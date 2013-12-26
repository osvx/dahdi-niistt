/*
 * NIISTT PCIe Quad E1 DAHDI Driver
 *
 * Written by Denis Scherbakov <scherbakov.ds@niistt.ru>
 * Based on previous works, designs, and archetectures conceived and
 * written by Jim Dixon <jim@lambdatel.com>.
 *
 * Copyright (C) 2001 Jim Dixon / Zapata Telephony.
 * Copyright (C) 2001-2013, NIISTT, CJSC.
 *
 */

/*
 * See http://www.asterisk.org for more information about
 * the Asterisk project. Please do not directly contact
 * any of the maintainers of this project for assistance;
 * the project provides a web site, mailing lists and IRC
 * channels for your use.
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */
 
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/kthread.h> 
#include <linux/sched.h> 
#include <linux/time.h> 
#include <linux/semaphore.h>

#include <dahdi/kernel.h>
#define NEED_PCI_IDS

#include "niistt_e1.h"

/* #define ENABLE_TASKLETS */
/* this stuff needs to work for 64 bit systems, however using the macro causes
   it to take twice as long */
/* #define FIXTHISFOR64 */  /* as of now, un-comment for 32 bit only system */

#define SPANS_PER_CARD  4
#define MAX_SPANS       16

//---------------------------------------------------------------------------------
#define dev_blck_offset 0x0000
	#define int_status_reg 0x02
#define idt_blck_offset 0x400
	#define idt_ctrl 0x01
		#define IDT_RST_N 0x00
		#define IDT_TCLR_N 0x01
		#define IDT_IN_sel 0x02
		#define IDT_MODE_sel0 0x03
		#define IDT_MODE_sel1 0x04	
	
	#define idt_status 0x02
		#define IDT_Lock 0x00
		#define IDT_MON_out0 0x01
		#define IDT_MON_out1 0x02
		#define IDT_Normal 0x03
		#define IDT_Holdover 0x04
		#define IDT_Freerun 0x05
#define spi_blck_offset 0x800
#define max_blck_offset 0xC00
//--------------------------------------------------------------------------------

struct niistt_e1_dev_chan {
	struct niistt_e1_dev *e1_dev;
	int span;
};

struct niistt_e1_dev_span {
	struct niistt_e1_dev *e1_dev;
	int span;
	struct dahdi_span dahdi_span;
};

struct niistt_e1_dev {
	/* This structure exists one per card */	
	struct pci_dev *pci;			/* Pointer to PCI device */
	int num;						/* Which card we are */
	int syncsrc;					/* active sync source */
	int syncs[SPANS_PER_CARD];		/* sync sources */
	int psyncs[SPANS_PER_CARD];		/* span-relative sync sources */
	int alarmtimer[SPANS_PER_CARD];	/* Alarm timer */
	char *type;	
	int irq;						/* IRQ used by device */
	int order;						/* Order */
	int flags;						/* Device flags */
	
	int syncpos[SPANS_PER_CARD];	/* span-relative sync sources */
	
	unsigned long plx_region;		/*PIO mem*/
	unsigned long plx_len;			
	unsigned int *plx;
	
	unsigned long xilinx32_len;		/*DMA mem*/
	
	unsigned int *tx_mem32;			/*DMA mem*/
	unsigned int tx_mem32_phy;
	
	unsigned int *rx_mem32;			/*DMA mem*/
	unsigned int rx_mem32_phy;
	
	spinlock_t spi_lock; 			/*spi spinlock*/
	
	struct task_struct *NIISTT_E1_BW;/*background worker*/
	int loopcnt;
		
	struct dahdi_device *ddev;
	struct niistt_e1_dev_span tspans[SPANS_PER_CARD];		/* Span data */
	struct dahdi_chan **chans[SPANS_PER_CARD]; 				/* Pointers to card channels */
	struct niistt_e1_dev_chan tchans[32 * SPANS_PER_CARD];	/* Channel user data */
	unsigned char txsigs[SPANS_PER_CARD][16];				/* Copy of tx sig registers */

	int spansstarted;		/* number of spans started */
		
	unsigned char leds;		/* copy of LED register */
	
	unsigned char ec_chunk1[SPANS_PER_CARD][32][DAHDI_CHUNKSIZE]; /* first EC chunk buffer */
	unsigned char ec_chunk2[SPANS_PER_CARD][32][DAHDI_CHUNKSIZE]; /* second EC chunk buffer */
	
#ifdef ENABLE_TASKLETS
	int taskletrun;
	int taskletsched;
	int taskletpending;
	int taskletexec;
	int txerrors;
	struct tasklet_struct niistt_e1_dev_tlet;
#endif
	unsigned int *datxlt;	/* pointer to datxlt structure */
	unsigned int passno;	/* number of interrupt passes */
};


#ifdef ENABLE_TASKLETS
static void niistt_e1_dev_tasklet(unsigned long data);
#endif

#define	LEDRED	2
#define	LEDGREEN 1

#define MAX_CARDS 64

static struct niistt_e1_dev *cards[MAX_CARDS];

static int loopback = 0;
static int debug = 0;
static int highestorder;

static int niistt_e1_dev_startup(struct file *file, struct dahdi_span *span);
static int niistt_e1_dev_shutdown(struct dahdi_span *span);
static int niistt_e1_dev_rbsbits(struct dahdi_chan *chan, int bits);
static int niistt_e1_dev_maint(struct dahdi_span *span, int cmd);
static int niistt_e1_dev_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data);
DAHDI_IRQ_HANDLER(niistt_e1_dev_intr);

/////////////////////////////////////////////////////////////////////////////////
static int bw_thread_init (struct niistt_e1_dev *e1_dev);
static int bw_thread_stop (struct niistt_e1_dev *e1_dev); 
static int bw_thread_fn (void *e1_dev);
/////////////////////////////////////////////////////////////////////////////////

/* translations of data channels for 30/31 channels in a 32 bit PCM highway */
static unsigned datxlt_e1[] = {1 ,2 ,3 ,4 ,5 ,6 ,7 ,8 ,9 ,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31 };

//Процедуры чтения/записи по SPI для DS21Q58
///////////////////////////////////////////////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------
unsigned int reverse(register unsigned int x)
{
        x = (((x & 0xaaaaaaaa) >> 1) | ((x & 0x55555555) << 1));
        x = (((x & 0xcccccccc) >> 2) | ((x & 0x33333333) << 2));
        x = (((x & 0xf0f0f0f0) >> 4) | ((x & 0x0f0f0f0f) << 4));
        x = (((x & 0xff00ff00) >> 8) | ((x & 0x00ff00ff) << 8));
        return((((x >> 16) | (x << 16)))>>16);
}
//-------------------------------------------------------------------------------

//-------------------------------------------------------------------------------
unsigned char t1in(struct niistt_e1_dev *e1_dev, int span, unsigned char addr) 
{
	unsigned int* ptr;
	ptr = &e1_dev->plx[max_blck_offset];
	writel(span-1, &ptr[9]);
	ptr = &e1_dev->plx[spi_blck_offset];
	writel(0x190050, &ptr[1]);
	writel(reverse(((0xFF00)|(((addr<<1)|(0x01))&0xff))), &ptr[3]);
	writel(0x01, &ptr[2]);
	while(readl(&e1_dev->plx[spi_blck_offset + 2])!=0x00);
	return (unsigned char)((reverse(readl(&e1_dev->plx[spi_blck_offset + 4])&0xFF))>>8);
}
//---------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
unsigned char t1in_sl(struct niistt_e1_dev *e1_dev, int span, unsigned char addr) 
{
	unsigned char ret =0;

	spin_lock(&e1_dev->spi_lock);
	ret = t1in(e1_dev,span,addr);
	spin_unlock(&e1_dev->spi_lock);
		
	return ret;
}
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
void t1out(struct niistt_e1_dev *e1_dev, int span, unsigned char addr, unsigned char val)
{	 
	unsigned int* ptr;
	ptr = &e1_dev->plx[max_blck_offset];
	writel(span-1, &ptr[9]);
	ptr = &e1_dev->plx[spi_blck_offset];
	writel(0x190050, &ptr[1]);
	writel(reverse(((val<<8)|((addr<<1)&0x7e))), &ptr[3]);
	writel(0x01, &ptr[2]);
	while(readl(&e1_dev->plx[spi_blck_offset + 2])!=0x00); 
}
//----------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------
void t1out_sl(struct niistt_e1_dev *e1_dev, int span, unsigned char addr, unsigned char val)
{
	spin_lock(&e1_dev->spi_lock);
	t1out(e1_dev,span,addr,val);
	spin_unlock(&e1_dev->spi_lock);
}
//-----------------------------------------------------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	
///////////////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_spanconfig(struct file *file, struct dahdi_span *span, struct dahdi_lineconfig *lc)
{	
	int i;
	struct niistt_e1_dev_span *p = container_of(span, struct niistt_e1_dev_span, dahdi_span);
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_spanconfig\n");
	
	printk(KERN_INFO "NIISTT: Configuring span %d\n", span->spanno);

	if ((lc->sync < 0) || (lc->sync > SPANS_PER_CARD)) 
	{
		printk(KERN_WARNING "NIISTT: %s %d: invalid span timing value %d.\n",
				THIS_MODULE->name, span->spanno, lc->sync);
		return -EINVAL;
	}
	span->syncsrc = p->e1_dev->syncsrc;
	
	/* remove this span number from the current sync sources, if there */
	for (i = 0; i < SPANS_PER_CARD; i++)
	{
		if (p->e1_dev->syncs[i] == span->spanno) 
		{
			p->e1_dev->syncs[i] = 0;
			p->e1_dev->psyncs[i] = 0;
		}
	}
	p->e1_dev->syncpos[p->span] = lc->sync;
	/* if a sync src, put it in the proper place */
	if (lc->sync) {
		p->e1_dev->syncs[lc->sync - 1] = span->spanno;
		p->e1_dev->psyncs[lc->sync - 1] = p->span + 1;
	}
	/* If we're already running, then go ahead and apply the changes */
	if (span->flags & DAHDI_FLAG_RUNNING)
		return niistt_e1_dev_startup(file, span);

	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_chanconfig(struct file *file, struct dahdi_chan *chan, int sigtype)
{
	int alreadyrunning;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_chanconfig\n");
	
	alreadyrunning = chan->span->flags & DAHDI_FLAG_RUNNING;

	if (alreadyrunning)
		printk(KERN_INFO "NIISTT: Reconfigured channel %d (%s) sigtype %d\n", chan->channo, chan->name, sigtype);
	else
		printk(KERN_INFO "NIISTT: Configured channel %d (%s) sigtype %d\n", chan->channo, chan->name, sigtype);
		
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_open(struct dahdi_chan *chan)
{
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_open\n");
	return 0;
}

static int niistt_e1_dev_close(struct dahdi_chan *chan)
{
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_close\n");
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////

static const struct dahdi_span_ops niistt_e1_dev_span_ops = {
	.owner = THIS_MODULE,
	.spanconfig = niistt_e1_dev_spanconfig,
	.chanconfig = niistt_e1_dev_chanconfig,
	.startup = niistt_e1_dev_startup,
	.shutdown = niistt_e1_dev_shutdown,
	.rbsbits = niistt_e1_dev_rbsbits,
	.maint = niistt_e1_dev_maint,
	.open = niistt_e1_dev_open,
	.close  = niistt_e1_dev_close,
	.ioctl = niistt_e1_dev_ioctl,
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void init_spans(struct niistt_e1_dev *e1_dev)
{
	int x, y, c;
	if(debug)printk(KERN_DEBUG "NIISTT: init_spans\n");
		
	/* TODO: a debug printk macro */
	for (x = 0; x < SPANS_PER_CARD; x++) 
	{
		struct dahdi_span *const s = &e1_dev->tspans[x].dahdi_span;
		sprintf(s->name, "NIISTT: Quad E1 Module /%d/%d", e1_dev->num, x + 1);
		snprintf(s->desc, sizeof(s->desc) - 1, "NIISTT: Quad E1 Card %d Span %d", e1_dev->num, x + 1);
			 
		s->channels = 31;
		s->deflaw = DAHDI_LAW_ALAW;
		s->linecompat = DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4;
		s->spantype = SPANTYPE_DIGITAL_E1;

		s->chans = e1_dev->chans[x];
		s->flags = DAHDI_FLAG_RBS;
		s->ops = &niistt_e1_dev_span_ops;

		e1_dev->tspans[x].e1_dev = e1_dev;
		e1_dev->tspans[x].span = x;
		
		for (y = 0; y < s->channels; y++) 
		{
			struct dahdi_chan *mychans = e1_dev->chans[x][y];
			sprintf(mychans->name, "NIISTT Quad E1 Module /%d/%d/%d", e1_dev->num, x + 1, y + 1);
			mychans->sigcap = DAHDI_SIG_EM | DAHDI_SIG_CLEAR | DAHDI_SIG_FXSLS | DAHDI_SIG_FXSGS | DAHDI_SIG_FXSKS |
 									 DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_FXOKS | DAHDI_SIG_CAS | DAHDI_SIG_SF | DAHDI_SIG_EM_E1;
			c = (x * s->channels) + y;
			mychans->pvt = &e1_dev->tchans[c];
			mychans->chanpos = y + 1;
			e1_dev->tchans[c].span = x;
			e1_dev->tchans[c].e1_dev = e1_dev;
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int __devinit niistt_e1_dev_launch(struct niistt_e1_dev *e1_dev)
{
	int res;
	struct dahdi_span *s;
	int i;
	
	if(debug) printk(KERN_DEBUG "NIISTT: niistt_e1_dev_launch\n");

	if (test_bit(DAHDI_FLAGBIT_REGISTERED, &e1_dev->tspans[0].dahdi_span.flags))
		return 0;

	e1_dev->ddev = dahdi_create_device();
	e1_dev->ddev->location = kasprintf(GFP_KERNEL, "NIISTT: PCI Bus %02d Slot %02d",
				       e1_dev->pci->bus->number,
				       PCI_SLOT(e1_dev->pci->devfn) + 1);

	if (!e1_dev->ddev->location)
		return -ENOMEM;

	printk(KERN_INFO "NIISTT: Launching card: %d\n", e1_dev->order);
	for (i = 0; i < SPANS_PER_CARD; ++i) {
		s = &e1_dev->tspans[i].dahdi_span;
		list_add_tail(&s->device_node, &e1_dev->ddev->spans);
	}

	res = dahdi_register_device(e1_dev->ddev, &e1_dev->pci->dev);
	if (res) {
		dev_err(&e1_dev->pci->dev, "NIISTT: Unable to register with DAHDI.\n");
		return res;
	}
		
	/* enable interrupt */
	//ptr = &e1_dev->plx[max_blck_offset];
	//writel(0x1b, &ptr[1]);

#ifdef ENABLE_TASKLETS
	tasklet_init(&e1_dev->niistt_e1_dev_tlet, niistt_e1_dev_tasklet, (unsigned long)e1_dev);
#endif
	return 0;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void free_niistt_e1_dev(struct niistt_e1_dev *e1_dev)
{
	unsigned int x, f;
	
	if(debug)printk(KERN_DEBUG "NIISTT: free_niistt_e1_dev\n");
	
	for (x = 0; x < SPANS_PER_CARD; x++) {
		for (f = 0; f < 31; f++) {
			if (e1_dev->chans[x][f]) {
				kfree(e1_dev->chans[x][f]);
			}
		}
		if (e1_dev->chans[x])
			kfree(e1_dev->chans[x]);
	}
	kfree(e1_dev->tx_mem32);
	kfree(e1_dev->rx_mem32);
	kfree(e1_dev->ddev->location);
	dahdi_free_device(e1_dev->ddev);
	kfree(e1_dev);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int __devinit niistt_e1_dev_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	int res,x,f;
	int ret = -ENODEV;
	struct niistt_e1_dev *e1_dev;
	unsigned int* ptr;
	
	if (debug) printk(KERN_DEBUG "NIISTT: niistt_e1_dev_probe\n");
	
	res = pci_enable_device(pdev);
	if (res)return res;
	
	res = pci_enable_msi(pdev);
	if (res)
	{
        printk(KERN_CRIT"NIISTT: Could not allocate MSI IRQ\n");
		return res;
    }
	
	e1_dev = kmalloc(sizeof(struct niistt_e1_dev), GFP_KERNEL);
	if (!e1_dev)return -ENOMEM;
	memset(e1_dev,0,sizeof(struct niistt_e1_dev));
	
	spin_lock_init(&e1_dev->spi_lock);
	
	e1_dev->pci = pdev;
	e1_dev->irq = pdev->irq;
	
	if (e1_dev->irq < 1) 
	{
		printk(KERN_ERR "NIISTT: No IRQ allocated for device\n");
		goto err_out_free_niistt_e1_dev;
	}
	
	e1_dev->plx_region = pci_resource_start(pdev, 0);
	e1_dev->plx_len = pci_resource_len(pdev, 0);
	e1_dev->plx = (unsigned int *)ioremap(e1_dev->plx_region, e1_dev->plx_len);
	
	e1_dev->type = (char *)ent->driver_data;
	
	if (!e1_dev->plx_region || !e1_dev->plx || (pci_resource_flags(pdev, 0) & IORESOURCE_IO)) 
	{
		printk(KERN_ERR "NIISTT: Invalid Quad E1 Module Base resource\n");
		goto err_out_free_niistt_e1_dev;
	}
	
	if (!request_mem_region(e1_dev->plx_region, e1_dev->plx_len, e1_dev->type)) {
		printk(KERN_ERR "NIISTT: Unable to reserve Quad E1 Module memory %08lx window at %08lx\n",
		       e1_dev->plx_len, e1_dev->plx_region);
		goto err_out_free_niistt_e1_dev;
	}
	
	e1_dev->xilinx32_len = 4096;
	ptr = &e1_dev->plx[max_blck_offset];
	
	e1_dev->tx_mem32 = kmalloc(e1_dev->xilinx32_len, GFP_KERNEL | GFP_DMA32);	
	if (!e1_dev->tx_mem32) goto err_out_free_niistt_e1_dev;	
	
	e1_dev->tx_mem32_phy = dma_map_single(&pdev->dev, e1_dev->tx_mem32, e1_dev->xilinx32_len, DMA_TO_DEVICE);
	memset(e1_dev->tx_mem32,0x00,e1_dev->xilinx32_len);	
	
	if(e1_dev->tx_mem32_phy!=0)
	{
		printk(KERN_INFO "NIISTT: PHY TX address: %x\n",e1_dev->tx_mem32_phy);
	}
	else
	{
		printk(KERN_INFO "NIISTT: PHY TX address error\n");
		goto err_out_free_niistt_e1_dev;
	}	
	
	writel(e1_dev->tx_mem32_phy, &ptr[6]);
	
		
	e1_dev->rx_mem32 = kmalloc(e1_dev->xilinx32_len, GFP_KERNEL | GFP_DMA32);	
	if (!e1_dev->rx_mem32) goto err_out_free_niistt_e1_dev;	
	
	e1_dev->rx_mem32_phy = dma_map_single(&pdev->dev, e1_dev->rx_mem32, e1_dev->xilinx32_len, DMA_FROM_DEVICE);
	memset(e1_dev->rx_mem32,0x00,e1_dev->xilinx32_len);
	
	if(e1_dev->rx_mem32_phy!=0)
	{
		printk(KERN_INFO "NIISTT: PHY RX address: %x\n",e1_dev->rx_mem32_phy);
	}
	else
	{
		printk(KERN_INFO "NIISTT: PHY RX address error\n");
		goto err_out_free_niistt_e1_dev;
	}	

	writel(e1_dev->rx_mem32_phy, &ptr[3]);	
	
	pci_set_drvdata(pdev, e1_dev);
		
	printk(KERN_INFO "NIISTT: Detected %s at 0x%lx irq %d\n", e1_dev->type, e1_dev->plx_region, e1_dev->irq);
	
	for (x = 0; x < MAX_CARDS; x++) 
	{
		if (!cards[x]) break;
	}
	if (x >= MAX_CARDS) 
	{
		printk(KERN_ERR "NIISTT: No cards[] slot available!!\n");
		goto err_out_release_all;
	}
	
	e1_dev->num = x;
	cards[x] = e1_dev;

	printk(KERN_INFO "NIISTT: Identification number: %x\n", (unsigned int)readl(&e1_dev->plx[0]));

	writel(0x00, &e1_dev->plx[idt_blck_offset + idt_ctrl]);
	writel(0x13, &e1_dev->plx[idt_blck_offset + idt_ctrl]);
		
	if (request_irq(e1_dev->irq, niistt_e1_dev_intr, DAHDI_IRQ_SHARED_DISABLED|IRQF_NOBALANCING, "NIISTT_E1_card", e1_dev)) 
	{
		printk(KERN_ERR "NIISTT: Unable to request IRQ %d\n", e1_dev->irq);
		goto err_out_release_all;
	}

	printk(KERN_INFO "NIISTT: NIISTT Quad E1 Card\n");
	e1_dev->datxlt = datxlt_e1;

	for (x = 0; x < SPANS_PER_CARD; x++) 
	{
		int num_chans = 31;
		
		if (!(e1_dev->chans[x] = kmalloc(num_chans * sizeof(*e1_dev->chans[x]), GFP_KERNEL))) 
		{
			printk(KERN_ERR "NIISTT: Not enough memory for chans[%d]\n", x);
			ret = -ENOMEM;
			goto err_out_release_all;
		}
		for (f = 0; f < (num_chans); f++) 
		{
			if (!(e1_dev->chans[x][f] = kmalloc(sizeof(*e1_dev->chans[x][f]), GFP_KERNEL))) 
			{
				printk(KERN_ERR "NIISTT: Not enough memory for chans[%d][%d]\n", x, f);
				ret = -ENOMEM;
				goto err_out_release_all;
			}
			memset(e1_dev->chans[x][f], 0, sizeof(*e1_dev->chans[x][f]));
		}
	}

	init_spans(e1_dev); 

	e1_dev->order = highestorder; 
	
	bw_thread_init(e1_dev);
	
	printk(KERN_INFO "NIISTT: Detected Card number: %d\n", e1_dev->order);

	niistt_e1_dev_launch(cards[e1_dev->num]);

	/* If we found at least one, increment the highest order and search again, otherwise stop */
	highestorder++;

	return 0;

err_out_release_all:	
err_out_free_niistt_e1_dev:
	pci_disable_msi(pdev);
	if (e1_dev->plx) iounmap((void*)e1_dev->plx);
	if (e1_dev) {free_niistt_e1_dev(e1_dev);}
	return ret;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct pci_driver niistt_e1_dev_driver;
static void __devexit niistt_e1_dev_remove(struct pci_dev *pdev)
{	
	struct niistt_e1_dev *e1_dev;
	unsigned int* ptr;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_remove\n");
	
	e1_dev = pci_get_drvdata(pdev);
	if (!e1_dev) BUG();
	
	bw_thread_stop(e1_dev);
	
	/*disable interrupt*/
	ptr = &e1_dev->plx[max_blck_offset];
	writel(0x00, &ptr[1]);
	
	/*leds off*/
	ptr = &e1_dev->plx[max_blck_offset];
	writel(0x00, &ptr[7]);
		
	free_irq(e1_dev->irq, e1_dev);
	
	dma_unmap_single(&pdev->dev, e1_dev->tx_mem32_phy, e1_dev->xilinx32_len, DMA_TO_DEVICE);
	dma_unmap_single(&pdev->dev, e1_dev->rx_mem32_phy, e1_dev->xilinx32_len, DMA_FROM_DEVICE);
	
	dahdi_unregister_device(e1_dev->ddev);
	
	release_mem_region(e1_dev->plx_region, e1_dev->plx_len);
	if (e1_dev->plx) iounmap(e1_dev->plx);
	
	cards[e1_dev->num] = NULL;
	pci_disable_msi(pdev);
	
	pci_set_drvdata(pdev, NULL);
	free_niistt_e1_dev(e1_dev);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
static struct pci_driver niistt_e1_dev_driver = {
	.name = "NIISTT Quad E1 Module",
	.probe = niistt_e1_dev_probe,
	.remove = __devexit_p(niistt_e1_dev_remove),
	.id_table = niistt_e1_dev_pci_ids,
};

static int __init niistt_e1_dev_init(void) {
	int res;
	res = dahdi_pci_module(&niistt_e1_dev_driver);
	printk(KERN_INFO "NIISTT: Registered NIISTT Quad E1 Module PCIe\n");
	return res;
}

static void __exit niistt_e1_dev_cleanup(void) {
	pci_unregister_driver(&niistt_e1_dev_driver);
	printk(KERN_INFO "NIISTT: Unregistered NIISTT Quad E1 Module PCIe\n");
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_rbsbits(struct dahdi_chan *chan, int bits)
{	
	u_char m,c;
	int k,n,b;
	struct niistt_e1_dev_chan *p = chan->pvt;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_rbsbits\n");
		
	printk(KERN_DEBUG "NIISTT: Setting bits to %d on channel %s\n", bits, chan->name);

		if (chan->chanpos == 16) return 0;
		n = chan->chanpos - 1;
		if (chan->chanpos > 16) n--;
		k = p->span;
		b = (n % 15) + 1;
		c = p->e1_dev->txsigs[k][b];
		m = (n / 15) * 4; /* nibble selector */
		c &= (15 << m); /* keep the other nibble */
		c |= (bits & 15) << (4 - m); /* put our new nibble here */
		p->e1_dev->txsigs[k][b] = c;
		/* output them to the chip */
		t1out_sl(p->e1_dev,k + 1,0x30 + b,c);
		return 0;						
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_shutdown(struct dahdi_span *span)
{
	int i;
	int tspan;
	int wasrunning;
	unsigned int* ptr;
	struct niistt_e1_dev_span *const p = container_of(span, struct niistt_e1_dev_span, dahdi_span);
	struct niistt_e1_dev *const e1_dev = p->e1_dev;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_shutdown\n");
	
	tspan = p->span + 1;
	if (tspan < 0) 
	{
		printk(KERN_DEBUG "NIISTT: Span '%d' isn't us?\n", span->spanno);
		return -1;
	}

	wasrunning = span->flags & DAHDI_FLAG_RUNNING;

	span->flags &= ~DAHDI_FLAG_RUNNING;
	
	/* Zero out all registers */
	for (i = 0; i < 64; i++)
		t1out_sl(e1_dev, tspan, i, 0);

	if (wasrunning)	e1_dev->spansstarted--;
		
	if (!(e1_dev->tspans[0].dahdi_span.flags & DAHDI_FLAG_RUNNING) &&
	    !(e1_dev->tspans[1].dahdi_span.flags & DAHDI_FLAG_RUNNING) &&
	    !(e1_dev->tspans[2].dahdi_span.flags & DAHDI_FLAG_RUNNING) &&
	    !(e1_dev->tspans[3].dahdi_span.flags & DAHDI_FLAG_RUNNING))
		{
			/*disable interrupt*/
			ptr = &e1_dev->plx[max_blck_offset];
			writel(0x00, &ptr[1]); 
		}
	
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_startup(struct file *file, struct dahdi_span *span)
{
	int i;
	int tspan;
	char *coding;
	char *framing;
	char *crcing;
	int alreadyrunning;
	unsigned int* ptr;
	struct niistt_e1_dev_span *p = container_of(span, struct niistt_e1_dev_span, dahdi_span);
	unsigned char tcr,ccr1,ccr6,tcr2;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_startup\n");
	
	tspan = p->span + 1;
	if (tspan < 0) 
	{
		printk(KERN_DEBUG "NIISTT: Span '%d' isn't us?\n", span->spanno);
		return -1;
	}
	
	alreadyrunning = span->flags & DAHDI_FLAG_RUNNING;

	/* initialize the start value for the entire chunk of last ec buffer */
	for (i = 0; i < span->channels; i++)
	{
		memset(p->e1_dev->ec_chunk1[p->span][i],
			DAHDI_LIN2X(0,span->chans[i]),DAHDI_CHUNKSIZE);
		memset(p->e1_dev->ec_chunk2[p->span][i],
			DAHDI_LIN2X(0,span->chans[i]),DAHDI_CHUNKSIZE);
	}
		
	if (!alreadyrunning) 
	{		
		/* Zero out all registers */
		for (i = 0; i < 64; i++) 
			t1out_sl(p->e1_dev,tspan, i, 0);
			
		/* re-evaluate active sync src (no cable version) */
		p->e1_dev->syncsrc = p->e1_dev->psyncs[0];
				
		//sync src config		
		if(p->e1_dev->syncsrc==0)
		{
			t1out_sl(p->e1_dev,1,0x1D, 0x00);
			writel(0x13, &p->e1_dev->plx[idt_blck_offset + idt_ctrl]);
		}
		else
		{
			t1out_sl(p->e1_dev,1,0x1D, (p->e1_dev->syncsrc&0x07));
			writel(0x07, &p->e1_dev->plx[idt_blck_offset + idt_ctrl]);
		}
		
		//receive control register
		t1out_sl(p->e1_dev,tspan,0x10, 0x30);
		
		//transmit control register
		t1out_sl(p->e1_dev,tspan,0x11, 0x08);
				
		//Line inerface control register
		t1out_sl(p->e1_dev,tspan,0x17, 0x23);			
	}
		
	ccr1 = 0x00;
	ccr6 = 0x00;
	crcing = "";
	tcr = 0x08; /* base TCR1 value: TSis mode */
	tcr2 = 0x00;
	
	if (span->lineconfig & DAHDI_CONFIG_CCS) 
	{
		ccr1 |= 0x08;
		coding = "CCS";
	} else 
	{
		ccr6 |= 0x08;
		coding = "CAS";
	}
	
	if (span->lineconfig & DAHDI_CONFIG_HDB3) 
	{
		ccr1 |= 0x44; /* CCR1: TX and RX HDB3 */
		framing = "HDB3";
	}else 
	{
		framing = "AMI";
	}
	
	if (span->lineconfig & DAHDI_CONFIG_CRC4) 
	{
		ccr1 |= 0x11; /* CCR1: TX and TX CRC4 */
		tcr |= 0x20; /* TCR2: CRC4 bit auto */
		crcing = "/CRC4";
	} 
	
	t1out_sl(p->e1_dev,tspan,0x11,tcr);
	t1out_sl(p->e1_dev,tspan,0x12,ccr1);
	t1out_sl(p->e1_dev,tspan,0x2f,ccr6);

	if (!alreadyrunning) 
	{

		t1out_sl(p->e1_dev,tspan,0x15,0x80); /* CCR4: LIRST*/
		t1out_sl(p->e1_dev,tspan,0x20,0x1b); /* TAFR */
		t1out_sl(p->e1_dev,tspan,0x21,0x5f); /* TNAFR */
		t1out_sl(p->e1_dev,tspan,0x30,0x0b); /* TSR1 */
		for (i = 0x31; i <= 0x3f; i++) t1out_sl(p->e1_dev,tspan,i,0x55);
		
		t1out_sl(p->e1_dev,tspan,0x15,0x00); 
		t1out_sl(p->e1_dev,tspan,0x1b,0x60); /* CCR3: set also ESR */
			
		span->flags |= DAHDI_FLAG_RUNNING;
		p->e1_dev->spansstarted++;
		
		/* enable interrupt */
		ptr = &p->e1_dev->plx[max_blck_offset];
		writel(0x1b, &ptr[1]);
	}
	
	if (p->e1_dev->syncs[0] == span->spanno) printk(KERN_INFO "NIISTT: SPAN %d: Primary Sync Source\n",span->spanno);
	if (p->e1_dev->syncs[1] == span->spanno) printk(KERN_INFO "NIISTT: SPAN %d: Secondary Sync Source\n",span->spanno);
	if (p->e1_dev->syncs[2] == span->spanno) printk(KERN_INFO "NIISTT: SPAN %d: Tertiary Sync Source\n",span->spanno);
	if (p->e1_dev->syncs[3] == span->spanno) printk(KERN_INFO "NIISTT: SPAN %d: Quaternary Sync Source\n",span->spanno);
		
	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_maint(struct dahdi_span *span, int cmd)
{
	struct niistt_e1_dev_span *p = container_of(span, struct niistt_e1_dev_span, dahdi_span);
	
	int tspan = p->span + 1;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_maint\n");

	switch(cmd) 
	{
		case DAHDI_MAINT_NONE:
			t1out_sl(p->e1_dev,tspan,0x14,0); /* no loops */
		break;
		case DAHDI_MAINT_LOCALLOOP:
			t1out_sl(p->e1_dev,tspan,0x14,0x40); /* local loop */
		break;
		case DAHDI_MAINT_REMOTELOOP:
			t1out_sl(p->e1_dev,tspan,0x14,0x80); /* remote loop */
		break;
		case DAHDI_MAINT_LOOPUP:
		case DAHDI_MAINT_LOOPDOWN:
			return -ENOSYS;
		default:
			printk(KERN_NOTICE "NIISTT: Unknown maint command: %d\n", cmd);
		break;
	}
	
	return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////
static inline void niistt_e1_dev_run(struct niistt_e1_dev *e1_dev)
{
	int x,y;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_run\n");
		
	for (x = 0; x < SPANS_PER_CARD; x++) 
	{
		struct dahdi_span *const s = &e1_dev->tspans[x].dahdi_span;
		if (s->flags & DAHDI_FLAG_RUNNING) 
		{

			for (y = 0; y < s->channels; y++) 
			{
				dahdi_ec_chunk(s->chans[y], s->chans[y]->readchunk, e1_dev->ec_chunk2[x][y]);
				memcpy(e1_dev->ec_chunk2[x][y],e1_dev->ec_chunk1[x][y], DAHDI_CHUNKSIZE);
				memcpy(e1_dev->ec_chunk1[x][y],s->chans[y]->writechunk, DAHDI_CHUNKSIZE);
			}
			dahdi_receive(s);
		}
	}
	for (x = 0; x < SPANS_PER_CARD; x++) 
	{
		struct dahdi_span *const s = &e1_dev->tspans[x].dahdi_span;
		if (s->flags & DAHDI_FLAG_RUNNING)
			dahdi_transmit(s);
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef ENABLE_TASKLETS
static void niistt_e1_dev_tasklet(unsigned long data)
{
	struct niistt_e1_dev *e1_dev = (struct niistt_e1_dev *)data;
	
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_tasklet\n");
	
	e1_dev->taskletrun++;
	if (e1_dev->taskletpending) {
		e1_dev->taskletexec++;
		niistt_e1_dev_run(e1_dev);
	}
	e1_dev->taskletpending = 0;
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IRQ_HANDLER
////////////////////////////////////////////////////////////////////////////////////////////////////////////
DAHDI_IRQ_HANDLER(niistt_e1_dev_intr)
{
	int n, i;
	unsigned int rxword,txword;
	struct niistt_e1_dev *e1_dev = (struct niistt_e1_dev *) dev_id;	
	
		
		if(loopback) 
		{
			//memcpy(e1_dev->tx_mem32, e1_dev->rx_mem32, 4096);
		
			for (n = 0; n < e1_dev->tspans[0].dahdi_span.channels; n++) 
			{
				for (i = 0; i < DAHDI_CHUNKSIZE; i++) 
				{
					e1_dev->tspans[3].dahdi_span.chans[n]->readchunk[i] = e1_dev->tspans[3].dahdi_span.chans[n]->writechunk[i];	
					e1_dev->tspans[2].dahdi_span.chans[n]->readchunk[i] = e1_dev->tspans[2].dahdi_span.chans[n]->writechunk[i];
					e1_dev->tspans[1].dahdi_span.chans[n]->readchunk[i] = e1_dev->tspans[1].dahdi_span.chans[n]->writechunk[i];
					e1_dev->tspans[0].dahdi_span.chans[n]->readchunk[i] = e1_dev->tspans[0].dahdi_span.chans[n]->writechunk[i];
				}
			}
		}
		else
		{
			/* do the transmit output */
			for (n = 0; n < e1_dev->tspans[0].dahdi_span.channels; n++) 
			{
		
				for (i = 0; i < DAHDI_CHUNKSIZE; i++) 
				{

					/* span 1 */
					txword = e1_dev->tspans[3].dahdi_span.chans[n]->writechunk[i] << 24;
					/* span 2 */
					txword |= e1_dev->tspans[2].dahdi_span.chans[n]->writechunk[i] << 16;
					/* span 3 */
					txword |= e1_dev->tspans[1].dahdi_span.chans[n]->writechunk[i] << 8;
					/* span 4 */
					txword |= e1_dev->tspans[0].dahdi_span.chans[n]->writechunk[i];
					/* write to part */
			
				#ifdef FIXTHISFOR64
					e1_dev->tx_mem32[e1_dev->datxlt[n] + (32 * i)] = txword;
				#else
					writel(txword, &e1_dev->tx_mem32[e1_dev->datxlt[n] + (32 * i)]);
				#endif		
				}
			}
			
			/* Do the receive input */
			for (n = 0; n < e1_dev->tspans[0].dahdi_span.channels; n++) 
			{
				for (i = 0; i < DAHDI_CHUNKSIZE; i++) 
				{
					/* read from */
				#ifdef FIXTHISFOR64
					rxword = e1_dev->rx_mem32[e1_dev->datxlt[n] + (32 * i)];
				#else
					rxword = readl(&e1_dev->rx_mem32[e1_dev->datxlt[n] + (32 * i)]);										
				#endif 

					/* span 1 */
					e1_dev->tspans[3].dahdi_span.chans[n]->readchunk[i] = rxword >> 24;
					/* span 2 */
					e1_dev->tspans[2].dahdi_span.chans[n]->readchunk[i] = (rxword & 0xff0000) >> 16;
					/* span 3 */
					e1_dev->tspans[1].dahdi_span.chans[n]->readchunk[i] = (rxword & 0xff00) >> 8;
					/* span 4 */
					e1_dev->tspans[0].dahdi_span.chans[n]->readchunk[i] = rxword & 0xff;
				}
								
			}
		}
		
		e1_dev->passno++;
	
	#ifdef ENABLE_TASKLETS
		if (!e1_dev->taskletpending) 
		{
			e1_dev->taskletpending = 1;
			e1_dev->taskletsched++; 
			tasklet_hi_schedule(&e1_dev->niistt_e1_dev_tlet);
		} else 
		{
			e1_dev->txerrors++;
		}
	#else
		niistt_e1_dev_run(e1_dev);
	#endif
			
	return IRQ_RETVAL(1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//BACKGROUND THREAD
static int bw_thread_init (struct niistt_e1_dev *e1_dev) 
{
	if(debug)printk(KERN_DEBUG "NIISTT: bw_thread_init\n");

    e1_dev->NIISTT_E1_BW = kthread_run(bw_thread_fn, (void *)e1_dev, "NIISTT_E1_BW");
	
    return 0;
}

static int bw_thread_stop (struct niistt_e1_dev *e1_dev) 
{
	if(debug)printk(KERN_DEBUG "NIISTT: bw_thread_stop\n");

	if( e1_dev->NIISTT_E1_BW != NULL ) kthread_stop(e1_dev->NIISTT_E1_BW);
	
    return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int bw_thread_fn (void *arg_void) 
{	
	int i,j,k,n;
	unsigned char rxc, c, f;
	unsigned int* ptr;
	struct niistt_e1_dev *e1_dev = (struct niistt_e1_dev*)arg_void;
		
	if(debug)printk(KERN_DEBUG "NIISTT: bw_thread_fn\n");
		
	while( !kthread_should_stop() )
	{		
		f = 0;
								
		//CAS_SIGNALING------------------------------------------------------------------------------
		//not tested
		for (i = 1; i <= SPANS_PER_CARD; i++)
		{
			if(t1in(e1_dev, i, 0x0b)&0x80)
			{	
				f = 1;
				
				for (j = 0; j < 15; j++) 
				{
					//RECEIVE SIGNALING REGISTER
					c = t1in(e1_dev,i,0x31 + j);
					
					rxc = c & 15;
					if (rxc != e1_dev->tspans[i - 1].dahdi_span.chans[j + 16]->rxsig) 
					{
						/* Check for changes in received bits */
						if (!(e1_dev->tspans[i - 1].dahdi_span.chans[j + 16]->sig & DAHDI_SIG_CLEAR))
							dahdi_rbsbits(e1_dev->tspans[i - 1].dahdi_span.chans[j + 16], rxc);
					}
					
					rxc = c >> 4;
					if (rxc != e1_dev->tspans[i - 1].dahdi_span.chans[j]->rxsig) 
					{
						/* Check for changes in received bits */
						if (!(e1_dev->tspans[i - 1].dahdi_span.chans[j]->sig & DAHDI_SIG_CLEAR))
							dahdi_rbsbits(e1_dev->tspans[i - 1].dahdi_span.chans[j], rxc);
					}
				}
			}
		}
		//-----------------------------------------------------------------------------------------------
	
		//turn off yel all-------------------------------------------------------------------------------
		for (i = 0; i < SPANS_PER_CARD; i++) 
		{ 	
			if (e1_dev->alarmtimer[i]) 
			{
				if (!--e1_dev->alarmtimer[i]) 
				{
					/* clear recover status */
					e1_dev->tspans[i].dahdi_span.alarms &= ~DAHDI_ALARM_RECOVER;
					t1out_sl(e1_dev,i + 1,0x21,0x5f); /* turn off yel */
					dahdi_alarm_notify(&e1_dev->tspans[i].dahdi_span);  /* let them know */
				}
			}
		}
		//------------------------------------------------------------------------------------------------
		
		if(!f)
		{
			//------------------------------------------------------------------------------------------------
			if(e1_dev->loopcnt == 20)
			{
				for (i = 0; i < SPANS_PER_CARD; i++) 
				{
					j = 0;  /* clear this alarm status */

					c = t1in(e1_dev,i + 1,0x0a);  /* get the status */
				
					if (c & 9) /* if red alarm */
						j |= DAHDI_ALARM_RED;

					if (c & 2) /* if blue alarm */
						j |= DAHDI_ALARM_BLUE;
						
					/*only consider previous carrier alarm state */
					e1_dev->tspans[i].dahdi_span.alarms &= (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_NOTOPEN);
					n = 1;
				
					if (e1_dev->tspans[i].dahdi_span.lineconfig & DAHDI_CONFIG_NOTOPEN) 
					{
						/* go thru all chans, and count # open */
						for (n = 0, k = 0; k < e1_dev->tspans[i].dahdi_span.channels; k++) 
						{
							if (((e1_dev->chans[i][k])->flags & DAHDI_FLAG_OPEN) || dahdi_have_netdev(e1_dev->chans[i][k])) n++;
						}
						/* if none open, set alarm condition */
						if (!n) j |= DAHDI_ALARM_NOTOPEN;
					}
		
					/* if no more alarms, and we had some */
					if ((!j) && e1_dev->tspans[i].dahdi_span.alarms)
							e1_dev->alarmtimer[i] = DAHDI_ALARMSETTLE_TIME; 
					
					if (e1_dev->alarmtimer[i]) j |= DAHDI_ALARM_RECOVER;
	
					/* if going into alarm state, set yellow alarm */
					if ((j) && (!e1_dev->tspans[i].dahdi_span.alarms)) 
					{
						t1out_sl(e1_dev,i + 1,0x21,0x7f);
					}
		
					if (c & 4) /* if yellow alarm */
						j |= DAHDI_ALARM_YELLOW;
					
					if (e1_dev->tspans[i].dahdi_span.maintstat || e1_dev->tspans[i].dahdi_span.mainttimer)
						j |= DAHDI_ALARM_LOOPBACK;
				
					e1_dev->tspans[i].dahdi_span.alarms = j;
			
					dahdi_alarm_notify(&e1_dev->tspans[i].dahdi_span);
				}
			}
			//-------------------------------------------------------------------------------------
		
			//LEDS--------------------------------------------------------------------------------------
			if(e1_dev->loopcnt == 40)
			{
				for (i = 0; i < SPANS_PER_CARD; i++)
				{
				
					e1_dev->leds &= ~((LEDRED | LEDGREEN) << (2 * i));		
					if (e1_dev->tspans[i].dahdi_span.flags & DAHDI_FLAG_RUNNING) 
					{
						if(t1in(e1_dev,i+1,0x09)&(1<<2))
						{
							e1_dev->leds |= LEDGREEN << (2 * i);
						}
						else
						{
							e1_dev->leds |= (LEDRED | LEDGREEN) << (2 * i);
						}
					}

				}
				
				ptr = &e1_dev->plx[max_blck_offset];
				writel(e1_dev->leds, &ptr[7]);
			}
			//-------------------------------------------------------------------------------------------
		
			//STATISTIC-----------------------------------------------------------------------------------
			if(e1_dev->loopcnt == 60)
			{
				for (i = 1; i <= SPANS_PER_CARD; i++)
				{
					e1_dev->tspans[i - 1].dahdi_span.count.bpv += t1in(e1_dev, i, 1) + 
															(t1in(e1_dev, i, 0)<<8);
															
					if (e1_dev->tspans[i - 1].dahdi_span.lineconfig & DAHDI_CONFIG_CRC4)
					{
						e1_dev->tspans[i - 1].dahdi_span.count.crc4 += t1in(e1_dev, i, 3) +
																	((t1in(e1_dev, i, 2) & 3) << 8);
						e1_dev->tspans[i - 1].dahdi_span.count.ebit += t1in(e1_dev, i, 5) +
																	((t1in(e1_dev, i, 4) & 3) << 8);
					}
				
					e1_dev->tspans[i - 1].dahdi_span.count.fas += (t1in(e1_dev, i, 4) >> 2) +
																((t1in(e1_dev, i, 2) & 0x3F) << 6);
				}	
			}
			//-------------------------------------------------------------------------------------------
					
			if(++e1_dev->loopcnt > 60) e1_dev->loopcnt = 0;
		}
				
		msleep(1);
				
	}
	
	return 0;

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int niistt_e1_dev_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	if(debug)printk(KERN_DEBUG "NIISTT: niistt_e1_dev_ioctl\n");

	switch(cmd) 
	{
		default:
			return -ENOTTY;
	}
	return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

MODULE_AUTHOR("Denis Scherbakov");
MODULE_DESCRIPTION("NIISTT PCIe Quad E1 DAHDI Driver");
MODULE_LICENSE("GPL v2");

module_param(loopback, int, 0600);
module_param(debug, int, 0600);

MODULE_DEVICE_TABLE(pci, niistt_e1_dev_pci_ids);

module_init(niistt_e1_dev_init);
module_exit(niistt_e1_dev_cleanup);
