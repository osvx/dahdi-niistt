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
 
#ifndef _NIISTT_E1_H

	#define _NIISTT_E1_H
	
	#ifdef NEED_PCI_IDS
	
		#define PCI_VENDOR_ID_XILINX	0x10ee
		
		#ifdef __KERNEL__
			static DEFINE_PCI_DEVICE_TABLE(niistt_e1_dev_pci_ids) =
		#else
			#define PCI_ANY_ID -1
			static struct tor2_pci_id 
			{
				int vendor;
				int device;
				int subvendor;
				int subdevice;
				int class;
				int classmask;
				unsigned long driver_data;
			} niistt_e1_dev_pci_ids[] =
		#endif		
			{
				{ PCI_VENDOR_ID_XILINX, 0x0007, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long)"NIISTT Quad E1 Module" },
				{ 0, }
			};

		#ifndef __KERNEL__
		
			/* We provide a simple routine to match the given ID's */
			static inline int niistt_e1_dev_pci_match(int vendorid, int deviceid, char **variant)
			{
				/* Returns 1 if this is a tormenta card or 0 if it isn't */
				int x;
				for (x = 0; x< sizeof(niistt_e1_dev_pci_ids) / sizeof(niistt_e1_dev_pci_ids[0]); x++)
					if (((niistt_e1_dev_pci_ids[x].vendor == PCI_ANY_ID) || 
						(niistt_e1_dev_pci_ids[x].vendor == vendorid)) &&
						((niistt_e1_dev_pci_ids[x].device == PCI_ANY_ID) ||
						(niistt_e1_dev_pci_ids[x].device == deviceid))) 
					{
						*variant = (char *)niistt_e1_dev_pci_ids[x].driver_data;
						return 1;
					}
			
				if (variant)
					*variant = NULL;
				return 0;
			}
		#endif
	#endif
#endif

