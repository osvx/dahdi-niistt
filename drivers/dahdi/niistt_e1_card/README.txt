Driver for NIISTT PCIe Quard E1 card

Configuration
-------------
 For use HDLC uncomment  CONFIG_DAHDI_NET in ./include/dahdi/dahdi_config.h

Installation
-------------
 make MODULES_EXTRA="niistt_e1_card/niistt_e1"
 make MODULES_EXTRA="niistt_e1_card/niistt_e1" install

Other information
-----------------
 For more information see: http://www.niistt.ru/p-01/addon-cards/comm/add-e1x4/
