# M031BSP_ISP_XMODEM_DUAL_BANK
 M031BSP_ISP_XMODEM_DUAL_BANK

update @ 2022/06/06

1. modify project from https://github.com/released/M031BSP_ISP_XMODEM_APROM

	- use M032SE EVB , UART0 as Xmodem reeeive , UART1 as debug port print message
	
	- applicatin code addr : 0x0000 , size : 60K , a.k.a bank0

	- UNDER LDROM , by receive Xmodem to upgrade one empty MCU section (bank1 , addr : 0xF000 , size : 60K)
   
	- after Xmodem receive data finish , MCU will reset to process firmware update (from bank1 to bank0)
	
	- after update finish , follow regular power on flow , to verify checksum and jump to application code : bank0

2. APROM project : modify Srecord file : generateChecksum.cmd , generateCRCbinary.cmd

and calculate CRC from 0 to 0xEFFC , and put CRC vaue at 0xEFFC ( 0xF000 - 4 ) 

3. Flash allocation

	- dataflash : 0x1F800 , size : 2K

	- boot loader CODE : LDROM 4K , extra storage in APROM at 0x1E000 (size : 6K)
	
	- application code : APROM addr 0x0000 (size : 60K) , checksum : 0xEFFC , a.k.a bank0 
	
	- application code : APROM addr 0xF000 (size : 60K) , checksum : 0x1DFFC , a.k.a bank1


3. screen capture:

under LDROM , recieve Xmodem DATA
	
![image](https://github.com/released/M031BSP_ISP_XMODEM_DUAL_BANK/blob/main/LDROM_receive_Xmodem.jpg)		
		
under LDROM , recieve Xmodem finish , and process upgrade data from bank 1 to bank 0

![image](https://github.com/released/M031BSP_ISP_XMODEM_DUAL_BANK/blob/main/LDROM_upgrade_from_bank1_to_bank0.jpg)		
		
under LDROM , when upgrade bank 0 finish , and jump to application code 01

![image](https://github.com/released/M031BSP_ISP_XMODEM_DUAL_BANK/blob/main/LDROM_upgrade_finish_jump_to_application01.jpg)		
		
under LDROM , when upgrade bank 0 finish , and jump to application code 02

![image](https://github.com/released/M031BSP_ISP_XMODEM_DUAL_BANK/blob/main/LDROM_upgrade_finish_jump_to_application02.jpg)		
		
under APROM , press digit 'z' , to reboot to LDROM , and wait for Xmodem DATA

![image](https://github.com/released/M031BSP_ISP_XMODEM_DUAL_BANK/blob/main/APROM_reset_from_application_code.jpg)		

