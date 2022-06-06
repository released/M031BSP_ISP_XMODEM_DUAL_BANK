/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "NuMicro.h"
#include "EEPROM_Emulate.h"

#define APROM_FW_VER_ADDR      		        (FMC_FLASH_PAGE_SIZE)
//put F/W verison at 2nd page , for indicator
const uint8_t FW_Version[] __attribute__((at(APROM_FW_VER_ADDR))) = "FW_VER_V002.002";

// #define APROM_1
#define APROM_2

#define ENABLE_EMULATE_EEPROM
// #define ENABLE_RTC

#define DEBUG_UART_PORT							(UART1)
#define DEBUG_UART_PORT_IRQn					(UART13_IRQn)
#define DEBUG_UART_IRQHandler					(UART13_IRQHandler)

volatile uint32_t counter_tick = 0;

#define DATA_FLASH_OFFSET  						(0x1F800)

#define DATA_FLASH_AMOUNT						(48)
#define DATA_FLASH_PAGE  						(4)     // M031 page : 0x200 (512)

#if defined (ENABLE_EMULATE_EEPROM)
int set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];
	
    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");
        return -1;
    }

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;

    FMC_ENABLE_CFG_UPDATE();

    au32Config[0] &= ~0x1;         /* CONFIG0[0] = 0 (Enabled) / 1 (Disabled) */
    au32Config[1] = u32DFBA;

    /* Update User Configuration settings. */
    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;

    FMC_ReadConfig(au32Config, 2);

    /* Check if Data Flash is enabled (CONFIG0[0]) and is expected address (CONFIG1) */
    if (((au32Config[0] & 0x01) == 1) || (au32Config[1] != u32DFBA))
    {
        printf("Error: Program Config Failed!\n");
        /* Disable FMC ISP function */
        FMC_Close();
        SYS_LockReg();
        return -1;
    }


    printf("\nSet Data Flash base as 0x%x.\n", u32DFBA);

    /* To check if all the debug messages are finished */
    while(!UART_IS_TX_EMPTY(DEBUG_UART_PORT));//while(!IsDebugFifoEmpty());

    /* Perform chip reset to make new User Config take effect */
    SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    return 0;
}

void Emulate_EEPROM(void)
{
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    if (set_data_flash_base(DATA_FLASH_OFFSET) < 0)
    {
        printf("Failed to set Data Flash base address!\r\n");
    }

	/* Test Init_EEPROM() */
	Init_EEPROM(DATA_FLASH_AMOUNT, DATA_FLASH_PAGE);
	Search_Valid_Page();	
}
#endif

void SystemReboot_CHIP_RST(void)
{
    while(!UART_IS_TX_EMPTY(DEBUG_UART_PORT));
        
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable FMC ISP function */
    FMC_Open();

    FMC_SET_LDROM_BOOT();

    // NVIC_SystemReset();
    // SYS_ResetCPU();  
    SYS_ResetChip();      
}

uint8_t read_magic_tag(void)
{
    uint8_t tag = 0;

    #if defined (ENABLE_EMULATE_EEPROM)
    
    Read_Data(EEP_ADDR_MAGIC_TAG , &tag);

    #endif

    #if defined (ENABLE_RTC)
    RTC_EnableSpareAccess();

    RTC->RWEN = RTC_WRITE_KEY;
    while(!(RTC->RWEN & RTC_RWEN_RWENF_Msk));
    
    tag =  RTC_READ_SPARE_REGISTER(0);

    #endif
    
    printf("Read MagicTag <0x%02X>\r\n", tag);
    
    return tag;
}

void write_magic_tag(uint8_t tag)
{

    #if defined (ENABLE_EMULATE_EEPROM)

    Write_Data(EEP_ADDR_MAGIC_TAG , tag);

	// /* Disable FMC ISP function */
	// FMC_Close();

	// /* Lock protected registers */
	// SYS_LockReg();

    #endif

	#if defined (ENABLE_RTC)
    RTC_EnableSpareAccess();

    RTC->RWEN = RTC_WRITE_KEY;
    while(!(RTC->RWEN & RTC_RWEN_RWENF_Msk));
    
    RTC_WRITE_SPARE_REGISTER(0, tag);
    
    #endif

    printf("Write MagicTag <0x%02X>\r\n", tag);	
}


void tick_counter(void)
{
	counter_tick++;
}

uint32_t get_tick(void)
{
	return (counter_tick);
}

void set_tick(uint32_t t)
{
	counter_tick = t;
}


void UARTx_Process(void)
{
	uint8_t res = 0;

	res = UART_READ(DEBUG_UART_PORT);

	if (res > 0x7F)
	{
		printf("invalid command\r\n");
	}
	else
	{
		switch(res)
		{
	
			case '1':

				break;	

			case 'X':
			case 'x':
			case 'Z':
			case 'z':
    			read_magic_tag();			
	            write_magic_tag(0xA5);
	        
	            printf("Perform CHIP_RST to enter BOOTLOADER\r\n");
	            SystemReboot_CHIP_RST();
			
				break;		
			
		}
	}
}

void DEBUG_UART_IRQHandler(void)
{
    if(UART_GET_INT_FLAG(DEBUG_UART_PORT, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))     /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(DEBUG_UART_PORT) == 0)
        {
//			set_flag(flag_uart_rx,ENABLE);
			UARTx_Process();
        }
    }

    if(DEBUG_UART_PORT->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART_ClearIntFlag(DEBUG_UART_PORT, (UART_INTSTS_RLSINT_Msk| UART_INTSTS_BUFERRINT_Msk));
    }
}

void DEBUG_UART_Init(void)
{
    SYS_ResetModule(UART1_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_UART_PORT, 115200);
	UART_EnableInt(DEBUG_UART_PORT, UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);
	NVIC_EnableIRQ(DEBUG_UART_PORT_IRQn);

	printf("\r\nCLK_GetCPUFreq : %8d\r\n",CLK_GetCPUFreq());
	printf("CLK_GetHXTFreq : %8d\r\n",CLK_GetHXTFreq());
	printf("CLK_GetLXTFreq : %8d\r\n",CLK_GetLXTFreq());	
	printf("CLK_GetPCLK0Freq : %8d\r\n",CLK_GetPCLK0Freq());
	printf("CLK_GetPCLK1Freq : %8d\r\n",CLK_GetPCLK1Freq());	
}


void LED_Init(void)
{
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB14MFP_Msk)) | (SYS_GPB_MFPH_PB14MFP_GPIO);
    GPIO_SetMode(PB, BIT14, GPIO_MODE_OUTPUT);	
}

void TMR1_IRQHandler(void)
{
	static uint32_t LOG = 0;	
	
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        TIMER_ClearIntFlag(TIMER1);

		tick_counter();

		#if defined (APROM_1)
		if ((get_tick() % 1000) == 0)
		{
        	printf("1)%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}
		#endif

		#if defined (APROM_2)
		if ((get_tick() % 500) == 0)
		{
        	printf("2)%s : %4d\r\n",__FUNCTION__,LOG++);
			PB14 ^= 1;
		}
		#endif

    }
}


void TIMER1_Init(void)
{
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1000);
    TIMER_EnableInt(TIMER1);
    NVIC_EnableIRQ(TMR1_IRQn);	
    TIMER_Start(TIMER1);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

//    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);	

//    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
//    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);	

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    // CLK_EnableModuleClock(UART0_MODULE);
    // CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART clock source from HXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    CLK_EnableModuleClock(TMR1_MODULE);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_HIRC, 0);


    //CLK_EnableModuleClock(RTC_MODULE);
    //CLK_SetModuleClock(RTC_MODULE, CLK_CLKSEL3_RTCSEL_LIRC,  NULL);
	
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
//    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
//    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
	
	DEBUG_UART_Init();

	LED_Init();
	TIMER1_Init();

    #if defined (ENABLE_EMULATE_EEPROM)
    Emulate_EEPROM();
    #endif

    while (1)
    {

    };
	
}
