/**
 * \file   dmtimerCounter.c
 *
 * \brief  Sample application for DMTimer. The application will
 *         count down from 9-0.
 *
 *         Application Configuration:
 *
 *             Modules Used:
 *                 DMTimer2
 *                 UART0
 *
 *             Configurable parameters:
 *                 None.
 *
 *             Hard-coded configuration of other parameters:
 *                 Mode of Timer - Timer mode(Auto reload)
 *
 *         Application Use Case:
 *             The application demonstrates DMTimer in Autoreload mode
 *             of operation. In the example for every overflow of DMTimer
 *             the counter register is reloaded with contents of overflow
 *             register. This sequence is continued 10 times and at each
 *             overflow a decrementing value is printed on the serial
 *             console showcasing the DMTimer as a down counter.
 *
 *         Running the example:
 *             On execution, the example will count down from 9 - 0 and stop.
 *             The time interval between each count is approximate to 700ms.
 *
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/ 
*/
/* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#include "consoleUtils.h"
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "interrupt.h"
#include "dmtimer.h"
#include "error.h"
#include "gpio_v2.h"
#include "hw_types.h"
#include "hw_cm_per.h"
#include "hw_cm_wkup.h"
#include "hw_control_AM335x.h"
#include "tsc_adc.h"
#include "locator.h"
#include "echod.h"
#include "lwiplib.h"
#include "lwipopts.h"
#include "cache.h"
#include "delay.h"
#include "mmu.h"
#include "cpsw.h"

/******************************************************************************
**                      INTERNAL MACRO DEFINITIONS
*******************************************************************************/
#define TIMER_INITIAL_COUNT             	(0xFFFFED12u)		// 0xFFFFED12u 4kHz
#define TIMER_RLD_COUNT                 	(0xFFFFFD12u)

#define GPIO_INSTANCE_ADDRESS           	(SOC_GPIO_1_REGS)

#define RESOL_X_MILLION            			(439u)

#define LEN_IP_ADDR                        	(4u)
#define ASCII_NUM_IDX                      	(48u) 
#define START_ADDR_DDR                     	(0x80000000)
#define START_ADDR_DEV                     	(0x44000000)
#define START_ADDR_OCMC                    	(0x40300000)
#define NUM_SECTIONS_DDR                   	(512)
#define NUM_SECTIONS_DEV                   	(960)
#define NUM_SECTIONS_OCMC                  	(1)

/******************************************************************************
**                      INTERNAL FUNCTION PROTOTYPES
*******************************************************************************/
static void DMTimerAintcConfigure(void);
static void DMTimerSetUp(void);
static void DMTimerIsr_2(void);

static void ADCIsr();
static void SetupIntc(void);
static void ADCStart(void);
static void CleanUpInterrupts(void);
static void StepConfigure(unsigned int, unsigned int, unsigned int);
void TSCADCModuleClkConfig(void);
unsigned int TSCADCPinMuxSetUp(void);

static void CPSWCore0RxIsr(void);
static void CPSWCore0TxIsr(void);
static void IpAddrDisplay(unsigned int ipAddr);
static void MMUConfigAndEnable(void);
static void Delay(unsigned int count);

/******************************************************************************
**                      INTERNAL VARIABLE DEFINITIONS
*******************************************************************************/
static volatile unsigned int cntValue = 10;
static volatile unsigned int flagIsr = 0;

volatile unsigned int flag = 1;
unsigned int sample1;
unsigned int sample2;
unsigned int sample3;
unsigned int val1;
unsigned int val2;
unsigned int val3;

short int state = 0;

/* page tables start must be aligned in 16K boundary */
#ifdef __TMS470__
#pragma DATA_ALIGN(pageTable, 16384);
static volatile unsigned int pageTable[4*1024];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=16384
static volatile unsigned int pageTable[4*1024];
#else
static volatile unsigned int pageTable[4*1024] __attribute__((aligned(16*1024)));
#endif

/* Ethernet Interface structure */ 
struct netif netIF;
/* Ethernet frame that we are going to use to send raw data */
struct pbuf *p_pbuf;
/* Ethernet raw data */
u8_t data[0x50] =
    {    
        0xff,0xff,0xff,0xff,0xff,0xff,0x88,0xc2,0x55,0x72,0x8c,0xef,0x81,0x00,0x80,0x01,
		0x88,0xba,0x40,0x00,0x00,0x3e,0x00,0x00,0x00,0x00,0x60,0x34,0x80,0x01,0x01,0xa2,
		0x2f,0x30,0x2d,0x80,0x04,0x34,0x30,0x30,0x31,0x82,0x02,0x01,0x18,0x83,0x04,0x00,
		0x00,0x00,0x01,0x85,0x01,0x02,0x87,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
    };

/******************************************************************************
**                          FUNCTION DEFINITIONS
*******************************************************************************/
int main(void)
{
	unsigned int ipAddr;
    LWIP_IF lwipIfPort1;

	/* Enabling functional clocks for GPIO1 instance. */
    GPIO1ModuleClkConfig();

    /* Selecting GPIO1[13] pin for use. */
    GPIO1PinMuxSetup(13);
	
    /* Enabling the GPIO module. */
    GPIOModuleEnable(GPIO_INSTANCE_ADDRESS);

    /* Resetting the GPIO module. */
	GPIOModuleReset(GPIO_INSTANCE_ADDRESS);
	
	/* Setting the GPIO1 pin 13 as an output pin. */
    GPIODirModeSet(GPIO_INSTANCE_ADDRESS, 13, GPIO_DIR_OUTPUT);

    /* This function will enable clocks for the DMTimer2 instance */
    IntMasterIRQEnable();

    /* Register DMTimer2 interrupts on to AINTC */
    DMTimerAintcConfigure();

    /* Perform the necessary configurations for DMTimer */
    DMTimerSetUp();

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

    //ConsoleUtilsPrintf("Tencounter: ");

    /* Start the DMTimer */
    DMTimerEnable(SOC_DMTIMER_2_REGS); 

	/* Enable the clock for touch screen */
    TSCADCModuleClkConfig();

    TSCADCPinMuxSetUp();
	
    /* Configures ADC to 12Mhz */
    TSCADCConfigureAFEClock(SOC_ADC_TSC_0_REGS, 24000000, 12000000);

    TSCADCStepIDTagConfig(SOC_ADC_TSC_0_REGS, 1);
	
    /* Disable Write Protection of Step Configuration regs*/
    TSCADCStepConfigProtectionDisable(SOC_ADC_TSC_0_REGS);

    /* Configure step 1 for channel 1(AN0)*/
    StepConfigure(0, TSCADC_FIFO_0, TSCADC_POSITIVE_INP_CHANNEL1);

    /* Configure step 2 for channel 2(AN1)*/
    StepConfigure(1, TSCADC_FIFO_1, TSCADC_POSITIVE_INP_CHANNEL2);
	
	/* Configure step 3 for channel 2(AN2)*/
    StepConfigure(2, TSCADC_FIFO_0, TSCADC_POSITIVE_INP_CHANNEL3);

    /* General purpose inputs */
    TSCADCTSModeConfig(SOC_ADC_TSC_0_REGS, TSCADC_GENERAL_PURPOSE_MODE);
	
	/* Clear the status of all interrupts */
    CleanUpInterrupts();	
	
    MMUConfigAndEnable();

    CPSWPinMuxSetup();
	
    CPSWClkEnable();

    // Initialize console for communication with the Host Machine
    ConsoleUtilsInit();

    // Select the console type based on compile time check
    ConsoleUtilsSetType(CONSOLE_UART);

    // Chip configuration RGMII selection
    EVMPortMIIModeSelect();

    // Get the MAC address
    EVMMACAddrGet(0, lwipIfPort1.macArray); 

    DelayTimerSetup();

    ConsoleUtilsPrintf("\n\rStarterWare Ethernet Echo Application. \n\r\n\r" );
   
    ConsoleUtilsPrintf("Acquiring IP Address for Port 1... \n\r" );

	/* configure our interface */
    lwipIfPort1.instNum = 0;
    lwipIfPort1.slvPortNum = 1; 
	/* Static IP: in this case 192.168.10.12 */
    lwipIfPort1.ipAddr = ((u32_t)((192) & 0xff) << 24) | \	
                         ((u32_t)((168) & 0xff) << 16) | \
                         ((u32_t)((10) & 0xff) << 8)  | \
                          (u32_t)((12) & 0xff);
    lwipIfPort1.netMask = 0; 
    lwipIfPort1.gwAddr = 0; 
	
	/* Specify as STATIC */
    lwipIfPort1.ipMode = IPADDR_USE_STATIC; 
	
	/* Initialize Ethernet-relate hardware registers of the BeagleBone Black */
    ipAddr = lwIPInit(&lwipIfPort1);
	
	/* In file lwiplib.c (line 250) we initialized our ethernet interface as en0.*/
	netIF = *netif_find("en0");
	
	/* Check that Ethernet initialized correctly */
	if(ipAddr)
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Assigned: ");
        IpAddrDisplay(ipAddr);
    }
    else
    {
        ConsoleUtilsPrintf("\n\r\n\rPort 1 IP Address Acquisition Failed.");
    }
	/* Prepare buffer for future raw Ethernet packet sending */
	p_pbuf = pbuf_alloc(PBUF_RAW, 0x50, PBUF_POOL);
	
	/* Fullfill buffer with the data */
	pbuf_take(p_pbuf, data, 0x50);
	
	/* Start ADC convertion */
	ADCStart();

    while(cntValue)
    {
        if(flagIsr == 1)
        {
			/* Change the state of the pin GPIO_45 (P8_11) to 1 (HIGH) */
			GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 13, GPIO_PIN_HIGH);
			/* Send raw Ethernet frame */
			netIF.linkoutput(&netIF, p_pbuf);
			/* Change the state of the pin GPIO_45 (P8_11) to 0 (LOW) */
			GPIOPinWrite(GPIO_INSTANCE_ADDRESS, 13, GPIO_PIN_LOW);
			/* free buffer that has been sent / prepear it for next data package */
			pbuf_free(p_pbuf);
			/* Check that ADC conversion is over */
			while(flag);
			/* Switch off ADC / prepear it for next conversion*/
			TSCADCModuleStateSet(SOC_ADC_TSC_0_REGS, TSCADC_MODULE_DISABLE);
            
			/* bring data to mV (milli Volt) scalse */
			val1 = (sample1 * RESOL_X_MILLION) / 1000;
			val2 = (sample2 * RESOL_X_MILLION) / 1000;
			val3 = (sample3 * RESOL_X_MILLION) / 1000;
			
			/* Locate ADC data in future Ethernet frame */
			data[58] = ((val1 >> 8)& 0xFF);
			data[59] = (val1 & 0xFF);
			data[66] = ((val2 >> 8)& 0xFF);
			data[67] = (val2 & 0xFF);
			data[74] = ((val3 >> 8)& 0xFF);
			data[75] = (val3 & 0xFF);
			/* Prepare buffer for future raw Ethernet packet sending */
			p_pbuf = pbuf_alloc(PBUF_RAW, 0x50, PBUF_POOL);
			/* Fullfill buffer with the data */
			pbuf_take(p_pbuf, data, 0x50);
			
			/* Start ADC convertion */
			flag = 1;
			ADCStart();
			
			flagIsr = 0;
        }
    }
}

/*
** Do the necessary DMTimer configurations on to AINTC.
*/
static void DMTimerAintcConfigure(void)
{
    /* Initialize the ARM interrupt control */
    IntAINTCInit();

    /* Registering DMTimerIsr */
    IntRegister(SYS_INT_TINT2, DMTimerIsr_2);
	
	/* Registering ADCIsr */
	IntRegister(SYS_INT_ADC_TSC_GENINT, ADCIsr);
	
	/* Register the Receive ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWRXINT0, CPSWCore0RxIsr);
  
    /* Register the Transmit ISR for Core 0 */
    IntRegister(SYS_INT_3PGSWTXINT0, CPSWCore0TxIsr);

    /* Set the priority */
    IntPrioritySet(SYS_INT_TINT2, 1, AINTC_HOSTINT_ROUTE_IRQ);
	IntPrioritySet(SYS_INT_ADC_TSC_GENINT, 2, AINTC_HOSTINT_ROUTE_IRQ);
	
	/* Set the priority */
    IntPrioritySet(SYS_INT_3PGSWTXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);
    IntPrioritySet(SYS_INT_3PGSWRXINT0, 0, AINTC_HOSTINT_ROUTE_IRQ);

    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_TINT2);	
	IntSystemEnable(SYS_INT_ADC_TSC_GENINT);
	
    /* Enable the system interrupt */
    IntSystemEnable(SYS_INT_3PGSWTXINT0);
    IntSystemEnable(SYS_INT_3PGSWRXINT0);
}

/*
** Setup the timer for one-shot and compare mode.
*/
static void DMTimerSetUp(void)
{
    /* Load the counter with the initial count value */
    DMTimerCounterSet(SOC_DMTIMER_2_REGS, TIMER_INITIAL_COUNT);

    /* Load the load register with the reload count value */
    DMTimerReloadSet(SOC_DMTIMER_2_REGS, TIMER_RLD_COUNT);

    /* Configure the DMTimer for Auto-reload and compare mode */
    DMTimerModeConfigure(SOC_DMTIMER_2_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);
}

/*
** DMTimer interrupt service routine. This will send a character to serial 
** console.
*/    
static void DMTimerIsr_2(void)
{
    /* Disable the DMTimer interrupts */
    DMTimerIntDisable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);

    /* Clear the status of the interrupt flags */
    DMTimerIntStatusClear(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_IT_FLAG);

    flagIsr = 1;

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

/* ADC is configured */
static void ADCStart(void)
{
	/* Enable step 1 */
    TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, 1, 1);

    /* Enable step 2 */
    TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, 2, 1);
	
	/* Enable step 3 */
    TSCADCConfigureStepEnable(SOC_ADC_TSC_0_REGS, 3, 1);

    /* End of sequence interrupt is enable */
    TSCADCEventInterruptEnable(SOC_ADC_TSC_0_REGS, TSCADC_END_OF_SEQUENCE_INT);

    /* Enable the TSC_ADC_SS module*/
    TSCADCModuleStateSet(SOC_ADC_TSC_0_REGS, TSCADC_MODULE_ENABLE);
}

/* Configures the step */
void StepConfigure(unsigned int stepSel, unsigned int fifo,
                   unsigned int positiveInpChannel)
{
    /* Configure ADC to Single ended operation mode */
    TSCADCTSStepOperationModeControl(SOC_ADC_TSC_0_REGS,
                                  TSCADC_SINGLE_ENDED_OPER_MODE, stepSel);
	
	TSCADCTSStepConfig(SOC_ADC_TSC_0_REGS, stepSel, TSCADC_NEGATIVE_REF_VSSA,
                    positiveInpChannel, TSCADC_NEGATIVE_INP_CHANNEL1, TSCADC_POSITIVE_REF_VDDA);

    /* XPPSW Pin is on, Which pull up the AN0, AN1 and AN2 lines*/
    TSCADCTSStepAnalogSupplyConfig(SOC_ADC_TSC_0_REGS, TSCADC_XPPSW_PIN_ON, TSCADC_XNPSW_PIN_ON,
                                TSCADC_YPPSW_PIN_ON, stepSel);
								
    /* select fifo 0 or 1*/
    TSCADCTSStepFIFOSelConfig(SOC_ADC_TSC_0_REGS, stepSel, fifo);

    /* Configure ADC to one shot mode */
    TSCADCTSStepModeConfig(SOC_ADC_TSC_0_REGS, stepSel,  TSCADC_ONE_SHOT_SOFTWARE_ENABLED);
}

/* Clear status of all interrupts */
static void CleanUpInterrupts(void)
{
    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS, 0x7FF);
    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS ,0x7FF);
    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS, 0x7FF);
}

/* Reads the data from FIFO 0 and FIFO 1 */
static void ADCIsr()
{
   volatile unsigned int status;

    status = TSCADCIntStatus(SOC_ADC_TSC_0_REGS);

    TSCADCIntStatusClear(SOC_ADC_TSC_0_REGS, status);

    if(status & TSCADC_END_OF_SEQUENCE_INT)
    {
         /* Read data from fifo 0 */
         sample1 = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);
         /* Read data from fif 1 */
         sample2 = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_1);
         /* Read data from fif 0 */
         sample3 = TSCADCFIFOADCDataRead(SOC_ADC_TSC_0_REGS, TSCADC_FIFO_0);

         flag = 0;
    }
}

static void SetupIntc(void)
{
    /* Enable IRQ in CPSR.*/
    IntMasterIRQEnable();

    /* Register DMTimer2 interrupts on to AINTC */
    DMTimerAintcConfigure();

    /* Perform the necessary configurations for DMTimer */
    DMTimerSetUp();

    /* Enable the DMTimer interrupts */
    DMTimerIntEnable(SOC_DMTIMER_2_REGS, DMTIMER_INT_OVF_EN_FLAG);
}

unsigned int TSCADCPinMuxSetUp(void)
{

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN0) = CONTROL_CONF_AIN0_CONF_AIN0_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN1) = CONTROL_CONF_AIN1_CONF_AIN1_RXACTIVE;

    HWREG( SOC_CONTROL_REGS +  CONTROL_CONF_AIN2)= CONTROL_CONF_AIN2_CONF_AIN2_RXACTIVE;

    /*HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN3) = CONTROL_CONF_AIN3_CONF_AIN3_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN4) = CONTROL_CONF_AIN4_CONF_AIN4_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN5) = CONTROL_CONF_AIN5_CONF_AIN5_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN6) = CONTROL_CONF_AIN6_CONF_AIN6_RXACTIVE;

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_AIN7) = CONTROL_CONF_AIN7_CONF_AIN7_RXACTIVE;*/

    HWREG( SOC_CONTROL_REGS + CONTROL_CONF_VREFP)= CONTROL_CONF_VREFP_CONF_VREFP_RXACTIVE;

    HWREG( SOC_CONTROL_REGS +  CONTROL_CONF_VREFN)= CONTROL_CONF_VREFN_CONF_VREFN_RXACTIVE;
    return TRUE;
}

void TSCADCModuleClkConfig(void)
{
    /* Configuring L3 Interface Clocks. */

    /* Writing to MODULEMODE field of CM_PER_L3_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |=
          CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) &
           CM_PER_L3_CLKCTRL_MODULEMODE));

    /* Writing to MODULEMODE field of CM_PER_L3_INSTR_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |=
          CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |=
          CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /* Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_OCPWP_L3_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |=
          CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |=
          CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
           CM_PER_L3S_CLKSTCTRL_CLKTRCTRL));

    /* Checking fields for necessary values.  */

    /* Waiting for IDLEST field in CM_PER_L3_CLKCTRL register to be set to 0x0. */
    while((CM_PER_L3_CLKCTRL_IDLEST_FUNC << CM_PER_L3_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) & CM_PER_L3_CLKCTRL_IDLEST));

    /*
    ** Waiting for IDLEST field in CM_PER_L3_INSTR_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_L3_INSTR_CLKCTRL_IDLEST_FUNC <<
           CM_PER_L3_INSTR_CLKCTRL_IDLEST_SHIFT)!=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) &
           CM_PER_L3_INSTR_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_GCLK field in CM_PER_L3_CLKSTCTRL register to
    ** attain the desired value.
    */
    while(CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) &
           CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_OCPWP_L3_GCLK field in CM_PER_OCPWP_L3_CLKSTCTRL
    ** register to attain the desired value.
    */
    while(CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) &
           CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L3S_GCLK field in CM_PER_L3S_CLKSTCTRL register
    ** to attain the desired value.
    */
    while(CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) &
          CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));


    /* Configuring registers related to Wake-Up region. */

    /* Writing to MODULEMODE field of CM_WKUP_CONTROL_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) |=
          CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_CONTROL_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_MODULEMODE));

    /* Writing to CLKTRCTRL field of CM_PER_L3S_CLKSTCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) |=
          CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKTRCTRL));

    /* Writing to CLKTRCTRL field of CM_L3_AON_CLKSTCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) |=
          CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP;

    /*Waiting for CLKTRCTRL field to reflect the written value. */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL_SW_WKUP !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKTRCTRL));

    /* Writing to MODULEMODE field of CM_WKUP_TSC_CLKCTRL register. */
    HWREG(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL) |=
          CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL) &
           CM_WKUP_ADC_TSC_CLKCTRL_MODULEMODE));

    /* Verifying if the other bits are set to required settings. */

    /*
    ** Waiting for IDLEST field in CM_WKUP_CONTROL_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_CONTROL_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_CONTROL_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CONTROL_CLKCTRL) &
           CM_WKUP_CONTROL_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L3_AON_GCLK field in CM_L3_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L3_AON_CLKSTCTRL) &
           CM_WKUP_CM_L3_AON_CLKSTCTRL_CLKACTIVITY_L3_AON_GCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_L4WKUP_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_L4WKUP_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_L4WKUP_CLKCTRL) &
           CM_WKUP_L4WKUP_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_GCLK field in CM_WKUP_CLKSTCTRL register
    ** to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_L4_WKUP_GCLK));

    /*
    ** Waiting for CLKACTIVITY_L4_WKUP_AON_GCLK field in CM_L4_WKUP_AON_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL) &
           CM_WKUP_CM_L4_WKUP_AON_CLKSTCTRL_CLKACTIVITY_L4_WKUP_AON_GCLK));

    /*
    ** Waiting for CLKACTIVITY_ADC_FCLK field in CM_WKUP_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_WKUP_CLKSTCTRL_CLKACTIVITY_ADC_FCLK !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_CLKSTCTRL) &
           CM_WKUP_CLKSTCTRL_CLKACTIVITY_ADC_FCLK));

    /*
    ** Waiting for IDLEST field in CM_WKUP_ADC_TSC_CLKCTRL register to attain
    ** desired value.
    */
    while((CM_WKUP_ADC_TSC_CLKCTRL_IDLEST_FUNC <<
           CM_WKUP_ADC_TSC_CLKCTRL_IDLEST_SHIFT) !=
          (HWREG(SOC_CM_WKUP_REGS + CM_WKUP_ADC_TSC_CLKCTRL) &
           CM_WKUP_ADC_TSC_CLKCTRL_IDLEST));
}

/*
** Function to setup MMU. This function Maps three regions (1. DDR
** 2. OCMC and 3. Device memory) and enables MMU.
*/
void MMUConfigAndEnable(void)
{
    /*
    ** Define DDR memory region of AM335x. DDR can be configured as Normal
    ** memory with R/W access in user/privileged modes. The cache attributes
    ** specified here are,
    ** Inner - Write through, No Write Allocate
    ** Outer - Write Back, Write Allocate
    */
    REGION regionDdr = {
                        MMU_PGTYPE_SECTION, START_ADDR_DDR, NUM_SECTIONS_DDR,
                        MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                         MMU_CACHE_WB_WA),
                        MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                        (unsigned int*)pageTable
                       };
    /*
    ** Define OCMC RAM region of AM335x. Same Attributes of DDR region given.
    */
    REGION regionOcmc = {
                         MMU_PGTYPE_SECTION, START_ADDR_OCMC, NUM_SECTIONS_OCMC,
                         MMU_MEMTYPE_NORMAL_NON_SHAREABLE(MMU_CACHE_WT_NOWA,
                                                          MMU_CACHE_WB_WA),
                         MMU_REGION_NON_SECURE, MMU_AP_PRV_RW_USR_RW,
                         (unsigned int*)pageTable
                        };

    /*
    ** Define Device Memory Region. The region between OCMC and DDR is
    ** configured as device memory, with R/W access in user/privileged modes.
    ** Also, the region is marked 'Execute Never'.
    */
    REGION regionDev = {
                        MMU_PGTYPE_SECTION, START_ADDR_DEV, NUM_SECTIONS_DEV,
                        MMU_MEMTYPE_DEVICE_SHAREABLE,
                        MMU_REGION_NON_SECURE,
                        MMU_AP_PRV_RW_USR_RW  | MMU_SECTION_EXEC_NEVER,
                        (unsigned int*)pageTable
                       };

    /* Initialize the page table and MMU */
    MMUInit((unsigned int*)pageTable);

    /* Map the defined regions */
    MMUMemRegionMap(&regionDdr);
    MMUMemRegionMap(&regionOcmc);
    MMUMemRegionMap(&regionDev);

    /* Now Safe to enable MMU */
    MMUEnable((unsigned int*)pageTable);
}

static void Delay(volatile unsigned int count)
{
    while(count--);
}
/*
** Interrupt Handler for Core 0 Receive interrupt
*/
static void CPSWCore0RxIsr(void)
{
	lwIPRxIntHandler(0);
}

/*   
** Interrupt Handler for Core 0 Transmit interrupt
*/
static void CPSWCore0TxIsr(void)
{
    lwIPTxIntHandler(0);
}

/*
** Displays the IP addrss on the Console
*/
static void IpAddrDisplay(unsigned int ipAddr) 
{
    ConsoleUtilsPrintf("%d.%d.%d.%d", (ipAddr & 0xFF), ((ipAddr >> 8) & 0xFF),
                       ((ipAddr >> 16) & 0xFF), ((ipAddr >> 24) & 0xFF));
}

/***************************** End Of File ***********************************/
