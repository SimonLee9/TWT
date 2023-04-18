//###########################################################################
//
// FILE:	DSP2833x_Sci.c
//
// TITLE:	DSP2833x SCI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2833x/F2823x Header Files and Peripheral Examples V142 $
// $Release Date: November  1, 2016 $
// $Copyright: Copyright (C) 2007-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//
// InitSci - This function initializes the SCI(s) to a known state.
//
void 
InitSci(void)
{
    //
    // Initialize SCI-A
    //

    ////////////////// Sensor SCI-A ////////////////////
    SciaRegs.SCICCR.all = 0x0007;       // 1 stop bit, No loopback
                                        // No parity, 8 char bits,
                                        // async mode, idle-line protocol
                                        // 한 통신 패킷에 8개 비트를 담겠다는 뜻

    SciaRegs.SCICTL1.all = 0x003;      // enable TX, RX, internal SCICLK,
                                        // Disable RX ERR, SLEEP, TXWAKE
                                        // 초기화 핀 및 Rx, Tx 사용 여부 설정

    SciaRegs.SCICTL2.bit.RXBKINTENA =1;


    SciaRegs.SCIHBAUD = 0x0000;         // 115200 (LSPCLK = 25Mhz)
    SciaRegs.SCILBAUD = 0x0027;         // 통신 속도 설정 = LSPCSK/ [레지스터 값 +1]*8

    SciaRegs.SCIFFTX.bit.SCIRST = 1;    // 다시 재 기능을 하게 됨
    SciaRegs.SCIFFTX.bit.SCIFFENA=1;    // FIFO 기능 활성화

    SciaRegs.SCIFFRX.bit.RXFFIENA =1;   // FFRX interrupt 허용
    SciaRegs.SCIFFRX.bit.RXFFIL=1;      // 수신 FIFO에 몇 바이트가 차면 인터럽트를 발생 할 것인지 설정

    SciaRegs.SCIFFCT.all=0x00;
    SciaRegs.SCICTL1.all=0x0023;        // Relinquish SCI from Reset

    SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;// 송신 FIFO 포인터를 초기화
    SciaRegs.SCIFFRX.bit.RXFIFORESET=1; // 수신 FIFO 포인터를 초기화

    //
    // Initialize SCI-B
    //

    ////////////////// Sensor SCI-B ////////////////////
    ScibRegs.SCICCR.all = 0x0007;       // 1 stop bit, No loopback
                                        // No parity, 8 char bits,
                                        // async mode, idle-line protocol
                                        // 한 통신 패킷에 8개 비트를 담겠다는 뜻

    ScibRegs.SCICTL1.all = 0x003;      // enable TX, RX, internal SCICLK,
                                        // Disable RX ERR, SLEEP, TXWAKE
                                        // 초기화 핀 및 Rx, Tx 사용 여부 설정

    ScibRegs.SCICTL2.bit.RXBKINTENA =1;
    ScibRegs.SCICTL2.bit.TXINTENA =1;


    ScibRegs.SCIHBAUD = 0x0001;         // 9600 (LSPCLK = 20Mhz)
    ScibRegs.SCILBAUD = 0x00E7;         // 통신 속도 설정 = LSPCSK/ [레지스터 값 +1]*8

    ScibRegs.SCIFFTX.bit.SCIRST = 1;    // 다시 재 기능을 하게 됨
    ScibRegs.SCIFFTX.bit.SCIFFENA=1;    // FIFO 기능 활성화

    ScibRegs.SCIFFRX.bit.RXFFIENA =1;   // FFRX interrupt 허용
    ScibRegs.SCIFFRX.bit.RXFFIL=1;      // 수신 FIFO에 몇 바이트가 차면 인터럽트를 발생 할 것인지 설정

    ScibRegs.SCIFFCT.all=0x00;
    ScibRegs.SCICTL1.all=0x0023;        // Relinquish SCI from Reset

    ScibRegs.SCIFFTX.bit.TXFIFOXRESET=1;// 송신 FIFO 포인터를 초기화
    ScibRegs.SCIFFRX.bit.RXFIFORESET=1; // 수신 FIFO 포인터를 초기화


    //
    // Initialize SCI-C
    //
}	

//
// InitSciGpio - This function initializes GPIO to function as SCI-A, SCI-B, or
// SCI-C
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 
// Caution: 
// Only one GPIO pin should be enabled for SCITXDA/B operation.
// Only one GPIO pin shoudl be enabled for SCIRXDA/B operation. 
// Comment out other unwanted lines.
//
void 
InitSciGpio()
{
    InitSciaGpio();
#if DSP28_SCIB   
    InitScibGpio();
#endif // if DSP28_SCIB  

#if DSP28_SCIC
    InitScicGpio();
#endif // if DSP28_SCIC
}

//
// InitSciaGpio - This function initializes GPIO pins to function as SCI-A pins
//
void 
InitSciaGpio()
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled disabled by the user.  
    // This will enable the pullups for the specified pins.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;  // Enable pull-up for GPIO28 (SCIRXDA)
    //GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	 // Enable pull-up for GPIO29 (SCITXDA)

    GpioCtrlRegs.GPBPUD.bit.GPIO35 = 0;
    GpioCtrlRegs.GPBPUD.bit.GPIO36 = 0;
    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.  
    // This will select asynch (no qualification) for the selected pins.
    //
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (SCIRXDA)
    
    GpioCtrlRegs.GPBQSEL1.bit.GPIO36 = 3;

    //
    // Configure SCI-A pins using GPIO regs
    // This specifies which of the possible GPIO pins will be SCI functional
    // pins.
    //
    //GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;   // Configure GPIO28 to SCIRXDA
    //GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;   // Configure GPIO29 to SCITXDA

    GpioCtrlRegs.GPBMUX1.bit.GPIO35 =1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO36 =1;


    EDIS;
}

#if DSP28_SCIB
//
// InitScibGpio - This function initializes GPIO pins to function as SCI-B pins
//
void 
InitScibGpio()
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins 
    // Pull-ups can be enabled or disabled disabled by the user.  
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;  //Enable pull-up for GPIO9  (SCITXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0; //Enable pull-up for GPIO14 (SCITXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;	 //Enable pull-up for GPIO18 (SCITXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0; //Enable pull-up for GPIO22 (SCITXDB)

    //GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0; //Enable pull-up for GPIO11 (SCIRXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0; //Enable pull-up for GPIO15 (SCIRXDB)
    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;	 //Enable pull-up for GPIO19 (SCIRXDB)
    //GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0; //Enable pull-up for GPIO23 (SCIRXDB)

    //
    // Set qualification for selected pins to asynch only
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO11 = 3;  // Asynch input GPIO11 (SCIRXDB)
    //GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (SCIRXDB)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;  // Asynch input GPIO19 (SCIRXDB)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 3;  // Asynch input GPIO23 (SCIRXDB)

    //
    // Configure SCI-B pins using GPIO regs
    // This specifies which of the possible GPIO pins will be SCI functional 
    // pins.
    // Comment out other unwanted lines.
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 2;  //Configure GPIO9 to SCITXDB 
    //GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 2; //Configure GPIO14 to SCITXDB
    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 2;  //Configure GPIO18 to SCITXDB
    //GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 3; //Configure GPIO22 to SCITXDB

    //GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 2;  //Configure GPIO11 for SCIRXDB
    //GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 2;  //Configure GPIO15 for SCIRXDB
    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 2;   //Configure GPIO19 for SCIRXDB
    //GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 3;  //Configure GPIO23 for SCIRXDB

    EDIS;
}
#endif // if DSP28_SCIB 

#if DSP28_SCIC
//
// InitScicGpio - This function initializes GPIO pins to function as SCI-C pins
//
void 
InitScicGpio()
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled disabled by the user.  
    // This will enable the pullups for the specified pins.
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO62 = 0;  // Enable pull-up for GPIO62 (SCIRXDC)
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;	 // Enable pull-up for GPIO63 (SCITXDC)

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.  
    // This will select asynch (no qualification) for the selected pins.
    //
    GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 3;  // Asynch input GPIO62 (SCIRXDC)

    //
    // Configure SCI-C pins using GPIO regs
    // This specifies which of the possible GPIO pins will be SCI functional 
    // pins.
    //
    GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 1;   // Configure GPIO62 to SCIRXDC
    GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 1;   // Configure GPIO63 to SCITXDC

    EDIS;
}
#endif // if DSP28_SCIC 
	
//
// End of file
//

