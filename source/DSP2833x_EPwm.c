//###########################################################################
//
// FILE:   DSP2833x_EPwm.c
//
// TITLE:  DSP2833x ePWM Initialization & Support Functions.
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
// InitEPwm - This function initializes the ePWM(s) to a known state.
//
void 
InitEPwm(void)
{
    //
    // Initialize ePWM1/2/3/4/5/6
    //

    //Initialize ePWM1
    EPwm1Regs.TBPRD = 7499;                             //주기 설정 Period = 46875 TBCLK counts(200Hz)
    EPwm1Regs.CMPA.half.CMPA =0;                        //Compare A 6562 (700us) 21562(2.3ms)
    EPwm1Regs.CMPB =0;                                  //Compare B
    EPwm1Regs.TBPHS.half.TBPHS = 0x0000;                //동기입력신호 time-base counter 사용하지 않음. Set Phase register to zero
    EPwm1Regs.TBCTR = 0;                                //현재의 time-base counter 사용하지 않음. Clear TB counter
    EPwm1Regs.TBCTL.bit.CTRMODE=TB_COUNT_UP;            //Up-Counter 모드
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;             //Phase 레지스터로부터 로드 하지 않음. Phase loading disabled
    EPwm1Regs.TBCTL.bit.PRDLD=TB_SHADOW;
    EPwm1Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_DISABLE;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1;             //High speed time-base click 분주비트 설정 TBCLK = SYSCLK/16
    EPwm1Regs.TBCTL.bit.CLKDIV=TB_DIV1;                 //Time-base Clock 분주 비트 설정 TBCLK = SYSCLK/16

    // Set up shadow register load on ZERO
    EPwm1Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
   // EPwm1Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    EPwm1Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;         // load on CTR = Zero
   // EPwm1Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;         // load on CTR = Zero

    // Set actions
    EPwm1Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm1Regs.AQCTLA.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
    EPwm1Regs.AQCTLB.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm1Regs.AQCTLB.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력

    //Initialize ePWM2
    EPwm2Regs.TBPRD = 7499;                             //주기 설정 Period = 46875 TBCLK counts(200Hz)
    EPwm2Regs.CMPA.half.CMPA =0;                        //Compare A 6562 (700us) 21562(2.3ms)
    EPwm2Regs.CMPB =0;                                  //Compare B
    EPwm2Regs.TBPHS.half.TBPHS = 0x0000;                //동기입력신호 time-base counter 사용하지 않음. Set Phase register to zero
    EPwm2Regs.TBCTR = 0;                                //현재의 time-base counter 사용하지 않음. Clear TB counter
    EPwm2Regs.TBCTL.bit.CTRMODE=TB_COUNT_UP;            //Up-Counter 모드
    EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE;             //Phase 레지스터로부터 로드 하지 않음. Phase loading disabled
    EPwm2Regs.TBCTL.bit.PRDLD=TB_SHADOW;
    EPwm2Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_DISABLE;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV =TB_DIV1;             //High speed time-base click 분주비트 설정 TBCLK = SYSCLK/16
    EPwm2Regs.TBCTL.bit.CLKDIV=TB_DIV1;                 //Time-base Clock 분주 비트 설정 TBCLK = SYSCLK/16

    // Set up shadow register load on ZERO
    EPwm2Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    //EPwm2Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    EPwm2Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;         // load on CTR = Zero
   // EPwm2Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;         // load on CTR = Zero

    // Set actions
    EPwm2Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm2Regs.AQCTLA.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
   // EPwm2Regs.AQCTLB.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
   // EPwm2Regs.AQCTLB.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력


    //Initialize ePWM3 -///////////////////////////// for bldc ///////////////////////////////////
    EPwm3Regs.TBPRD = 46874;                            // Period = 46875 TBCLK counts(200Hz)
    EPwm3Regs.CMPA.half.CMPA =0;                        // Compare A 6562 (700us) 21562(2.3ms)
    EPwm3Regs.CMPB =0;                                  // Compare B
    EPwm3Regs.TBPHS.half.TBPHS = 0x0000;                // Set phase register to zero
    EPwm3Regs.TBCTR = 0;                                // 현재의 time-base counter 사용하지 않음. Clear TB counter
    EPwm3Regs.TBCTL.bit.CTRMODE=TB_COUNT_UP;            // Up-Counter 모드
    EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Phase 레지스터로부터 로드 하지 않음. Phase loading disabled
    EPwm3Regs.TBCTL.bit.PRDLD=TB_SHADOW;
    EPwm3Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_DISABLE;
    EPwm3Regs.TBCTL.bit.HSPCLKDIV =2;                   // TBCLK = SYSCLKz
    EPwm3Regs.TBCTL.bit.CLKDIV=2;//TB_DIV4              //Time-base Clock 분주 비트 설정 TBCLK = SYSCLK/16

    // Set up shadow register load on ZERO
    EPwm3Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    //EPwm3Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    EPwm3Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;         // load on CTR = Zero
    //EPwm3Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;         // load on CTR = Zero

    // Set actions
    //EPwm3Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    //EPwm3Regs.AQCTLA.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
    //EPwm3Regs.AQCTLB.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    //EPwm3Regs.AQCTLB.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력

    // Set actions
    //EPwm3Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    //EPwm3Regs.AQCTLA.bit.CAU=AQ_SET;                  // 카운터가 증가되면서, LOW 값 출력
    //EPwm3Regs.AQCTLA.bit.CAD=AQ_CLEAR;
    //EPwm3Regs.AQCTLB.bit.CBU=AQ_SET;                  // 카운터가 증가되면서, LOW 값 출력
    //EPwm3Regs.AQCTLB.bit.CBD=AQ_CLEAR;

    // Set actions - 원복
    EPwm3Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm3Regs.AQCTLA.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
   // EPwm3Regs.AQCTLB.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
   // EPwm3Regs.AQCTLB.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력

    //Initialize ePWM4 -///////////////////////////// for bldc ///////////////////////////////////
    EPwm4Regs.TBPRD = 46874;                            // Period = 46875 TBCLK counts(200Hz)
    EPwm4Regs.CMPA.half.CMPA =0;                        // Compare A 6562 (700us) 21562(2.3ms)
    EPwm4Regs.CMPB =0;                                  // Compare B
    EPwm4Regs.TBPHS.half.TBPHS = 0x0000;                // Set phase register to zero
    EPwm4Regs.TBCTR = 0;                                // 현재의 time-base counter 사용하지 않음. Clear TB counter
    EPwm4Regs.TBCTL.bit.CTRMODE=TB_COUNT_UP;            // Up-Counter 모드
    EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Phase 레지스터로부터 로드 하지 않음. Phase loading disabled
    EPwm4Regs.TBCTL.bit.PRDLD=TB_SHADOW;
    EPwm4Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_DISABLE;
    EPwm4Regs.TBCTL.bit.HSPCLKDIV =2;                   // TBCLK = SYSCLKz
    EPwm4Regs.TBCTL.bit.CLKDIV=2;//TB_DIV4              //Time-base Clock 분주 비트 설정 TBCLK = SYSCLK/16

    // Set up shadow register load on ZERO
    EPwm4Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    //EPwm4Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    EPwm4Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;         // load on CTR = Zero
    //EPwm4Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;         // load on CTR = Zero


    // Set actions - 원복
    EPwm4Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm4Regs.AQCTLA.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
    //EPwm4Regs.AQCTLB.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    //EPwm4Regs.AQCTLB.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력

    //Initialize ePWM5 ///////////////////////Servo motor - L //////////////////////////////////////////////
    EPwm5Regs.TBPRD = 46874;                             //주기 설정 Period = 46875 TBCLK counts(200Hz)
    EPwm5Regs.CMPA.half.CMPA =0;                        //Compare A 6562 (700us) 21562(2.3ms)
    EPwm5Regs.CMPB =0;                                  //Compare B
    EPwm5Regs.TBPHS.half.TBPHS = 0x0000;                //동기입력신호 time-base counter 사용하지 않음. Set Phase register to zero
    EPwm5Regs.TBCTR = 0;                                //현재의 time-base counter 사용하지 않음. Clear TB counter
    EPwm5Regs.TBCTL.bit.CTRMODE=TB_COUNT_UP;            //Up-Counter 모드
    EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE;             //Phase 레지스터로부터 로드 하지 않음. Phase loading disabled
    EPwm5Regs.TBCTL.bit.PRDLD=TB_SHADOW;
    EPwm5Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_DISABLE;
    EPwm5Regs.TBCTL.bit.HSPCLKDIV =2;             //High speed time-base click 분주비트 설정 TBCLK = SYSCLK/16
    EPwm5Regs.TBCTL.bit.CLKDIV=2;                 //Time-base Clock 분주 비트 설정 TBCLK = SYSCLK/16

    // Set up shadow register load on ZERO
    EPwm5Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    EPwm5Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    EPwm5Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;         // load on CTR = Zero
    EPwm5Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;         // load on CTR = Zero

    // Set actions
    EPwm5Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm5Regs.AQCTLA.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
    EPwm5Regs.AQCTLB.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm5Regs.AQCTLB.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력

    //Initialize ePWM6 ///////////////////////Servo motor - R //////////////////////////////////////////////
    EPwm6Regs.TBPRD = 46874;                            // Period = 46875 TBCLK counts(200Hz)
    EPwm6Regs.CMPA.half.CMPA =0;                        // Compare A 6562 (700us) 21562(2.3ms)
    EPwm6Regs.CMPB =0;                                  // Compare B
    EPwm6Regs.TBPHS.half.TBPHS = 0x0000;                // Set phase register to zero
    EPwm6Regs.TBCTR = 0;                                // 현재의 time-base counter 사용하지 않음. Clear TB counter
    EPwm6Regs.TBCTL.bit.CTRMODE=TB_COUNT_UP;            // Up-Counter 모드
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;             // Phase 레지스터로부터 로드 하지 않음. Phase loading disabled
    EPwm6Regs.TBCTL.bit.PRDLD=TB_SHADOW;
    EPwm6Regs.TBCTL.bit.SYNCOSEL=TB_SYNC_DISABLE;
    EPwm6Regs.TBCTL.bit.HSPCLKDIV =2;                   // TBCLK = SYSCLKz
    EPwm6Regs.TBCTL.bit.CLKDIV=2;//TB_DIV4              //Time-base Clock 분주 비트 설정 TBCLK = SYSCLK/16

    // Set up shadow register load on ZERO
    EPwm6Regs.CMPCTL.bit.SHDWAMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    //EPwm6Regs.CMPCTL.bit.SHDWBMODE=CC_SHADOW;           // 쉐도우 모드, 두개의 버퍼?로 동작
    EPwm6Regs.CMPCTL.bit.LOADAMODE=CC_CTR_ZERO;         // load on CTR = Zero
    //EPwm6Regs.CMPCTL.bit.LOADBMODE=CC_CTR_ZERO;         // load on CTR = Zero


    // Set actions - 원복
    EPwm6Regs.AQCTLA.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    EPwm6Regs.AQCTLA.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
    //EPwm6Regs.AQCTLB.bit.ZRO=AQ_SET;                    // 카운터가 zero 값과 같을 때, 다음과 같이 동작을 함(HIGH값 출력)
    //EPwm6Regs.AQCTLB.bit.CAU=AQ_CLEAR;                  // 카운터가 증가되면서, LOW 값 출력
}

//
// InitEPwmGpio - This function initializes GPIO pins to function as ePWM pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 
void 
InitEPwmGpio(void)
{
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
#if DSP28_EPWM4
    InitEPwm4Gpio();
#endif // endif DSP28_EPWM4
#if DSP28_EPWM5    
    InitEPwm5Gpio();
#endif // endif DSP28_EPWM5
#if DSP28_EPWM6
    InitEPwm6Gpio();
#endif // endif DSP28_EPWM6 
}

//
// InitEPwm1Gpio - This function initializes GPIO pins to function as ePWM1
// 
void 
InitEPwm1Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pull-up on GPIO0 (EPWM1A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pull-up on GPIO1 (EPWM1B)

    //
    // Configure ePWM-1 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM1 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    EDIS;
}

//
// InitEPwm2Gpio - This function initializes GPIO pins to function as ePWM2
//
void 
InitEPwm2Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
    //GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)

    //
    // Configure ePWM-2 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM2 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    //GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    EDIS;
}

//
// InitEPwm3Gpio - This function initializes GPIO pins to function as ePWM3
//
void 
InitEPwm3Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)

    //
    // Configure ePWM-3 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM3 functional
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
   // GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

    EDIS;
}

#if DSP28_EPWM4
//
// InitEPwm4Gpio - This function initializes GPIO pins to function as ePWM4
//
void 
InitEPwm4Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;    // Enable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 0;    // Enable pull-up on GPIO7 (EPWM4B)

    //
    // Configure ePWM-4 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM4 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure GPIO6 as EPWM4A
   // GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure GPIO7 as EPWM4B

    EDIS;
}
#endif // endif DSP28_EPWM4  

#if DSP28_EPWM5
//
// InitEPwm5Gpio - This function initializes GPIO pins to function as ePWM5
//
void 
InitEPwm5Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pull-up on GPIO9 (EPWM5B)

    //
    // Configure ePWM-5 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM5 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure GPIO8 as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure GPIO9 as EPWM5B

    EDIS;
}
#endif // endif DSP28_EPWM5

#if DSP28_EPWM6
//
// InitEPwm6Gpio - This function initializes GPIO pins to function as ePWM6
//
void 
InitEPwm6Gpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;    // Enable pull-up on GPIO10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;    // Enable pull-up on GPIO11 (EPWM6B)

    //
    // Configure ePWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be ePWM6 functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A
   // GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B

    EDIS;
}
#endif // endif DSP28_EPWM6  

//
// InitEPwmSyncGpio - This function initializes GPIO pins to function as ePWM 
// Synch pins
//
void 
InitEPwmSyncGpio(void)
{
    EALLOW;

    //
    // Configure EPWMSYNCI
    //

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;  //Enable pull-up on GPIO6 (EPWMSYNCI)
    //GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;//Enable pull-up on GPIO32 (EPWMSYNCI)    

    //
    // Set qualification for selected pins to asynch only
    // This will select synch to SYSCLKOUT for the selected pins.
    // Comment out other unwanted lines.
    //

    //
    // Synch to SYSCLKOUT GPIO6 (EPWMSYNCI)
    //
    GpioCtrlRegs.GPAQSEL1.bit.GPIO6 = 0;

    //
    //Synch to SYSCLKOUT GPIO32 (EPWMSYNCI)
    //
    //GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 0;   

    //
    // Configure EPwmSync pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPwmSync 
    // functional pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 2;  //Enable pull-up on GPIO6(EPWMSYNCI)
    //GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 2;//Enable pull-up on GPIO32(EPWMSYNCI)    

    //
    // Configure EPWMSYNC0
    //

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //

    //
    // Enable pull-up on GPIO6 (EPWMSYNC0)
    //
    //GpioCtrlRegs.GPAPUD.bit.GPIO6 = 0;

    //
    // Enable pull-up on GPIO33 (EPWMSYNC0)
    //
    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;

    //
    // Enable pull-up on GPIO6 (EPWMSYNC0)
    //
    //GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 3;

    //
    // Enable pull-up on GPIO33 (EPWMSYNC0)
    //
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 2;
}

//
// InitTzGpio -  This function initializes GPIO pins to function as Trip Zone
// (TZ) pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.  
// 
void 
InitTzGpio(void)
{
    EALLOW;

    //
    // Enable internal pull-up for the selected pins
    // Pull-ups can be enabled or disabled by the user. 
    // This will enable the pullups for the specified pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;    // Enable pull-up on GPIO12 (TZ1)
    GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;    // Enable pull-up on GPIO13 (TZ2)
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;    // Enable pull-up on GPIO14 (TZ3)
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;    // Enable pull-up on GPIO15 (TZ4)

    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;    // Enable pull-up on GPIO16 (TZ5)
    //GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;    // Enable pull-up on GPIO28 (TZ5)

    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;    // Enable pull-up on GPIO17 (TZ6) 
    //GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;    // Enable pull-up on GPIO29 (TZ6)  

    //
    // Set qualification for selected pins to asynch only
    // Inputs are synchronized to SYSCLKOUT by default.  
    // This will select asynch (no qualification) for the selected pins.
    // Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3;  // Asynch input GPIO12 (TZ1)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3;  // Asynch input GPIO13 (TZ2)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3;  // Asynch input GPIO14 (TZ3)
    GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3;  // Asynch input GPIO15 (TZ4)

    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;  // Asynch input GPIO16 (TZ5)
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;  // Asynch input GPIO28 (TZ5)

    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;  // Asynch input GPIO17 (TZ6) 
    //GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;  // Asynch input GPIO29 (TZ6)  

    //
    // Configure TZ pins using GPIO regs
    // This specifies which of the possible GPIO pins will be TZ functional 
    // pins. Comment out other unwanted lines.
    //
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // Configure GPIO12 as TZ1
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // Configure GPIO13 as TZ2
    GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // Configure GPIO14 as TZ3
    GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // Configure GPIO15 as TZ4

    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 3;  // Configure GPIO16 as TZ5
    //GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 3;  // Configure GPIO28 as TZ5

    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 3;  // Configure GPIO17 as TZ6               
    //GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 3;  // Configure GPIO29 as TZ6  

    EDIS;
}

//
// End of file
//

