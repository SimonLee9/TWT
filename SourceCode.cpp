// Backup
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <math.h>
#include <string.h>             // strtok
#include <stdio.h>
#include <stdlib.h>             // header for using atof

//Control period
#define Control_Period 0.01

#define _CRT_SECURE_NO_WARNINGS //  Provide compile error about strtok warning

////////////Motor direction Setting//////////
////////////////////////// Wheel - GPIO /////////////////////////////////
#define Motor_Move          GpioDataRegs.GPBSET.bit.GPIO38 = 1; // Motor Driver_Stanby_On
#define Motor_Stop          GpioDataRegs.GPBCLEAR.bit.GPIO38 = 1; // Motor Driver_Stanby_Off

#define motorDirL1Set       GpioDataRegs.GPBSET.bit.GPIO39 = 1; // MotorA_INA2
#define motorDirL1Clear     GpioDataRegs.GPBCLEAR.bit.GPIO39 = 1; // MotorA_INA2

#define motorDirL2Set       GpioDataRegs.GPCSET.bit.GPIO87 = 1;     //MotorA_INA1
#define motorDirL2Clear     GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;   //MotorA_INA1

#define motorDirR2Set       GpioDataRegs.GPBSET.bit.GPIO40 = 1;     //MotorB_INB1
#define motorDirR2Clear     GpioDataRegs.GPBCLEAR.bit.GPIO40 = 1;   //MotorB_INB1

#define motorDirR1Set       GpioDataRegs.GPBSET.bit.GPIO41 = 1;     //MotorB_INB2
#define motorDirR1Clear     GpioDataRegs.GPBCLEAR.bit.GPIO41 = 1;   //MotorB_INB2

////////////////////////// Ducted-fan_BLDC /////////////////////////////////
#define BLDC_R_On          GpioDataRegs.GPASET.bit.GPIO04 = 1; //
#define BLDC_R_Clear       GpioDataRegs.GPACLEAR.bit.GPIO04 = 1; //

#define BLDC_L_On          GpioDataRegs.GPASET.bit.GPIO05 = 1; //
#define BLDC_L_Clear       GpioDataRegs.GPACLEAR.bit.GPIO05 = 1; //


////////////////////////// Servo - Tilting /////////////////////////////////
#define Servo_R_Tilting_On      GpioDataRegs.GPASET.bit.GPIO08 = 1; //
#define Servo_R_Tilting_Clear   GpioDataRegs.GPACLEAR.bit.GPIO08 = 1; //

#define Servo_L_Tilting_On      GpioDataRegs.GPASET.bit.GPIO10 = 1; //
#define Servo_L_Tilting_Clear   GpioDataRegs.GPACLEAR.bit.GPIO10 = 1; //


//Interrupts
__interrupt void Cpu_Timer0_Isr(void);  //Period setting
__interrupt void SciaRxFifoIsr(void);   //Getting a data of EBIMU Sensor
__interrupt void ScibRxFifoIsr(void);   //Bluetooh
__interrupt void ScicTxFifoIsr(void);   //Send Data to PC


// functions
void IsrSet(void);              //ISR: Interrupt Service Routine
void CpuTimer_set(void);
void SciaSend_Data(void);
void Encoder_cal(void);
void AdcResult(void);

void Line_Drive(void);
void Line_Drive1(void);

void Angle_PID(void);

void Position_PID(void);

void Speed_PID(void);

void Hadding_PID(void);

void Led_Check(void);

void Motor_Control(void);

void Test_Left_Go(void);
void Test_Right_Go(void);

void BLDC_fan(void);

void Fan_Angle_PID(void); // Ducted fan
void Fan_Control(void); // Ducted fan

void Tilting_Wheel_Angle_PID(void); // Angle PID of Change_Mode_motorcycle

void Tilting(void); // Mode Change\

void bicycle(void);

void segway(void);

void Converting(void);

void Tiliting_Value(void);

void Wheel_Angle_Gain_Activation(void);
void Wheel_Position_Gain_Activation(void);
void Fan_Gain_Activation(void);

void Wheel_Angle_Gain_DeActivation(void);
void Wheel_Position_Gain_DeActivation(void);
void Fan_Gain_DeActivation(void);

void Fan_Gain_Offset_Go_Back_Activation(void);
void Fan_Gain_Offset_Turn_Right_Activation(void);
void Fan_Gain_Offset_Turn_Left_Activation(void);

void Fan_Servo_Tilting_Segway(void);
void Fan_Servo_Tilting_Bicycle(void);
void Fan_Servo_Tilting_Bicycle_Right(void);
void Fan_Servo_Tilting_Bicycle_Left(void);

void Sliding_Mode_Contorl_Bicycle_Roll(void);  //==================SMC제어_추가_덕티드팬제어===============

void Bicycle_Heading_PID(void);

//void SerialTX(void); //==========================================???

void SendDataCal(float32 data, float32 data2, float32 data3);//========================================Sensor Data, Serial Tx Set

//******* Variables *******
//cnt
unsigned char Cnt_500us     =   0;
unsigned char Cnt_Tx        =   0;


//gain
float32 AKp =0;
float32 AKd =0; //20; 33;
float32 AKi = 0;//

/////Tilting Angle_gain
float32 T_AKp =0;
float32 T_AKd =0; //20; 33;
float32 T_AKi = 0;//
////////////////////////

float32 PKp = 0;
float32 PKd = 0;
float32 PKi = 0;//50;

float32 SKp = 0;
float32 SKi = 0;

float32 HKp = 0;
float32 HKd = 0;
float32 HKi = 0;

Uint16 Duty1A = 0; // PWM A
Uint16 Duty1B = 0; // PWM B

Uint16 Test1 = 0;
Uint16 Test2 = 0;

Uint16 Tilt_Go_R = 0; // PWM A
Uint16 Tilt_Go_L = 0; // PWM B

float32 Duct_Fan_Acc_flag = 0;
float32 Duct_Fan_Dec_flag = 0;
float32 Duct_Fan_Acc    = 8500;
float32 Fan_R_Cali = 8500;  //  default:8500    8700
float32 Fan_L_Cali = 9000;  //  default:8500    8000
Uint16 Fan_Test1 = 0;

//Offset
Uint16 Fan_R_Value = 0;//100;   // default: 100    1000
Uint16 Fan_L_Value = 200;//1200;  // default: 1200    1500

Uint16 flag = 0;
Uint16 Blue_Turn_flag=0;
float32 L =0;
float32 R =0;

float32 Duty = 0;
float32 Duty_h = 3;

float32 adcloop = 0;

//Flag===========================Check for data communication
unsigned char Sample_10ms_Flag          = 0;    // Control Flag
unsigned char Scia_TxDataSave_Flag      = 1;
unsigned char Scia_TxReady_Flag         = 0;
unsigned char Scia_SensorDataReady_Flag = 0;

unsigned char ScicTxDataSave_flag    =   1;
unsigned char ScicTxReady_flag       =   0;

unsigned char Scia_RxMode         = 0;
unsigned int Scia_RxRemainder     =   0;

unsigned char Scib_RxMode         = 0;
unsigned int Scib_RxRemainder     =   0;
unsigned char Scib_SensorDataReady_Flag = 0;
//=======================================================================================

int16   Angle_Temp              =   0;
int16   Angular_Temp            =   0;
float32 Angle                   =   0;
float32 Angular                 =   0;

float32 Angle_desired           =   0;//-1;
float32 AngleErr                =   0;
float32 AngleErrSum               =   0;
float32 Angle_dt               =   0;
float32 pre_AngleErr            =0;
float32 AngleControl            =0;

float32 P_Err                   =0;
float32 P_Err_Sum               =0;
float32 P_Err_dt               =0;
float32 pre_P_Err               =0;
float32 P_Control               =0;
float32 P_desired               =0;

float32 Speed                   =0;
float32 S_Err                   =0;
float32 S_Err_Sum               =0;
float32 S_Control               =0;
float32 S_desired               =0;
float32 Pre_Position            =0;

float32 H_Err                   =0;
float32 H_Err_Sum               =0;
float32 H_Err_dt               =0;
float32 pre_H_Err               =0;
float32 H_Control               =0;
float32 H_desired               =0;
float32 Heading_Angle           =0;

float32 Motor_Control_temp_L    =0;
float32 Motor_Control_temp_R    =0;
float32 Total_Control_L         =0;
float32 Total_Control_R         =0;

int16   R_Angle_Temp            =   0;
float32 R_Angle                 =   0;

int32   count1_EQep1            =0;
int32   count1_EQep2            =0;

float32 L_position              =0;
float32 R_position              =0;

//wheel Speed filtering
float32 P_Err_dt_filter         =0;

float32 P_Err_dt_x1             =0;

float32 P_Err_dt_y1             =0;

float32 Speed_filter             =0;

float32 Speed_x1                =0;
float32 Speed_y1                =0;
float32 H_Err_dt_filter         =0;
float32 H_Err_dt_x1             =0;
float32 H_Err_dt_y1             =0;

int Scia_SensorData[12]         =   {0};

unsigned char Scia_TxBuff[13]   =   {0};
unsigned char Buff[20]          =   {0};

unsigned char Sdata[25]        =   {0};  ; //==================Data Send to PC==============

// ====================================================EBIMU Sensor part=======================
#define SBUF_SIZE 64
unsigned int Data_Reset= 0;
int16   c = 0;
int16   a = 0;
int16   b = 0;
int16   d = 0;
unsigned char ff          =   0;
char *addr = 0;
unsigned char addr_a = 0;
float   Sensor_Data [3] = {0};
int number_of_item = 3;         // number_of_item : EBIMU의 1PACEKT 당 수신 항목 수를 입력, euler3축 출력 : 3, euler3축,가속도3축 출력 : 6
int i = 0;
int n = 0;
int result=0;
float item [3] = {0};
signed int sbuf_cnt=0;
char sbuf[SBUF_SIZE];

float Roll = 0;
float Pitch = 0;
float Yaw = 0;
//============================================================================================
//============================================================================================

//=================================================Test parameter=============================
int test_for=0;
int test_isrset=0;
int test_cpu=0;
int test_interrupt_cpu=0;
int test_interrupt_scia=0;
int test_motor_control=0;
int test_Control_Flag=0;
int test_main=0;
int Fan_Control_test=0;
int Fan_Angle_PID_test=0;
//==============================================================================================
//==============================================================================================

//=================================================Ducted Fan===================================
float32 Fan_AngleErr = 0;
float32 Fan_Angle_desired = 0;
float32 Fan_AngleErrSum = 0;
float32 Fan_Angle_dt = 0;
float32 Fan_AngleControl = 0;
float32 Fan_Pre_AngleErr =0;
float32 Fan_Angle_dt2=0;

float32 Total_Fan_Control_L = 0;
float32 Total_Fan_Control_R = 0;

float32 Fan_Control_temp_L = 0;
float32 Fan_Control_temp_R = 0;

float32 Fan_AKp = 0;
float32 Fan_AKd = 0;
float32 Fan_AKi = 0;
float32 Fan_AKa = 0;

float32 Fan_Max_Value =11000;
//==============================================================================================
//==============================================================================================


////Tilting Angle PID Gain ////////
int16   T_Angle_Temp              =   0;
int16   T_Angular_Temp            =   0;
float32 T_Angle                   =   0;
float32 T_Angular                 =   0;

float32 T_Angle_desired           =   -1;
float32 T_AngleErr                =   0;
float32 T_AngleErrSum               =   0;
float32 T_Angle_dt               =   0;
float32 T_pre_AngleErr            =0;
float32 T_AngleControl            =0;

float32 Tilting_Test1=0;
float32 Tilting_Test2=0;

float32 Tilting_Change_Mode1 = 0;   //segway mode
float32 Tilting_Change_Mode3 = 0;

float32 Tilting_Change_Mode2 = 0;   //bicycle
float32 Tilting_Change_Mode4 = 0;

float32 Tilting_Change_Mode5 = 0;   //bicycle
float32 Tilting_Change_Mode6 = 0;

float32 Tilting_Toal_Value_L =0;
float32 Tilting_Toal_Value_R =0;

float32 Tilting_Ducted_Fan= 0;

/////////////for Bicycle Angle Control-Fan///////////////////
float32 Pup=0;
float32 Pdown=0;

float32 Iup=0;
float32 Idown=0;

float32 Dup=0;
float32 Ddown=0;

/////////////for Bicycle Angle Control-Servo motor///////////////////
int16 R1=0;
int16 R2=0;
int16 R3=0;

int16 L1=0;
int16 L2=0;
int16 L3=0;

float32 mode=1;
float32 next_step=0;

//===============================================main=====================================

void DSP_init(void){
    InitSysCtrl();
    InitGpio();
    InitEPwmGpio(); //EPWM Port setting
    InitSciGpio();  // SCI Port setting
    InitEQepGpio(); // eQEP Port setting

    DINT;
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags
    IER = 0x0000;
    IFR = 0x0000;

    InitPieVectTable();

    EALLOW;

    EDIS;

    CpuTimer_set();     //CPU Timer & Timer ISR setting

    InitSci();          //SCI
    InitEPwm();         // EPWM Setting
    InitEQep();         // eQEP1 Setting
    DELAY_US(500000);   // Delay for ARS Sensor
    IsrSet();           // Interrupt service routine

    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
}

void main(void)
{

    DSP_init();
    for(;;)
    {
        Motor_Move;

        if(Sample_10ms_Flag==1) // Control Flag
        {
            Sample_10ms_Flag =0;

            //SendDataCal(Roll,Pitch,Yaw);
            //            SendDataCal(99,88,77);
            //SendDataCal(Roll,Yaw,flag); //22.09.26
            SendDataCal(Roll,L_position,R_position); //22.09.26
            Encoder_cal();
            Hadding_PID();
            Motor_Control();
            Speed_PID();

            /////////////// Wheel //////////// /*Pause for Testing*/
            EPwm1Regs.CMPA.half.CMPA = Total_Control_R+Duty1A+Test1+Tilt_Go_R; // PWM A - Motor Wheel
            EPwm2Regs.CMPA.half.CMPA = Total_Control_L+Duty1B+Test2+Tilt_Go_L; // PWM B - Motor Wheel

            /////////////////// BLDC Fan ////////////////////////////////
            Fan_Angle_PID();
            Fan_Control();

            EPwm3Regs.CMPA.half.CMPA = Fan_R_Cali + Total_Fan_Control_R + Fan_R_Value; // A = R
            EPwm4Regs.CMPA.half.CMPA = Fan_L_Cali + Total_Fan_Control_L + Fan_L_Value; // B = L

            ///////////////////Servo motor_Tilting////////////////////////
            EPwm5Regs.CMPA.half.CMPA = Tilting_Change_Mode1 - Tilting_Change_Mode3 + Tilting_Change_Mode5;
            EPwm5Regs.CMPB = Tilting_Change_Mode2 - Tilting_Change_Mode4 + Tilting_Change_Mode6;

            EPwm6Regs.CMPA.half.CMPA = Tilting_Ducted_Fan;
        }
    }
}



void Wheel_Angle_Gain_Activation(void){
    AKp=900;
    AKd=12;
    AKi=1;
}
void Wheel_Angle_Gain_DeActivation(void){
    AKp=0;
    AKd=0;
    AKi=0;
}
void Wheel_Position_Gain_Activation(void){
    PKp=400;
    PKd=200;
    PKi=1;
}
void Wheel_Position_Gain_DeActivation(void){
    PKp=0;
    PKd=0;
    PKi=0;

}
void Fan_Gain_Activation(void){
    Fan_AKp=400;
    Fan_AKd=100;
    Fan_AKi=0;
    Fan_AKa=1;

}
void Fan_Gain_DeActivation(void){
    Fan_AKp=0;
    Fan_AKd=0;
    Fan_AKi=0;
    Fan_AKa=0;
}

void Fan_Gain_Offset_Go_Back_Activation(void){
    Fan_R_Value =660;
    Fan_L_Value =200;
    Fan_R_Cali  =8500;
    Fan_L_Cali  =9500;
}

void Fan_Gain_Offset_Turn_Right_Activation(void){
    Fan_R_Value =660;
    Fan_L_Value =200;
    Fan_R_Cali  =9000;
    Fan_L_Cali  =9500;
}

void Fan_Gain_Offset_Turn_Left_Activation(void){
    Fan_R_Value =660;
    Fan_L_Value =300;
    //Fan_R_Cali  =8500;
    Fan_R_Cali  =9000;
    Fan_L_Cali  =9500;
}

void Fan_Servo_Tilting_Segway(void){
    Tilting_Ducted_Fan = 8000;
}

void Fan_Servo_Tilting_Bicycle(void){
    Tilting_Ducted_Fan = 16500;
}

void Fan_Servo_Tilting_Bicycle_Right(void){
    //Tilting_Ducted_Fan = 12250;
    Tilting_Ducted_Fan = 19000;
}

void Fan_Servo_Tilting_Bicycle_Left(void){
    //Tilting_Ducted_Fan = 19000;
    Tilting_Ducted_Fan = 12250;
}

void Fan_Angle_PID()
{
    Fan_AngleErr = Fan_Angle_desired-Roll;
    Fan_AngleErrSum += Fan_AngleErr * Control_Period;
    Fan_Angle_dt = (Fan_AngleErr - Fan_Pre_AngleErr)/Control_Period;
    Fan_Angle_dt2 = Fan_Angle_dt/Control_Period;

    Fan_AngleControl = Fan_AKp*Fan_AngleErr+Fan_AKd*Fan_Angle_dt+Fan_AKi*Fan_AngleErrSum+Fan_AKa*Fan_Angle_dt2; //+ 8500;//Fan_Offset

    Fan_Pre_AngleErr = Fan_AngleErr;
}

void Fan_Control(void) // Ducted Fan Control
{
    Fan_Control_temp_L = Fan_AngleControl;
    Fan_Control_temp_R = Fan_AngleControl;

    Total_Fan_Control_L = Fan_Control_temp_L;
    Total_Fan_Control_R = Fan_Control_temp_R;

    if(Total_Fan_Control_L<0) // Go back
    {
        Total_Fan_Control_L=0;
        Total_Fan_Control_L = -Total_Fan_Control_L;
        if(Total_Fan_Control_L > Fan_Max_Value) Total_Fan_Control_L = Fan_Max_Value; // Limit
    }

    else // 앞으로감
    {
        Total_Fan_Control_R=0;
        if(Total_Fan_Control_L>Fan_Max_Value) Total_Fan_Control_L = Fan_Max_Value;
    }

    if(Total_Fan_Control_R<0)
    {
        Total_Fan_Control_L=0;
        Total_Fan_Control_R =-Total_Fan_Control_R;
        if(Total_Fan_Control_R>Fan_Max_Value) Total_Fan_Control_R=Fan_Max_Value;
    }

    else
    {
        Total_Fan_Control_R=0;
        if(Total_Fan_Control_R>Fan_Max_Value) Total_Fan_Control_R = Fan_Max_Value;
    }

    if (Total_Fan_Control_R>10000) Total_Fan_Control_R = 10000;
    if (Total_Fan_Control_L>10000) Total_Fan_Control_L = 10000;
}

void Tilting_Wheel_Angle_PID(void)
{
    T_AngleErr = T_Angle_desired-Roll;
    T_AngleErrSum += T_AngleErr * Control_Period;
    T_Angle_dt = (T_AngleErr - T_pre_AngleErr)/Control_Period;

    T_AngleControl = -(T_AKp*T_AngleErr+T_AKd*T_Angle_dt+T_AKi*T_AngleErrSum);

    T_pre_AngleErr = T_AngleErr;
}

void IsrSet(void)
{
    test_isrset++;

    EALLOW; // This is needed to write to EALLOW protected registers
    PieVectTable.SCIRXINTA = &SciaRxFifoIsr;
    PieVectTable.SCIRXINTB = &ScibRxFifoIsr;
    PieVectTable.SCITXINTC = &ScicTxFifoIsr;

    EDIS;   // This is needed to disable write to EALLOW protected registers

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;          // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;          // Enable PIE Group 9, int1, SCIRXINTA
    PieCtrlRegs.PIEIER9.bit.INTx3 = 1;          // Enable PIE Group 9, int3, SCIRXINTB
    PieCtrlRegs.PIEIER8.bit.INTx6 = 1;          // Enable PIE Group 8, int6, SCITXINTC
    PieCtrlRegs.PIEIER8.bit.INTx5 = 1;          // Enable PIE Group 8, int5, SCIRXINTC

    PieCtrlRegs.PIEIER1.bit.INTx6 = 1;          // adc

    IER |= M_INT9;                              // Enable INT9
    IER |= M_INT8;                              // Enable INT8
    IER |= M_INT1;
    EINT;
}

void Encoder_cal()      //  Qep_ Rotary encoder of moving direction of local position and axis
{   //Encoder: 34*4*11[34:Gear, 4:채배, 11:Encoder pulse]
    count1_EQep1 = EQep1Regs.QPOSCNT;   //Left
    L_position = 0.01321003963011889035667107001321*(float)count1_EQep1;     // EQep value of divide when it 40cm wheel

    count1_EQep2 = EQep2Regs.QPOSCNT;   //Right
    R_position = 0.01321003963011889035667107001321*(float)count1_EQep2;     // EQep value of divide when it 40cm wheel
}

void Position_PID(void)
{  //Position Control
    P_Err = P_desired - (L_position + R_position)/2;
    P_Err_Sum += P_Err*Control_Period;
    P_Err_dt = (P_Err - pre_P_Err)/Control_Period;

    // low-pass filter, wheel Motor1 Speed filtering - cut-1Hz LPF @ control freq 100Hz LPF
    P_Err_dt_filter = 0.9391*P_Err_dt_y1 + 0.0305*P_Err_dt +0.0305 *P_Err_dt_x1;
    P_Err_dt_y1 = P_Err_dt_filter;
    P_Err_dt_x1 = P_Err_dt;

    P_Control = PKp * P_Err + PKd *P_Err_dt_filter + PKi*P_Err_Sum;

    pre_P_Err = P_Err;
}

void Speed_PID(void)
{
    Speed = (((L_position + R_position)/2) - Pre_Position) / Control_Period;

    Speed_filter = 0.9391* Speed_y1 + 0.0305 *Speed+0.0305 *Speed_x1;
    Speed_y1 = Speed_filter;
    Speed_x1 = Speed;

    S_Err = S_desired - Speed_filter;
    S_Err_Sum += S_Err*Control_Period;

    S_Control = S_Err*SKp + S_Err_Sum *SKi;

    Pre_Position = (L_position + R_position)/2;
}

void Hadding_PID(void) //Heading Control, Heading
{
    Heading_Angle = ((L_position - R_position)/21.4)/3.141592*180;
    H_Err = H_desired - Heading_Angle; // 21.4cm = distance of tire to tire
    H_Err_Sum += H_Err*Control_Period;
    H_Err_dt = (H_Err - pre_H_Err)/Control_Period;

    H_Control = HKp*H_Err + HKd*H_Err_dt + HKi*H_Err_Sum;
    pre_H_Err = H_Err;
}

//void Bicycle_Hadding_PID(void)
//{//Heading Control
//    Bicycle_Heading_Angle = Bicycle_Heading_Position/3.141592*180;
//    Bicycle_Heading_Err = Bicycle_Heading_desired - Bicycle_Heading_Angle; // 21.4cm = distance of tire to tire
//    Bicycle_Heading_Err_Sum += Bicycle_Heading_Err*Control_Period;
//    Bicycle_Heading_Err_dt = (Bicycle_Heading_Err - pre_Bicycle_Heading_Err)/Control_Period;
//
//    Bicycle_Heading_Control = BHKp*H_Err + BHKd*H_Err_dt + BHKi*Bicycle_Heading_Err_Sum;
//    pre_Bicycle_Heading_Err = Bicycle_Heading_Err;
//}

void Angle_PID(void)
{
    AngleErr = Angle_desired-Roll;
    AngleErrSum += AngleErr * Control_Period;
    Angle_dt = (AngleErr - pre_AngleErr)/Control_Period;

    AngleControl = AKp*AngleErr+AKd*Angle_dt+AKi*AngleErrSum;

    pre_AngleErr = AngleErr;
}

void bicycle(void) //=========================================================================================
{

    /////////////////////
    //test
    Fan_Servo_Tilting_Bicycle();

    Tilting_Change_Mode1 +=10;
    Tilting_Change_Mode2 +=10;

    Tilting_Change_Mode3 = 0;

    Tilting_Change_Mode4 = 0;

    Tilting_Change_Mode5 = 2600;
    Tilting_Change_Mode6 = 2600;


    if(Tilting_Change_Mode1>9631) Tilting_Change_Mode1 = 9631;
    if(Tilting_Change_Mode2>9631) Tilting_Change_Mode2 = 9631;

    if(Blue_Turn_flag==1){//Turn Right
        S_desired = -13;
        SKp=70;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;

        Tilting_Change_Mode6 = 5400;
        Tilting_Change_Mode4 = 0;

        Fan_Servo_Tilting_Bicycle_Right();

        //Tuning 01.17.21
        //Fan_Gain_Activation();
        Fan_Gain_Offset_Turn_Right_Activation();
    }

    else if(Blue_Turn_flag==2){ // Turn Left
        //////////////////////////////////////////
        S_desired = -13;
        SKp=70;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;

        Tilting_Change_Mode4 = 5400;
        Tilting_Change_Mode6 = 5400;

        Fan_Servo_Tilting_Bicycle_Left();

        //Tuning 01.17.21
        //Fan_Gain_Activation();
        Fan_Gain_Offset_Turn_Left_Activation();
    }

    else if(Blue_Turn_flag==3) { // Go, Back
        S_desired = -13;
        SKp=60;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;


        Tilting_Change_Mode6 = 2600;
        Tilting_Change_Mode4 = 0;

        Fan_Servo_Tilting_Bicycle(); // 2022-01-12

        //Tuning 01.17.21
        //Fan_Gain_Activation();
        Fan_Gain_Offset_Go_Back_Activation();
    }

    else if(Blue_Turn_flag==4) { // STOP for Tuning
        S_desired = 0;
        SKp=0;

        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();
        //Fan_Gain_DeActivation();

        Tilting_Change_Mode6 = 2600;
        Tilting_Change_Mode4 = 0;
    }

    //2022-01-12
    else{
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();
        Fan_Gain_DeActivation();

        Tilting_Change_Mode6 = 2600;
        Tilting_Change_Mode4 = 0;
    }
}

void segway(void)
{
    Tilting_Change_Mode1 = 0;
    Tilting_Change_Mode2 = 0;

    Tilting_Change_Mode3 +=10;
    Tilting_Change_Mode4 +=10;

    Fan_Servo_Tilting_Segway();

    if(Tilting_Change_Mode3>7031) Tilting_Change_Mode3 = 7031;
    if(Tilting_Change_Mode4>7031) Tilting_Change_Mode4 = 7031;
}

void Converting(void) // Segway --> Converting --> bicycle
{
    Tilting_Change_Mode5 +=10;
    Tilting_Change_Mode6 +=10;

    Fan_Servo_Tilting_Bicycle();

    if(Tilting_Change_Mode5>2600) Tilting_Change_Mode5 = 2600;
    if(Tilting_Change_Mode6>2600) Tilting_Change_Mode6 = 2600;
}

void Motor_Control(void) // Flag Command, Segway & Bicycle

{
    if(flag==3) //bicycle mode
    {
        // Duty1A = 0;
        // Duty1B = 0;

        bicycle(); //bicycle mode
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Fan_Gain_Activation();

        //Fan_Gain_DeActivation();
        Fan_Angle_PID();

        Motor_Control_temp_L =  S_Control ;
        Motor_Control_temp_R =  S_Control ;

    }


    else if (flag==2) // 45 degree tilting
    {
        Angle_PID();
        Position_PID();

        Wheel_Angle_Gain_Activation();

        Motor_Control_temp_L = AngleControl-P_Control- H_Control;// - S_Control ;
        Motor_Control_temp_R = AngleControl-P_Control+ H_Control;//- S_Control ;

        Converting();

        Tilting_Change_Mode1 = 5400;
        Tilting_Change_Mode2 = 5400;
    }


    else if(flag==1) // Segway mode
    {
        // Test
        Fan_Servo_Tilting_Segway();

        Wheel_Angle_Gain_Activation();
        //Wheel_Position_Gain_Activation();
        Fan_Gain_DeActivation();

        Duty1A = 480;
        Duty1B = 470;

        //function
        Angle_PID();

        Position_PID();

        Motor_Control_temp_L = AngleControl-P_Control - H_Control;// - S_Control ;
        Motor_Control_temp_R = AngleControl-P_Control + H_Control;//- S_Control ;


        Tilting_Change_Mode1 = 5400;
        Tilting_Change_Mode2 = 5400;

        Tilting_Change_Mode5 =0;
        Tilting_Change_Mode6 =0;
    }

    else if(flag==4) // Segway mode Speed Control
    {
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Angle_PID();

        Hadding_PID();
        S_desired=10;
        SKp=40;

        Speed_PID();

        Motor_Control_temp_L = AngleControl-P_Control - H_Control - S_Control ;
        Motor_Control_temp_R = AngleControl-P_Control + H_Control - S_Control ;
    }

    else if(flag==5){
        S_desired = -15;
        SKp=90;
        Wheel_Position_Gain_DeActivation();
        Wheel_Angle_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;
    }

    else if(flag==6){
        S_desired = 15;
        SKp=90;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;
    }

    else if(flag==7){ // Turn_Right
        //S_desired = -15;
        //SKp=90;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;

        Tilting_Change_Mode6 = 5400;
        Tilting_Change_Mode4 = 0;

        Fan_Servo_Tilting_Bicycle_Right();
        //Fan_Gain_Activation();
        /*
        Fan_AKp=400;
        Fan_AKd=100;
        Fan_AKi=0;
        Fan_AKa=1;
         * */
        Fan_Gain_Offset_Turn_Right_Activation();

    }

    else if(flag==8){ // Turn_Left
        //S_desired = -15;
        //SKp=90;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;

        Tilting_Change_Mode4 = 5400;
        Tilting_Change_Mode6 = 5400;

        Fan_Servo_Tilting_Bicycle_Left();
        //Fan_Gain_Activation();
        /*
        Fan_AKp=400;
        Fan_AKd=100;
        Fan_AKi=0;
        Fan_AKa=1;
         * */
        Fan_Gain_Offset_Turn_Left_Activation();

    }

    else if(flag==11){ // Gain Tuning---------------//2022.01.12 - for Turn RIGHT
        S_desired = -20;
        //SKp=60;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;

        Tilting_Change_Mode4 = 5400;
        //Tilting_Change_Mode6 = 5400;
        Tilting_Change_Mode6 = 0;

        Fan_Servo_Tilting_Bicycle_Right();
        Fan_Gain_Activation();
        Fan_Gain_Offset_Turn_Right_Activation();
    }

    else if(flag==12){ // // Gain Tuning------------//2022.01.12 - for Turn LEFT
        S_desired = -20;
        //SKp=60;
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;

        Tilting_Change_Mode4 = 5400;
        Tilting_Change_Mode6 = 5400;

        Fan_Servo_Tilting_Bicycle_Left();
        Fan_Gain_Activation();
        Fan_Gain_Offset_Turn_Left_Activation();
    }

    else if(flag==99){ // // Gain Tuning---------------------------------------------------
        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = AngleControl-P_Control - H_Control;// - S_Control ;
        Motor_Control_temp_R = AngleControl-P_Control + H_Control;//- S_Control ;

        if(mode==1){
            next_step=0;
            Fan_Servo_Tilting_Segway();

            Fan_Gain_DeActivation();

            Tilting_Change_Mode1 = 5400;
            Tilting_Change_Mode2 = 5400;

            Tilting_Change_Mode5 =0;
            Tilting_Change_Mode6 =0;

            next_step=1;
            if (next_step==1){
                mode=2;
            }

        }
        else if(mode==2){
            next_step=0;

            Converting();

            Tilting_Change_Mode1 = 5400;
            Tilting_Change_Mode2 = 5400;

            next_step=1;
            if (next_step==1){
                mode=3;
            }

        }
        else if(mode==3){
            bicycle(); //bicycle mode

            //Fan_Gain_Activation();

            Fan_Angle_PID();

        }

    }

    else if(flag==0){ //Stop
        S_desired = 0;
        SKp=0;

        Wheel_Angle_Gain_DeActivation();
        Wheel_Position_Gain_DeActivation();

        Motor_Control_temp_L = S_Control;
        Motor_Control_temp_R = S_Control;
    }

    else
    {
        Fan_Servo_Tilting_Segway();

        Wheel_Angle_Gain_Activation();
        Wheel_Position_Gain_Activation();
        Fan_Gain_DeActivation();

        Duty1A = 480;
        Duty1B = 470;

        //function
        Angle_PID();

        Position_PID();

        Motor_Control_temp_L = AngleControl-P_Control - H_Control;// - S_Control ;
        Motor_Control_temp_R = AngleControl-P_Control + H_Control;//- S_Control ;


        Tilting_Change_Mode1 = 5400;
        Tilting_Change_Mode2 = 5400;

        Tilting_Change_Mode5 =0;
        Tilting_Change_Mode6 =0;

        /*
        Fan_Servo_Tilting_Segway();

        Wheel_Angle_Gain_Activation();
        Wheel_Position_Gain_Activation();

        Duty1A = 480;
        Duty1B = 470;

        //function
        Angle_PID();
        //Hadding_PID();
        Position_PID();

        Motor_Control_temp_L = AngleControl-P_Control;// - H_Control - S_Control ;
        Motor_Control_temp_R = AngleControl-P_Control;// + H_Control- S_Control ;

        //Segway mode Pause for Testing
        Tilting_Change_Mode1 = 5400;
        Tilting_Change_Mode2 = 5400;

        Tilting_Change_Mode5 =0;
        Tilting_Change_Mode6 =0;
         */

    }

    Total_Control_L = Motor_Control_temp_L;
    Total_Control_R = Motor_Control_temp_R;

    if(Total_Control_L<0) // Go Back
    {
        motorDirL2Set;
        motorDirL1Clear;

        Total_Control_L = -Total_Control_L;

        if(Total_Control_L > 7499) Total_Control_L = 7499;
    }

    else // Go Front
    {
        motorDirL1Set;
        motorDirL2Clear;

        if(Total_Control_L>7499) Total_Control_L = 7499;
    }

    if(Total_Control_R<0)
    {
        motorDirR1Set;
        motorDirR2Clear;

        Total_Control_R =-Total_Control_R;

        if(Total_Control_R>7499) Total_Control_R=7499;
    }

    else
    {
        motorDirR2Set;
        motorDirR1Clear;

        if(Total_Control_R>7499) Total_Control_R = 7499;
    }
}

void Tilting_Value()
{
    Tilting_Toal_Value_L = Tilting_Change_Mode1 - Tilting_Change_Mode3 + Tilting_Change_Mode5;
    Tilting_Toal_Value_R = Tilting_Change_Mode2 - Tilting_Change_Mode4 + Tilting_Change_Mode6;
}

void CpuTimer_set()
{
    test_cpu++;

    EALLOW;
    PieVectTable.TINT0 =&Cpu_Timer0_Isr;
    EDIS;
    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 150, 500);    //CPU Timer Setting
    CpuTimer0Regs.TCR.all = 0x4000;         //Use Write_Only Instruct
    IER |= M_INT1;
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
}

int16 as=0;

void SendDataCal(float32 data, float32 data2, float32 data3)
{
    static unsigned char First_shot = 1;

    Sdata[0] = '[';
    if(data<0)
    {
        Sdata[1]='-';
        data*=-1;
    }

    else    Sdata[1]=data/1000+48;

    Sdata[2]=((int)data%1000)/100+48;
    Sdata[3]=((int)data%100)/10+48;
    Sdata[4]=((int)data%10)/1+48;
    Sdata[5]='.';
    Sdata[6]=(Uint32)(data*10)%10+48;
    Sdata[7]=(Uint32)(data*100)%10+48;

    Sdata[8]=',';

    if(data2<0)
    {
        Sdata[9]='-';
        data2*=-1;
    }

    else    Sdata[9]=data2/1000+48;

    Sdata[10]=((int)data2%1000)/100+48;
    Sdata[11]=((int)data2%100)/10+48;
    Sdata[12]=((int)data2%10)/1+48;
    Sdata[13]='.';
    Sdata[14]=(Uint32)(data2*10)%10+48;
    Sdata[15]=(Uint32)(data2*100)%10+48;

    Sdata[16]=',';

    if(data3<0)
    {
        Sdata[17]='-';
        data3*=-1;
    }

    else    Sdata[17]=data3/1000+48;

    Sdata[18]=((int)data3%1000)/100+48;
    Sdata[19]=((int)data3%100)/10+48;
    Sdata[20]=((int)data3%10)/1+48;
    Sdata[21]='.';
    Sdata[22]=(Uint32)(data3*10)%10+48;
    Sdata[23]=(Uint32)(data3*100)%10+48;
    Sdata[24]=']';
    Sdata[25]=0x0D;

    if(First_shot == 1){
        ScicRegs.SCIFFTX.bit.TXFIFOXRESET=1;
        First_shot = 0;
    }
    else ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;
}

__interrupt void Cpu_Timer0_Isr(void)
{
    test_interrupt_cpu++;

    Cnt_Tx++;
    Cnt_500us++;

    if(Cnt_Tx==20&&ScicTxDataSave_flag&&!ScicTxReady_flag)
    {
        //   SerialTX();
        Cnt_Tx=0;
    }
    // if(ScicTxReady_flag==1)  Send_Data();
    if(Cnt_500us==20){
        Sample_10ms_Flag=1; // Control Flag
        Cnt_500us=0;
    }
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    SpiaRegs.SPITXBUF = 0;
    // Acknowledge this interrupt to receive more interrupts from group 1
}

//=====================================EBIMU Sensor data ======================================================
__interrupt void SciaRxFifoIsr(void)
{
    //test_interrupt_scia++;
    unsigned char Scia_Temp = 0;        // Read data
    sbuf[sbuf_cnt] = SciaRegs.SCIRXBUF.all; // Get Character

    if(sbuf[sbuf_cnt]==0x0a)
    {
        addr = strtok(sbuf,",");

        for(i=0;i<number_of_item;i++)
        {
            item[i] = atof(addr);
            addr = strtok(NULL,",");
            b += 1;
        }
        result = 1;
    }

    else if(sbuf[sbuf_cnt]=='*')
    {   sbuf_cnt=-1;
    }

    sbuf_cnt++;
    if(sbuf_cnt>=SBUF_SIZE)
    {
        sbuf_cnt=0;
        Scib_SensorDataReady_Flag =1; // ?

        //d++;
    }

    Data_Reset++;
    if(Data_Reset>=Scia_Temp)
    {
        Data_Reset=0;
    }

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;      // clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;      // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP9;  // Issue PIE ack

    Roll = item[0];
    Pitch = item[1];
    Yaw = item[2];

    result = 0;
}
//============================================================================================================


__interrupt void ScibRxFifoIsr(void) // Bluetooth, Get the Command
{
    unsigned char Blue_flag = 0;
    Blue_flag = ScibRegs.SCIRXBUF.all; // Get Character

    if(Blue_flag=='b')  // Segway
    {
        flag=1;
    }

    else if(Blue_flag=='c') // Tilting
    {
        flag=2;
    }

    else if(Blue_flag=='d') //Bicycle
    {
        flag=3;
    }

    else if(Blue_flag=='L') // Left
    {
        Blue_Turn_flag=2;

        S_desired = -15;
        SKp = 80;
    }

    else if(Blue_flag=='R') // Right
    {
        Blue_Turn_flag=1;

        S_desired = -15;
        SKp = 80;
    }

    else if(Blue_flag=='F') // Go
    {
        S_desired = -10;
        SKp = 80;
        // Fan_AKp = 500;
        Blue_Turn_flag=3;

        if(Roll>4 || Roll<-4)
        {
            SKp = 105;
        }
    }

    else if(Blue_flag=='B') // Back
    {
        S_desired = +10;
        SKp = 80;
        //Fan_AKp = 500;

        Blue_Turn_flag=3;

        if(Roll>4 || Roll<-4)
        {
            SKp = 105;
        }

    }

    else if(Blue_flag=='a') // Stop
    {
        S_desired =0;
        SKp = 0;

        //   Fan_AKp = 700;

        Blue_Turn_flag=3;

        if(Roll>4 || Roll<-4)
        {
            SKp = 105;
        }

    }

    /*
        else if(Blue_flag=='Pup') //
        {
            Fan_AKp += 10;
        }
        else if(Blue_flag=='Pdown') //
        {
            Fan_AKp -= 10;
        }
        else if(Blue_flag=='Iup') //
        {
            Fan_AKi += 1;
        }
        else if(Blue_flag=='Idown') //
        {
            Fan_AKi -= 1;
        }
        else if(Blue_flag=='Dup') //
        {
            Fan_AKd += 2;
        }
        else if(Blue_flag=='Ddown') //
        {
            Fan_AKd -= 2;
        }


        else if(Blue_flag=='R1') //
        {
            Tilting_Change_Mode1 +=2;
        }
        else if(Blue_flag=='R2') //
        {
            Tilting_Change_Mode1 +=5;
        }
        else if(Blue_flag=='R3') //
        {
            Tilting_Change_Mode1 +=10;
        }
        else if(Blue_flag=='L1') //
        {
            Tilting_Change_Mode1 -=2;
        }
        else if(Blue_flag=='L2') //
        {
            Tilting_Change_Mode1 -=5;
        }
        else if(Blue_flag=='L3') //
        {
            Tilting_Change_Mode1 -=5;
        }
     */
    ScibRegs.SCIFFRX.bit.RXFFOVRCLR=1;      // clear Overflow flag
    ScibRegs.SCIFFRX.bit.RXFFINTCLR=1;      // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=PIEACK_GROUP8;  // Issue PIE ack

    result = 0;
}

__interrupt void ScicTxFifoIsr(void) // Data transfer to PC
{
    static unsigned char send_cnt = 0;
    unsigned char i;

    if(send_cnt == 0){
        for(i=0;i<16;i++)
            ScicRegs.SCITXBUF = Sdata[i];

        send_cnt++;
        ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;    // Clear SCI Interrupt flag
    }
    else if(send_cnt == 1){
        for(i=16;i<26;i++)
            ScicRegs.SCITXBUF = Sdata[i];

        ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;    // Clear SCI Interrupt flag
        send_cnt = 0;
    }



    PieCtrlRegs.PIEACK.all |= PIEACK_GROUP8;      // Issue PIE ACK

    //        unsigned char i;
    //        for(i=0;i<26;i++)
    //        {
    //            ScicRegs.SCITXBUF = Sdata[i];
    //        }
    //
    //
    //        ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;    // Clear SCI Interrupt flag
    //        //PieCtrlRegs.PIEACK.all|=0x100;
    //        PieCtrlRegs.PIEACK.all |= PIEACK_GROUP8;      // Issue PIE ACK
}
