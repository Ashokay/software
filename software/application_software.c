#include "application_software.h"
#include "fastmath.h"
#include "tuning.h"
#include "math.h"
#include "stdlib.h"
#include "driver_GPIO.h"
#include "default_ESC_config.h"
#include "VCU_CANdata.h"

#define OFF 0
#define RPM_CONTROL 1
#define TORQUE_CONTROL 2
#define VEHICLE_CONTROL 3
#define ROTOR_POS_ID 4

#define NOT_READY 0
#define READY 1
#define READY_AND_ON 3

#define COSN45 0.707106781
#define SINN45 0.707106781

static int poles = 10;
static int driving_mode = OFF;
static int driving_interface = 0;

static float rpm_max = 7000;
static float Iph_max = 100;
static float motor_derateC = 90, ESC_derateC = 90;
static float Tfault_motorC = 110, Tfault_ESCC = 110;
static float Ibat_max = 50, Ibat_regen = 50;
static float Vbat_max = 60, Vbat_min = 36;

static float abs_max_rpm = 7000;
static float throttle_zero = 1.0, throttle_max = 4.0; 
static float brake_zero = 2.0, brake_max = 4.0; 
static float kp_rpm = 0.1, ki_rpm = 0.2;
static float kp_torque = 5.0, ki_torque = 100.0;

static float Id_desired = 0, Iq_desired = 0;
static float rpm = 0, torque = 0, abs_rpm;
static float Id_desired_integrator = 0;

static float Id_rms = 0, Iq_rms = 0;
static float Id_limit = 50, Iq_limit = 50, fId_limit = 50;
static float PfId_limit = 0, NfId_limit = 0;

static float reverse_rpm = 10, immob_rpm = 10;
static float Leco_rpm = 100, eco_rpm = 100;

static float Ilimit = 8, rpm_limit = 650;
static float rpm_to_kmph = 1;

static float VM_rpm_limit = 600, VM_Iregen = IBAT_REGEN; 
static float Vbrake_derate = 54.0, k_brake = 1.0;

static float safe_rpm = 50;

static float Iph_actual = 0;

void drive_init(int * p_adr, float * throttle_zero_adr, float * throttle_max_adr, float motor_TfaultC, float ESC_TfaultC, float * rpm_to_speed_adr, float *brake_zero_adr, float *brake_max_adr)
{   
    poles = *p_adr; 
    
    rpm_max = RPM_MAX;
    Iph_max = IPH_MAX;
    motor_derateC = MOTOR_DERATEC;
    ESC_derateC = ESC_DERATEC;
    
    Vbat_max = VBAT_MAX;
    Vbat_min = VBAT_MIN;
    Ibat_max = IBAT_MAX;
    Ibat_regen = IBAT_REGEN;
    
    driving_mode = DRIVING_MODE;
    driving_interface = DRIVING_INTERFACE;
    
    Leco_rpm = LECO_RPM;
    eco_rpm = ECO_RPM;
    
    throttle_zero = THROTTLE_ZERO;
    throttle_max = THROTTLE_MAX;
    
    brake_zero = THROTTLE_ZERO;
    brake_max = THROTTLE_MAX;
        
    Vbrake_derate = VBRAKE_DERATE;
    reverse_rpm = REVERSE_RPM;
    immob_rpm = IMMOB_RPM;
    rpm_to_kmph = RPM_TO_KMPH;
    
    kp_rpm = KP_RPM;
    ki_rpm = KI_RPM;
    
    kp_torque = KP_TORQUE;
    ki_torque = KI_TORQUE;
    
    tune_powertrain_variables(&poles, &Iph_max, &rpm_max, &rpm_to_kmph, &motor_derateC, &ESC_derateC, &Vbat_max, &Vbat_min, &Ibat_max, &Ibat_regen);
    
    tune_vehicle_veriables(&driving_mode, &immob_rpm, &Leco_rpm, &eco_rpm, &throttle_zero, &throttle_max, &Vbrake_derate, &reverse_rpm, &kp_rpm, &ki_rpm, &kp_torque, &ki_torque);
       
    *p_adr = poles;     
    *throttle_zero_adr = throttle_zero;
    *throttle_max_adr = throttle_max;
    abs_max_rpm = fabs(rpm_max);
    rpm_limit = rpm_max;
    Tfault_motorC = motor_TfaultC;
    Tfault_ESCC = ESC_TfaultC;
    *rpm_to_speed_adr = rpm_to_kmph;
    
    VM_Iregen = Ibat_regen; 
    VM_rpm_limit = rpm_max;
    
    safe_rpm = rpm_max * SAFE_RPM/100.0;
}

void drive_reset(float * adr_Idref, float * adr_Iqref)
{    
    *adr_Iqref = 0;
    *adr_Idref = 0;
    
    Id_desired = 0;
    Iq_desired = 0;
    
    Id_desired_integrator = 0;      //reset rpm control and torque control integration
    
    Id_rms = 0;
    Iq_rms = 0;
    
    GPIO_reverse(Iph_max, rpm_max, reverse_rpm, &VM_rpm_limit); //check for reverse
    CAN_reverse(Iph_max, rpm_max, reverse_rpm, &VM_rpm_limit); //check for reverse

    GPIO_immob_EN(Iph_max, &VM_rpm_limit); //check for immob
    CAN_immob_EN(Iph_max, &VM_rpm_limit); //check for immob
}

void torque_rpm_calculation(float Vdc, float Idc, float w, int p, float * rpm_adr, float * Torq_adr, float * Idc_f_adr, float * Iph_act_adr )
{
    static float itorque = 0;
    static float fIdc = 0;
    
    #define TORQUE_SCALE 18         //15Nm = 270Amp
    
    rpm = w/2.0/3.14159265*120.0/p;
    
    abs_rpm = fabs(rpm);
        
    *rpm_adr = rpm;
    
    fIdc = fIdc + 0.01*(Idc - fIdc);
    *Idc_f_adr = fIdc;
    
    itorque = Iq_rms / 5.5;  
       
    torque = torque + 0.2*(itorque - torque);
    
    *Torq_adr = torque;
    
    *Iph_act_adr = Iph_actual;
}

int drive_condition(float a_input)
{    
    static int current_drive_state = 0;
    static int stop_wait_counter = 0;
    
    if(driving_mode > 4) driving_mode = 3; //default mode Iref_control
    if(driving_mode < 0) driving_mode = 3;
    
    if(current_drive_state == OFF)    
    {
        stop_wait_counter++;    
    
        if(stop_wait_counter > 2000)
        {                    
            stop_wait_counter = 2000;
     
            if(driving_mode > 0)
            {        
                if((rpm < safe_rpm) & (rpm > -safe_rpm)) //safe rpm to turn on or off
                {
                    //stop_wait_counter = 2000;
                    if((a_input > 0) || (VM_rpm_limit == 0))// || rpm > 50) 
                    {   
                        current_drive_state = driving_mode;
                        stop_wait_counter = 0;
                    }
                }
            }            
        }         
    }         
    else
    {
        static float turnoff_timer = 0;
        
        if((rpm < safe_rpm) & (rpm > -safe_rpm))
        {
            if(fabs(rpm) < 50) turnoff_timer++;
            else turnoff_timer = 0;
            
            if((a_input == 0) & (VM_rpm_limit !=0) & (turnoff_timer > 1000))
            {
                turnoff_timer = 0;
                current_drive_state = OFF;
            }
        }
    }
    return current_drive_state;
}


void current_ref_generator(float Vdc, float * adr_Idref, float * adr_Iqref, float Tesc, float Tm, float Dduty, float Qduty, float DQmax, float Id_act, float Iq_act, float Ibat_act, float Imosfet)
{
    float DQduty = 0;
    float fIbat_act = 0, abs_Ibat_act = 0;
    float IbatP_limiter = 200, IbatN_limiter = -200;        
    float ESC_derateC_factor = 1, motor_derateC_factor = 1;  
    float Vbrake_margine = 1.0;
    static float fIq_act = 0;
    
    Iph_actual = sqrt(Id_act * Id_act + Iq_act * Iq_act);
    
//Temperature limits============================================================    
    if((Tesc >= ESC_derateC) & (Tfault_ESCC > (ESC_derateC + 10)))
    {
        ESC_derateC_factor = 1 - (Tesc - ESC_derateC)/(Tfault_ESCC - ESC_derateC) ;
    }
    else  ESC_derateC_factor = 1;
    
    if((Tm >= motor_derateC) & (Tfault_motorC > (motor_derateC + 10)))
    {
        motor_derateC_factor = 1 - (Tm - motor_derateC)/(Tfault_motorC-motor_derateC) ;
    }
    else  motor_derateC_factor = 1;    
    
    Ilimit = ESC_derateC_factor * motor_derateC_factor * Iph_max;  
        
    if(Ilimit > Imosfet) Ilimit = Imosfet;
    
    fIq_act = fIq_act + 0.01*(Iq_act - fIq_act);
              
    Id_limit = sqrt(Ilimit * Ilimit - fIq_act * fIq_act);
              
//Brake Regen limit===================================================================
    
    Vbrake_margine = Vbat_max - Vbrake_derate;
    if(Vbrake_margine < 1.0) Vbrake_margine = 1.0;
    Vbrake_derate = Vbat_max - Vbrake_margine;
    if(Vdc > Vbrake_derate)
    {
        k_brake =  (1 - (Vdc - Vbrake_derate)/(Vbrake_margine));   
    }
    if(k_brake < 0) k_brake = 0;
    
//Battery limits================================================================    
    
    if(Ibat_regen > VM_Iregen) Ibat_regen = VM_Iregen;
    
    Ibat_regen = Ibat_regen * k_brake;
    
    #define IBAT_REGEN_MIN 0.0
    if(Ibat_regen < IBAT_REGEN_MIN) Ibat_regen = IBAT_REGEN_MIN;
    
    fIbat_act = fIbat_act + 0.02*(Ibat_act - fIbat_act);
    abs_Ibat_act = fabs(fIbat_act);
    
    #define ABS_IBAT_MIN 0.01
    if(abs_Ibat_act < ABS_IBAT_MIN) abs_Ibat_act = ABS_IBAT_MIN; 
    
    #define ABS_ID_ACT_MIN 1
    float abs_Id_act = 0;
    abs_Id_act = fabs(Id_act);
    if(abs_Id_act < ABS_ID_ACT_MIN) abs_Id_act = ABS_ID_ACT_MIN; 
    
    IbatP_limiter = abs_Id_act + abs_Id_act * (Ibat_max - Ibat_act)/abs_Ibat_act;
    if(IbatP_limiter < 10) IbatP_limiter = 10;
    IbatN_limiter = abs_Id_act + abs_Id_act * (Ibat_act - (- Ibat_regen))/abs_Ibat_act;
    if(IbatN_limiter < 0) IbatN_limiter = 0;

    static float Id_batlimit = 10;
    //if(IbatP_limiter < IbatN_limiter) 
    Id_batlimit = IbatP_limiter;
    //else Id_batlimit = IbatN_limiter;
    
    if(Id_limit > Id_batlimit) Id_limit = Id_batlimit;

//Vehicle mode==================================================================
    GPIO_Vmode(rpm_max, eco_rpm, Leco_rpm, &VM_rpm_limit);
    CAN_Vmode(rpm_max, rpm_to_kmph, &VM_rpm_limit, &VM_Iregen); 
    GPIO_immob_DIS(&VM_rpm_limit);
    CAN_immob_DIS(&VM_rpm_limit);        
    
    rpm_limit =  VM_rpm_limit; //imp for mode control
    
//Id, Iq referance==============================================================    
    
    #define DI 0.1
    
    fId_limit = fId_limit + 0.1*(Id_limit - fId_limit);             //filter
    PfId_limit = fId_limit;
    if(PfId_limit < 5) PfId_limit = 5;
    NfId_limit = -0*fId_limit;        
    
        
    if(Id_desired < NfId_limit) Id_desired = NfId_limit;
    if(Id_desired > PfId_limit) Id_desired = PfId_limit;
        
    if(Id_rms < Id_desired) Id_rms = Id_rms + DI;
    else if (Id_rms > Id_desired ) Id_rms = Id_rms - DI;

//==============================================================================  
//Iq calcultaion    
    float Iq_req = 0;
    
    Iq_limit = fabs(Id_act*1);
    DQduty = sqrt(Dduty*Dduty + Qduty*Qduty);
    if(DQduty > DQmax*0.8)
        Iq_req = (DQduty - DQmax*0.8)/(DQmax-DQmax*.8)*Iq_limit;
    else Iq_req = 0;
    
    Iq_req =0;

    Iq_desired = Iq_desired + 0.1*(Iq_req - Iq_desired);

    if(Iq_desired > Iq_limit ) Iq_desired = Iq_limit;
    if(Iq_desired < -Iq_limit ) Iq_desired = -Iq_limit;
    
    if(Iq_rms < Iq_desired) Iq_rms = Iq_rms + DI;
    else if (Iq_rms > Iq_desired ) Iq_rms = Iq_rms - DI;
    
    //if(Id_rms < -fId_limit*k_brake) Id_rms = -fId_limit*k_brake;
    *adr_Idref = Id_rms/0.5775; //peak value and transformation factor
    *adr_Iqref = Iq_rms/0.5775;
} 
    
void rpm_control(float a_input, float b_input, float dt)
{

    
    Id_desired = 0;
    
 
}
       
void torque_control(float a_input, float b_input, float dt)
{  
    Id_desired = a_input*100;

}

void vehicle_control(float a_input, float b_input, float dt)
{
    Id_desired = 0;
}

void angle_finder(float Scos, float Ssin, int current_dirMR)
{
    static int angle_finder_step = 0;
    static int hold_time_counter = 0;
    float Zcos, Zsin;
    static float theta, theta0, theta1, theta_diff;
    static int correct_dirMR;
    hold_time_counter++;
    
    switch(angle_finder_step)
    {
        case 0:            
            Id_desired = 15;
            Iq_desired = 15;
        
            if(hold_time_counter >= 10000)
            {
                hold_time_counter = 0;
                angle_finder_step = 1;
                
                Zcos = Scos/0.816496610641479;
                Zsin = Ssin/0.816496610641479;
                theta0 = find_ang(Zcos, Zsin);                                
            }            
        break;
        
        case 1:            
            Id_desired = 15;
            Iq_desired = -15;
        
            if(hold_time_counter >= 10000)
            {
                hold_time_counter = 0;
                angle_finder_step = 2;
                
                Zcos = Scos/0.816496610641479;
                Zsin = Ssin/0.816496610641479;
                theta1 = find_ang(Zcos, Zsin);
            }            
        break; 
        
        case 2: 
            Id_desired = 15;
            Iq_desired = 15;
        
            if(hold_time_counter >= 10000)
            {   
                hold_time_counter = 0;
                angle_finder_step = 0; 
                
                theta_diff = theta0 - theta1;
                if(theta_diff > 180) theta_diff = theta_diff - 360;
                if(theta_diff < -180) theta_diff = theta_diff + 360;
                
                Zcos = Scos /0.816496610641479;
                if((theta_diff) < 0)  
                {
                    Zsin = -Ssin /0.816496610641479;
                    if(current_dirMR == 0) correct_dirMR = 1;
                    else correct_dirMR = 0;
                }
                else
                {
                    Zsin = Ssin /0.816496610641479;
                    correct_dirMR = current_dirMR;
                }
                theta = find_ang(Zcos, Zsin);
                theta = 360 - theta + 135;
                //theta0 = Zcos;
                //theta1 = Zsin;
                send_measured_angle(theta, theta0, theta1, correct_dirMR);                
            }            
        break;    
            
        default:
            hold_time_counter = 0;
            angle_finder_step = 0; 
        break; 
    }
}

void current_ref_hold(float * adr_Idref, float * adr_Iqref)
{   
    if(Id_desired < -50 ) Id_desired = -50;
    if(Id_desired > 50) Id_desired = 50;
    
    if(Id_rms < Id_desired) Id_rms = Id_rms + 0.05;
    else if (Id_rms > Id_desired ) Id_rms = Id_rms - 0.05;


    if(Iq_desired > 50 ) Iq_desired = 50;
    if(Iq_desired < -50 ) Iq_desired = -50;
    
    if(Iq_rms < Iq_desired) Iq_rms = Iq_rms + 0.05;
    else if (Iq_rms > Iq_desired ) Iq_rms = Iq_rms - 0.05;
    
    *adr_Idref = Id_rms/0.5775; //peak value and transformation factor
    *adr_Iqref = Iq_rms/0.5775;
} 


