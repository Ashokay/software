#include "application_software.h"
#include "fastmath.h"
#include "tuning.h"
#include "math.h"
#include "stdlib.h"
#include "driver_GPIO.h"
#include "default_ESC_config.h"
#include "VCU_CANdata.h"
#include "driver_CANbus.h"

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
static float L_acceleration = 1.0, M_acceleration = 1.0, k_auto_brake = 0.0;

static float Id_desired = 0, Iq_desired = 0;
static float rpm = 0, torque = 0, abs_rpm;
static float Id_desired_integrator = 0;

static float Id_rms = 0, Iq_rms = 0;
static float Id_limit = 50, Iq_limit = 50;
static float PfId_limit = 0, NfId_limit = 0;
static float Iph_limit = 100, Ibat_limit = 50;

static float reverse_rpm = 10;
static float L_rpm = 100, M_rpm = 100;

static float Ilimit = 8, rpm_limit = 650;
static float rpm_to_kmph = 1;

static float Vbrake_derate = 54.0, k_brake = 1.0;

static float safe_rpm = 50;

static float Iph_actual = 0;

static float L_Ibat = 1, M_Ibat = 1;
static float L_Iph = 1, M_Iph = 1;

static float pdi = 0.2, ndi = 0.2;

static int Vmode = 0, Immob = 0, reverse = 0;

static int CAN_baud = 500;

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
    reverse_rpm = REVERSE_RPM;
    
    L_rpm = L_RPM;
    M_rpm = M_RPM;
    
    throttle_zero = THROTTLE_ZERO;
    throttle_max = THROTTLE_MAX;
    
    brake_zero = THROTTLE_ZERO;
    brake_max = THROTTLE_MAX;
        
    Vbrake_derate = VBRAKE_DERATE;
    k_auto_brake = K_AUTO_BRAKE/100.0; 
    
    rpm_to_kmph = RPM_TO_KMPH;
    
    kp_rpm = KP_RPM;
    ki_rpm = KI_RPM;
    
    L_acceleration = L_ACCELERATION;
    M_acceleration = M_ACCELERATION;
    
    L_Ibat = L_IBAT;
    M_Ibat = M_IBAT;
    
    L_Iph = L_IPH;
    M_Iph = M_IPH;
    
    CAN_baud = CANBAUD;
    
    tune_powertrain_variables(&poles, &Iph_max, &rpm_max, &rpm_to_kmph, &motor_derateC, &ESC_derateC, &Vbat_max, &Vbat_min, &Ibat_max, &Ibat_regen, &CAN_baud);
    
    if(CAN_baud == 250) CAN1_Initialize250();
    else CAN1_Initialize500();
    
    tune_vehicle_veriables(&driving_mode, &reverse_rpm, &L_rpm, &M_rpm, &throttle_zero, &throttle_max, &Vbrake_derate, &k_auto_brake, &kp_rpm, &ki_rpm, &L_acceleration, &M_acceleration, &L_Ibat, &M_Ibat, &L_Iph, &M_Iph);
       
    *p_adr = poles;     
    *throttle_zero_adr = throttle_zero;
    *throttle_max_adr = throttle_max;
    abs_max_rpm = fabs(rpm_max);
    Tfault_motorC = motor_TfaultC;
    Tfault_ESCC = ESC_TfaultC;
    *rpm_to_speed_adr = rpm_to_kmph;
    
    pdi = DI;
    ndi = DI;
    
    rpm_limit = rpm_max;
    Ibat_limit = Ibat_max;
    Iph_limit = Iph_max;
    
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
}

void torque_rpm_calculation(float Vdc, float Idc, float w, int p, float * rpm_adr, float * Torq_adr, float * Idc_f_adr, float * Iph_act_adr )
{
    static float itorque = 0;
    static float fIdc = 0;
        
    rpm = w/2.0/3.14159265*120.0/p;
    
    abs_rpm = fabs(rpm);
        
    *rpm_adr = rpm;
    
    fIdc = fIdc + 0.01*(Idc - fIdc);
    *Idc_f_adr = fIdc;
    
    itorque = Iph_actual/20.0;
       
    torque = torque + 0.3*(itorque - torque);
    
    *Torq_adr = torque;
    
    *Iph_act_adr = Iph_actual;
}

int drive_condition(float a_input, float b_input, int fault_id)
{    
    static int current_drive_state = 0;
    static int stop_wait_counter = 0;
    static long int b_count = 0;
    
    if(driving_mode > 4) driving_mode = 3;  //default mode Iref_control
    if(driving_mode < 0) driving_mode = 3;
        
    Immob = GPIO_immob();        
    Vmode = GPIO_Vmode();
    reverse = GPIO_reverse(); 
    
    if(a_input == 0) 
    {
        if(Immob == 1)
        {
            Nop();
        }
        else if(reverse == 1)
        {
            Nop();
        }
        else if(Vmode == 2)
        {
            rpm_limit = rpm_max;
            Ibat_limit = Ibat_max;
            Iph_limit = Iph_max;
            pdi = ndi;
        }
        else if(Vmode == 1)
        {
            rpm_limit = L_rpm * rpm_max/100.0;
            Ibat_limit = L_Ibat * Ibat_max/100.0;
            Iph_limit = L_Iph * Iph_max/100.0;
            pdi = L_acceleration * ndi/100.0;
        }
        else
        {
            rpm_limit = M_rpm * rpm_max/100.0;
            Ibat_limit = M_Ibat * Ibat_max/100.0;
            Iph_limit = M_Iph * Iph_max/100.0;
            pdi = M_acceleration * ndi/100.0;
        }
    }
    
    #define BCOUNT 1920000      //10 min
    if(b_input > 0) b_count = BCOUNT;
    else b_count = b_count - 1;
    if(b_count < 0) b_count = 0; 
    
    if(current_drive_state == OFF)    
    {
        if(Immob == 1)
        {
            rpm_limit = 0;
            Ibat_limit = Ibat_max;
            Iph_limit = Iph_max;
            pdi = ndi;
        }
        else if(reverse == 1)
        {
            rpm_limit = -reverse_rpm * rpm_max/100;
            Ibat_limit = Ibat_max;
            Iph_limit = Iph_max;
            pdi = ndi;
        }
        
        if((b_count > 0))
        {
            Parkmode_indicator(0);
        }
        else Parkmode_indicator(1); 
        
        stop_wait_counter++;
        if(fault_id != 0) stop_wait_counter = 0;
    
        if(stop_wait_counter > 1500)
        {      
            stop_wait_counter = 1500;
     
            if(driving_mode > 0)
            {        
                if((rpm < safe_rpm) & (rpm > -safe_rpm)) //safe rpm to turn on or off
                {
                    if((a_input > 0)  & (rpm_limit != 0) & (b_count > 0)) 
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
            
            if((a_input == 0) &  (turnoff_timer > 700))
            {
                turnoff_timer = 0;
                current_drive_state = OFF;
                b_count = BCOUNT;
            }
        }
    }
    return current_drive_state;
}

void current_ref_generator(float Vdc, float * adr_Idref, float * adr_Iqref, float Tesc, float Tm, float Dduty, float Qduty, float DQmax, float Id_act, float Iq_act, float Ibat_act, float Imosfet)
{
      
    float ESC_derateC_factor = 1, motor_derateC_factor = 1;  
    float Vbrake_margine = 1.0;
    static float fIq_act = 0;
    
    Iph_actual = sqrt(Id_act * Id_act + Iq_act * Iq_act);
    
//Temperature limits============================================================    
    if((Tesc >= ESC_derateC) & (Tfault_ESCC > (ESC_derateC + 1)))
    {
        ESC_derateC_factor = 1 - (Tesc - ESC_derateC)/(Tfault_ESCC - ESC_derateC) ;
    }
    else  ESC_derateC_factor = 1;
    
    if((Tm >= motor_derateC) & (Tfault_motorC > (motor_derateC + 1)))
    {
        motor_derateC_factor = 1 - (Tm - motor_derateC)/(Tfault_motorC-motor_derateC) ;
    }
    else  motor_derateC_factor = 1;    
    
    if(ESC_derateC_factor < 0.1) ESC_derateC_factor = 0.1;
    if(motor_derateC_factor < 0.1) motor_derateC_factor = 0.1;
    
    Ilimit = ESC_derateC_factor * motor_derateC_factor * Iph_limit;  
        
    if(Ilimit > Imosfet) Ilimit = Imosfet;
    
    fIq_act = Iq_desired;//fIq_act + 0.1*(Iq_act - fIq_act);
              
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
    
    Ibat_regen = Ibat_regen * k_brake;
    
  


    PfId_limit = 0;
    NfId_limit = 0;
    
   
    
//Id, Iq referance==============================================================    

    if(Id_desired < -Id_limit) Id_desired = -Id_limit;
    if(Id_desired > Id_limit) Id_desired = Id_limit;
    
    if(Id_rms < Id_desired) Id_rms = Id_rms + pdi;
    else if (Id_rms > Id_desired) Id_rms = Id_rms - ndi;

//==============================================================================  
//Iq calcultaion    
    float Iq_req = 0;
    
    Iq_desired = Iq_desired + 0.1*(Iq_req - Iq_desired);

    if(Iq_desired > Iq_limit ) Iq_desired = Iq_limit;
    if(Iq_desired < -Iq_limit ) Iq_desired = -Iq_limit;
    
    if(Iq_rms < Iq_desired) Iq_rms = Iq_rms + pdi;
    else if (Iq_rms > Iq_desired ) Iq_rms = Iq_rms - ndi;
    
    *adr_Idref = Id_rms/0.5775; //peak value and transformation factor
    *adr_Iqref = Iq_rms/0.5775;
} 
    
void vehicle_control(float a_input, float b_input, float dt)
{
    float rpm_ref = 0;
    float rpm_err = 0;
    
    float P_Ilimit = 0;
    float N_Ilimit = 0;
    
    float brake_rpm = 1, krpm_brake = 0;   //to avoid electric braking at low rpms
    float regen_brake = 0.0;
        
    float fkp_rpm = 0.1;
    float fki_rpm = 0.1;
    
    //Peak current limit============================================================
    #define OVL_RPM_MAX 1000
    #define OVL_TIME_LIMIT 5
    #define OVL_DIS_TIME 1 
    #define OVL_IPH 300
    
    static float ovl_time = 0;
    static float ovl_rpm = 0, fovl_rpm = 0;
    static float ovl_prt_EN = 0;
    
    ovl_rpm = abs_rpm;
    if(ovl_rpm > OVL_RPM_MAX) ovl_rpm = OVL_RPM_MAX;
    fovl_rpm = fovl_rpm + 0.02*(ovl_rpm - fovl_rpm);
        
    ovl_time = ovl_time + 2 * (Iph_actual/OVL_IPH - 0.5) * (OVL_RPM_MAX - fovl_rpm )/ OVL_RPM_MAX * dt;
    
    if(ovl_time > OVL_TIME_LIMIT) ovl_prt_EN = 1;
    if(ovl_time < OVL_DIS_TIME) ovl_prt_EN = 0;
    if(ovl_prt_EN == 1)    a_input = 0;
    
    
    //==========================================================================
    fkp_rpm = kp_rpm;
    if(fkp_rpm < KP_RPM_MIN) fkp_rpm = KP_RPM_MIN;
    fki_rpm = ki_rpm;
    if(fki_rpm < KI_RPM_MIN) fki_rpm = KI_RPM_MIN;
    
    regen_brake = k_auto_brake * Id_limit;    
    
    if(b_input == 0) rpm_ref = a_input * rpm_limit;     
    else rpm_ref = 0;   
    
    brake_rpm = BRAKE_RPM * rpm_limit/100.0;
    if(brake_rpm < 1) brake_rpm = 1;
    
    krpm_brake = fabs(rpm)/brake_rpm;
    if(krpm_brake < 0)krpm_brake = 0;
    if(krpm_brake > 1)krpm_brake = 1;
    
    if(rpm_limit > 0)       //in case of forward
    {
        P_Ilimit = a_input * PfId_limit;         
        if(a_input == 0) N_Ilimit = krpm_brake * regen_brake;
        else N_Ilimit = 0;
        if(N_Ilimit > NfId_limit) N_Ilimit = NfId_limit;
    }
    else if(rpm_limit < 0)  //in case of reverse
    {
        N_Ilimit = a_input * PfId_limit;
        if(a_input == 0) P_Ilimit = krpm_brake * regen_brake;
        else P_Ilimit = 0;
        if(P_Ilimit > NfId_limit) P_Ilimit = NfId_limit;
    }
    else                    //in case of immobolize
    {
        P_Ilimit = 0; 
        N_Ilimit = 0; 
    }
    
    rpm_err = rpm_ref - rpm;
    
    Id_desired_integrator = Id_desired_integrator + rpm_err * dt * fki_rpm;
    
    if(Id_desired_integrator > P_Ilimit) Id_desired_integrator = P_Ilimit;
    if(Id_desired_integrator < -N_Ilimit) Id_desired_integrator = -N_Ilimit;
    
    Id_desired = Id_desired_integrator + rpm_err * fkp_rpm;
        
    if(Id_desired > P_Ilimit) Id_desired = P_Ilimit;
    if(Id_desired < -N_Ilimit) Id_desired = -N_Ilimit;         
}
       
void torque_control(float a_input, float b_input, float dt)
{
    dt = dt;
        Id_desired = a_input * PfId_limit;
    Iq_desired = 0;
    
    if(Id_desired > PfId_limit) Id_desired = PfId_limit;
    if(Id_desired < -NfId_limit) Id_desired = -NfId_limit;
}

void rpm_control(float a_input, float b_input, float dt)
{
    Id_desired = 0;
    Iq_desired = 0;
    Nop();
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


