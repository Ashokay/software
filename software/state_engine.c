#include "driver_ADC.h"	
#include "driver_PWM.h"
#include "state_engine.h"
#include "fault.h"
#include "device.h"
#include "math.h"
#include "driver_GPIO.h"
#include "application_software.h"
#include "tuning.h"
#include "driver_UART.h"
#include "diagnostics.h"
#include "driver_SPI.h"
#include "driver_CANbus.h"
#include "VCU_CANdata.h"
#include "default_ESC_config.h"

    #define SLOW_ROUTINE_COUNT 5

    #define FAULT 0
	#define CALIBERATION 1
	#define STOP 2
	#define PMSM 3
    #define ROTOR_HOLD 4

    #define OFF 0
    #define RPM_CONTROL 1
    #define TORQUE_CONTROL 2
    #define VEHICLE_CONTROL 3
    #define ROTOR_POS_ID 4

//non tunable parameters========================================================

    #define KP_SLESS_CON 50.0
    #define KI_SLESS_CON 200.0

    #define DW_SLESS_SAT 1000.0

    #define KP_ICON 0.002           //test with expendable controller 
    #define KI_ICON 0.5             //Kp: 0.003, Ki: 0.5
    #define DQ_DUTY_MAX 1.41        //including vector factor

//state variables===============================================================

	static int system_state = CALIBERATION;
	static int fault_id = 0;
	static int slow_routine_counter = 0;

//motor variables===============================================================	
    static int poles = 10;
	static float Vdc = 0, Ibat = 0, fIbat = 0;
	static float Iu = 0, Iv = 0, Iw = 0; 
	static float cosMR, sinMR;
    static int rotation_direction = 0, sensor_direction = 0;
    static float zero_angle = 310;
	static float MRcos0, MRsin0;
    static float wMR = 0;	
	
    static float Tm = 0, Tesc = 0;
    static float Imosfet = 200;
    static float Rph = 0.002, Ld = 0.000018, Lq = 0.000018, PHIph = 0.0068; 
    
//control variables=============================================================	    
	static float dt = 0.00004;
    static float du = 0, dv = 0, dw =0;
    static float dutyD = 0, dutyQ = 0;
    static float OV_fault = 60.0, UV_fault = 36.0, Ibat_fault = 400.0, Iph_fault = 500.0, ESC_TfaultC = 110, motor_TfaultC = 110.0;
    static float rpm_fault = 10000;
    
    static float kp_sless_con = 20.0, ki_sless_con = 200.0;
    static float dw_sless_sat = 2000;
    static float ang_margine = 30.0;
    static float kp_icon = 0.0005, ki_icon = 0.2;
    static float duty_sat = 0.9, dq_duty_sat = 1.41;
    
    static float throttle_zero = 1.0, throttle_max = 4.0;
    static float brake_zero = 2.0, brake_max = 4.0; 
    
    static float a_input = 0, b_input = 0;	
	static float Idref = 5.0, Iqref = 0;
    static float Id = 0, Iq = 0;
    static int drive_state = 0;
    
    static float rpm = 0, Torq = 0;
    static float rpm_to_speed = 1;
    
    static float Iph_act = 0;
    
//Functions=====================================================================    
    
void state_init(void)  //To be called first in main()
{
    
    system_state = CALIBERATION;
    drive_state = 0;
    
    PWM_init(&dt, &duty_sat);
    dq_duty_sat = DQ_DUTY_MAX*duty_sat;   
    
    kp_sless_con = KP_SLESS_CON;
    ki_sless_con = KI_SLESS_CON;
    dw_sless_sat = DW_SLESS_SAT;
    
    kp_icon = KP_ICON;
    ki_icon = KI_ICON;


 
//Default values of tunable parameters==========================================
           
    poles = POLES;
    
    Rph = RPH;
    PHIph = PHIPH;
    Ld = LD;    
    Lq = LQ;
    ang_margine = ANG_MARGINE;
    zero_angle = ZERO_ANGLE;   
    
    sensor_direction = SENSOR_DIRECTION;  
    rotation_direction = ROTATION_DIRECTION;
    
    OV_fault = OVFLT;
    UV_fault = UVFLT;
    
    Iph_fault = IPHFLT * 1.414213;
    Ibat_fault = IBATFLT;
    ESC_TfaultC = ESC_TFLTC;  
    motor_TfaultC = MOTOR_TFLTC;
    
    rpm_fault = RPM_FLT;
    
    
//==============================================================================    
    
    tune_state_variables(&Rph, &Ld, &Lq, &PHIph, &zero_angle, &sensor_direction, &ang_margine, &rotation_direction);
    tune_faults(&rpm_fault, &OV_fault, &UV_fault, &Iph_fault, &Ibat_fault, &ESC_TfaultC, &motor_TfaultC);
    
    if(rotation_direction == 1)    //for reversing the direction of rotation
    {
        zero_angle = 180 - zero_angle;
        if(sensor_direction == 1) sensor_direction = 0;
        else sensor_direction = 1;
    }
    
    MRcos0 = cos(zero_angle * 3.14159265/180);
    MRsin0 = sin(zero_angle * 3.14159265/180);  
    
    drive_init(&poles, &throttle_zero, &throttle_max, motor_TfaultC, ESC_TfaultC, &rpm_to_speed, &brake_zero, &brake_max);    
}
	

void fast_routines(void)
{
	static int caliberation_counter = 0;
	
	slow_routine_counter++;
	       
	fast_measurement_VI(&Vdc, &Ibat, &Iu, &Iv, &Iw);
	fast_measurement_RP(&cosMR, &sinMR, sensor_direction);

	if(fault_id != 0) system_state = FAULT;
    
    if((system_state != FAULT) & (system_state != CALIBERATION)) fault_id = fast_fault_check();	
    
	switch(system_state)
	{
		case CALIBERATION:		          
            PWM_override_OFF();     //keep phases off
            ADC_calib();            //caliberate current sensors
            caliberation_counter++;
            if(caliberation_counter >= 10000)
            {
                caliberation_counter = 0;
                system_state = STOP;
            }			
		break;
		
		case STOP:		
            PWM_override_OFF();        
		break;
		
		case PMSM:		            
            
            duty_cal_update(du, dv, dw);
		break;
        
        case ROTOR_HOLD: 
            measurement_constants_update(1, 0, poles, throttle_zero, throttle_max, brake_zero, brake_max);
            duty_cal_update(du, dv, dw);            
        break;        
		
		case FAULT:		
            fault(fault_id);            
		break;
		
		default:            
            system_state = FAULT;
			fault_id = 9;
		break;	
	}    
}

void slow_routines(void)
{
	if(slow_routine_counter >= 5) 
	{
		slow_routine_counter = 0;
        
        system_state_indicator(system_state, fault_id);
        
		slow_measurement_T(&Tesc, &Imosfet, &Tm);
        
		slow_measurement_input((dt*5), &wMR, &a_input, &b_input);	
                
		if((system_state != FAULT) & (system_state != CALIBERATION)) 
        {
            fault_id = slow_fault_check();
        }
        
        drive_state = drive_condition(a_input);       //throttle on-off
        
        torque_rpm_calculation(Vdc, Ibat, wMR, poles, &rpm, &Torq, &fIbat, &Iph_act);        
        
        fill_diag_data(system_state, Vdc, fIbat, rpm, Torq, Tesc, Tm);
        send_byte_uart();
        recieve_command_uart();
        
        fill_data_for_VCU(rpm_to_speed, rpm, fIbat, Iph_act, Tesc, fault_id);
        CANbus_read();
        
		switch(system_state)
	    {	
			case CALIBERATION:
				fault_limits_update(OV_fault, UV_fault, Ibat_fault, Iph_fault, ESC_TfaultC, rpm_fault, motor_TfaultC);
			break;
			
			case STOP:			
                measurement_constants_update(MRcos0, MRsin0, poles, throttle_zero, throttle_max, brake_zero, brake_max);         //update parameters
                fault_limits_update(OV_fault, UV_fault, Ibat_fault, Iph_fault, ESC_TfaultC, rpm_fault, motor_TfaultC);

                if(drive_state > 0)
                {
                    if(drive_state == ROTOR_POS_ID) system_state = ROTOR_HOLD;
                    else system_state = PMSM;
                    
                    //PWMpins_enable();
                }
			break;
			
			case PMSM:                          
                
                if(drive_state == RPM_CONTROL) torque_control(a_input, b_input, dt*5);
                else if (drive_state == TORQUE_CONTROL) torque_control(a_input, b_input, dt*5);
                //else if (drive_state == VEHICLE_CONTROL) vehicle_control(a_input, b_input, dt*5);
                else system_state = STOP;
                
                current_ref_generator(Vdc, &Idref, &Iqref, Tesc, Tm, dutyD, dutyQ, dq_duty_sat, Id*0.5775, Iq*0.5775, Ibat, Imosfet);                
			break;
			
			case FAULT:
			
			break;
            
            case ROTOR_HOLD:
                if(drive_condition(a_input) == OFF) system_state = STOP;
                current_ref_hold(&Idref, &Iqref);
                angle_finder(cosMR, sinMR, sensor_direction);
            break;
                        		
			default:
                
			break;	            
		}	        
	}
}

