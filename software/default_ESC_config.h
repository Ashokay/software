

#ifndef _DEFAULT_ESC_CONFIG_H    /* Guard against multiple inclusion */
#define _DEFAULT_ESC_CONFIG_H


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    #define POLES 10

    #define RPH 0.016
    #define LD 0.000018
    #define LQ 0.000018
    #define PHIPH 0.0068

    #define SENSOR_DIRECTION 0         
    #define ROTATION_DIRECTION 0       //reverse rule: 180 - zero_angle

    #define ANG_MARGINE 20.0  
    #define ZERO_ANGLE 255.0  

    #define OVFLT 60.0
    #define OVFLT_MAX 65
    #define UVFLT 42.0

    #define IPHFLT 300.0
    #define IPH_FLT_MAX 400.0 
    #define IBATFLT 150.0
    #define IBAT_FLT_MAX 250.0

    #define ESC_TFLTC 90.0
    #define ESC_TFLTC_MAX 125.0
    #define MOTOR_TFLTC 250.0

    #define RPM_FLT 7500  
    #define RPM_TO_KMPH .012
    
    #define RPM_MAX 7000
    #define IPH_MAX 200    
    #define IPH_MAX_HLIMIT 400          //to limit user entry

    #define MOTOR_DERATEC 250
    #define ESC_DERATEC 70

    #define VBAT_MAX 58
    #define VBAT_MIN 44

    #define IBAT_MAX 100
    #define IBAT_REGEN 80

    #define DRIVING_MODE 1
    #define DRIVING_INTERFACE 0
    #define LECO_RPM 60
    #define ECO_RPM 30    

    #define THROTTLE_ZERO 1.0
    #define THROTTLE_ZERO_LL 0.2    
    #define THROTTLE_ZERO_HL 1.5
    #define THROTTLE_MAX 4.0
    #define THROTTLE_MAX_LL 1.6
    #define THROTTLE_MAX_HL 4.5

    #define VBRAKE_DERATE 54
    #define IMMOB_RPM 5
    #define REVERSE_RPM 5

    #define KP_RPM 0.05
    #define KI_RPM_MAX 2.0
    #define KI_RPM_MIN 0.00001     

    #define KI_RPM 0.2
    #define KP_RPM_MAX 0.5
    #define KP_RPM_MIN 0.00001    

    #define KP_TORQUE 5.0
    #define KI_TORQUE 100.0

    #define BRAKE_ZERO 2.0
    #define BRAKE_ZERO_LL 1.0    
    #define BRAKE_ZERO_HL 3.0
    #define BRAKE_MAX 4.0
    #define BRAKE_MAX_LL 3.6
    #define BRAKE_MAX_HL 4.5
    
    #define SAFE_RPM 2               //% of max rpm for safe turn on/turn off

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _DEFAULT_ESC_CONFIG_H */

/* *****************************************************************************
 End of File
 */
