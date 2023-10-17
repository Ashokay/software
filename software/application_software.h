
#ifndef APPLICATION_SOFTWARE_H
#define	APPLICATION_SOFTWARE_H

#ifdef	__cplusplus
extern "C" {
#endif

void drive_init(int *, float*, float*, float, float, float*, float*, float*);
void drive_reset(float *, float *);
int drive_condition(float);    
void current_ref_generator(float, float *, float *, float, float, float, float, float, float, float, float, float);
//Vdc, * adr_Idref, * adr_Iqref, Tesc, Tm, Dduty, Qduty, DQmax, Id_act, Iq_act, Ibat_act, Imosfet
void torque_rpm_calculation(float, float, float, int, float *, float *, float *, float *);
void rpm_control(float, float, float);
void torque_control(float, float, float);
void vehicle_control(float, float, float);
void angle_finder(float, float, int);
void current_ref_hold(float *, float *);

#ifdef	__cplusplus
}
#endif

#endif	/* APPLICATION_SOFTWARE_H */

