

#ifndef _STATES_ENGINE_H    /* Guard against multiple inclusion */
#define _STATES_ENGINE_H


#ifdef __cplusplus
extern "C" {
#endif

    void state_init(void);      //initialize state variables, state
	void fast_routines(void);   //to be called in the interrupt, contains calls to all fast routines based on state
	void slow_routines(void);   //to be called in main loop: so executes on when 5 counts are completed in fast_routines 

#ifdef __cplusplus
}
#endif

#endif 




