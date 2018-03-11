/*
 * firefly_height_PD_control.c
 */

#include "firefly_height_PD_control.h"
#include "sdk.h"

short calculateThrust(int desired_height, int current_height, int current_velocity){
	// need to change the unit to m

	// for safety test, make sure the UAV doesn't suddenly fall down
	// soft landing, the thrust need to be determined by test fly
	if(current_velocity < 0){
		return 15.4*4095/36;
	}
	else{
		int error_position = current_height - desired_height;
		int error_velocity = current_velocity;
		short thrust = mass*gravity - K_position*error_position - K_velocity*error_velocity;
		return thrust*4095/36;		// max thrust: 36N.  0..4095 = 0..100%
	}
}
