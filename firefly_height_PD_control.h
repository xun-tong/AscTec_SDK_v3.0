/*
 * firefly_height_PD_control.h
 */

#ifndef FIREFLY_HEIGHT_PD_CONTROL_H_
#define FIREFLY_HEIGHT_PD_CONTROL_H_

short calculateThrust(int desired_height, int current_height, int current_velocity);

#define K_position 6
#define K_velocity 4.7

#define gravity 9.80665
#define mass 1.56779

#endif /* FIREFLY_HEIGHT_PD_CONTROL_H_ */
