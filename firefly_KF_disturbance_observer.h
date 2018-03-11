/*
 * firefly_KF_disturbance_observer.h
 */

#ifndef FIREFLY_KF_DISTURBANCE_OBSERVER_H_
#define FIREFLY_KF_DISTURBANCE_OBSERVER_H_

#include "CMatrixLib.h"

typedef struct {
	matrix_float * P;	// state covariance
//	matrix_float * F_continous_time;	// continous system matrix
	matrix_float * F;	// system matrix
	matrix_float * Q;	// process noise covariance
	matrix_float * H; 	// measurement matrix
	matrix_float * R; 	// measurement noise covariance
	int kStateSize;
	int kMeasurementSize;
	matrix_float * drag_coefficients_matrix;
	vector_float * state;
	vector_float * position;
	vector_float * velocity;
	vector_float * attitude;
	vector_float * angular_velocity;
	vector_float * external_forces;
	vector_float * external_moments;
	vector_float * command_roll_pitch_yaw_thrust;
	vector_float * measurement;
	matrix_float * rotation_matrix;
} kalmanHandler;

#define kGravity 9.8066

  // covariance of initial state
#define P0_position 0.1
#define P0_velocity 0.1
#define P0_attitude 0.1
#define P0_angular_velocity 0.1
#define P0_force 0.1
#define P0_torque 0.1

  // process noise
#define q_position 0.01
#define q_velocity 0.025
#define q_attitude 0.015
#define q_angular_velocity 0.02
#define q_force 0.1
#define q_torque 0.1

  // measurement noise
#define r_position 0.001
#define r_velocity 0.0012
#define r_attitude 0.01

  // limits
#define omega_limit_x   3.000000
#define omega_limit_y	3.000000
#define omega_limit_z	2.000000
#define external_forces_limit_x     5.000000       // m/s^2
#define external_forces_limit_y		5.000000
#define external_forces_limit_z		3.000000
#define external_moments_limit_x    20.000000  // rad/s^2
#define external_moments_limit_y	20.000000
#define external_moments_limit_z	20.000000

  // model from system identification (2nd order attitude model)
#define Ax  0.010000	// drag_coefficients
#define Ay	0.010000
#define Az	0
#define roll_omega_  7.07
#define roll_damping_  0.72
#define roll_gain_  0.8
#define pitch_omega_  7.07
#define pitch_damping_  0.76
#define pitch_gain_  0.84
#define yaw_omega_  5.000000
#define yaw_damping_  0.950000
#define yaw_gain_  1.000000

void initialize(void);
void updateEstimator(void);
void systemDynamics(float dt);

#endif /* FIREFLY_KF_DISTURBANCE_OBSERVER_H_ */
