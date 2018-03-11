/*
 * firefly_KF_disturbance_observer.c
 */

#include "firefly_KF_disturbance_observer.h"

kalmanHandler KalmanHandler;

void initialize(void){
	KalmanHandler.kStateSize = 18;
	KalmanHandler.kMeasurementSize = 9;

	/* continous system matrix F_continous_time */
/*	matrix_float_set_zero(KalmanHandler.F_continous_time);
	matrix_float_set(KalmanHandler.F_continous_time, 1, 4, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 2, 5, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 3, 6, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 4, 4, -Ax);
	matrix_float_set(KalmanHandler.F_continous_time, 5, 5, -Ay);
	matrix_float_set(KalmanHandler.F_continous_time, 6, 6, -Az);
	matrix_float_set(KalmanHandler.F_continous_time, 4, 13, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 5, 14, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 6, 15, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 7, 10, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 8, 11, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 9, 12, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 10, 7, -roll_omega_ * roll_omega_);
	matrix_float_set(KalmanHandler.F_continous_time, 11, 8, -pitch_omega_ * pitch_omega_);
	matrix_float_set(KalmanHandler.F_continous_time, 12, 9, -yaw_omega_ * yaw_omega_);
	matrix_float_set(KalmanHandler.F_continous_time, 10, 10, -2.0*roll_omega_ * roll_damping_);
	matrix_float_set(KalmanHandler.F_continous_time, 11, 11, -2.0*pitch_omega_ * pitch_damping_);
	matrix_float_set(KalmanHandler.F_continous_time, 12, 12, -2.0*yaw_omega_ * yaw_damping_);
	matrix_float_set(KalmanHandler.F_continous_time, 10, 16, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 11, 17, 1);
	matrix_float_set(KalmanHandler.F_continous_time, 12, 18, 1);	*/

	/* discrete system matrix F */
	// sampling_time not determined yet ??
	KalmanHandler.F->height = KalmanHandler.kStateSize;
	KalmanHandler.F->width = KalmanHandler.kStateSize;
	matrix_float_set_all(KalmanHandler.F, 1);
	matrix_float_set(KalmanHandler.F, 1, 4, 1.0101);
	matrix_float_set(KalmanHandler.F, 2, 5, 1.0101);
	matrix_float_set(KalmanHandler.F, 3, 6, 1.0101);
	matrix_float_set(KalmanHandler.F, 7, 10, 1.0101);
	matrix_float_set(KalmanHandler.F, 8, 11, 1.0101);
	matrix_float_set(KalmanHandler.F, 9, 12, 1.0101);
	matrix_float_set(KalmanHandler.F, 4, 13, 1.0101);
	matrix_float_set(KalmanHandler.F, 5, 14, 1.0101);
	matrix_float_set(KalmanHandler.F, 6, 15, 1.0101);
	matrix_float_set(KalmanHandler.F, 10, 16, 1.0101);
	matrix_float_set(KalmanHandler.F, 11, 17, 1.0101);
	matrix_float_set(KalmanHandler.F, 12, 18, 1.0101);
	matrix_float_set(KalmanHandler.F, 4, 4, 0.9999);
	matrix_float_set(KalmanHandler.F, 5, 5, 0.9999);
	matrix_float_set(KalmanHandler.F, 10, 7, 0.6066);
	matrix_float_set(KalmanHandler.F, 11, 8, 0.6066);
	matrix_float_set(KalmanHandler.F, 12, 9, 0.7788);
	matrix_float_set(KalmanHandler.F, 10, 10, 0.9032);
	matrix_float_set(KalmanHandler.F, 11, 11, 0.8981);
	matrix_float_set(KalmanHandler.F, 12, 12, 0.9094);

	/* measurement matrix H */
	KalmanHandler.H->height = KalmanHandler.kMeasurementSize;
	KalmanHandler.H->width = KalmanHandler.kStateSize;
	matrix_float_set_zero(KalmanHandler.H);
	for(int i = 1; i < 10; i++){
		matrix_float_set(KalmanHandler.H, i, i, 1);
	}


	/* state covariance P */
	KalmanHandler.P->height = KalmanHandler.kStateSize;
	KalmanHandler.P->width = KalmanHandler.kStateSize;
	matrix_float_set_zero(KalmanHandler.P);
	for(int i = 1; i < 4; i++){
		matrix_float_set(KalmanHandler.P, i, i, P0_position);
		matrix_float_set(KalmanHandler.P, i+3, i+3, P0_velocity);
		matrix_float_set(KalmanHandler.P, i+6, i+6, P0_attitude);
		matrix_float_set(KalmanHandler.P, i+9, i+9, P0_angular_velocity);
		matrix_float_set(KalmanHandler.P, i+12, i+12, P0_force);
		matrix_float_set(KalmanHandler.P, i+15, i+15, P0_torque);
	}

	/* process noise covariance Q */
	KalmanHandler.Q->height = KalmanHandler.kStateSize;
	KalmanHandler.Q->width = KalmanHandler.kStateSize;
	matrix_float_set_zero(KalmanHandler.Q);
	for(int i = 1; i < 4; i++){
		matrix_float_set(KalmanHandler.Q, i, i, q_position);
		matrix_float_set(KalmanHandler.Q, i+3, i+3, q_velocity);
		matrix_float_set(KalmanHandler.Q, i+6, i+6, q_attitude);
		matrix_float_set(KalmanHandler.Q, i+9, i+9, q_angular_velocity);
		matrix_float_set(KalmanHandler.Q, i+12, i+12, q_force);
		matrix_float_set(KalmanHandler.Q, i+15, i+15, q_torque);
	}

	/* measurement noise covariance R */
	KalmanHandler.R->height = KalmanHandler.kMeasurementSize;
	KalmanHandler.R->width = KalmanHandler.kMeasurementSize;
	matrix_float_set_zero(KalmanHandler.R);
	for(int i = 1; i < 4; i++){
		matrix_float_set(KalmanHandler.R, i, i, r_position);
		matrix_float_set(KalmanHandler.R, i+3, i+3, r_velocity);
		matrix_float_set(KalmanHandler.R, i+6, i+6, r_attitude);
	}

	/* drag_coefficients_matrix */
	KalmanHandler.drag_coefficients_matrix->height = 3;
	KalmanHandler.drag_coefficients_matrix->width = 3;
	matrix_float_set(KalmanHandler.drag_coefficients_matrix, 1, 1, Ax);
	matrix_float_set(KalmanHandler.drag_coefficients_matrix, 2, 2, Ay);
	matrix_float_set(KalmanHandler.drag_coefficients_matrix, 3, 3, Az);

	/* set command_roll_pitch_yaw_thrust to zero */
	KalmanHandler.command_roll_pitch_yaw_thrust->length = 4;
	KalmanHandler.command_roll_pitch_yaw_thrust->orientation = 0;
	vector_float_set_zero(KalmanHandler.command_roll_pitch_yaw_thrust);

	/* set initial state to zero */
	KalmanHandler.state->length = KalmanHandler.kStateSize;
	KalmanHandler.state->orientation = 0;		// set orientation to vertical
	vector_float_set_zero(KalmanHandler.state);

	/* set initial position to zero */
	KalmanHandler.position->length = 3;
	KalmanHandler.position->orientation = 0;		// set orientation to vertical
	vector_float_set_zero(KalmanHandler.position);

	/* set initial velocity to zero */
	KalmanHandler.velocity->length = 3;
	KalmanHandler.velocity->orientation = 0;		// set orientation to vertical
	vector_float_set_zero(KalmanHandler.velocity);

	/* set initial attitude to zero */
	KalmanHandler.attitude->length = 3;
	KalmanHandler.attitude->orientation = 0;		// set orientation to vertical
	vector_float_set_zero(KalmanHandler.attitude);

	/* set initial angular_velocity to zero */
	KalmanHandler.angular_velocity->length = 3;
	KalmanHandler.angular_velocity->orientation = 0;		// set orientation to vertical
	vector_float_set_zero(KalmanHandler.angular_velocity);

	/* set initial external_forces to zero */
	KalmanHandler.external_forces->length = 3;
	KalmanHandler.external_forces->orientation = 0;		// set orientation to vertical
	vector_float_set_zero(KalmanHandler.external_forces);

	/* set initial external_moments to zero */
	KalmanHandler.external_moments->length = 3;
	KalmanHandler.external_moments->orientation = 0;		// set orientation to vertical
	vector_float_set_zero(KalmanHandler.external_moments);
}

void updateEstimator(void){
	/* P = F*P*F' + Q */
	matrix_float * temp1;
	temp1->height = KalmanHandler.F->height;
	temp1->width = KalmanHandler.P->width;
	matrix_float_mul(KalmanHandler.F, KalmanHandler.P, temp1);	// temp1 = F*P
	matrix_float_mul_trans(temp1, KalmanHandler.F, KalmanHandler.P);		// P = temp1*F' = F*P*F'
	matrix_float_add(KalmanHandler.P, KalmanHandler.Q);		// P = P + Q = F*P*F' + Q

	/* predict state */
	systemDynamics(0.01);	//??? dt need to be set

	/* S = H*P*H' + R */
	matrix_float * S;
	S->height = KalmanHandler.R->height;
	S->width = KalmanHandler.R->width;
	matrix_float * temp2;
	temp2->height = KalmanHandler.H->height;
	temp2->width = KalmanHandler.P->width;
	matrix_float_mul(KalmanHandler.H, KalmanHandler.P, temp2);	// temp2 = H*P
	matrix_float_mul_trans(temp2, KalmanHandler.H, S);		// S = temp2*H' = H*P*H'
	matrix_float_add(S, KalmanHandler.R);		// S = S + R = H*P*H' + R

	/* K = P*H'*inv(S) */
	matrix_float * K;
	K->height = KalmanHandler.P->height;
	K->width = S->width;
	matrix_float * temp3;
	temp3->height = KalmanHandler.P->height;
	temp3->width = KalmanHandler.H->height;
	matrix_float_mul_trans(KalmanHandler.P, KalmanHandler.H, temp3); // temp3 = P*H'
	matrix_float_inverse(S);	// inv(S)
	matrix_float_mul(temp3, S, K);		// K = temp3*inv(S) = P*H'*inv(S)

	/* update states with measurements:  x = x + K*(z - H*x) */
	vector_float * temp4;
	temp4->length = KalmanHandler.measurement->length;
	temp4->orientation = 0;
	matrix_float_mul_vec_right(KalmanHandler.H, KalmanHandler.state, temp4);		// temp4 = H*x
	vector_float * temp5;
	temp5->length = KalmanHandler.measurement->length;
	vector_float_copy(temp5, KalmanHandler.measurement);		// temp5 = z
	vector_float_subtract(temp5, temp4);	// temp5 = temp5 - temp4 = z - H*x
	vector_float * temp6;
	temp6->length = KalmanHandler.state->length;
	temp6->orientation = 0;
	matrix_float_mul_vec_right(K, temp5, temp6);	// temp6 = K*temp5 = K*(z - H*x)
	vector_float_add(KalmanHandler.state, temp6);	// x = x + temp6 = x + K*(z - H*x)

	/* update state covariance P = (I - K*H)*P */
	matrix_float * I;
	I->height = K->height;
	I->width = KalmanHandler.H->width;
	matrix_float_set_identity(I);
	matrix_float * temp7;
	temp7->height = K->height;
	temp7->width = KalmanHandler.H->width;
	matrix_float_mul(K, KalmanHandler.H, temp7);		// temp7 = K*H
	matrix_float_sub(I, temp7);		// I = I - temp7 = I - K*H
	matrix_float * temp8;
	temp8->height = KalmanHandler.P->height;
	temp8->width = KalmanHandler.P->width;
	matrix_float_copy(temp8, KalmanHandler.P);		// temp8 = P
	matrix_float_mul(I, temp8, KalmanHandler.P);		// P = I*temp8 = (I - K*H)*P


	/* limits */
	// omega_limit_x
	if(vector_float_get(KalmanHandler.state, 10) < -omega_limit_x){
		vector_float_set(KalmanHandler.state, 10, -omega_limit_x);
	}
	else if(vector_float_get(KalmanHandler.state, 10) > omega_limit_x){
		vector_float_set(KalmanHandler.state, 10, omega_limit_x);
	}
	// omega_limit_y
	if(vector_float_get(KalmanHandler.state, 11) < -omega_limit_y){
		vector_float_set(KalmanHandler.state, 11, -omega_limit_y);
	}
	else if(vector_float_get(KalmanHandler.state, 11) > omega_limit_y){
		vector_float_set(KalmanHandler.state, 11, omega_limit_y);
	}
	// omega_limit_z
	if(vector_float_get(KalmanHandler.state, 12) < -omega_limit_z){
		vector_float_set(KalmanHandler.state, 12, -omega_limit_z);
	}
	else if(vector_float_get(KalmanHandler.state, 12) > omega_limit_z){
		vector_float_set(KalmanHandler.state, 12, omega_limit_z);
	}

	// external_forces_limit_x
	if(vector_float_get(KalmanHandler.state, 13) < -external_forces_limit_x){
		vector_float_set(KalmanHandler.state, 13, -external_forces_limit_x);
	}
	else if(vector_float_get(KalmanHandler.state, 13) > external_forces_limit_x){
		vector_float_set(KalmanHandler.state, 13, external_forces_limit_x);
	}
	// external_forces_limit_y
	if(vector_float_get(KalmanHandler.state, 14) < -external_forces_limit_y){
		vector_float_set(KalmanHandler.state, 14, -external_forces_limit_y);
	}
	else if(vector_float_get(KalmanHandler.state, 14) > external_forces_limit_y){
		vector_float_set(KalmanHandler.state, 14, external_forces_limit_y);
	}
	// external_forces_limit_z
	if(vector_float_get(KalmanHandler.state, 15) < -external_forces_limit_z){
		vector_float_set(KalmanHandler.state, 15, -external_forces_limit_z);
	}
	else if(vector_float_get(KalmanHandler.state, 15) > external_forces_limit_z){
		vector_float_set(KalmanHandler.state, 15, external_forces_limit_z);
	}

	// external_moments_limit_x
	if(vector_float_get(KalmanHandler.state, 16) < -external_moments_limit_x){
		vector_float_set(KalmanHandler.state, 16, -external_moments_limit_x);
	}
	else if(vector_float_get(KalmanHandler.state, 16) > external_moments_limit_x){
		vector_float_set(KalmanHandler.state, 16, external_moments_limit_x);
	}
	// external_moments_limit_y
	if(vector_float_get(KalmanHandler.state, 17) < -external_moments_limit_y){
		vector_float_set(KalmanHandler.state, 17, -external_moments_limit_y);
	}
	else if(vector_float_get(KalmanHandler.state, 17) > external_moments_limit_y){
		vector_float_set(KalmanHandler.state, 17, external_moments_limit_y);
	}
	// external_moments_limit_z
	if(vector_float_get(KalmanHandler.state, 18) < -external_moments_limit_z){
		vector_float_set(KalmanHandler.state, 18, -external_moments_limit_z);
	}
	else if(vector_float_get(KalmanHandler.state, 18) > external_moments_limit_z){
		vector_float_set(KalmanHandler.state, 18, external_moments_limit_z);
	}


	/* update */
	vector_float_set(KalmanHandler.position, 1, vector_float_get(KalmanHandler.state, 1));
	vector_float_set(KalmanHandler.position, 2, vector_float_get(KalmanHandler.state, 2));
	vector_float_set(KalmanHandler.position, 3, vector_float_get(KalmanHandler.state, 3));

	vector_float_set(KalmanHandler.velocity, 1, vector_float_get(KalmanHandler.state, 4));
	vector_float_set(KalmanHandler.velocity, 2, vector_float_get(KalmanHandler.state, 5));
	vector_float_set(KalmanHandler.velocity, 3, vector_float_get(KalmanHandler.state, 6));

	vector_float_set(KalmanHandler.attitude, 1, vector_float_get(KalmanHandler.state, 7));
	vector_float_set(KalmanHandler.attitude, 2, vector_float_get(KalmanHandler.state, 8));
	vector_float_set(KalmanHandler.attitude, 3, vector_float_get(KalmanHandler.state, 9));

	vector_float_set(KalmanHandler.angular_velocity, 1, vector_float_get(KalmanHandler.state, 10));
	vector_float_set(KalmanHandler.angular_velocity, 2, vector_float_get(KalmanHandler.state, 11));
	vector_float_set(KalmanHandler.angular_velocity, 3, vector_float_get(KalmanHandler.state, 12));

	vector_float_set(KalmanHandler.external_forces, 1, vector_float_get(KalmanHandler.state, 13));
	vector_float_set(KalmanHandler.external_forces, 2, vector_float_get(KalmanHandler.state, 14));
	vector_float_set(KalmanHandler.external_forces, 3, vector_float_get(KalmanHandler.state, 15));

	vector_float_set(KalmanHandler.external_moments, 1, vector_float_get(KalmanHandler.state, 16));
	vector_float_set(KalmanHandler.external_moments, 2, vector_float_get(KalmanHandler.state, 17));
	vector_float_set(KalmanHandler.external_moments, 3, vector_float_get(KalmanHandler.state, 18));
}

void systemDynamics(float dt){
/* acceleration = rotation_matrix * (0, 0, T)' + (0, 0, -g)' - drag_coefficients_matrix * old_velocity + old_external_forces */
	// thrust = (0, 0, T)'
	vector_float * thrust;
	thrust->length = 3;
	thrust->orientation = 0;
	vector_float_set_zero(thrust);
	float thrust_value = vector_float_get(KalmanHandler.command_roll_pitch_yaw_thrust, 4);
	vector_float_set(thrust, 3, thrust_value);

	vector_float * acceleration;
	acceleration->length = 3;
	acceleration->orientation = 0;

	// acceleration = rotation_matrix * (0, 0, T)'
	matrix_float_mul_vec_right(KalmanHandler.rotation_matrix, thrust, acceleration);

	// gravity = (0, 0, -g)'
	vector_float * gravity;
	gravity->length = 3;
	gravity->orientation = 0;
	vector_float_set_zero(gravity);
	vector_float_set(gravity, 3, -kGravity);

	// acceleration = acceleration + (0, 0, -g)'
	vector_float_add(acceleration, gravity);

	// temp1 = drag_coefficients_matrix * old_velocity
	vector_float * temp1;
	temp1->length = 3;
	temp1->orientation = 0;
	matrix_float_mul_vec_right(KalmanHandler.drag_coefficients_matrix, KalmanHandler.velocity, temp1);

	// acceleration = acceleration - drag_coefficients_matrix * old_velocity
	vector_float_subtract(acceleration, temp1);		// ?? I think subtract, not add

	// acceleration = acceleration + old_external_forces
	vector_float_add(acceleration, KalmanHandler.external_forces);

/* update velocity and position */
	// temp2 = acceleration * dt
	vector_float * temp2;
	temp2->length = 3;
	vector_float_copy(temp2, acceleration);
	vector_float_times(temp2, dt);

	// temp3 = old_velocity (save for calculating new position later)
	vector_float * temp3;
	temp3->length = 3;
	vector_float_copy(temp3, KalmanHandler.velocity);

	// new_velocity = velocity + acceleration * dt
	vector_float_add(KalmanHandler.velocity, temp2);

	// temp3 = old_velocity * dt
	vector_float_times(temp3, dt);

	// temp2 = 0.5 * acceleration * dt * dt
	vector_float_times(temp2, 0.5 * dt);

	// new_position = position + velocity * dt + 0.5 * acceleration * dt * dt
	vector_float_add(KalmanHandler.position, temp3);
	vector_float_add(KalmanHandler.position, temp2);


/* calculate angular_acceleration */
	vector_float * angular_acceleration;
	angular_acceleration->length = 3;
	angular_acceleration->orientation = 0;

	// get old angular velocity, attitude, attitude command, external_moments
	float p = vector_float_get(KalmanHandler.angular_velocity, 1);
	float q = vector_float_get(KalmanHandler.angular_velocity, 2);
	float r = vector_float_get(KalmanHandler.angular_velocity, 3);

	float roll = vector_float_get(KalmanHandler.attitude, 1);
	float pitch = vector_float_get(KalmanHandler.attitude, 2);
	float yaw = vector_float_get(KalmanHandler.attitude, 3);

	float roll_cmd = vector_float_get(KalmanHandler.command_roll_pitch_yaw_thrust, 1);
	float pitch_cmd = vector_float_get(KalmanHandler.command_roll_pitch_yaw_thrust, 2);
	float yaw_cmd = vector_float_get(KalmanHandler.command_roll_pitch_yaw_thrust, 3);

	float dm1 = vector_float_get(KalmanHandler.external_moments, 1);
	float dm2 = vector_float_get(KalmanHandler.external_moments, 2);
	float dm3 = vector_float_get(KalmanHandler.external_moments, 3);

	float acc_roll = -2.0 * roll_damping_ * roll_omega_ * p - roll_omega_ * roll_omega_ * roll + roll_gain_ * roll_omega_ * roll_omega_ * roll_cmd + dm1;
	float acc_pitch = -2.0 * pitch_damping_ * pitch_omega_ * q - pitch_omega_ * pitch_omega_ * pitch + pitch_gain_ * pitch_omega_ * pitch_omega_ * pitch_cmd + dm2;
	float acc_yaw = -2.0 * yaw_damping_ * yaw_omega_ * r - yaw_omega_ * yaw_omega_ * yaw + yaw_gain_ * yaw_omega_ * yaw_omega_ * yaw_cmd + dm3;

	vector_float_set(angular_acceleration, 1, acc_roll);
	vector_float_set(angular_acceleration, 2, acc_pitch);
	vector_float_set(angular_acceleration, 3, acc_yaw);

/* update angular velocity and attitude */
	// temp4 = angular_acceleration * dt
	vector_float * temp4;
	temp4->length = 3;
	vector_float_copy(temp4, angular_acceleration);
	vector_float_times(temp4, dt);

	// temp5 = old_angular_velocity (save for calculating new attitude later)
	vector_float * temp5;
	temp5->length = 3;
	vector_float_copy(temp5, KalmanHandler.angular_velocity);

	// new_angular_velocity = angular_velocity + angular_acceleration * dt
	vector_float_add(KalmanHandler.angular_velocity, temp4);

	// temp5 = old_angular_velocity * dt
	vector_float_times(temp5, dt);

	// temp4 = 0.5 * angular_acceleration * dt * dt
	vector_float_times(temp4, 0.5 * dt);

	// new_attitude = attitude + angular_velocity * dt + 0.5 * angular_acceleration * dt * dt
	vector_float_add(KalmanHandler.attitude, temp5);
	vector_float_add(KalmanHandler.attitude, temp4);


/* update the state vector */
	vector_float_set(KalmanHandler.state, 1, vector_float_get(KalmanHandler.position, 1));
	vector_float_set(KalmanHandler.state, 2, vector_float_get(KalmanHandler.position, 2));
	vector_float_set(KalmanHandler.state, 3, vector_float_get(KalmanHandler.position, 3));

	vector_float_set(KalmanHandler.state, 4, vector_float_get(KalmanHandler.velocity, 1));
	vector_float_set(KalmanHandler.state, 5, vector_float_get(KalmanHandler.velocity, 2));
	vector_float_set(KalmanHandler.state, 6, vector_float_get(KalmanHandler.velocity, 3));

	vector_float_set(KalmanHandler.state, 7, vector_float_get(KalmanHandler.attitude, 1));
	vector_float_set(KalmanHandler.state, 8, vector_float_get(KalmanHandler.attitude, 2));
	vector_float_set(KalmanHandler.state, 9, vector_float_get(KalmanHandler.attitude, 3));

	vector_float_set(KalmanHandler.state, 10, vector_float_get(KalmanHandler.angular_velocity, 1));
	vector_float_set(KalmanHandler.state, 11, vector_float_get(KalmanHandler.angular_velocity, 2));
	vector_float_set(KalmanHandler.state, 12, vector_float_get(KalmanHandler.angular_velocity, 3));
}
