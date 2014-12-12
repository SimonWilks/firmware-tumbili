/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "vtol_transition_controller.h"

VtolTransitionControl::VtolTransitionControl() {
	memset(&_local_pos,0,sizeof(_local_pos));
	memset(&_local_pos_sp,0,sizeof(_local_pos_sp));
	memset(&_att,0,sizeof(_att));
	memset(&_att_sp,0,sizeof(_att_sp));

	_initialized = false;

	_position.zero();
	_position_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_R.identity();
	_euler.zero();
	_q_des.from_euler(0.0f,0.0f,0.0f);
	_thrust_des = 0;

	// initalize parameters
	_tau_p(0) = 0.40f;
	_tau_p(1) = 0.40f;
	_tau_p(2) = 0.30f;

	_zeta_p(0) = 1.10f;
	_zeta_p(1) = 1.10f;
	_zeta_p(2) = 0.80f;
}


void VtolTransitionControl::set_current_position(struct vehicle_local_position_s &pos) {
	memcpy(&_local_pos,&pos,sizeof(_local_pos));
	_position(0) = pos.y;
	_position(1) = pos.x;
	_position(2) = pos.z;
	_vel(0) = pos.vx;
	_vel(1) = pos.vy;
	_vel(2) = pos.vz;

}

void VtolTransitionControl::set_target_position(struct vehicle_local_position_setpoint_s &pos_sp) {
	memcpy(&_local_pos_sp,&pos_sp,sizeof(_local_pos_sp));
	_position_sp(0) = pos_sp.x;
	_position_sp(1) = pos_sp.y;
	_position_sp(2) = pos_sp.z;
}

void VtolTransitionControl::set_attitude(struct vehicle_attitude_s &att) {
	memcpy(&_att,&att,sizeof(_att));
	_R.set(_att.R);
	_euler = _R.to_euler();
}

void VtolTransitionControl::get_attitude_setpoint(struct vehicle_attitude_setpoint_s &att_sp) {

}

bool VtolTransitionControl::is_initialized() {
	return !_initialized;
}

void VtolTransitionControl::initialize(struct vehicle_local_position_s &pos) {
	_local_pos_sp.x = cosf(_att.yaw)*L;
	_local_pos_sp.y = sinf(_att.yaw)*L;
	_local_pos_sp.z -= H;	// z axis points down
	_local_pos_sp.yaw = _att.yaw;
	_initialized = true;
}

void VtolTransitionControl::deinitialize() {
	_initialized = false;
}

void VtolTransitionControl::get_desired_acceleration(math::Vector<3> &acc) {
	// provisional feedforward, not used yet
	math::Vector<3> acc_feedforward;
	memset(&acc_feedforward,0,sizeof(acc_feedforward));

	// compute position and velocity error
	math::Vector<3> pos_error = _position - _position_sp;
	math::Vector<3> vel_error = _R*_vel - _vel_sp;	// in body frame

	for (int i = 0;i < 3;i++) {
		acc(i) = acc_feedforward(i) - 2.0f*_zeta_p(i)/_tau_p(i) * vel_error(i)
			- 1/(_tau_p(i)*_tau_p(i))*pos_error(i);
	}

	// add gravity
	acc(2) += GRAVITY;
}

void VtolTransitionControl::get_aerodynamic_force(math::Vector<3> &f) {
	// for now pretend there are no aerodynamic forces
	memset(&f,0,sizeof(f));
}

int VtolTransitionControl::run_controller() {
	// get desired acceleration
	math::Vector<3> a_des;
	get_desired_acceleration(a_des);

	// substract estimated aerodynamic acceleration
	math::Vector<3> f_air;
	get_aerodynamic_force(f_air);
	a_des = a_des - (_R*f_air)/MASS;

	// constrain acceleration to never be negative along body z axis
	a_des = _R*a_des;
	a_des(2) = a_des(2) < 0.1f ? 0.1f : a_des(2);
	a_des = _R.transposed()*a_des;

	// compute desired yaw
	math::Vector<3> v_global = _R.transposed()*_vel;
	float yaw_des;
	if(v_global(0)*v_global(0) + v_global(1)*v_global(1) > 0.1f) { // are we moving in the ground plane?
		yaw_des = atan2f(_vel(1),_vel(0)) - M_PI_2_F;
		float yaw_diff = yaw_des - _euler(2);
		while (yaw_diff > M_PI_F) { yaw_diff = yaw_diff - 2.0f*M_PI_F;}
		while (yaw_diff < -M_PI_F) {yaw_diff = yaw_diff + 2.0f*M_PI_F;}
		if (fabsf(yaw_diff) > M_PI_2_F) {
			yaw_des = yaw_des > 0.0f ? yaw_des - M_PI_F : yaw_des + M_PI_F;
		}
	}
	else {
		yaw_des = _euler(2);	// take momentary heading
	}

	// rotate desired acceleration such that nose of vehicle is in flight direction
	math::Quaternion q_yaw;
	q_yaw.from_euler(0.0f,0.0f,yaw_des);
	math::Matrix<3,3> R_yaw = q_yaw.to_dcm();
	math::Vector<3> a_des_yaw = R_yaw*a_des;

	// compute desired rotation angle
	math::Vector<3> zb_des = a_des_yaw.normalized();
	math::Vector<3> zi = {0.0f,0.0f,1.0f};
	float inner_prod = zi*zb_des;
	inner_prod = math::constrain(inner_prod,-1.0f,1.0f);
	float alpha = acosf(inner_prod);

	math::Vector<3> rot_axis;
	// if our heading is more or less correct, only rotate around pitch to ensure we fly with roll ~= 0
	// this probably needs tuning
	if (fabsf(alpha) < 0.0001f || fabsf(alpha + M_PI_F) < 0.0001f || fabsf(alpha - M_PI_F) < 0.0001f) {
		rot_axis(0) = 0.0f;
		rot_axis(1) = 1.0f;
		rot_axis(2) = 0.0f;
	}
	else {
		rot_axis = zi%zb_des;
		rot_axis.normalize();
	}

	// compute desired quaternion
	math::Quaternion q_xy;
	q_xy(0) = cosf(0.5f*alpha);
	rot_axis*= sinf(0.5f*alpha);
	q_xy(1) = rot_axis(0);
	q_xy(2) = rot_axis(1);
	q_xy(3) = rot_axis(2);

	_q_des = q_xy*q_yaw;	// combine the two rotations

	// rotate body z-axis to global frame
	zb_des = R_yaw.transposed()*zb_des;

	// compute desired thrust (project desired to actual axis)
	math::Vector<3> zb = _R.transposed()*zi;
	_thrust_des = zb*zb_des;
	_thrust_des *= MASS*a_des.length(); // this is a force!

	return 0;
}

void VtolTransitionControl::write_att_sp(struct vehicle_attitude_setpoint_s &att_sp) {
	math::Matrix<3,3> R_sp = _q_des.to_dcm();
	math::Vector<3> euler = R_sp.to_euler();
	att_sp.R_body[0][0] = R_sp(0,0);
	att_sp.R_body[0][1] = R_sp(0,1);
	att_sp.R_body[0][2] = R_sp(0,2);
	att_sp.R_body[1][0] = R_sp(1,0);
	att_sp.R_body[1][1] = R_sp(1,1);
	att_sp.R_body[1][2] = R_sp(1,2);
	att_sp.R_body[2][0] = R_sp(2,0);
	att_sp.R_body[2][1] = R_sp(2,1);
	att_sp.R_body[2][2] = R_sp(2,2);
	att_sp.R_valid = true;

	att_sp.roll_body = euler(0);
	att_sp.pitch_body = euler(1);
	att_sp.yaw_body = euler(2);

	att_sp.thrust = math::constrain(_thrust_des*THRUST_SCALING,0.4f,0.7f);	// limit this for now
}