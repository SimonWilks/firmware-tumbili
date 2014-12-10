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

/**
 * Vtol transition controller
 *
 * @author Roman Bapst 		<bapstr@ethz.ch>
 * 
 */

 #include <uORB/topics/vehicle_local_position.h>
 #include <uORB/topics/vehicle_local_position_setpoint.h>
 #include <uORB/topics/vehicle_attitude.h>
 #include <uORB/topics/vehicle_attitude_setpoint.h>
 #include <lib/mathlib/mathlib.h>
 #include <math.h>

extern "C" __EXPORT int VtolTransitionControl(int argc, char *argv[]);

// for now parameters are hardcoded
#define GRAVITY 9.81f

class VtolTransitionControl {
public:
	VtolTransitionControl();
	~VtolTransitionControl();

	// public member functions
	void set_current_state(struct &vehicle_local_position_s pos);
	void set_target_position(struct &vehicle_local_position_setpoint_s pos_sp);
	void set_attitude(struct &vehicle_attitude att);
	void get_attitude_setpoint(struct &vehicle_attitude_setpoint_s att_sp);
	int run_controller();

private:
	// data containers
	struct vehicle_local_position_s _local_pos;
	struct vehicle_local_position_setpoint_s _local_pos_sp;
	struct vehicle_attitude_s 	_att;
	struct vehicle_attitude_setpoint_s _app_sp;

	math::Vector<3> _position;
	math::Vector<3> _position_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Matrix<3,3> _R;
	math::Vector<3> _euler;

	// main values computed
	math::Vector<3> _q_des;
	float _thrust_des;

	// provisional parameters
	math::Vector<3> _tau_p;
	math::Vector<3> _zeta_p;
	
	// private member functions
	void get_desired_acceleration(math::Vector<3> &acc);
	void get_aerodynamic_force(math::Vector<3> &f);
	void run_controller();

};

VtolTransitionControl::VtolTransitionControl() {
	memset(&_local_pos,0,sizeof(_local_pos));
	memset(&_local_pos_sp,0,sizeof(_local_pos_sp));
	memset(&_att,0,sizeof(_att));
	memset(&_att_sp,0,sizeof(_att_sp));

	// initalize parameters
	_tau_p(0) = 0.40f;
	_tau_p(1) = 0.40f;
	_tau_p(2) = 0.30f;

	_zeta_p(0) = 1.10f;
	_zeta_p(1) = 1.10f;
	_zeta_p(2) = 0.80f;
}


void VtolTransitionControl::set_current_state(struct &vehicle_local_position_s pos) {
	memcpy(&_local_pos,pos,sizeof(_local_pos));
	_position(0) = pos.y;
	_position(1) = pos.x;
	_position(2) = -pos.z;
	_vel(0) = pos.vx;
	_vel(1) = pos.vy;
	_vel(2) = pos.vz;

}

void VtolTransitionControl::set_target_position(struct &vehicle_local_position_setpoint_s pos_sp) {
	memcpy(&_local_pos_sp,pos_sp,sizeof(_local_pos_sp));
	_position_sp(0) = pos_sp(0);
	_position_sp(1) = pos_sp(1);
	_position_sp(2) = pos_sp(2);
}

void VtolTransitionControl::set_attitude(struct &vehicle_attitude att) {
	memcpy(&_att,att,sizeof(_att));
	_R.set(&_att.R);
	_euler = _R.to_euler();
}

void VtolTransitionControl::get_attitude_setpoint(struct &vehicle_attitude_setpoint_s att_sp) {

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

	// compute desired body z azis
	math::Vector<3> zb_des = a_des;
	zb_des.normalize();

	// substract estimated aerodynamic acceleration
	math::Vector<3> f_air;
	get_aerodynamic_force(f_air);
	a_des = a_des - (_R*f_air)/_params.m;

	// constrain acceleration to never be negative along body z axis
	a_des = _R*a_des;
	a_des(2) = a_des(2) < 0.1f ? 0.1f : a_des(2);
	a_des = R.transpose()*a_des;

	// compute desired yaw
	Vector<3> v_global = _R.transpose()*_vel;

	if(v_global(0)*v_global(0) + v_global(1)*v_global(1) > 0.1) { // are we moving in the ground plane?
		float yaw_des = atan2f(_vel(1),_vel(0)) - M_PI_2;
		float yaw_diff = yaw_des - _euler(2);
		while (yaw_diff > M_PI) { yaw_diff = yaw_diff - 2.0f*M_PI;}
		while (yaw_diff < -M_PI) {yaw_diff = yaw_diff + 2.0f*pi;}
		if (fabsf(yaw_diff) > M_PI_2) {
			yaw_des = yaw_des > 0.0f ? yaw_des - M_PI : yaw_des + pi;
		}
	}
	else {
		yaw_des = _euler(2);	// take momentary heading
	}

	// rotate desired acceleration such that nose of vehicle is in flight direction
	math::Quaternion q_yaw;
	q_yaw.from_euler(0.0f,0.0f,yaw_des);
	math::Matrix<3,3> R_yaw = q_yaw.to_dcm();
	float a_des_yaw = R_yaw*a_des;

	// compute desired rotation angle
	math::Vector<3> zb_des = a_des_yaw.normalized();
	math::Vector<3> zi = {0.0f,0.0f,1.0f};
	float inner_prod = zi*zb_des;
	inner_prod = math::constrain(inner_prod,-1.0f,1.0f);
	float alpha = math::acosf(inner_prod);

	math::Vector<3> rot_axis;
	// if our heading is more or less correct, only rotate around pitch to ensure we fly with roll ~= 0
	// this probably needs tuning
	if (fabsf(alpha) < 0.0001 || fabsf(alpha + M_PI) < 0.0001 || fabsf(alpha - M_PI) < 0.0001) {
		rot_axis.set({0.0f,1.0f,0.0f});
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
	zb_des = R_yaw.transpose()*zb_des;

	// compute desired thrust (project desired to actual axis)
	math::Vector<3> zb = _R.transpose()*zi;
	_thrust_des = zb*zb_des;
	_thrust_des *= _params.m*a_des.length(); // this is a force!
}