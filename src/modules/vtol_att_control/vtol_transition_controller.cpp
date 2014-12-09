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

extern "C" __EXPORT int VtolTransitionControl(int argc, char *argv[]);

#define GRAVITY 9.81f

class VtolTransitionControl {
public:
	VtolTransitionControl();
	~VtolTransitionControl();
	void set_current_state(struct &vehicle_local_position_s pos);
	void set_target_position(struct &vehicle_local_position_setpoint_s pos_sp);
	void set_attitude(struct &vehicle_attitude att);
	void get_attitude_setpoint(struct &vehicle_attitude_setpoint_s att_sp);
	int run_controller();

private:

	struct vehicle_local_position_s _local_pos;
	struct vehicle_local_position_setpoint_s _local_pos_sp;
	struct vehicle_attitude_s 	_att;
	struct vehicle_attitude_setpoint_s _app_sp;

	math::Vector<3> _position;
	math::Vector<3> _position_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Matrix<3,3> _R;
	
	void get_desired_acceleration(math::Vector<3> &acc);
};

VtolTransitionControl::VtolTransitionControl() {
	memset(&_local_pos,0,sizeof(_local_pos));
	memset(&_local_pos_sp,0,sizeof(_local_pos_sp));
	memset(&_att,0,sizeof(_att));
	memset(&_att_sp,0,sizeof(_att_sp));
}


void VtolTransitionControl::set_current_state(struct &vehicle_local_position_s pos) {
	memcpy(&_local_pos,pos,sizeof(_local_pos));
}

void VtolTransitionControl::set_target_position(struct &vehicle_local_position_setpoint_s pos_sp) {
	memcpy(&_local_pos_sp,pos_sp,sizeof(_local_pos_sp));
}

void VtolTransitionControl::set_attitude(struct &vehicle_attitude att) {
	memcpy(&_att,att,sizeof(_att));
}

void VtolTransitionControl::get_attitude_setpoint(struct &vehicle_attitude_setpoint_s att_sp) {

}

void VtolTransitionControl::get_desired_acceleration(math::Vector<3> &acc) {
	// provisional feedforward, not used yet
	math::Vector<3> acc_feedforward;
	memset(&acc_feedforward,0,sizeof(acc_feedforward));

	// compute position and velocity error
	math::Vector<3> pos_error = _position - _position_sp;
	math::Vector<3> vel_error = _vel - _vel_sp; // check in which frame _vel is, may need to rotate !!!!!!!!!!!!!!!!!!!!!!!!!!!!

	for (int i = 0;i < 3;i++) {
		acc(i) = acc_feedforward(i) - 2.0f*_params.zeta_p(i)/_params.tau_p(i) * vel_error(i)
			- 1/(_params.tau_p(i)*_params.tau_p(i))*pos_error(i);
	}

	// add gravity
	acc(2) += GRAVITY;
}

int VtolTransitionControl::run_controller() {
	// get desired acceleration
	math::Vector<3> a_des;
	get_desired_acceleration(a_des);

	// compute desired body z azis
	math::Vector<3> zb_des(a_des);
	zb_des.normalize();

	// substract estimated aerodynamic acceleration
	math::Vector<3> f_air;
	get_aerodynamic_force(f_air);
	a_des = a_des - (_R*f_air)/_params.m;

	// constrain acceleration to never be negative along body z axis
	a_des = _R*a_des;
	a_des(2) = a_des(2) < 0.1 ? 0.1 : a_des(2);
	a_des = R.transpose()*a_des;

	// compute desired yaw
	Vector<3> v_global = _R.transpose()*_vel;

	if(v_global(0)*v_global(0) + v_global(1)*v_global(1) > 0.1) {
		float yaw_des = atan2(_vel(1),_vel(0)) - M_PI_2;
		float yaw_curr = 
	}





}