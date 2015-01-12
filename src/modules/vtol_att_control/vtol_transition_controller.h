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

// for now parameters are hardcoded
#define GRAVITY 9.81f
#define MASS 0.70f
#define THRUST_SCALING 0.02f
#define L 10
#define H 5

class VtolTransitionControl {
public:
	VtolTransitionControl();
	~VtolTransitionControl();

	// public member functions
	void set_current_position(struct vehicle_local_position_s &pos);
	void set_target_position(struct vehicle_local_position_setpoint_s &pos_sp);
	void set_attitude(struct vehicle_attitude_s &att);
	void get_attitude_setpoint(struct vehicle_attitude_setpoint_s &att_sp);
	bool is_initialized();
	void initialize(struct vehicle_local_position_s &pos);
	void deinitialize();
	int run_controller();
	void write_att_sp(struct vehicle_attitude_setpoint_s &att_sp);
	math::Matrix<4,4> get_quat_mult_mat(math::Quaternion &q);

private:
	// data containers
	struct vehicle_local_position_s _local_pos;
	struct vehicle_local_position_setpoint_s _local_pos_sp;
	struct vehicle_attitude_s 	_att;
	struct vehicle_attitude_setpoint_s _att_sp;

	math::Vector<3> _position;
	math::Vector<3> _position_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Matrix<3,3> _R;
	math::Vector<3> _euler;

	// main values computed
	math::Quaternion _q_des;
	float _thrust_des;

	// provisional parameters
	math::Vector<3> _tau_p;
	math::Vector<3> _zeta_p;

	bool _initialized;		// has the transition maneuver been initialized?
	
	// private member functions
	void get_desired_acceleration(math::Vector<3> &acc);
	void get_aerodynamic_force(math::Vector<3> &f);
};