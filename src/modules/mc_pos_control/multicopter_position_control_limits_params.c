/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Maximum horizontal velocity
 *
 * Absolute maximum for all velocity controlled modes.
 * Any higher value is truncated.
 *
 * @unit m/s
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_XY_VEL_MAX, 10.f);

/**
 * Maximum ascent velocity
 *
 * Absolute maximum for all climb rate controlled modes.
 * In manually piloted modes full stick deflection commands this velocity.
 *
 * For default autonomous velocity see MPC_Z_V_AUTO_UP
 *
 * @unit m/s
 * @min 0.1
 * @max 8
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_UP, 0.7f);

/**
 * Maximum horizontal acceleration
 *
 * Absolute maximum for all acceleration controlled modes.
 * Any higher value is truncated.
 *
 * @unit m/s
 * @min 0
 * @max 100.0
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
 PARAM_DEFINE_FLOAT(MPC_XY_ACC_MAX, 5.f);

 /**
  * Maximum ascent acceleration
  *
  * Absolute maximum for all climb rate dot controlled modes.
  * In manually piloted modes full stick deflection commands this acceleration.
  *
  * For default autonomous acceleration see MPC_Z_V_AUTO_UP
  *
  * @unit m/s
  * @min 0.5
  * @max 100.0
  * @increment 0.1
  * @decimal 1
  * @group Multicopter Position Control
  */
 PARAM_DEFINE_FLOAT(MPC_Z_ACC_MAX_UP, 80.f);

/**
 * Maximum descent velocity
 *
 * Absolute maximum for all climb rate controlled modes.
 * In manually piloted modes full stick deflection commands this velocity.
 *
 * For default autonomous velocity see MPC_Z_V_AUTO_UP
 *
 * @unit m/s
 * @min 0.1
 * @max 100.0
 * @increment 0.1
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_Z_VEL_MAX_DN, 0.1f);

/**
 * Maximum tilt angle in air
 *
 * Absolute maximum for all velocity or acceleration controlled modes.
 * Any higher value is truncated.
 *
 * @unit deg
 * @min 20
 * @max 89
 * @decimal 0
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_AIR, 45.f);

/**
 * Maximum tilt during inital takeoff ramp
 *
 * Tighter tilt limit during takeoff to avoid tip over.
 *
 * @unit deg
 * @min 5
 * @max 89
 * @decimal 0
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_TILTMAX_LND, 12.f);

/**
 * Minimum collective thrust in climb rate controlled modes
 *
 * Too low thrust leads to loss of roll/pitch/yaw torque control authority.
 * With airmode enabled this parameters can be set to 0
 * while still keeping torque authority (see MC_AIRMODE).
 *
 * @unit norm
 * @min 0.05
 * @max 15.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MIN, 1.0f); //0.12f

/**
 * Maximum collective thrust in climb rate controlled modes
 *
 * Limit allowed thrust e.g. for indoor test of overpowered vehicle.
 *
 * @unit norm
 * @min 0
 * @max 50.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_THR_MAX, 50.f); //1.f

/**
 * Acceleration to tilt coupling
 *
 * Set to decouple tilt from vertical acceleration.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_ACC_DECOUPLE, 1);
