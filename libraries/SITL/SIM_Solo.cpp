/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  multicopter simulator class
*/

#include "SIM_Solo.h"
#include "VehicleBuild.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

static const Motor quad_plus_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_4, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
};

static const Motor quad_x_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
};

static const Motor hexa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2, 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_3,-120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_4,  60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_5, -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_6, 120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static const Motor hexax_motors[] =
{
    Motor(AP_MOTORS_MOT_1,  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_2, -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_3, -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6),
    Motor(AP_MOTORS_MOT_4, 150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3),
    Motor(AP_MOTORS_MOT_5,  30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_6,-150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4)
};

static const Motor octa_motors[] =
{
    Motor(AP_MOTORS_MOT_1,    0,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1),
    Motor(AP_MOTORS_MOT_2,  180,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5),
    Motor(AP_MOTORS_MOT_3,   45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2),
    Motor(AP_MOTORS_MOT_4,  135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_5,  -45,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6, -135,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6),
    Motor(AP_MOTORS_MOT_7,  -90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_8,   90,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3)
};

static const Motor octa_quad_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1),
    Motor(AP_MOTORS_MOT_2,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7),
    Motor(AP_MOTORS_MOT_3, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3),
    Motor(AP_MOTORS_MOT_5,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8),
    Motor(AP_MOTORS_MOT_6,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2),
    Motor(AP_MOTORS_MOT_7,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4),
    Motor(AP_MOTORS_MOT_8, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6)
};

/*
  table of supported frame types
 */
static Frame supported_frames[] =
{
    Frame("+",         4, quad_plus_motors),
    Frame("quad",      4, quad_plus_motors),
    Frame("copter",    4, quad_plus_motors),
    Frame("x",         4, quad_x_motors),
    Frame("hexa",      6, hexa_motors),
    Frame("hexax",     6, hexax_motors),
    Frame("octa",      8, octa_motors),
    Frame("octa-quad", 8, octa_quad_motors)
};

void Frame::init(float _mass, float hover_throttle, float _terminal_velocity, float _terminal_rotation_rate)
{
    mass = _mass;

    /*
       scaling from total motor power to Newtons. Allows the copter
       to hover against gravity when each motor is at hover_throttle
    */
    thrust_scale = (mass * GRAVITY_MSS) / (num_motors * hover_throttle);

    terminal_velocity = _terminal_velocity;
    terminal_rotation_rate = _terminal_rotation_rate;
}

/*
  find a frame by name
 */
Frame *Frame::find_frame(const char *name)
{
    for (uint8_t i=0; i < ARRAY_SIZE(supported_frames); i++) {
        if (strcasecmp(name, supported_frames[i].name) == 0) {
            return &supported_frames[i];
        }
    }
    return NULL;
}

Solo::Solo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(NULL)
{
    frame = Frame::find_frame(frame_str);
    if (frame == NULL) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }
    frame->init(1.53, 0.51, 15, 4*radians(360));
    frame_height = 0.1;
}

/*
  update the Solo simulation by one time step (1/1000 sec) with high-fidelity dynamics
 */
void Solo::update(const struct sitl_input &input)
{
    VehicleBuild_U.VoltageCommands.M1 = constrain_float((input.servos[motor_offset+motors[0].servo]-1000)/1000.0, 0, 1);
    VehicleBuild_U.VoltageCommands.M2 = constrain_float((input.servos[motor_offset+motors[1].servo]-1000)/1000.0, 0, 1);
    VehicleBuild_U.VoltageCommands.M3 = constrain_float((input.servos[motor_offset+motors[2].servo]-1000)/1000.0, 0, 1);
    VehicleBuild_U.VoltageCommands.M4 = constrain_float((input.servos[motor_offset+motors[3].servo]-1000)/1000.0, 0, 1);

    VehicleBuild_step();

    // assign gyro
    gyro[0] = VehicleBuild_Y.SensorData_o.omega_body[0];
    gyro[1] = VehicleBuild_Y.SensorData_o.omega_body[1];
    gyro[2] = VehicleBuild_Y.SensorData_o.omega_body[2];

    // assign dcm
    Quaternion(VehicleBuild_Y.State.World.Q[0],VehicleBuild_Y.State.World.Q[1],VehicleBuild_Y.State.World.Q[2],VehicleBuild_Y.State.World.Q[3]).rotation_matrix(&dcm);

    // assign accel_earth
    accel_earth[0] = VehicleBuild_Y.State.World.ax;
    accel_earth[1] = VehicleBuild_Y.State.World.ay;
    accel_earth[2] = VehicleBuild_Y.State.World.az;

    // assign accel_body
    accel_body[0] = VehicleBuild_Y.SensorData_o.accel_body[0];
    accel_body[1] = VehicleBuild_Y.SensorData_o.accel_body[1];
    accel_body[2] = VehicleBuild_Y.SensorData_o.accel_body[2];
    
    // assign earth_frame velocity
    velocity_ef[0] = VehicleBuild_Y.State.World.vx;
    velocity_ef[1] = VehicleBuild_Y.State.World.vy;
    velocity_ef[2] = VehicleBuild_Y.State.World.vz;
    
    // assign position
    position[0] = VehicleBuild_Y.State.World.x;
    position[1] = VehicleBuild_Y.State.World.y;
    position[2] = VehicleBuild_Y.State.World.z;
    
    // update lat/lon/altitude
    update_position();
}


