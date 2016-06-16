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

namespace SITL {

Solo::Solo(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(NULL)
{
    frame = Frame::find_frame(frame_str);
    if (frame == NULL) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }
    frame->init(1.5, 0.51, 15, 4*radians(360));
    frame_height = 0.1;

    VehicleBuild_initialize();
}

/*
  update the Solo simulation by one time step (1/1000 sec) with high-fidelity dynamics
 */
void Solo::update(const struct sitl_input &input)
{
    VehicleBuild_U.VoltageCommands.M1 = constrain_float((input.servos[frame->motor_offset+frame->motors[0].servo]-1000)/1000.0, 0, 1);
    VehicleBuild_U.VoltageCommands.M2 = constrain_float((input.servos[frame->motor_offset+frame->motors[1].servo]-1000)/1000.0, 0, 1);
    VehicleBuild_U.VoltageCommands.M3 = constrain_float((input.servos[frame->motor_offset+frame->motors[2].servo]-1000)/1000.0, 0, 1);
    VehicleBuild_U.VoltageCommands.M4 = constrain_float((input.servos[frame->motor_offset+frame->motors[3].servo]-1000)/1000.0, 0, 1);

    VehicleBuild_step();

    // assign gyro
    gyro[0] = VehicleBuild_Y.SensorData_o.omega_body[0];
    gyro[1] = VehicleBuild_Y.SensorData_o.omega_body[1];
    gyro[2] = VehicleBuild_Y.SensorData_o.omega_body[2];

    // assign dcm
    Quaternion(VehicleBuild_Y.State.World.Q[0],VehicleBuild_Y.State.World.Q[1],VehicleBuild_Y.State.World.Q[2],VehicleBuild_Y.State.World.Q[3]).rotation_matrix(dcm);


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

}