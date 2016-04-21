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

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>

#include <stdio.h>

using namespace SITL;

MultiCopter::MultiCopter(const char *home_str, const char *frame_str) :
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
}

// calculate rotational and linear accelerations
<<<<<<< HEAD
void Frame::calculate_forces(const Aircraft &aircraft,
                             const Aircraft::sitl_input &input,
                             Vector3f &rot_accel,
                             Vector3f &body_accel)
{
    // rotational acceleration, in rad/s/s, in body frame
    float thrust = 0.0f; // newtons

    for (uint8_t i=0; i<num_motors; i++) {
        float motor_speed = constrain_float((input.servos[motor_offset+motors[i].servo]-1000)/1000.0, 0, 1);
        rot_accel.x  += -radians(5000.0) * sinf(radians(motors[i].angle)) * motor_speed;
        rot_accel.y  +=  radians(5000.0) * cosf(radians(motors[i].angle)) * motor_speed;
        rot_accel.z += motors[i].yaw_factor * motor_speed * radians(400.0);
        thrust += motor_speed * thrust_scale; // newtons
    }

    body_accel = Vector3f(0, 0, -thrust / mass);

    if (terminal_rotation_rate > 0) {
        // rotational air resistance
        const Vector3f &gyro = aircraft.get_gyro();
        rot_accel.x -= gyro.x * radians(400.0) / terminal_rotation_rate;
        rot_accel.y -= gyro.y * radians(400.0) / terminal_rotation_rate;
        rot_accel.z -= gyro.z * radians(400.0) / terminal_rotation_rate;
    }

    if (terminal_velocity > 0) {
        // air resistance
        Vector3f air_resistance = -aircraft.get_velocity_ef() * (GRAVITY_MSS/terminal_velocity);
        body_accel += aircraft.get_dcm().transposed() * air_resistance;
    }

    // add some noise
    const float gyro_noise = radians(0.1);
    const float accel_noise = 0.3;
    const float noise_scale = thrust / (thrust_scale * num_motors);
    rot_accel += Vector3f(aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1),
                          aircraft.rand_normal(0, 1)) * gyro_noise * noise_scale;
    body_accel += Vector3f(aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1),
                           aircraft.rand_normal(0, 1)) * accel_noise * noise_scale;
}

// calculate rotational and linear accelerations
=======
>>>>>>> c262d6a... SITL: break up multicopter into Motor/Frame/Multicopter classes
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel);
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    // how much time has passed?
    Vector3f rot_accel;

    calculate_forces(input, rot_accel, accel_body);

    update_dynamics(rot_accel);

    if (on_ground(position)) {
        // zero roll/pitch, but keep yaw
        float r, p, y;
        dcm.to_euler(&r, &p, &y);
        dcm.from_euler(0, 0, y);

        position.z = -(ground_level + frame_height - home.alt*0.01f);
    }
    
    // update lat/lon/altitude
    update_position();
}


