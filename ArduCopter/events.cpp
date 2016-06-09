// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 *       This event will be called when the failsafe changes
 *       boolean failsafe reflects the current state
 */
void Copter::failsafe_radio_on_event()
{
    // if motors are not armed there is nothing to do
    if( !motors.armed() ) {
        return;
    }

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else {
        if ((control_mode == AUTO || (control_mode == GUIDED && !failsafe.guided_sp_expired)) && g.failsafe_throttle == FS_THR_ENABLED_CONTINUE_MISSION) {
            // continue mission or guided mode
        } else if (control_mode == LAND && g.failsafe_battery_enabled == FS_BATT_LAND && failsafe.battery) {
            // continue landing
        } else {
            if (g.failsafe_throttle == FS_THR_ENABLED_ALWAYS_LAND) {
                set_mode(LAND, MODE_REASON_RADIO_FAILSAFE);
            } else {
                if (!set_mode(RTL, MODE_REASON_RADIO_FAILSAFE)) {
                    set_mode(LAND, MODE_REASON_RADIO_FAILSAFE);
                }
            }
        }
    }

    // log the error to the dataflash
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_off_event - respond to radio contact being regained
// we must be in AUTO, LAND or RTL modes
// or Stabilize or ACRO mode but with motors disarmed
void Copter::failsafe_radio_off_event()
{
    // no need to do anything except log the error as resolved
    // user can now override roll, pitch, yaw and throttle and even use flight mode switch to restore previous flight mode
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_RADIO, ERROR_CODE_FAILSAFE_RESOLVED);
}

void Copter::failsafe_battery_event(void)
{
    // return immediately if low battery event has already been triggered
    if (failsafe.battery) {
        return;
    }

    // failsafe check
    if (g.failsafe_battery_enabled != FS_BATT_DISABLED && motors.armed()) {
        if (should_disarm_on_failsafe()) {
            init_disarm_motors();
        } else {
            if (control_mode != LAND) {
                if (g.failsafe_battery_enabled == FS_BATT_RTL || control_mode == AUTO) {
                    if (!set_mode(RTL, MODE_REASON_BATTERY_FAILSAFE)) {
                        set_mode(LAND, MODE_REASON_BATTERY_FAILSAFE);
                    }
                } else {
                    set_mode(LAND, MODE_REASON_BATTERY_FAILSAFE);
                }
            }
        }
    }

    // set the low battery flag
    set_failsafe_battery(true);

    // warn the ground station and log to dataflash
    gcs_send_text(MAV_SEVERITY_WARNING,"Low battery");
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_BATT, ERROR_CODE_FAILSAFE_OCCURRED);

}

// failsafe_gcs_check - check for ground station failsafe
void Copter::failsafe_gcs_check()
{
    uint32_t last_gcs_update_ms;

    // return immediately if gcs failsafe is disabled, gcs has never been connected or we are not overriding rc controls from the gcs and we are not in guided mode
    // this also checks to see if we have a GCS failsafe active, if we do, then must continue to process the logic for recovery from this state.
    if ((!failsafe.gcs)&&(g.failsafe_gcs == FS_GCS_DISABLED || failsafe.last_heartbeat_ms == 0 || (!failsafe.rc_override_active && control_mode != GUIDED))) {
        return;
    }

    // calc time since last gcs update
    // note: this only looks at the heartbeat from the device id set by g.sysid_my_gcs
    last_gcs_update_ms = millis() - failsafe.last_heartbeat_ms;

    // check if all is well
    if (last_gcs_update_ms < FS_GCS_TIMEOUT_MS) {
        // check for recovery from gcs failsafe
        if (failsafe.gcs) {
            failsafe_gcs_off_event();
            set_failsafe_gcs(false);
        }
        return;
    }

    // do nothing if gcs failsafe already triggered or motors disarmed
    if (failsafe.gcs || !motors.armed()) {
        return;
    }

    // GCS failsafe event has occured
    // update state, log to dataflash
    set_failsafe_gcs(true);
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_OCCURRED);

    // clear overrides so that RC control can be regained with radio.
    hal.rcin->clear_overrides();
    failsafe.rc_override_active = false;

    if (should_disarm_on_failsafe()) {
        init_disarm_motors();
    } else {
        if (control_mode == AUTO && g.failsafe_gcs == FS_GCS_ENABLED_CONTINUE_MISSION) {
            // continue mission
        } else if (g.failsafe_gcs != FS_GCS_DISABLED) {
            if (!set_mode(RTL, MODE_REASON_GCS_FAILSAFE)) {
                set_mode(LAND, MODE_REASON_GCS_FAILSAFE);
            }
        }
    }
}

// failsafe_gcs_off_event - actions to take when GCS contact is restored
void Copter::failsafe_gcs_off_event(void)
{
    // log recovery of GCS in logs?
    Log_Write_Error(ERROR_SUBSYSTEM_FAILSAFE_GCS, ERROR_CODE_FAILSAFE_RESOLVED);
}

void Copter::failsafe_gps_glitch_check() {
    bool glitch = ahrs.getGpsGlitchStatus();

    if (glitch && !failsafe.gps_glitch) {
        failsafe.gps_glitch = true;

        bool in_action_mode = false;
        switch(control_mode) {
            case LAND:
                in_action_mode = true;
                break;
            case RTL:
                in_action_mode = (rtl_state == RTL_Land);
                break;
            case GUIDED: // GUIDED for solo because shots modes use GUIDED
            case LOITER:
            case DRIFT:
            case BRAKE:
            case POSHOLD:
                in_action_mode = true;
                break;
            default:
                break;
        }

        if (motors.armed() && in_action_mode) {
            if (should_disarm_on_failsafe()) {
                init_disarm_motors();
            } else if (!failsafe.radio) {
                set_mode(ALT_HOLD, MODE_REASON_GPS_GLITCH);
            }
        }
    } else if (!glitch && position_ok() && failsafe.gps_glitch) {
        failsafe.gps_glitch = false;

        if (failsafe.radio) {
            failsafe_radio_on_event();
        } else if (control_mode_reason == MODE_REASON_GPS_GLITCH) {
            set_mode(LOITER, MODE_REASON_GPS_GLITCH_RECOVERED);
        }
    }
}

bool Copter::should_disarm_on_failsafe() {
    if (ap.in_arming_delay) {
        return true;
    }

    switch(control_mode) {
        case STABILIZE:
        case ACRO:
            // if throttle is zero OR vehicle is landed disarm motors
            return ap.throttle_zero || ap.land_complete;
            break;
        case AUTO:
            // if mission has not started AND vehicle is landed, disarm motors
            return !ap.auto_armed && ap.land_complete;
            break;
        default:
            // used for AltHold, Guided, Loiter, RTL, Circle, Drift, Sport, Flip, Autotune, PosHold
            // if landed disarm
            return ap.land_complete;
            break;
    }
}

void Copter::update_events()
{
    ServoRelayEvents.update_events();
}

