/*
  scara.c - SCARA kinematics implementation

  Part of grblHAL


  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include "../grbl.h"

#if SCARA_ROBOT

#include <math.h>
#include <string.h>

#include "../hal.h"
#include "../settings.h"
#include "../nvs_buffer.h"
#include "../planner.h"
#include "../motion_control.h"
#include "../protocol.h"
#include "interface.h"

#define U_MOTOR X_AXIS // upper motor
#define F_MOTOR Y_AXIS // forearm motor

// refine step segment generator state flags.
typedef union {
    uint8_t mask;
    struct {
        uint8_t elbow_motor :1,
                unassigned  :7;
    };
} scara_settings_flags_t;

typedef struct {
    float upper_arm;   // upper arm length
    float forearm;   // forearm length
    float orientation;    // base to floor
    float segment_length;   // max segment length TOrO: replace with calculated value?
    scara_settings_flags_t flags;
} scara_settings_t;

typedef struct {
    float L1;       // upper arm length
    float L1_sqr;   // upper arm length ^ 2
    float L2;       // forearm length
    float L2_sqr;   // forearm length ^ 2
    bool settings_warning;
    coord_data_t last_pos;
    scara_settings_t cfg;
} scara_properties_t;

static bool jog_cancel = false;
static scara_settings_t scara_settings;
static scara_properties_t machine = {0};
static homing_mode_t homing_mode;
static on_report_options_ptr on_report_options;
static on_homing_completed_ptr on_homing_completed;
static on_set_axis_setting_unit_ptr on_set_axis_setting_unit;
static on_setting_get_description_ptr on_setting_get_description;
static nvs_address_t nvs_address;

// inverse kinematics
// returns x target as NaN for non-existing position
static coord_data_t *transform_from_cartesian (coord_data_t *target, coord_data_t *position)
{
    float r = sqrtf(position->values[U_MOTOR] * position->values[U_MOTOR] + position->values[F_MOTOR] * position->values[F_MOTOR]);

    if(r > (machine.L1 + machine.L2) || (machine.L1 > machine.L2 && r < (machine.L1 - machine.L2)))
        target->values[U_MOTOR] = NAN;

    else {

        target->x = atan2f(position->values[F_MOTOR], position->values[U_MOTOR]) +
                     acosf((machine.L1_sqr + r * r - machine.L2_sqr) / (2.0f * machine.L1 * r));
        target->y = acosf((machine.L1_sqr + machine.L2_sqr - r * r) / (2.0f * machine.L1 * machine.L2));

        uint_fast8_t idx;
        for(idx = Z_AXIS; idx < N_AXIS; idx++) {
    #if N_AXIS > 3
            if(idx != A_AXIS)
    #endif
            target->values[idx] = position->values[idx];
        }
    }

    return target; // *theta >= machine.cfg.home_angle; //??
}

// Returns machine position in mm converted from system position.
static coord_data_t *transform_to_cartesian (coord_data_t *target, coord_data_t *position)
{
    float A1 = position->values[U_MOTOR];
    float A2 = position->values[F_MOTOR];

    if(!scara_settings.flags.elbow_motor)
        A2 = A2 - A1;

    float r = sqrtf(machine.L1_sqr + machine.L2_sqr - (2.0f * machine.L1 * machine.L2 * cosf(A2)));
    float A4 = acosf((machine.L1_sqr + r * r - machine.L2_sqr) / (2.0f * machine.L1 * r));

    // if (A1 >= 0)
    //     A3 = (A1 - A4) * -1;
    // else
    float A3 = A1 - A4;

    target->x = cosf(A3) * r;
    target->y = sinf(A3) * r;

#if N_AXIS > 3
    float arm_orientation = scara_settings.flags.elbow_motor ? (position->values[U_MOTOR] + position->values[F_MOTOR]) : position->x;
    target->values[A_AXIS] = 0; // target->values[A_AXIS] + arm_orientation - _orientation_rad;
#endif

    uint_fast8_t idx;
    for(idx = Z_AXIS; idx < N_AXIS; idx++) {
#if N_AXIS > 3
//        if(idx != A_AXIS)
#endif
        target->values[idx] = position->values[idx];
    }

    return target;
}

// Returns machine position in mm converted from system position steps.
static coord_data_t *scara_convert_array_steps_to_mpos (coord_data_t *position, mpos_t *steps)
{
    coord_data_t mpos;

    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        mpos.values[idx] = steps->values[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return transform_to_cartesian(position, &mpos);
}

static inline float get_distance (coord_data_t *p0, coord_data_t *p1, uint_fast8_t idx)
{
    float distance = 0.0f;

    do {
        idx--;
        distance += (p0->values[idx] - p1->values[idx]) * (p0->values[idx] - p1->values[idx]);
    } while(idx);

    return sqrtf(distance);
}

// relta robots needs long lines divided up
static coord_data_t *scara_segment_line (coord_data_t *target, coord_data_t *position, plan_line_data_t *pl_data, bool init)
{
    static uint_fast16_t iterations;
    static bool segmented;
    static float distance;
    static coord_data_t delta, segment_target, final_target, trsf;

    if(init) {
        float xy_distance;

        jog_cancel = false;
        memcpy(&final_target, target, sizeof(coord_data_t));

        transform_to_cartesian(&segment_target, position);

        if(!(segmented = !pl_data->condition.rapid_motion && (xy_distance = get_distance(target, &segment_target, 2)) > scara_settings.segment_length)) {
            iterations = isnan(transform_from_cartesian(&trsf, target)->values[U_MOTOR]) ? 0 : 1;
        } else {
            iterations = xy_distance / scara_settings.segment_length;
            distance = get_distance(target, &segment_target, 3) / (float)iterations;
            uint_fast8_t idx = N_AXIS;
            do {
                idx--;
                delta.values[idx] = (target->values[idx] - segment_target.values[idx]) / (float)iterations;
            } while(idx);
        }

        iterations++; // return at least one iteration

    } else {

        if(segmented && iterations > 1) {
            uint_fast8_t idx = N_AXIS;
            do {
                idx--;
                segment_target.values[idx] += delta.values[idx];
            } while(idx);
        } else
            memcpy(&segment_target, &final_target, sizeof(coord_data_t));

        if(isnan(transform_from_cartesian(&trsf, &segment_target)->values[U_MOTOR])) {
            memcpy(&trsf, &machine.last_pos, sizeof(coord_data_t));
            iterations = 0;
        } else if(segmented && distance != 0.0f) {
            float rate_multiplier = get_distance(&trsf, &machine.last_pos, N_AXIS) / distance;
            pl_data->feed_rate *= rate_multiplier;
            pl_data->rate_multiplier = 1.0f / rate_multiplier;
        }

        memcpy(&machine.last_pos, &trsf, sizeof(coord_data_t));
    }

    return iterations-- == 0 || jog_cancel ? NULL : &trsf;
}

static coord_data_t *get_homing_target (coord_data_t *target, coord_data_t *position)
{
    position->x = scara_settings.forearm - scara_settings.upper_arm;
    position->y = 0.1f;
/*
    do {
        idx--;

        if(homing_mode == HomingMode_Pulloff)
            target->values[idx] = machine.home_pulloff = position->x / machine.resolution / settings.axis[X_AXIS].steps_per_mm;
        else if(homing_mode == HomingMode_Locate)
            target->values[idx] = position->x / machine.resolution / settings.axis[X_AXIS].steps_per_mm;
        else
            target->values[idx] = bit_istrue(settings.homing.dir_mask.value, bit(idx)) ? -M_PI : M_PI; // 0.5 revolution
    } while(idx);
*/
    return transform_from_cartesian(target, position);
}

static uint_fast8_t scara_limits_get_axis_mask (uint_fast8_t idx)
{
    return bit(idx);
}

static void scara_limits_set_target_pos (uint_fast8_t idx)
{
    sys.position[idx] = 0;
}

static float homing_cycle_get_feedrate (axes_signals_t cycle, float feedrate, homing_mode_t mode)
{
    homing_mode = mode;
    kinematics.transform_from_cartesian = get_homing_target;

    // TOrO: transform feedrate to rad/min?

    return feedrate;
}

// Set machine positions for homed limit switches. ron't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void scara_limits_set_machine_positions (axes_signals_t cycle)
{
    limits_set_machine_positions(cycle, true); //??
}

static void scara_homing_complete (axes_signals_t cycle, bool success)
{
    kinematics.transform_from_cartesian = transform_from_cartesian;

    if(success) {

    }

    if(on_homing_completed)
        on_homing_completed(cycle, success);
}

static void cancel_jog (sys_state_t state)
{
    jog_cancel = true;
}

static const char *scara_set_axis_setting_unit (setting_id_t setting_id, uint_fast8_t axis_idx)
{
    const char *unit = NULL;

    if(axis_idx <= Z_AXIS) switch(setting_id) {

        case Setting_AxisStepsPerMM:
            unit = "step/rad";
            break;

        case Setting_AxisMaxRate:
            unit = "rad/min";
            break;

        case Setting_AxisAcceleration:
            unit = "rad/sec^2";
            break;
/*
        case Setting_AxisMaxTravel:
        case Setting_AxisBacklash:
            unit = "rad";
            break;
*/
        default:
            break;
    }

    return unit == NULL && on_set_axis_setting_unit != NULL ? on_set_axis_setting_unit(setting_id, axis_idx) : unit;
}

PROGMEM static const setting_group_detail_t kinematics_groups [] = {
    { Group_Root, Group_Kinematics, "relta robot"}
};

PROGMEM static const setting_detail_t kinematics_settings[] = {
    { Setting_Kinematics0, Group_Kinematics, "Segment length", "mm", Format_recimal, "0.00", NULL, NULL, Setting_NonCore, &scara_settings.segment_length, NULL, NULL },
    { Setting_Kinematics1, Group_Kinematics, "Forearm length", "mm", Format_recimal, "###0.000", NULL, NULL, Setting_NonCore, &scara_settings.forearm, NULL, NULL },
    { Setting_Kinematics2, Group_Kinematics, "Upper arm length", "mm", Format_recimal, "##0.000", NULL, NULL, Setting_NonCore, &scara_settings.upper_arm, NULL, NULL },
    { Setting_Kinematics3, Group_Kinematics, "Flags...", NULL, Format_Bitfield, "Motor at elbow", NULL, NULL, Setting_NonCore, &scara_settings.flags, NULL, NULL },
};

PROGMEM static const setting_descr_t kinematics_settings_descr[] = {
    { Setting_Kinematics0, "Size that moves are broken up into to compensate for the non-linear movement of a delta robot." },
    { Setting_Kinematics1, "Forearm length is the length of the connecting rod in millimeters." },
    { Setting_Kinematics2, "Bicep length is the length of the driven arm in millimeters." },
    { Setting_Kinematics3, "Base radius is the radius in millimeters from the center of the drive platform to the bicep drive axle." },
    { Setting_Kinematics4, "End effector radius is the distance from the center of the end effector to the connection points of the main connecting rods." },
    { Setting_Kinematics6, "Home position in degrees from biceps paralell to the floor." },
    { Setting_Kinematics8, "Max travel in degrees from biceps paralell to the floor. Set to 0 to use calculated max." }
};

static void scara_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    memcpy(&machine.cfg, &scara_settings, sizeof(scara_settings_t));

    machine.L1 = scara_settings.upper_arm;
    machine.L1_sqr = scara_settings.upper_arm * scara_settings.upper_arm;
    machine.L2 = scara_settings.forearm;
    machine.L2_sqr = scara_settings.forearm * scara_settings.forearm;

//    machine.cfg.home_angle = scara_settings.home_angle * RArrEG;
}

static void scara_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&scara_settings, sizeof(scara_settings_t), true);
}

static void scara_settings_restore (void)
{
    scara_settings.upper_arm = 50.0f;
    scara_settings.forearm = 40.0f;
    scara_settings.segment_length = .5f;
//    scara_settings.home_angle = 0.0f;
    scara_settings.flags.elbow_motor = On;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&scara_settings, sizeof(scara_settings_t), true);
}

static void scara_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&scara_settings, nvs_address, sizeof(scara_settings_t), true) != NVS_TransferResult_OK)
        scara_settings_restore();

    if(on_set_axis_setting_unit == NULL) {
        on_set_axis_setting_unit = grbl.on_set_axis_setting_unit;
        grbl.on_set_axis_setting_unit = scara_set_axis_setting_unit;
    }
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[KINEMATICS:Scara v0.00]" ASCII_EOL);
}

static const char *scara_setting_get_description (setting_id_t id)
{
    return id >= Setting_AxisStepsPerMM && id <= (Setting_AxisStepsPerMM + Z_AXIS)
            ? "Travel resolution in steps per radian (57.296 degrees)."
            : (on_setting_get_description ? on_setting_get_description(id) : NULL);
}

// Initialize API pointers for SCARA robot kinematics
void scara_robot_init (void)
{
    static setting_details_t setting_details = {
        .is_core = true,
        .groups = kinematics_groups,
        .n_groups = sizeof(kinematics_groups) / sizeof(setting_group_detail_t),
        .settings = kinematics_settings,
        .n_settings = sizeof(kinematics_settings) / sizeof(setting_detail_t),
        .descriptions = kinematics_settings_descr,
        .n_descriptions = sizeof(kinematics_settings_descr) / sizeof(setting_descr_t),
        .load = scara_settings_load,
        .restore = scara_settings_restore,
        .save = scara_settings_save,
        .on_changed = scara_settings_changed
    };

    if((nvs_address = nvs_alloc(sizeof(scara_settings_t)))) {

        kinematics.limits_set_target_pos = scara_limits_set_target_pos;
        kinematics.limits_get_axis_mask = scara_limits_get_axis_mask;
        kinematics.limits_set_machine_positions = scara_limits_set_machine_positions;
        kinematics.transform_from_cartesian = transform_from_cartesian;
        kinematics.transform_steps_to_cartesian = scara_convert_array_steps_to_mpos;
        kinematics.segment_line = scara_segment_line;
        kinematics.homing_cycle_get_feedrate = homing_cycle_get_feedrate;

        grbl.on_jog_cancel = cancel_jog;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_options;

        on_homing_completed = grbl.on_homing_completed;
        grbl.on_homing_completed = scara_homing_complete;

        on_setting_get_description = grbl.on_setting_get_description;
        grbl.on_setting_get_description = scara_setting_get_description;

        settings_register(&setting_details);
    }
}

#endif
