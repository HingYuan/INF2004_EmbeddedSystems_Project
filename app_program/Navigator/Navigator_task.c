#include <tk/tkernel.h>
#include <tk/device.h>
#include <tm/tmonitor.h>
#include <bsp/libbsp.h>

#include "Navigator_task.h"
#include "../robot_tasks.h"
#include "../Util/C_ring_buffer.h"
#include "../Motor/Motor_task.h"
#include "../Ultrasonic/Ultrasonic_task.h"
#include "../IMU/IMU_task.h"
#include "../Network/mqtt_client.h"

// Shared telemetry snapshot instance owned by Navigator
EXPORT volatile TELEMETRY_SNAPSHOT telemetry_snapshot = {0};

void clear_navigator_mbf(void)
{
    NAVIGATOR_MESSAGE discard_msg;
    while (tk_rcv_mbf(navigator_mbfid, &discard_msg, TMO_POL) >= 0) { }
}

static BYPASS_CONTEXT bypass;

static void bypass_abort(const char *reason);
static void bypass_stop(BYPASS_PHASE phase);
static void bypass_update(const NAVIGATOR_MESSAGE *message);
static void bypass_set_mode(MOVEMENT_MODE *mode, MOTOR_COMMAND *command);

_Static_assert(BYPASS_PIVOT_FROM_FRONT_CM <= ROBOT_LENGTH_CM, "pivot must lie within body");
_Static_assert(BYPASS_PEEK_DEG > 0U && BYPASS_PEEK_DEG <= 5U, "peek must not exceed 5 degrees");

/*
Note for Bryan, rmb to remove mqtt_get_movement_for_barcode function from network code
In case you need it
curr_speed_left = mot_cmd.target_speed_left;
curr_speed_right = mot_cmd.target_speed_right;
}
*/

/*
CHANGE m_mode
CHANGE instructions_count
CHANGE from instructions_fake to instructions
*/

EXPORT void navigator_task(INT stacd, void *exinf) 
{
    // General
    NAVIGATOR_MESSAGE n_msg;
    INT message_size;
    MOTOR_COMMAND mot_cmd;
    // MOVEMENT_MODE m_mode = MODE_WIFI_SEND_COMMS;
    // MOVEMENT_MODE m_mode = MODE_LINE_FOLLOWING;
    MOVEMENT_MODE m_mode = MODE_BARCODE_SCANNING;

    // Sensor specific telemetry cache
    char last_decoded_char = 0;
    UW last_obstacle_dist_cm = 0;
    UW last_closest_cm = 0;
    W last_closest_angle_deg = 0;
    UW last_estimated_width_cm = 0;
    float last_pitch_deg = 0.0f;
    float last_roll_deg = 0.0f;
    UB last_imu_event = 0;

    // Line following
    IR_STATE prev_ir_state = STATE_NULL;
    UH turning_rampup_count = 0;

    // Barcode decoding
    B instruction_idx = 0;                                                      // Iterate through instructions for each intersection
    B barcode_characters[NUMBER_OF_COMMANDS] = { 'A' };                              // Character key, sent from wifi
    DIRECTION barcode_instructions[NUMBER_OF_COMMANDS] = { DIR_LEFT };                    // Instruction mapping, sent from wifi
    B mapping_increment = 0;                                                    // 
    DIRECTION instructions[NUMBER_OF_COMMANDS] = {};                            // Instructions scanned from barcodes
    DIRECTION instructions_fake[NUMBER_OF_COMMANDS] = { DIR_RIGHT, DIR_LEFT, DIR_BACK, DIR_FRONT, DIR_STOP };
    for (int i=0; i<NUMBER_OF_COMMANDS; i++) instructions[i] = DIR_INVALID;     // Preset all instructions as DIR_INVALID
    B instructions_count = 0;   
    // B instructions_count = 5;

    // Decision making
    B num_rotations = 0;
    UW junction_ir_after_us = 0;

    // Hump
    BOOL hump_pop = 0;
    UH spam_mitigation = 0;
    UH spam_mitigation_threshold = 10;

    // Unused
    INT left_debounce_ticks = 0;
    INT right_debounce_ticks = 0;

    // Motion tracking for telemetry
    INT curr_speed_left = 0;
    INT curr_speed_right = 0;
    UW live_sonar_distance_cm = 0;
    BOOL live_sonar_distance_valid = FALSE;

    while (1) {
        BOOL bypass_active = m_mode == MODE_OBSTACLE_ASSESS || m_mode == MODE_WALL_HUGGING;
        message_size = tk_rcv_mbf(navigator_mbfid, (void *)&n_msg,
                                 bypass_active ? 20 : TMO_FEVR);

        if (message_size == sizeof(NAVIGATOR_MESSAGE)) {
            if (n_msg.type == MSG_REMOTE_CMD) {

                if(!motor_estop && m_mode != MODE_WALL_HUGGING && m_mode != MODE_SAFE_STOP) {
                    mot_cmd = n_msg.data.remote_cmd;

                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                }
            } else if (n_msg.type == MSG_IMU_UPDATE &&
                      (n_msg.data.imu_telemetry_msg.event == IMU_MSG_SHOCK_DETECTED ||
                       n_msg.data.imu_telemetry_msg.event == IMU_MSG_TILT_WARNING)) {
                motor_estop = TRUE;
                m_mode = MODE_SAFE_STOP;
                mot_cmd.target_speed_left = STOP_SPEED;
                mot_cmd.target_speed_right = STOP_SPEED;
                tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_POL);
                tm_printf((UB *)"[nav] EMERGENCY STOP triggered by IMU event %d\n", n_msg.data.imu_telemetry_msg.event);
            } else if (n_msg.type == MSG_SONAR_DISTANCE) {
                live_sonar_distance_valid = n_msg.data.sonar_distance.valid;
                
                if (live_sonar_distance_valid) {
                    live_sonar_distance_cm = n_msg.data.sonar_distance.distance_cm;
                }
            } else {
                switch (m_mode) {
                    case MODE_WIFI_SEND_COMMS:
                        instruction_idx = 0;

                        // Stop car
                        mot_cmd.target_speed_left =      STOP_SPEED;
                        mot_cmd.target_speed_right =     STOP_SPEED;
                        mot_cmd.turn_ticks_left =        0;
                        mot_cmd.turn_ticks_right =       0;

                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                        switch (n_msg.type) {
                            case MSG_WIFI_BARCODE_MAPPING:      // Note for bryan & future self: packed data must look like 0bxxxx xxxx yyyy yyyy, where x is direction and y is character.
                                DIRECTION direction = n_msg.data.packed_command_mapping >> 8;      // Cast int to enum
                                B code39_char = n_msg.data.packed_command_mapping & 0x00ff;        // bitwise & mask to get least-significant 8 bits

                                // Capitalize lowercase characters
                                if (code39_char >= 'a' && code39_char <= 'z') code39_char = code39_char - 32;
                                
                                //Sends temp log for debugging purposes, can be removed later ~ Bryan
                                tm_printf((UB *)"[nav] mapping received: %c -> %d\n",code39_char,direction);

                                barcode_characters[mapping_increment] = code39_char;
                                barcode_instructions[mapping_increment] = direction;
                                mapping_increment++;
                                instructions_count++;

                                if (mapping_increment >= NUMBER_OF_COMMANDS) {
                                    m_mode = MODE_BARCODE_SCANNING;
                                }

                                break;
                            default:
                                
                                break;
                        }

                        break;
                    case MODE_BARCODE_SCANNING:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:
                                switch (n_msg.data.ir_state) {
                                    case STATE_W_B:
                                        mot_cmd.target_speed_left =     RIGHT_TURN_L_BARCODE;
                                        mot_cmd.target_speed_right =   -RIGHT_TURN_R_BARCODE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    case STATE_B_W:
                                        mot_cmd.target_speed_left =     LEFT_TURN_L_BARCODE;
                                        mot_cmd.target_speed_right =   -LEFT_TURN_R_BARCODE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                    default:
                                        mot_cmd.target_speed_left =     FORWARD_SPEED_BARCODE;
                                        mot_cmd.target_speed_right =   -FORWARD_SPEED_BARCODE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        break;
                                }

                                break;
                            case MSG_BARCODE_DECODED:
                                last_decoded_char = (char)n_msg.data.decoded_char;

                                // Map detected character to instruction, then add to instruction array
                                for (B i=0; i<NUMBER_OF_COMMANDS; i++) {
                                    if (last_decoded_char == barcode_characters[i]) {
                                        instructions[instruction_idx] = barcode_instructions[i];
                                        instruction_idx++;
                                    }

                                    // If max instructions recorded
                                    if (instruction_idx > NUMBER_OF_COMMANDS) {
                                        m_mode = MODE_LINE_FOLLOWING;
                                        instruction_idx = 0;

                                        tk_dly_tsk(1000);
                                    }
                                }

                                break;
                        }

                        break;
                    case MODE_LINE_FOLLOWING:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:
                                IR_STATE curr_state = n_msg.data.ir_state;
      
                                    if (curr_state != prev_ir_state) {

                                    if (curr_state == STATE_W_W) {
                                        turning_rampup_count = 0;

                                        mot_cmd.target_speed_left =     FORWARD_SPEED;
                                        mot_cmd.target_speed_right =   -FORWARD_SPEED;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    } else if (curr_state == STATE_W_B) {
                                        // Car veering left, adjust right
                                        mot_cmd.target_speed_left =     RIGHT_TURN_L_BASE;
                                        mot_cmd.target_speed_right =   -RIGHT_TURN_R_BASE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    } else if (curr_state == STATE_B_W) {
                                        // Car veering right, adjust left
                                        mot_cmd.target_speed_left =     LEFT_TURN_L_BASE;
                                        mot_cmd.target_speed_right =   -LEFT_TURN_R_BASE;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    } else if (curr_state == STATE_B_B) {
                                        turning_rampup_count = 0;

                                        // Car at instersection
                                        mot_cmd.target_speed_left =     STOP_SPEED;
                                        mot_cmd.target_speed_right =   -STOP_SPEED;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        m_mode = MODE_DECISION_MAKING_ADVANCE;
                                    } else {
                                        // Errorneous state

                                    }

                                    // Update previous IR state
                                    prev_ir_state = curr_state;
                                } else {
                                    // Ramp up turning if stuck on turning
                                    if (curr_state == STATE_B_W || curr_state == STATE_W_B) {
                                        if (turning_rampup_count < TURNING_RAMPUP_LIMIT) {
                                            turning_rampup_count++;
                                        }

                                        if (turning_rampup_count == TURNING_RAMPUP_LIMIT) {
                                            turning_rampup_count++;     

                                            mot_cmd.turn_ticks_left =   0;
                                            mot_cmd.turn_ticks_right =  0;
                                            
                                            if (curr_state == STATE_B_W) {
                                                mot_cmd.target_speed_left =     LEFT_TURN_L_RAMP;
                                                mot_cmd.target_speed_right =   -LEFT_TURN_R_RAMP;
                                            } else if (curr_state == STATE_W_B) {
                                                mot_cmd.target_speed_left =     RIGHT_TURN_L_RAMP;
                                                mot_cmd.target_speed_right =   -RIGHT_TURN_R_RAMP;
                                            }

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                        }                                    
                                    }
                                }

                                break;
                            case MSG_OBSTACLE_DETECTED:
                                last_obstacle_dist_cm = n_msg.data.obstacle_distance_cm;
                                bypass = (BYPASS_CONTEXT){0};
                                bypass.started_us = time_us();
                                bypass_stop(BYPASS_IDLE);
                                m_mode = MODE_OBSTACLE_ASSESS;
                                mot_cmd = bypass.command;
                                break;

                            case MSG_IMU_UPDATE:
                                if (n_msg.data.imu_telemetry_msg.event == IMU_MSG_HUMP_DETECTED) {
                                    m_mode = MODE_GOING_OVER_HUMP;
                                }

                                break;
                            default:

                                break;
                        }

                        break;

                    case MODE_GOING_OVER_HUMP:
                        VB left_speed =     FORWARD_SPEED;
                        VB right_speed =   -FORWARD_SPEED;

                        VB pitch = n_msg.data.imu_telemetry_msg.pitch_deg;
                        VB roll = n_msg.data.imu_telemetry_msg.roll_deg;

                        switch (n_msg.data.imu_telemetry_msg.event) {
                            case IMU_MSG_HUMP_DETECTED:
                                if (pitch < 0) {
                                    mot_cmd.target_speed_left =  (left_speed + pitch) > 10 ? left_speed + pitch : 10;
                                    mot_cmd.target_speed_right = -right_speed;
                                } else {
                                    mot_cmd.target_speed_left =  left_speed;
                                    mot_cmd.target_speed_right = -((right_speed - pitch) < -10 ? right_speed - pitch : -10);
                                }

                                if (roll < -10 && hump_pop != 1) {
                                    hump_pop = 1;
                                } else if (roll > 10 && hump_pop != 0) {
                                    hump_pop = 0;
                                }
                                
                                mot_cmd.turn_ticks_left =        0;
                                mot_cmd.turn_ticks_right =       0;

                                spam_mitigation++;
                                if (spam_mitigation > spam_mitigation_threshold) {
                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                    spam_mitigation = 0;
                                }

                                break;
                            case IMU_MSG_NO_HUMP:
                                // tm_printf("NO HUMP RECEIVED\n");
                                if (hump_pop == 0) {
                                    // tm_printf("RECOVERED\n");
                                    m_mode = MODE_LINE_FOLLOWING;
                                } else {
                                    mot_cmd.turn_ticks_left =        0;
                                    mot_cmd.turn_ticks_right =       0;

                                    tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);
                                }

                                break;
                        }

                        break;

                    case MODE_DECISION_MAKING_ADVANCE:
                        tk_dly_tsk(250);

                        mot_cmd.target_speed_left =     FORWARD_SPEED;
                        mot_cmd.target_speed_right =   -FORWARD_SPEED;
                        mot_cmd.turn_ticks_left =       0;
                        mot_cmd.turn_ticks_right =      0;

                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                        tk_dly_tsk(400);

                        mot_cmd.target_speed_left =     0;
                        mot_cmd.target_speed_right =   -0;
                        mot_cmd.turn_ticks_left =       0;
                        mot_cmd.turn_ticks_right =      0;

                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                        tk_dly_tsk(250);

                        m_mode = MODE_DECISION_MAKING_TURN;
                        // Discard only old IR observations; retain queued obstacle/stop events.
                        junction_ir_after_us = time_us();

                        tk_dly_tsk(30);     // wait for next IR update

                        break;

                    // Junction Navigation
                    case MODE_DECISION_MAKING_TURN:

                        if (n_msg.type == MSG_IR_UPDATE &&
                            (W)(n_msg.sampled_at_us - junction_ir_after_us) < 0)
                            break;

                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:

                                IR_STATE curr_ir_state = n_msg.data.ir_state;
                                DIRECTION curr_decision = instructions[instruction_idx];
                                // DIRECTION curr_decision = instructions_fake[instruction_idx];

                                if (prev_ir_state == STATE_B_B) {       // should still be B_B after advance
                                    switch (curr_decision) {
                                        case DIR_FRONT:
                                            m_mode = MODE_LINE_FOLLOWING;

                                            break;
                                        case DIR_BACK:
                                            num_rotations = 2;

                                            mot_cmd.target_speed_left =    -LEFT_PIVOT_L;
                                            mot_cmd.target_speed_right =   -RIGHT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_LEFT:
                                            num_rotations = 1;

                                            mot_cmd.target_speed_left =    -LEFT_PIVOT_L;
                                            mot_cmd.target_speed_right =   -LEFT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_RIGHT:
                                            num_rotations = 1;

                                            mot_cmd.target_speed_left =     RIGHT_PIVOT_L;
                                            mot_cmd.target_speed_right =    RIGHT_PIVOT_R;
                                            mot_cmd.turn_ticks_left =       0;
                                            mot_cmd.turn_ticks_right =      0;

                                            tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                            break;
                                        case DIR_STOP:
                                            num_rotations = 0;
                                           
                                        default:
                                            m_mode = MODE_LINE_FOLLOWING;

                                            break;
                                    }

                                } else {
                                    if (curr_ir_state == STATE_B_W && prev_ir_state == STATE_W_W && (curr_decision == DIR_LEFT || curr_decision == DIR_BACK)) {
                                        num_rotations--;
                                    } else if (curr_ir_state == STATE_W_B && prev_ir_state == STATE_W_W && curr_decision == DIR_RIGHT) {
                                        num_rotations--;
                                    }
                                    
                                    if (num_rotations <= 0) {

                                        mot_cmd.target_speed_left =     STOP_SPEED;
                                        mot_cmd.target_speed_right =    STOP_SPEED;
                                        mot_cmd.turn_ticks_left =       0;
                                        mot_cmd.turn_ticks_right =      0;

                                        tk_snd_mbf(motor_mbfid, (void *)&mot_cmd, sizeof(MOTOR_COMMAND), TMO_FEVR);

                                        tk_dly_tsk(250);

                                        // Linger on last instruction on overflow
                                        if (++instruction_idx >= instructions_count) { instruction_idx--; }

                                        tm_printf("INSTRUCTIONS: %d %d\n", instruction_idx, instructions_count);
                                        // Reset variables
                                        num_rotations = 0;
                                        prev_ir_state = STATE_NULL;
                                        m_mode = MODE_LINE_FOLLOWING;
                                    }
                                }

                                if (prev_ir_state != curr_ir_state) { prev_ir_state = curr_ir_state; }

                                break;
                            default:
                                
                                break;
                        }

                        break;
                    case MODE_OBSTACLE_ASSESS:
                    case MODE_WALL_HUGGING:
                        switch (n_msg.type) {
                            case MSG_OBSTACLE_PROFILED:
                                last_closest_cm = n_msg.data.obstacle_profile.closest_cm;
                                last_closest_angle_deg = n_msg.data.obstacle_profile.closest_angle_deg;
                                last_estimated_width_cm = n_msg.data.obstacle_profile.width_valid ?
                                    n_msg.data.obstacle_profile.estimated_width_cm : 0U;
                                break;
                            case MSG_IR_UPDATE:
                                prev_ir_state = n_msg.data.ir_state;
                                break;
                            case MSG_SONAR_PROBED:
                                if (n_msg.data.sonar_probe.angle_deg == 0) {
                                    live_sonar_distance_valid = n_msg.data.sonar_probe.valid;
                                    if (live_sonar_distance_valid)
                                        live_sonar_distance_cm = n_msg.data.sonar_probe.distance_cm;
                                }
                                break;
                            default:
                                break;
                        }
                        bypass_update(&n_msg);
                        bypass_set_mode(&m_mode, &mot_cmd);
                        break;

                    case MODE_REENTRY_ADVANCE:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:

                                break;
                            default:

                                break;
                        }

                        break;

                    case MODE_REENTRY_PIVOT:
                        switch (n_msg.type) {
                            case MSG_IR_UPDATE:

                                break;
                            default:

                                break;
                        }

                        break;

                    default:

                        break;
                }

                
            }

            // Update shared telemetry snapshot
            // telemetry_snapshot.speed_left = curr_speed_left;
            // telemetry_snapshot.speed_right = curr_speed_right;
            telemetry_snapshot.speed_left = mot_cmd.target_speed_left;
            telemetry_snapshot.speed_right = mot_cmd.target_speed_right;

            telemetry_snapshot.ir_state = (uint8_t)prev_ir_state;
            telemetry_snapshot.movement_mode = (uint8_t)m_mode;
            telemetry_snapshot.bypass_phase = (uint8_t)bypass.phase;
            telemetry_snapshot.last_barcode = last_decoded_char;
            telemetry_snapshot.instruction_idx = (uint8_t)instruction_idx;
            telemetry_snapshot.enc_l = (int32_t)enc_l_ticks;
            telemetry_snapshot.enc_r = (int32_t)enc_r_ticks;
            telemetry_snapshot.obstacle_distance_cm = last_obstacle_dist_cm;
            telemetry_snapshot.closest_cm = last_closest_cm;
            telemetry_snapshot.closest_angle_deg = (int32_t)last_closest_angle_deg;
            telemetry_snapshot.estimated_width_cm = last_estimated_width_cm;
            telemetry_snapshot.seq++;
            telemetry_snapshot.obstacle_distance_valid = live_sonar_distance_valid;
            if (live_sonar_distance_valid) {
                telemetry_snapshot.obstacle_distance_cm = live_sonar_distance_cm;
            } else {
                telemetry_snapshot.obstacle_distance_cm = last_obstacle_dist_cm;
            }

        } else {
            // Bypass receive errors and timeouts are handled below.
        }

        // Keep timed moves bounded even when the common handlers consume messages.
        if (bypass_active && bypass.wait != BYPASS_WAIT_NONE) {
            if (message_size != sizeof(NAVIGATOR_MESSAGE) && message_size != E_TMOUT) {
                bypass_abort("Navigator receive failed");
            } else {
                const NAVIGATOR_MESSAGE *remote =
                    message_size == sizeof(NAVIGATOR_MESSAGE) && n_msg.type == MSG_REMOTE_CMD ?
                    &n_msg : NULL;
                bypass_update(remote);
            }
            bypass_set_mode(&m_mode, &mot_cmd);
            telemetry_snapshot.speed_left = mot_cmd.target_speed_left;
            telemetry_snapshot.speed_right = mot_cmd.target_speed_right;
            telemetry_snapshot.movement_mode = (uint8_t)m_mode;
            telemetry_snapshot.bypass_phase = (uint8_t)bypass.phase;
        }

    }
}

static void bypass_set_mode(MOVEMENT_MODE *mode, MOTOR_COMMAND *command)
{
    *command = bypass.command;
    if (bypass.phase == BYPASS_ABORT)
        *mode = MODE_SAFE_STOP;
    else if (bypass.phase == BYPASS_COMPLETE)
        *mode = MODE_REENTRY_ADVANCE;
    else if (bypass.phase != BYPASS_IDLE)
        *mode = MODE_WALL_HUGGING;
}

static void bypass_abort(const char *reason)
{
    motor_estop = TRUE;
    motor_stop_ack_requested = FALSE;
    bypass.command = (MOTOR_COMMAND){0};
    bypass.phase = BYPASS_ABORT;
    bypass.wait = BYPASS_WAIT_NONE;
    (void)tk_set_flg(ranger_flgid, SONAR_PAUSE_FLAG);
    (void)tk_snd_mbf(motor_mbfid, &bypass.command, sizeof(bypass.command), TMO_POL);
    tm_printf((UB *)"[bypass] STOP: %s\n", reason);
}

static void bypass_stop(BYPASS_PHASE phase)
{
    motor_estop = TRUE;
    motor_stop_ack_requested = TRUE;
    bypass.command = (MOTOR_COMMAND){0};
    bypass.phase = phase;
    bypass.wait = BYPASS_WAIT_STOP;
    bypass.deadline_us = time_us() + BYPASS_RESPONSE_MS * 1000U;
}

static UINT bypass_side_probe(void)
{
    return bypass.side > 0 ? SONAR_PROBE_RIGHT_FLAG : SONAR_PROBE_LEFT_FLAG;
}

static void bypass_probe(BYPASS_PHASE phase, UINT flag)
{
    if (phase != bypass.phase)
        bypass.attempts = 0;
    bypass.phase = phase;
    bypass.wait = BYPASS_WAIT_PROBE;
    bypass.probe_flag = flag;
    bypass.probe_angle = flag == SONAR_PROBE_LEFT_FLAG ? 45 :
                         flag == SONAR_PROBE_RIGHT_FLAG ? -45 : 0;
    bypass.requested_us = time_us();
    bypass.deadline_us = bypass.requested_us + BYPASS_RESPONSE_MS * 1000U;
    if (!motor_estop || tk_set_flg(ranger_flgid, flag) < E_OK)
        bypass_abort("probe request failed");
}

static void bypass_drive(BYPASS_PHASE phase, INT left, INT right, UW duration_ms)
{
    if (duration_ms == 0U ||
        (motor_fault_word & (FAULT_BIT_STALL | FAULT_BIT_ENCODER)) != 0U) {
        bypass_abort("invalid movement / motor fault");
        return;
    }
    bypass.command = (MOTOR_COMMAND){0};
    bypass.command.target_speed_left = left;
    bypass.command.target_speed_right = right;
    bypass.phase = phase;
    bypass.wait = BYPASS_WAIT_MOVE;
    bypass.segment_ms = duration_ms;
    bypass.deadline_us = time_us() + duration_ms * 1000U;
    motor_stop_ack_requested = FALSE;
    motor_estop = FALSE;
    if (tk_snd_mbf(motor_mbfid, &bypass.command, sizeof(bypass.command), TMO_POL) < E_OK) {
        bypass_abort("motor queue full");
        return;
    }
    tm_printf((UB *)"[bypass] phase=%d L=%d R=%d for %u ms\n",
              phase, left, right, duration_ms);
}

static void bypass_pivot(BYPASS_PHASE phase, INT direction, UW degrees)
{
    UW turn_ms = direction > 0 ? BYPASS_LEFT_90_MS : BYPASS_RIGHT_90_MS;
    INT left = direction > 0 ? -LEFT_PIVOT_L : RIGHT_PIVOT_L;
    INT right = direction > 0 ? -LEFT_PIVOT_R : RIGHT_PIVOT_R;
    // The right motor is mirrored: equal PWM signs produce a stationary pivot.
    bypass_drive(phase, left, right, (turn_ms * degrees + 89U) / 90U);
}

static UW bypass_swept_radius_cm(void)
{
    UW arm = BYPASS_PIVOT_FROM_FRONT_CM;
    UW rear = ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM;
    UW radius = ROBOT_HALF_WIDTH_CM;
    if (rear > arm) arm = rear;
    while (radius * radius < arm * arm + ROBOT_HALF_WIDTH_CM * ROBOT_HALF_WIDTH_CM)
        radius++;
    return radius;
}

static void bypass_corner_peek(void)
{
    UW arm = BYPASS_PIVOT_FROM_FRONT_CM;
    UW rear = ROBOT_LENGTH_CM - BYPASS_PIVOT_FROM_FRONT_CM;
    if (rear > arm) arm = rear;
    // sin(5 degrees) < 0.088; retain 2 cm during the small peek.
    UW peek_width = ROBOT_HALF_WIDTH_CM + (arm * 88U + 999U) / 1000U + 2U;
    if (!bypass.wall_seen || bypass.wall_cm * 707U / 1000U < peek_width) {
        bypass_abort("insufficient room for corner peek");
        return;
    }
    if (!bypass.corner_active) {
        bypass.corner_active = TRUE;
        // shortcut: rear clearance uses timed travel, replace with measured pose if slip is frequent.
        bypass.corner_remaining_ms = (bypass.wall_cm * 707U / 1000U +
            ROBOT_LENGTH_CM + WALL_BODY_GAP_CM) * BYPASS_MS_PER_CM;
    }
    bypass_pivot(BYPASS_PEEK_OUT, -bypass.side, BYPASS_PEEK_DEG);
}

static BOOL bypass_wall_present(const SONAR_PROBE_RESULT *probe)
{
    return probe->valid && probe->distance_cm >= 2U &&
           probe->distance_cm < BYPASS_WALL_TARGET_CM + BYPASS_EDGE_JUMP_CM;
}

static void bypass_track(void)
{
    bypass.phase = BYPASS_CHECK_SIDE;
    bypass.wait = BYPASS_WAIT_TRACK;
    bypass.requested_us = time_us();
    bypass.deadline_us = bypass.requested_us + BYPASS_RESPONSE_MS * 1000U;
    UINT flag = bypass.side > 0 ? SONAR_TRACK_RIGHT_FLAG : SONAR_TRACK_LEFT_FLAG;
    if (tk_set_flg(ranger_flgid, flag) < E_OK)
        bypass_abort("tracking request failed");
}

static void bypass_steer(UW cm)
{
    INT error = (INT)cm - (INT)BYPASS_WALL_TARGET_CM;
    if (error >= -BYPASS_WALL_DEADBAND_CM && error <= BYPASS_WALL_DEADBAND_CM)
        error = 0;
    INT correction = bypass.side * CLAMP(error, -BYPASS_STEER_LIMIT, BYPASS_STEER_LIMIT);
    bypass.command = (MOTOR_COMMAND){0};
    bypass.command.target_speed_left = FORWARD_SPEED_BARCODE + correction;
    bypass.command.target_speed_right = -(FORWARD_SPEED_BARCODE - correction);
    if (motor_estop || tk_snd_mbf(motor_mbfid, &bypass.command,
                                 sizeof(bypass.command), TMO_POL) < E_OK)
        bypass_abort("tracking interrupted / motor queue full");
}

static void bypass_after_stop(void)
{
    bypass.attempts = 0;
    switch (bypass.phase) {
    case BYPASS_IDLE:
        bypass.wait = BYPASS_WAIT_SCAN;
        bypass.deadline_us = time_us() + BYPASS_SCAN_MS * 1000U;
        if (tk_set_flg(ranger_flgid, SONAR_SCAN_REQUEST) < E_OK)
            bypass_abort("scan request failed");
        break;
    case BYPASS_TURN_OUT:
        bypass_probe(BYPASS_CHECK_OUT, SONAR_PROBE_FRONT_FLAG);
        break;
    case BYPASS_SHIFT_OUT:
        bypass.shift_remaining_ms -= bypass.segment_ms;
        if (bypass.shift_remaining_ms == 0U)
            bypass_pivot(BYPASS_TURN_PARALLEL, -bypass.side, 90U);
        else
            bypass_probe(BYPASS_CHECK_OUT, SONAR_PROBE_FRONT_FLAG);
        break;
    case BYPASS_TURN_PARALLEL:
    case BYPASS_ADVANCE:
        bypass_probe(BYPASS_CHECK_FORWARD, SONAR_PROBE_FRONT_FLAG);
        break;
    case BYPASS_CHECK_SIDE:
        bypass_probe(BYPASS_CHECK_SIDE, bypass_side_probe());
        break;
    case BYPASS_PEEK_OUT:
        bypass_probe(BYPASS_CHECK_PEEK, bypass_side_probe());
        break;
    case BYPASS_PEEK_BACK:
        bypass_probe(BYPASS_CHECK_CLEAR, SONAR_PROBE_FRONT_FLAG);
        break;
    case BYPASS_CLEAR_CORNER:
        bypass.corner_remaining_ms -= bypass.segment_ms;
        bypass_probe(BYPASS_CHECK_SIDE, bypass_side_probe());
        break;
    case BYPASS_TURN_IN:
    case BYPASS_FIND_LINE:
        bypass_probe(BYPASS_CHECK_RETURN, SONAR_PROBE_FRONT_FLAG);
        break;
    case BYPASS_LINE_FOUND:
        if (tk_set_flg(ranger_flgid, SONAR_PAUSE_FLAG) < E_OK) {
            bypass_abort("sonar pause failed");
            break;
        }
        bypass.phase = BYPASS_COMPLETE;
        bypass.wait = BYPASS_WAIT_NONE;
        tm_printf((UB *)"[bypass] Line found; stopped handover to reentry\n");
        break;
    default:
        bypass_abort("unexpected stop phase");
        break;
    }
}

static void bypass_profile(const OBSTACLE_PROFILE *profile)
{
    BOOL left = profile->left_edge_valid && profile->left_offset_valid;
    BOOL right = profile->right_edge_valid && profile->right_offset_valid;
    if (!profile->profile_valid || (!left && !right) ||
        profile->closest_cm + BYPASS_PIVOT_FROM_FRONT_CM <
        bypass_swept_radius_cm() + WALL_BODY_GAP_CM) {
        bypass_abort("invalid profile / no observed edge / pivot too close");
        return;
    }
    bypass.side = left && (!right || profile->left_offset_cm <= profile->right_offset_cm) ? 1 : -1;
    UW offset = bypass.side > 0 ? profile->left_offset_cm : profile->right_offset_cm;
    if (offset < ROBOT_HALF_WIDTH_CM + WALL_BODY_GAP_CM || offset > 100U) {
        bypass_abort("invalid lateral offset");
        return;
    }
    bypass.shift_remaining_ms = offset * BYPASS_MS_PER_CM;
    tm_printf((UB *)"[bypass] Pass %s; offset=%u cm, wall ray target=%u cm\n",
              bypass.side > 0 ? "left" : "right", offset, BYPASS_WALL_TARGET_CM);
    bypass_pivot(BYPASS_TURN_OUT, bypass.side, 90U);
}

static void bypass_probe_result(const SONAR_PROBE_RESULT *probe)
{
    if (probe->angle_deg != bypass.probe_angle) {
        bypass_abort("unexpected sonar angle");
        return;
    }
    BOOL side_check = bypass.phase == BYPASS_CHECK_SIDE || bypass.phase == BYPASS_CHECK_PEEK;
    if (!probe->valid || (side_check && !bypass_wall_present(probe))) {
        if (++bypass.attempts < BYPASS_SCAN_ATTEMPTS) {
            bypass_probe(bypass.phase, bypass.probe_flag);
        } else if (bypass.phase == BYPASS_CHECK_SIDE) {
            bypass_corner_peek();
        } else if (bypass.phase == BYPASS_CHECK_PEEK) {
            bypass_pivot(BYPASS_PEEK_BACK, bypass.side, BYPASS_PEEK_DEG);
        } else {
            bypass_abort("no valid forward clearance");
        }
        return;
    }
    bypass.attempts = 0;
    if (side_check) {
        if (bypass.phase == BYPASS_CHECK_PEEK) {
            // Still seeing the wall: restart the rear-clearance estimate at this position.
            bypass.corner_remaining_ms = (bypass.wall_cm * 707U / 1000U +
                ROBOT_LENGTH_CM + WALL_BODY_GAP_CM) * BYPASS_MS_PER_CM;
            bypass_pivot(BYPASS_PEEK_BACK, bypass.side, BYPASS_PEEK_DEG);
        } else {
            bypass.wall_seen = TRUE;
            bypass.wall_cm = probe->distance_cm;
            bypass.corner_active = FALSE;
            bypass_probe(BYPASS_CHECK_FORWARD, SONAR_PROBE_FRONT_FLAG);
        }
        return;
    }
    UW reserve = BYPASS_STOP_DISTANCE_CM + WALL_BODY_GAP_CM;
    if (probe->distance_cm <= reserve || probe->distance_cm > 400U) {
        bypass_abort("forward path blocked");
        return;
    }
    // Reserve stopping room, allowing twice the estimated forward speed between checks.
    UW budget = (probe->distance_cm - reserve) * BYPASS_MS_PER_CM / 2U;
    bypass.front_budget_ms = budget < BYPASS_FRONT_INTERVAL_MS ? budget : BYPASS_FRONT_INTERVAL_MS;
    UW duration = bypass.front_budget_ms;
    switch (bypass.phase) {
    case BYPASS_CHECK_OUT:
        if (duration > bypass.shift_remaining_ms) duration = bypass.shift_remaining_ms;
        bypass_drive(BYPASS_SHIFT_OUT, FORWARD_SPEED_BARCODE, -FORWARD_SPEED_BARCODE, duration);
        break;
    case BYPASS_CHECK_FORWARD:
        bypass_track();
        break;
    case BYPASS_CHECK_CLEAR:
        if (bypass.corner_remaining_ms == 0U) {
            if (probe->distance_cm + BYPASS_PIVOT_FROM_FRONT_CM <
                bypass_swept_radius_cm() + WALL_BODY_GAP_CM)
                bypass_abort("insufficient room for inward pivot");
            else
                bypass_pivot(BYPASS_TURN_IN, -bypass.side, 90U);
        } else {
            if (duration > BYPASS_SEARCH_STEP_CM * BYPASS_MS_PER_CM)
                duration = BYPASS_SEARCH_STEP_CM * BYPASS_MS_PER_CM;
            if (duration > bypass.corner_remaining_ms) duration = bypass.corner_remaining_ms;
            bypass_drive(BYPASS_CLEAR_CORNER, FORWARD_SPEED_BARCODE, -FORWARD_SPEED_BARCODE, duration);
        }
        break;
    case BYPASS_CHECK_RETURN:
        if (bypass.return_started_us == 0U) {
            bypass.return_started_us = time_us();
        }
        bypass.last_ir_us = time_us();
        bypass.black_count = 0;
        bypass_drive(BYPASS_FIND_LINE, FORWARD_SPEED_BARCODE, -FORWARD_SPEED_BARCODE, duration);
        break;
    default:
        bypass_abort("unexpected probe phase");
        break;
    }
}

static void bypass_update(const NAVIGATOR_MESSAGE *message) 
{
    UW now = time_us();
    if (bypass.wait == BYPASS_WAIT_NONE)
        return;
    if (message != NULL && (message->type == MSG_OBSTACLE_DETECTED ||
        (message->type == MSG_REMOTE_CMD && message->data.remote_cmd.target_speed_left == 0 &&
         message->data.remote_cmd.target_speed_right == 0))) {
        bypass_abort("obstacle / remote stop");
        return;
    }
    if ((UW)(now - bypass.started_us) >= BYPASS_TOTAL_MS * 1000U ||
        (bypass.return_started_us != 0U &&
         (UW)(now - bypass.return_started_us) >= BYPASS_RETURN_MS * 1000U) ||
        (motor_fault_word & (FAULT_BIT_STALL | FAULT_BIT_ENCODER)) != 0U) {
        bypass_abort("deadline / motor fault");
        return;
    }
    if (bypass.wait == BYPASS_WAIT_MOVE && motor_estop) {
        bypass_abort("external emergency stop");
        return;
    }
    if ((W)(now - bypass.deadline_us) >= 0) {
        if (bypass.wait == BYPASS_WAIT_MOVE) {
            if ((UW)(now - bypass.deadline_us) > 100000U)
                bypass_abort("movement timer serviced late");
            else
                bypass_stop(bypass.phase);
        } else {
            bypass_abort("missing stop / sonar response");
        }
        return;
    }
    if (bypass.phase == BYPASS_ADVANCE && bypass.wait == BYPASS_WAIT_MOVE &&
        (UW)(now - bypass.last_tracking_us) >= BYPASS_TRACK_STALE_MS * 1000U) {
        bypass_abort("stale wall tracking");
        return;
    }
    if (message == NULL)
        return;
    if (message->type == MSG_MOTOR_STOPPED && bypass.wait == BYPASS_WAIT_STOP) {
        bypass_after_stop();
    } else if (message->type == MSG_OBSTACLE_PROFILED && bypass.wait == BYPASS_WAIT_SCAN) {
        bypass_profile(&message->data.obstacle_profile);
    } else if (message->type == MSG_SONAR_PROBED && bypass.wait == BYPASS_WAIT_PROBE) {
        if ((W)(message->sampled_at_us - bypass.requested_us) >= 0 &&
            (UW)(now - message->sampled_at_us) < BYPASS_RESPONSE_MS * 1000U)
            bypass_probe_result(&message->data.sonar_probe);
    } else if (message->type == MSG_TRACKING_OBSTACLE &&
               (bypass.wait == BYPASS_WAIT_TRACK ||
                (bypass.phase == BYPASS_ADVANCE && bypass.wait == BYPASS_WAIT_MOVE))) {
        const SONAR_PROBE_RESULT *probe = &message->data.sonar_probe;
        if ((W)(message->sampled_at_us - bypass.requested_us) < 0 ||
            (UW)(now - message->sampled_at_us) >= BYPASS_TRACK_STALE_MS * 1000U)
            return;
        if (probe->angle_deg != -45 * bypass.side) {
            bypass_abort("unexpected tracking angle");
            return;
        }
        if (!bypass_wall_present(probe)) {
            bypass_stop(BYPASS_CHECK_SIDE);
            return;
        }
        if (probe->distance_cm * 707U / 1000U <= ROBOT_HALF_WIDTH_CM + 2U) {
            bypass_abort("wall too close");
            return;
        }
        bypass.wall_seen = TRUE;
        bypass.wall_cm = probe->distance_cm;
        bypass.last_tracking_us = message->sampled_at_us;
        if (bypass.wait == BYPASS_WAIT_TRACK)
            bypass_drive(BYPASS_ADVANCE, FORWARD_SPEED_BARCODE, -FORWARD_SPEED_BARCODE,
                         bypass.front_budget_ms);
        if (bypass.phase != BYPASS_ABORT)
            bypass_steer(probe->distance_cm);
    } else if (message->type == MSG_IR_UPDATE && bypass.phase == BYPASS_FIND_LINE &&
               bypass.wait == BYPASS_WAIT_MOVE &&
               (W)(message->sampled_at_us - bypass.last_ir_us) > 0 &&
               (UW)(now - message->sampled_at_us) < 100000U) {
        if ((UW)(message->sampled_at_us - bypass.last_ir_us) >= 100000U)
            bypass.black_count = 0;
        bypass.last_ir_us = message->sampled_at_us;
        IR_STATE state = message->data.ir_state;
        if (state == STATE_B_W || state == STATE_W_B || state == STATE_B_B)
            bypass.black_count++;
        else
            bypass.black_count = 0;
        if (bypass.black_count >= 2U)
            bypass_stop(BYPASS_LINE_FOUND);
    }
}
