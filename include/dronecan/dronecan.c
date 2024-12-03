#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <canard.h>
#include <dronecan.h>
#include <eeprom-pm.h>
#include "version.h"
#include "to_string.h"

extern HAL_Handle halHandle;
extern MOTOR_Vars_t gMotorVars;
extern USER_Params gUserParams;
extern uint64_t next_1hz_service_at;
extern uint64_t next_telem_service_at;
extern uint64_t next_DC_clear_service_at;
extern uint16_t peak_percent;
extern settings_t settings;
extern canstats_t canstats;
extern esc_t esc;
extern bool Flag_Latch_Save;
extern bool Flag_Update_Settings;
extern bool Flag_Arming;
extern bool Flag_Overheat;
extern int16_t RAWcmd;
extern uint64_t lastNonZeroRAWcmdTime;
extern uint8_t error;
extern bool Flag_nFaultDroneCan;
#ifdef DRV8305_SPI
extern uint16_t fault[4];
#endif

#define APP_VERSION_MAJOR 1
#define APP_VERSION_MINOR 0
#define APP_NODE_NAME "org.ipm.manta100"
#define UNIQUE_ID_LENGTH_BYTES 16

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_MOTOR_ID 111
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ARMING 112
#define UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_MOTOR_OFF 113

static CanardInstance canard;
static uint8_t canard_memory_pool[380];

static uint8_t previousUserErrorCode = 0;
static uint8_t previousCtrlState = 0;
static uint8_t previousEstState = 0;

static DeviceContext dc = {
    .last_tx_delay_time = 0,
    .tx_delay_duration_us = 5,
    .error = 0};

static bool Motor_Auto_ID = false;

static uint8_t req_index = 0;
static uint8_t setter = 0;

ProfileState currentState = PHASE_IDLE;

#pragma DATA_SECTION(str_node_id, "fstring");
const char str_node_id[] = "NODE_ID";
#pragma DATA_SECTION(str_esc_index, "fstring");
const char str_esc_index[] = "ESC_INDEX";
#pragma DATA_SECTION(str_arming, "fstring");
const char str_arming[] = "ARMING";
#pragma DATA_SECTION(str_telem_rate, "fstring");
const char str_telem_rate[] = "TELEM_RATE";
#pragma DATA_SECTION(str_can_speed, "fstring");
const char str_can_speed[] = "CAN_SPEED";
#pragma DATA_SECTION(str_max_speed, "fstring");
const char str_max_speed[] = "MAX_SPEED";
#pragma DATA_SECTION(str_controll_word, "fstring");
const char str_controll_word[] = "CONTROL_WORD";
#pragma DATA_SECTION(str_midle_point, "fstring");
const char str_midle_point[] = "MIDLE_POINT";
#pragma DATA_SECTION(str_acseleration, "fstring");
const char str_acseleration[] = "ACCELER";
#pragma DATA_SECTION(str_motor_poles, "fstring");
const char str_motor_poles[] = "MOTOR_POLES";
#pragma DATA_SECTION(str_Kp, "fstring");
const char str_Kp[] = "KP";
#pragma DATA_SECTION(str_Ki, "fstring");
const char str_Ki[] = "KI";

#pragma DATA_SECTION(str_res_est_current, "fstring");
const char str_res_est_current[] = "RES_EST_CURRENT";
#pragma DATA_SECTION(str_ind_est_current, "fstring");
const char str_ind_est_current[] = "IND_EST_CURRENT";
#pragma DATA_SECTION(str_mac_current, "fstring");
const char str_mac_current[] = "MAX_CURRENT";
#pragma DATA_SECTION(str_flux_est_freq, "fstring");
const char str_flux_est_freq[] = "FLUX_EST_FREQ";

#pragma DATA_SECTION(str_Rs, "fstring");
const char str_Rs[] = "Rs";
#pragma DATA_SECTION(str_Ld, "fstring");
const char str_Ld[] = "Ld";
#pragma DATA_SECTION(str_FLUX, "fstring");
const char str_FLUX[] = "FLUX";

static struct parameter {
    enum uavcan_protocol_param_Value_type_t type;
    float *value;
    const char *name;
    int decimal;
} parameters[] = {
    {UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.can_node, str_node_id, 0},              // CAN node ID
    {UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.esc_index, str_esc_index, 0},           // index in RawCommand
    {UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.require_arming, str_arming, 0},         // arming
    {UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.telem_rate, str_telem_rate, 0},         // telemetry rate
    {UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.can_speed, str_can_speed, 0},           // CAN speed
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.max_speed, str_max_speed, 1},              // max speed
    {UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.controll_word, str_controll_word, 0},   // control word
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.midle_point, str_midle_point, 0},          // midle point
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.acseleration, str_acseleration, 1},        // acseleration
    {UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, &settings.motor_poles, str_motor_poles, 0},       // motor poles
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.Kp, str_Kp, 3},                            // KP
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.Ki, str_Ki, 3},                            // KI
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.res_est_current, str_res_est_current, 2},  // MAX CURRENT RES EST
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.ind_est_current, str_ind_est_current, 2},  // MAX CURRENT IND EST
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.maxCurrent, str_mac_current, 1},           // Motor Max CURRENT
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &settings.fluxEstFreq_Hz, str_flux_est_freq, 0},     // FLUX EST FREQ
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &gUserParams.motor_Rs, str_Rs, 11},                  // Rs
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &gUserParams.motor_Ls_d, str_Ld, 11},                // Ld
    {UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE, &gUserParams.motor_ratedFlux, str_FLUX, 11},         // FLUX
};

#ifdef F2802xF
#pragma DATA_SECTION(parameters, "rom_accessed_data");
#endif

char buffer_str[50];

#ifdef F2802xF
#pragma DATA_SECTION(buffer_str, "rom_accessed_data");
#endif

static uint8_t msg_buffer[380];

static uint8_t midle_point_value = 0;
static uint16_t esc_min_value = 0;
static uint16_t esc_max_value = 0;

static struct uavcan_protocol_NodeStatus node_status;

// Определение строковых констант во Flash
// #pragma DATA_SECTION(str_can_node, "fstring");
// const char str_can_node[] = "NODE";

#pragma DATA_SECTION(str_DrvError0, "fstring");
const char str_DrvError0[] = "DrvError0:";
#pragma DATA_SECTION(str_DrvError1, "fstring");
const char str_DrvError1[] = "DrvError1:";
#pragma DATA_SECTION(str_DrvError2, "fstring");
const char str_DrvError2[] = "DrvError2:";
#pragma DATA_SECTION(str_DrvError3, "fstring");
const char str_DrvError3[] = "DrvError3:";

const char *str_DrvError[] = {str_DrvError0, str_DrvError1, str_DrvError2, str_DrvError3};

#pragma DATA_SECTION(str_UserErrorCode, "fstring");
const char str_UserErrorCode[] = "UserErrorCode:";
#pragma DATA_SECTION(str_CtrlState, "fstring");
const char str_CtrlState[] = "CtrlState:";
#pragma DATA_SECTION(str_EstState, "fstring");
const char str_EstState[] = "EstState:";

// Returns a pseudo random float in the range [0, 1].
float getRandomFloat(void) {
    static bool initialized = false;
    if (!initialized) {  // This is not thread safe, but a race condition here is not harmful.
        initialized = true;
        srand((unsigned)time(NULL));
    }
    // coverity[dont_call]
    return (float)rand() / (float)RAND_MAX;
}

static void can_printf(const char *message, uint8_t level) {
    struct uavcan_protocol_debug_LogMessage pkt = {0};

    uint32_t message_length = strlen(message);
    uint32_t copy_length = MIN(message_length, sizeof(pkt.text.data) - 1);
    pkt.level.value = level;
    strncpy((char *)pkt.text.data, message, copy_length);
    pkt.text.data[copy_length] = '\0';
    pkt.text.len = copy_length;
    uint16_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, msg_buffer);
    static uint8_t logmsg_transfer_id = 0;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                    &logmsg_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    msg_buffer,
                    len);
}

// float convertThrottle(int throttle, float max_speed, int middle_point) {
//     if (middle_point == 1) {
//         if (throttle < 0) {
//             throttle = 0;
//         }
//         return ((float)throttle - 4095.0f) * max_speed / 4095.0f;
//     } else {
//         return (float)throttle * max_speed / 8191.0f;
//     }
// }

uint32_t float_to_uint32(float value) {
    uint32_t result;
    memcpy(&result, &value, sizeof(float));
    return result;
}

void unpack_from_float(float packed_float, uint8_t *midle_point_value, uint16_t *esc_min_value, uint16_t *esc_max_value) {
    uint32_t packed_int = float_to_uint32(packed_float);
    *midle_point_value = (packed_int >> 31) & 0x01;
    *esc_min_value = (packed_int >> 21) & 0x03FF;
    *esc_max_value = (packed_int >> 8) & 0x1FFF;
}

#define ESC_MID 4095

float convertThrottle(int throttle, float max_speed, float middle_point) {
    // unpack_from_float(middle_point, &midle_point_value, &esc_min_value, &esc_max_value);

    float throttle_range;
    float normalized_throttle;

    float dir = (throttle < 0) ? -1.0f : 1.0f;
    throttle = abs(throttle);

    if (throttle < esc_min_value) {
        return 0.0f;
    }

    if (midle_point_value == 1) {
        if (throttle > ESC_MID) {
            throttle_range = (float)(esc_max_value - ESC_MID);
            normalized_throttle = (float)(throttle - ESC_MID) / throttle_range;
            if (dir == -1.0f) {
                dir = 1.0f;
            }

        } else {
            throttle_range = (float)(ESC_MID - esc_min_value);
            normalized_throttle = (float)(ESC_MID - throttle) / throttle_range;
            if (dir == 1.0f) {
                dir = -1.0f;
            }
            // else {
            //     dir = 1.0f;
            // }
        }
    } else {
        throttle_range = (float)(esc_max_value - esc_min_value);
        normalized_throttle = (float)(throttle - esc_min_value) / throttle_range;
    }

    return dir * (normalized_throttle * max_speed);
}

void readUniqueID(uint8_t *out_uid) {
    uint8_t i;
    for (i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++) {
        out_uid[i] = i;
    }
}

static void handle_RestartNode(CanardInstance *ins, CanardRxTransfer *transfer) {
    resetDevice(halHandle);
}

static void handle_param_Set(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    struct parameter *p = NULL;

    if (req.index < ARRAY_SIZE(parameters)) {
        p = &parameters[req.index];
        req_index = req.index;
        setter = 1;
        Flag_Update_Settings = true;
    }

    if (p != NULL && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        switch (p->type) {
            case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
                *p->value = req.value.integer_value;
                break;
            case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
                *p->value = req.value.boolean_value;
                break;
            case UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE:
                *p->value = req.value.real_value;
                break;
            default:
                return;
        }
    }
}

static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_protocol_GetNodeInfoResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = getMonotonicTimestampUSec() / 1000000ULL;
    pkt.status = node_status;
    pkt.software_version.major = APP_VERSION_MAJOR;
    pkt.software_version.minor = APP_VERSION_MINOR;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = GIT_HASH;
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    readUniqueID(pkt.hardware_version.unique_id);

    pkt.name.len = sizeof(APP_NODE_NAME) - 1;
    strcpy(pkt.name.data, APP_NODE_NAME);

    // pkt.name.len = strlen(str_node_name);
    // memcpy(pkt.name.data, str_node_name, pkt.name.len);

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, msg_buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           msg_buffer,
                           total_size);
}

static void handle_RawCommand(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_equipment_esc_RawCommand cmd;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &cmd)) {
        return;
    }
    if (cmd.cmd.len <= settings.esc_index) {
        return;
    }
    RAWcmd = cmd.cmd.data[(unsigned)settings.esc_index];
    esc.throttle = convertThrottle(RAWcmd, settings.max_speed, settings.midle_point);
    gMotorVars.SpeedRef_krpm = _IQ(esc.throttle);
    esc.last_update_us = getMonotonicTimestampUSec();
}

static void handle_ArmingStatus(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_equipment_safety_ArmingStatus cmd;
    if (uavcan_equipment_safety_ArmingStatus_decode(transfer, &cmd)) {
        return;
    }
    if (!settings.require_arming) {
        gMotorVars.Flag_enableSys = 1;
        gMotorVars.Flag_Run_Identify = 1;
        gMotorVars.Flag_enableUserParams = 1;
    }
    settings.require_arming = 0;
    // DroneCanActivityCounter++;
}

static void SaveProfile(void) {
    HAL_setupSpi_25AA02(halHandle);
    int pn = (int)EEPROM_getTargetProfile(halHandle);
    EEPROM_saveProfile(halHandle, pn, &gUserParams, &gMotorVars, &settings);
#ifdef DRV8301_SPI
    HAL_setupSpiA(halHandle);
#endif
}

static void SaveAndResetFlags() {
    SaveProfile();
    Flag_Latch_Save = 0;
}

static void CheckAndSaveProfile() {
    if (!Flag_Latch_Save) {
        return;
    }
    switch (currentState) {
        case PHASE_IDLE:
            gMotorVars.Flag_enableSys = 0;
            currentState = PHASE_SAVING;
            break;

        case PHASE_SAVING:
            if (gUserParams.motor_Rs != 0 && gUserParams.motor_Ls_d != 0) {
                SaveAndResetFlags();
                currentState = PHASE_IDLE;
                gMotorVars.Flag_enableSys = 1;
            }
            break;
        default:
            currentState = PHASE_IDLE;
            gMotorVars.Flag_enableSys = 1;
            break;
    }
}

static void handle_param_ExecuteOpcode(CanardInstance *ins, CanardRxTransfer *transfer) {
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        // here is where you would reset all parameters to defaults
        HAL_setupSpi_25AA02(halHandle);
        EEPROM_clear(halHandle);
#ifdef DRV8301_SPI
        HAL_setupSpiA(halHandle);
#endif
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        // here is where you would save all the changed parameters to permanent storage
        Flag_Latch_Save = 1;
    }

    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_MOTOR_ID) {
        Motor_Auto_ID = 1;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ARMING) {
        settings.require_arming = 1;
        Flag_Arming = 1;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_MOTOR_OFF) {
        gMotorVars.Flag_Run_Identify = false;
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, msg_buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           msg_buffer,
                           total_size);
}

void createStringFromParameter(const struct parameter *p, char *buffer, size_t buffer_size) {
    if (p == NULL || buffer == NULL || buffer_size == 0) {
        return;
    }

    size_t name_len = strlen(p->name);

    if (strcmp(p->name, "MIDLE_POINT") == 0) {
        unpack_from_float(settings.midle_point, &midle_point_value, &esc_min_value, &esc_max_value);
        if (name_len + 1 < buffer_size) {
            strncpy(buffer, p->name, buffer_size - 1);
            buffer[buffer_size - 1] = '\0';
            buffer[name_len] = ' ';
            size_t offset = name_len + 1;

            uintToStringInBuffer(midle_point_value, buffer + offset, buffer_size - offset);
            offset += strlen(buffer + offset);
            if (offset + 1 >= buffer_size) {
                buffer[0] = '\0';
                return;
            }
            buffer[offset++] = ' ';

            uint16ToStringInBuffer(esc_min_value, buffer + offset, buffer_size - offset);
            offset += strlen(buffer + offset);
            if (offset + 1 >= buffer_size) {
                buffer[0] = '\0';
                return;
            }
            buffer[offset++] = ' ';

            uint16ToStringInBuffer(esc_max_value, buffer + offset, buffer_size - offset);
        } else {
            buffer[0] = '\0';
        }
    } else {
        if (name_len + 1 < buffer_size) {
            strncpy(buffer, p->name, buffer_size - 1);
            buffer[buffer_size - 1] = '\0';

            buffer[name_len] = ' ';
            floatToStringInBuffer(buffer + name_len + 1, buffer_size - name_len - 1, *p->value, p->decimal);
        } else {
            buffer[0] = '\0';
        }
    }
}

static void ParameterToString(uint8_t index, char *buffer_str, size_t buffer_size) {
    struct parameter *p = NULL;
    if (index < ARRAY_SIZE(parameters)) {
        p = &parameters[index];
    }
    createStringFromParameter(p, buffer_str, buffer_size);
}

static void onTransferReceived(CanardInstance *ins,
                               CanardRxTransfer *transfer) {
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        switch (transfer->data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
                handle_GetNodeInfo(ins, transfer);
                break;
            }
            case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
                handle_param_Set(ins, transfer);
                if (setter) {
                    ParameterToString(req_index, buffer_str, sizeof(buffer_str));
                    can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO);
                    setter = 0;
                }
                break;
            }
            case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
                handle_param_ExecuteOpcode(ins, transfer);
                break;
            }
            case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
                handle_RestartNode(ins, transfer);
                break;
            }
        }
    }

    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        switch (transfer->data_type_id) {
            case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
                handle_RawCommand(ins, transfer);
                break;
            }
            case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
                handle_ArmingStatus(ins, transfer);
                break;
            }
        }
    }
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id) {
    if (transfer_type == CanardTransferTypeRequest) {
        switch (data_type_id) {
            case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
                return true;
            }
            case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
                return true;
            }
            case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
                return true;
            }
            case UAVCAN_PROTOCOL_RESTARTNODE_ID: {
                *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
                return true;
            }
        }
    }

    if (transfer_type == CanardTransferTypeBroadcast) {
        switch (data_type_id) {
            case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
                *out_data_type_signature = UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
                return true;
            }
            case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID: {
                *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
                return true;
            }
        }
    }
    return false;
}

static void send_ESCStatus(CanardInstance *ins) {
    struct uavcan_equipment_esc_Status pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.error_count = 0;
    pkt.voltage = _IQtoF(gMotorVars.VdcBus_kV) * 1000;
    pkt.current = _IQtoF(gMotorVars.Is_A);  // calcAvgCurrent();
    pkt.temperature = C_TO_KELVIN(measureTemperatureC());
    pkt.rpm = _IQtoF(gMotorVars.Speed_krpm) * 1000;
    pkt.power_rating_pct = (uint8_t)gMotorVars.CpuUsagePercentageAvg;
    pkt.esc_index = settings.esc_index;

    uint16_t len = uavcan_equipment_esc_Status_encode(&pkt, msg_buffer);
    static uint8_t transfer_id;
    canardBroadcast(ins,
                    UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    msg_buffer,
                    len);
}

static void send_NodeStatus(void) {
    node_status.uptime_sec = getMonotonicTimestampUSec() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    node_status.vendor_specific_status_code = 4343;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, msg_buffer);
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    msg_buffer,
                    len);
}

static void process1HzTasks(uint64_t timestamp_usec) {
    // canardCleanupStaleTransfers(&canard, timestamp_usec);
    // const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
    // peak_percent = 100U * stats.peak_usage_blocks / stats.capacity_blocks;
    send_NodeStatus();
}

static void processClearTasks(uint64_t timestamp_usec) {
    canardCleanupStaleTransfers(&canard, timestamp_usec);
    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
    peak_percent = 100U * stats.peak_usage_blocks / stats.capacity_blocks;
}

// void processTxRxOnce(int timeout_msec) {
//     // Transmitting
//     const CanardCANFrame *txf;

//     HAL_setupSpi_MCP2515(halHandle);
//     for (txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
//         uint64_t c;
//         for (c = 200LL * 32; c; c--) {  // 1000LL
//         }

//         const int tx_res = canardC2000Transmit(txf);

//         if (tx_res > 0) {  // Success - just drop the frame
//             canardPopTxQueue(&canard);
//         } else  // Timeout - just exit and try again later
//         {
//             break;
//         }
//     }

//     if (getRcvFlag(halHandle->mcp2515Handle)) {
//         // Receiving
//         CanardCANFrame rx_frame;
//         const uint64_t timestamp = getMonotonicTimestampUSec();
//         const int rx_res = canardC2000Receive(&rx_frame);

//         if (rx_res < 0) {  // Failure - report
//             error = 1;
//         } else if (rx_res > 0)  // Success - process the frame
//         {
//             canardHandleRxFrame(&canard, &rx_frame, timestamp);
//         } else {
//             ;  // Timeout - nothing to do
//         }
//         if (GPIO_read(halHandle->gpioHandle, halHandle->mcp2515Handle->gpio_INT)) {  // If MCP signals no more data in the buffer.
//             setRcvFlag(halHandle->mcp2515Handle, 0);
//             PIE_clearInt(((HAL_Obj *)halHandle)->pieHandle, PIE_GroupNumber_1);
//         } else {
//             setRcvFlag(halHandle->mcp2515Handle, 1);
//         }
//     }
//     HAL_setupSpiA(halHandle);
// }

void processTxRxOnce(int timeout_msec) {
    uint64_t start_time = getMonotonicTimestampUSec();
    uint64_t timeout_us = timeout_msec * 1000ULL;  // Convert timeout to microseconds

    // Transmitting
    const CanardCANFrame *txf;

    HAL_setupSpi_MCP2515(halHandle);
    for (txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        // Check if the delay time since the last transmission has elapsed
        if ((getMonotonicTimestampUSec() - dc.last_tx_delay_time) >= dc.tx_delay_duration_us) {
            // Update the last delay time
            dc.last_tx_delay_time = getMonotonicTimestampUSec();

            // Transmit the frame
            const int tx_res = canardC2000Transmit(txf);

            if (tx_res > 0) {  // Successful - remove the frame from the queue
                canardPopTxQueue(&canard);
            } else {  // Error - exit transmission, try again later
                break;
            }
        } else {
            // If the delay has not yet elapsed, exit the loop to avoid blocking
            break;
        }

        // Check the overall timeout
        if ((getMonotonicTimestampUSec() - start_time) >= timeout_us) {
            return;  // Time limit exceeded, exit the function
        }
    }

    // Check the data reception flag
    if (getRcvFlag(halHandle->mcp2515Handle)) {
        // Receiving
        CanardCANFrame rx_frame;
        const uint64_t timestamp = getMonotonicTimestampUSec();
        const int rx_res = canardC2000Receive(&rx_frame);

        if (rx_res < 0) {  // Error during reception - set the error flag
            dc.error = 1;
        } else if (rx_res > 0) {  // Successful reception - process the frame
            canardHandleRxFrame(&canard, &rx_frame, timestamp);
        }

        // Check if there is more data in the MCP2515
        if (GPIO_read(halHandle->gpioHandle, halHandle->mcp2515Handle->gpio_INT)) {
            // If there is no more data in the buffer
            setRcvFlag(halHandle->mcp2515Handle, 0);
            PIE_clearInt(((HAL_Obj *)halHandle)->pieHandle, PIE_GroupNumber_1);
        } else {
            // There is still data to read
            setRcvFlag(halHandle->mcp2515Handle, 1);
        }
    }
#ifdef DRV8301_SPI
    HAL_setupSpiA(halHandle);
#endif
}

void sendUpdateIfChanged(uint8_t currentValue, uint8_t *previousValue, const char *prefix, char *buffer_str, size_t buffer_size) {
    if (currentValue != *previousValue) {
        strncpy(buffer_str, prefix, buffer_size - 1);
        buffer_str[buffer_size - 1] = '\0';
        size_t prefix_len = strlen(buffer_str);
        uintToStringInBuffer(currentValue, buffer_str + prefix_len, buffer_size - prefix_len);
        can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG);
        *previousValue = currentValue;
    }
}

void sendFaultMessage(uint16_t currentValue, const char *prefix, char *buffer_str, size_t buffer_size) {
    strncpy(buffer_str, prefix, buffer_size - 1);
    buffer_str[buffer_size - 1] = '\0';
    size_t prefix_len = strlen(buffer_str);
    uint16ToStringInBuffer(currentValue, buffer_str + prefix_len, buffer_size - prefix_len);
    can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
}

void checkAndSendUpdates(char *buffer_str, size_t buffer_size) {
    sendUpdateIfChanged(gMotorVars.UserErrorCode, &previousUserErrorCode, str_UserErrorCode, buffer_str, buffer_size);
    sendUpdateIfChanged(gMotorVars.CtrlState, &previousCtrlState, str_CtrlState, buffer_str, buffer_size);
    sendUpdateIfChanged(gMotorVars.EstState, &previousEstState, str_EstState, buffer_str, buffer_size);
}

void SendArmingStatus(void) {
    if (Flag_Arming) {
        ParameterToString(2, buffer_str, sizeof(buffer_str));
        can_printf(buffer_str, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO);
        Flag_Arming = 0;
    }
}

void MotorAutoIdHandler(void) {
    static int phase = 0;
    if (Motor_Auto_ID) {
        switch (phase) {
            case 0:
                settings.require_arming = 1;
                gMotorVars.Flag_enableUserParams = false;
                gMotorVars.Flag_Run_Identify = false;
                gMotorVars.Flag_enableSys = false;
                phase = 1;
                break;
            case 1:
                gMotorVars.Flag_enableSys = true;
                gMotorVars.Flag_Run_Identify = true;
                phase = 2;
                break;
            case 2:
                if (!gMotorVars.Flag_Run_Identify) {
                    Motor_Auto_ID = false;
                    phase = 0;
                }
                break;
            default:
                phase = 0;
                break;
        }
    } else {
        phase = 0;
    }
}

void updateRAWcmdStatus(int16_t RAWcmd, uint64_t ts) {
    if (RAWcmd != 0) {
        lastNonZeroRAWcmdTime = ts;
    }
}

void canard_init() {
    canardInit(&canard,
               canard_memory_pool,
               sizeof(canard_memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);
    canardSetLocalNodeID(&canard, (uint8_t)settings.can_node);

    next_1hz_service_at = getMonotonicTimestampUSec();
    next_telem_service_at = getMonotonicTimestampUSec();
    next_DC_clear_service_at = getMonotonicTimestampUSec();
}

void canard_update() {
    processTxRxOnce(10);
    const uint64_t ts = getMonotonicTimestampUSec();

    if (ts >= next_1hz_service_at) {
        next_1hz_service_at += 1000000;
        // if (ts - lastNonZeroRAWcmdTime >= 1000000) {
        //     gMotorVars.Flag_Run_Identify = 0;
        // }

        // if (!settings.require_arming) {
        //     if (DroneCanActivityCounter == previousDroneCanActivityCounter) {
        //         gMotorVars.Flag_Run_Identify = 0;
        //     }
        // }
        // previousDroneCanActivityCounter = DroneCanActivityCounter;
        process1HzTasks(ts);
    }

    if (ts >= next_DC_clear_service_at) {
        next_DC_clear_service_at += 1000000 / 4;
        processClearTasks(ts);
    }

    if (Flag_nFaultDroneCan) {
        Flag_nFaultDroneCan = 0;

#ifdef DRV8305_SPI
        for (int i = 0; i < 4; i++) {
            if (fault[i] != 0) {
                sendFaultMessage(fault[i], str_DrvError[i], buffer_str, sizeof(buffer_str));
            }
            fault[i] = 0;
        }
#else
        can_printf("nFault", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
    }

#endif

        if (ts >= next_telem_service_at) {
            next_telem_service_at += 1000000ULL / settings.telem_rate;

            checkAndSendUpdates(buffer_str, sizeof(buffer_str));
            if (Flag_Update_Settings) {
                extractFlagsFromControlWord(&settings, &gMotorVars);
                SettingsToMotorVars(&settings, &gMotorVars, &gUserParams);
                Flag_Update_Settings = false;
            } else {
                settings.controll_word = updateControlWord(&gMotorVars);
                MotorVarsToSettings(&gMotorVars, &settings, &gUserParams);
            }
            send_ESCStatus(&canard);
        }
        if (Flag_Overheat) {
            can_printf("Overheat", UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_ERROR);
            Flag_Overheat = 0;
        }
        MotorAutoIdHandler();
        CheckAndSaveProfile();
        SendArmingStatus();
        unpack_from_float(settings.midle_point, &midle_point_value, &esc_min_value, &esc_max_value);
    }
