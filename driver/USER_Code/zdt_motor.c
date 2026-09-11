/* ZDT MODBUS V1.0.1 (2026-04-01), pp.7,10,12-14,22,34-35,38-39,44,53-54.
 * Register bytes are big endian, CRC is low byte first. X 32-bit values
 * exchange 16-bit words only. The manual's 0x6B examples are NOT RTU frames.
 */
#include "zdt_motor.h"
#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
#include "hcan.h"
#else
#include "usart.h"
#endif
#include "cmsis_os2.h"
#include <string.h>

#if ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_EMM || ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_X
#define ZDT_USING_MODBUS 1
#else
#define ZDT_USING_MODBUS 0
#endif

static ZdtMotorFault motor_fault;
static uint8_t fault_address, exception_code;
#if ZDT_USING_MODBUS
static uint8_t poll_address = 1U;
#endif
static uint8_t initialized;

#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
static const uint8_t wheel_addresses[4] = {
    ZDT_CAN_WHEEL_1_ADDRESS, ZDT_CAN_WHEEL_2_ADDRESS,
    ZDT_CAN_WHEEL_3_ADDRESS, ZDT_CAN_WHEEL_4_ADDRESS
};
static volatile uint8_t can_waiting, can_reply_ready, can_reply_valid;
static volatile uint8_t can_expected_address, can_expected_function;
static volatile uint8_t can_reply_status;
#endif

uint16_t ZdtMotor_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU, i;
    uint8_t bit;
    for(i = 0U; i < length; ++i) {
        crc ^= data[i];
        for(bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                               (uint16_t)(crc >> 1U);
    }
    return crc;
}

static void put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8U);
    out[1] = (uint8_t)value;
}

#if ZDT_USING_MODBUS
static uint16_t finish(uint8_t *out, uint16_t length)
{
    uint16_t crc = ZdtMotor_Crc16(out, length);
    out[length] = (uint8_t)crc;
    out[length + 1U] = (uint8_t)(crc >> 8U);
    return length + 2U;
}
#endif

uint16_t ZdtMotor_BuildSpeed(uint8_t *out, uint8_t address, int rpm, uint8_t sync)
{
    uint16_t magnitude;
    uint8_t direction;
    /* Clamp before negating: even INT_MIN is well defined. */
    if(rpm > (int)ZDT_MAX_RPM) rpm = (int)ZDT_MAX_RPM;
    if(rpm < -(int)ZDT_MAX_RPM) rpm = -(int)ZDT_MAX_RPM;
    direction = rpm < 0 ? 1U : 0U;
    magnitude = (uint16_t)(rpm < 0 ? -rpm : rpm);
    out[0] = address;
#if ZDT_MOTOR_PROTOCOL == ZDT_LEGACY_F6 || ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
    out[1] = 0xF6U;
    out[2] = direction;
    put16(out + 3, magnitude);
    out[5] = ZDT_EMM_ACCEL;
    out[6] = sync ? 1U : 0U;
    out[7] = 0x6BU;
    return 8U;
#else
    out[1] = 0x10U;
    out[2] = 0U;
#if ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_EMM
    out[3] = 0xF6U;
    put16(out + 4, 3U);
    out[6] = 6U;
    out[7] = direction;
    out[8] = ZDT_EMM_ACCEL;
    put16(out + 9, magnitude);
    out[11] = sync ? 1U : 0U;
    out[12] = 0U;
    return finish(out, 13U);
#else
    out[3] = 0xE6U;
    put16(out + 4, 4U);
    out[6] = 8U;
    put16(out + 7, direction);
    put16(out + 9, ZDT_X_ACCEL_RPM_S);
    put16(out + 11, (uint16_t)(magnitude * 10U));
    out[13] = sync ? 1U : 0U;
    out[14] = 0U;
    return finish(out, 15U);
#endif
#endif
}

uint16_t ZdtMotor_BuildStop(uint8_t *out, uint8_t address)
{
#if ZDT_MOTOR_PROTOCOL == ZDT_LEGACY_F6 || ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
    /* Emm free-protocol immediate stop: address, FE, 98, sync flag, 6B. */
    out[0] = address;
    out[1] = 0xFEU;
    out[2] = 0x98U;
    out[3] = 0U;
    out[4] = 0x6BU;
    return 5U;
#else
    out[0] = address; out[1] = 6U;
    put16(out + 2, 0xFEU); put16(out + 4, 0x9800U);
    return finish(out, 6U);
#endif
}

static uint16_t can_magnitude(int rpm, uint8_t *direction)
{
    if(rpm > (int)ZDT_MAX_RPM) rpm = (int)ZDT_MAX_RPM;
    if(rpm < -(int)ZDT_MAX_RPM) rpm = -(int)ZDT_MAX_RPM;
    *direction = rpm < 0 ? 1U : 0U;
    return (uint16_t)(rpm < 0 ? -rpm : rpm);
}

void ZdtMotor_BuildCanSpeed(ZdtCanFrame *out, uint8_t address, int rpm, uint8_t sync)
{
    uint8_t direction;
    uint16_t magnitude = can_magnitude(rpm, &direction);
    memset(out, 0, sizeof(*out));
    out->extended_id = (uint32_t)address << 8U;
    out->length = 7U;
    out->data[0] = 0xF6U;
    out->data[1] = direction;
    put16(out->data + 2, magnitude);
    out->data[4] = ZDT_EMM_ACCEL;
    out->data[5] = sync ? 1U : 0U;
    out->data[6] = 0x6BU;
}

void ZdtMotor_BuildCanStop(ZdtCanFrame *out, uint8_t address, uint8_t sync)
{
    memset(out, 0, sizeof(*out));
    out->extended_id = (uint32_t)address << 8U;
    out->length = 4U;
    out->data[0] = 0xFEU;
    out->data[1] = 0x98U;
    out->data[2] = sync ? 1U : 0U;
    out->data[3] = 0x6BU;
}

void ZdtMotor_BuildCanSync(ZdtCanFrame *out, uint8_t address)
{
    memset(out, 0, sizeof(*out));
    out->extended_id = (uint32_t)address << 8U;
    out->length = 3U;
    out->data[0] = 0xFFU;
    out->data[1] = 0x66U;
    out->data[2] = 0x6BU;
}

static void latch(ZdtMotorFault fault, uint8_t address)
{
    if(motor_fault == ZDT_FAULT_NONE) {
        motor_fault = fault;
        fault_address = address;
    }
}

#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
void ZdtMotor_OnCanFrame(uint32_t extended_id, uint8_t dlc, const uint8_t *data)
{
    uint8_t address = (uint8_t)(extended_id >> 8U);
    uint8_t packet = (uint8_t)extended_id;
    if(extended_id > 0xFFFFU) return; /* Emm uses one address and one packet byte. */
    if(address < ZDT_CAN_WHEEL_1_ADDRESS || address > ZDT_CAN_WHEEL_4_ADDRESS)
        return;
    if(!can_waiting || address != can_expected_address) return;
    can_reply_valid = 0U;
    if(packet == 0U && dlc == 3U && data != 0 &&
       data[0] == can_expected_function && data[2] == 0x6BU) {
        can_reply_status = data[1];
        can_reply_valid = 1U;
    }
    /* Publish last so the chassis task never observes a partial reply. */
    can_reply_ready = 1U;
}

static uint8_t can_send(const ZdtCanFrame *frame)
{
    if(!CAN_SendEXDataChecked(CAN_NUM, frame->extended_id,
                              frame->data, frame->length)) {
        latch(ZDT_FAULT_TX, (uint8_t)(frame->extended_id >> 8U));
        return 0U;
    }
    return 1U;
}

static uint8_t can_exchange(const ZdtCanFrame *frame)
{
    uint8_t address = (uint8_t)(frame->extended_id >> 8U);
    uint32_t start;
    can_expected_address = address;
    can_expected_function = frame->data[0];
    can_reply_status = 0U;
    can_reply_valid = 0U;
    can_reply_ready = 0U;
    can_waiting = 1U;
    if(!can_send(frame)) {
        can_waiting = 0U;
        return 0U;
    }
    start = HAL_GetTick();
    while(!can_reply_ready) {
        if((uint32_t)(HAL_GetTick() - start) >= ZDT_CAN_REPLY_TIMEOUT_MS) {
            can_waiting = 0U;
            latch(ZDT_FAULT_RX, address);
            return 0U;
        }
        osDelay(1U);
    }
    can_waiting = 0U;
    if(!can_reply_valid) {
        latch(ZDT_FAULT_REPLY, address);
        return 0U;
    }
    /* Emm free-protocol command status: 0x02 means accepted. Preserve any
     * other status byte for diagnosis and latch the link until reset. */
    if(can_reply_status != 0x02U) {
        exception_code = can_reply_status;
        latch(ZDT_FAULT_MOTOR, address);
        return 0U;
    }
    return 1U;
}
#else
void ZdtMotor_OnCanFrame(uint32_t extended_id, uint8_t dlc, const uint8_t *data)
{
    (void)extended_id; (void)dlc; (void)data;
}
#endif

#if ZDT_MOTOR_PROTOCOL != ZDT_EMM_CAN
static uint8_t transmit(uint8_t *frame, uint16_t length)
{
    /* 1 kHz RTOS: 3 ticks guarantee >=2ms even at a tick boundary.
     * Above 19200 baud RTU t3.5 is 1.75ms. Applies before EVERY request. */
#if ZDT_MOTOR_PROTOCOL == ZDT_LEGACY_F6
    osDelay(1U);
#else
    osDelay(3U);
#endif
    if(HAL_UART_Transmit(&huart4, frame, length, ZDT_TX_TIMEOUT_MS) != HAL_OK) {
        latch(ZDT_FAULT_TX, frame[0]);
        return 0U;
    }
    return 1U;
}
#endif

#if ZDT_USING_MODBUS
static uint16_t get16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

static void put32(uint8_t *out, uint32_t value)
{
#if ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_X
    put16(out, (uint16_t)value); put16(out + 2, (uint16_t)(value >> 16U));
#else
    put16(out, (uint16_t)(value >> 16U)); put16(out + 2, (uint16_t)value);
#endif
}

/* One bounded transaction. No retry of motion writes, no broadcast response.
 * RX uses the chassis task only, not the queue/UART interrupt receive task.
 * Any unexpected/late/partial reply faults instead of silently resyncing. */
static uint8_t exchange(uint8_t *request, uint16_t length, uint8_t *reply)
{
    uint32_t start, elapsed;
    uint16_t size = 3U, received = 0U, crc;
    if(!transmit(request, length)) return 0U;
    if(request[0] == 0U) return 1U;
    start = HAL_GetTick();
    while(received < size) {
        elapsed = (uint32_t)(HAL_GetTick() - start);
        if(elapsed >= ZDT_RX_TIMEOUT_MS ||
           HAL_UART_Receive(&huart4, reply + received, 1U,
                            ZDT_RX_TIMEOUT_MS - elapsed) != HAL_OK) {
            latch(ZDT_FAULT_RX, request[0]);
            return 0U;
        }
        ++received;
        if(received == 3U) {
            if(reply[1] == (uint8_t)(request[1] | 0x80U)) size = 5U;
            else if(request[1] == 4U && reply[1] == 4U) {
                if(reply[2] != (uint8_t)(get16(request + 4) * 2U) || reply[2] > 4U) {
                    latch(ZDT_FAULT_REPLY, request[0]); return 0U;
                }
                size = (uint16_t)(5U + reply[2]);
            } else size = 8U;
        }
    }
    crc = ZdtMotor_Crc16(reply, size - 2U);
    if(reply[0] != request[0] || reply[size - 2U] != (uint8_t)crc ||
       reply[size - 1U] != (uint8_t)(crc >> 8U)) {
        latch(ZDT_FAULT_REPLY, request[0]); return 0U;
    }
    if(reply[1] == (uint8_t)(request[1] | 0x80U)) {
        exception_code = reply[2];
        latch(ZDT_FAULT_EXCEPTION, request[0]); return 0U;
    }
    if(reply[1] != request[1] ||
       (request[1] != 4U && memcmp(reply, request, 6U) != 0)) {
        latch(ZDT_FAULT_REPLY, request[0]); return 0U;
    }
    return 1U;
}

static uint8_t write_regs(uint8_t address, uint16_t reg, const uint8_t *data, uint16_t count)
{
    uint8_t frame[17], reply[9];
    frame[0] = address; frame[1] = 0x10U;
    put16(frame + 2, reg); put16(frame + 4, count);
    frame[6] = (uint8_t)(count * 2U);
    memcpy(frame + 7, data, count * 2U);
    return exchange(frame, finish(frame, 7U + count * 2U), reply);
}

static uint8_t read_regs(uint8_t address, uint16_t reg, uint16_t count, uint8_t *reply)
{
    uint8_t frame[8];
    frame[0] = address; frame[1] = 4U; /* Also understood by older read implementations. */
    put16(frame + 2, reg); put16(frame + 4, count);
    return exchange(frame, finish(frame, 6U), reply);
}

static uint8_t write_single(uint8_t address, uint16_t reg, uint16_t value)
{
    uint8_t frame[8], reply[9];
    frame[0] = address; frame[1] = 6U;
    put16(frame + 2, reg); put16(frame + 4, value);
    return exchange(frame, finish(frame, 6U), reply);
}

static uint8_t check_options(uint8_t address, uint8_t check_scale)
{
    uint8_t reply[9];
    uint8_t firmware_bit = ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_EMM ? 2U : 0U;
    if(!read_regs(address, 0x1AU, 1U, reply)) return 0U;
    if((reply[4] & 2U) != firmware_bit || (reply[4] & 4U) == 0U ||
       (check_scale && firmware_bit && (reply[4] & 0x80U))) {
        latch(ZDT_FAULT_MOTOR, address); return 0U;
    }
    return 1U;
}

static uint8_t check_motor(uint8_t address)
{
    uint8_t reply[9];
    uint16_t reg = ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_EMM ? 0x3AU : 0x50U;
    if(!read_regs(address, reg, 1U, reply)) return 0U;
    /* bit0 enabled; bit2 stall; bit3 stall protection. bit1 is POSITION reached,
     * NOT proof that velocity mode has reached its setpoint. bit7 was cleared
     * at init; a new power loss invalidates volatile heartbeat/scale settings. */
    if(reply[3] != 0U || (reply[4] & 1U) == 0U || (reply[4] & 0x8CU) != 0U) {
        latch(ZDT_FAULT_MOTOR, address); return 0U;
    }
    return 1U;
}
#endif

static void best_effort_stop(void)
{
#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
    ZdtCanFrame can_frame;
    /* Address zero is the Emm broadcast address; broadcast commands do not
     * produce a reply. Send exactly once and never retry a motion frame. */
    ZdtMotor_BuildCanStop(&can_frame, 0U, 0U);
    (void)can_send(&can_frame);
#else
    uint8_t frame[17];
    uint16_t length;
#if ZDT_MOTOR_PROTOCOL == ZDT_LEGACY_F6
    uint8_t address;
    /* No legacy broadcast assumption: try every wheel even if one TX fails. */
    for(address = 1U; address <= 4U; ++address) {
        length = ZdtMotor_BuildStop(frame, address);
        (void)transmit(frame, length);
    }
#else
    length = ZdtMotor_BuildStop(frame, 0U);
    (void)transmit(frame, length);
#endif
#endif
}

void ZdtMotor_Init(void)
{
    if(initialized) return; /* An init call must not clear a latched fault. */
    initialized = 1U;
#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
    {
#if !ZDT_CAN_COMMISSIONED
        latch(ZDT_FAULT_NOT_COMMISSIONED, 0U);
        return;
#else
        uint8_t index;
        ZdtCanFrame can_frame;
        if(!CAN_Start(CAN_NUM)) {
            latch(ZDT_FAULT_TX, 0U);
            return;
        }
        /* A unicast stop/ACK proves every configured address before the
         * chassis task is allowed to stage any speed command. */
        for(index = 0U; index < 4U && motor_fault == ZDT_FAULT_NONE; ++index) {
            ZdtMotor_BuildCanStop(&can_frame, wheel_addresses[index], 0U);
            if(!can_exchange(&can_frame)) break;
        }
#endif
    }
#elif ZDT_USING_MODBUS
    {
        uint8_t address, reply[9], heartbeat[6] = {0x38U, 0U, 0U, 0U, 0U, 0U};
        const uint8_t enable[4] = {0xABU, 1U, 0U, 0U};
        if(!ZDT_MODBUS_COMMISSIONED) {
            latch(ZDT_FAULT_NOT_COMMISSIONED, 0U);
            return; /* Do not send a different protocol to an unverified bus. */
        }
        best_effort_stop();
        put32(heartbeat + 2, ZDT_HEARTBEAT_MS);
        for(address = 1U; address <= 4U && motor_fault == ZDT_FAULT_NONE; ++address) {
            if(!check_options(address, 0U) ||
               !write_regs(address, 0x68U, heartbeat, 3U) ||
               !read_regs(address, 0x16U, 2U, reply)) break;
            if(memcmp(reply + 3, heartbeat + 2, 4U) != 0) {
                latch(ZDT_FAULT_REPLY, address); break;
            }
#if ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_EMM
            {
                /* Volatile only: force whole-RPM inputs (manual p.44). */
                const uint8_t scale[4] = {0x71U, 0U, 0U, 0U};
                if(!write_regs(address, 0x4FU, scale, 2U)) break;
            }
#endif
            if(!check_options(address, 1U) ||
               !write_single(address, ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_EMM ? 0x50U : 0xDFU, 0U) ||
               !write_regs(address, ZDT_MOTOR_PROTOCOL == ZDT_MODBUS_EMM ? 0xF3U : 0xE0U,
                           enable, 2U) || !check_motor(address)) break;
        }
    }
#else
    best_effort_stop();
#endif
#if ZDT_MOTOR_PROTOCOL != ZDT_EMM_CAN || ZDT_CAN_COMMISSIONED
    if(motor_fault != ZDT_FAULT_NONE) best_effort_stop();
#endif
}

uint8_t ZdtMotor_SetSpeeds(const int rpm[4], uint8_t (*motion_valid)(void))
{
#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
    uint8_t index;
    ZdtCanFrame can_frame;
#else
    uint8_t address, frame[17];
    uint16_t length;
#endif
    if(!initialized || motor_fault != ZDT_FAULT_NONE) return 0U;
#if ZDT_MOTOR_PROTOCOL == ZDT_EMM_CAN
    if(rpm[0] == 0 && rpm[1] == 0 && rpm[2] == 0 && rpm[3] == 0) {
        best_effort_stop();
        return motor_fault == ZDT_FAULT_NONE;
    }
    for(index = 0U; index < 4U; ++index) {
        if(motion_valid && !motion_valid()) goto cancelled;
        ZdtMotor_BuildCanSpeed(&can_frame, wheel_addresses[index], rpm[index], 1U);
        if(!can_exchange(&can_frame)) goto fail;
    }
    /* Only commit after all four nodes accepted their staged setpoints. */
    if(motion_valid && !motion_valid()) goto cancelled;
    ZdtMotor_BuildCanSync(&can_frame, 0U);
    if(!can_send(&can_frame)) goto fail;
#elif ZDT_USING_MODBUS
    {
        uint8_t reply[9];
        uint8_t stopped = rpm[0] == 0 && rpm[1] == 0 && rpm[2] == 0 && rpm[3] == 0;
        if(stopped) {
            best_effort_stop(); /* Brake before waiting for a status reply. */
            if(motor_fault != ZDT_FAULT_NONE) return 0U;
        }
        if(!check_motor(poll_address)) goto fail;
        poll_address = poll_address == 4U ? 1U : poll_address + 1U;
        if(stopped) return 1U;
        for(address = 1U; address <= 4U; ++address) {
            if(motion_valid && !motion_valid()) goto cancelled;
            length = ZdtMotor_BuildSpeed(frame, address, rpm[address - 1U], 1U);
            if(!exchange(frame, length, reply)) goto fail;
        }
        /* All four unicast ACKs are required before the broadcast commit. */
        if(motion_valid && !motion_valid()) goto cancelled;
        frame[0] = 0U; frame[1] = 6U;
        put16(frame + 2, 0xFFU); put16(frame + 4, 0x6600U);
        if(!transmit(frame, finish(frame, 6U))) goto fail;
    }
#else
    for(address = 1U; address <= 4U; ++address) {
        if(motion_valid && !motion_valid()) goto cancelled;
        length = ZdtMotor_BuildSpeed(frame, address, rpm[address - 1U], 0U);
        if(!transmit(frame, length)) goto fail;
    }
#endif
    return 1U;
cancelled:
    best_effort_stop();
    return motor_fault == ZDT_FAULT_NONE;
fail:
    best_effort_stop();
    /* Stop all traffic after this attempt: a broken link must allow the
     * already configured driver heartbeat to expire. Never auto-enable. */
    return 0U;
}

ZdtMotorFault ZdtMotor_GetFault(void) { return motor_fault; }
uint8_t ZdtMotor_GetFaultAddress(void) { return fault_address; }
uint8_t ZdtMotor_GetException(void) { return exception_code; }
