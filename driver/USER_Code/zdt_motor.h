#ifndef ZDT_MOTOR_H
#define ZDT_MOTOR_H

#include <stdint.h>

#define ZDT_LEGACY_F6 0
#define ZDT_MODBUS_EMM 1
#define ZDT_MODBUS_X 2
#define ZDT_EMM_CAN 3

/* Keep the installed bus unchanged until the commissioning checklist is done.
 * MODBUS profiles below target X42S/Y42 with heartbeat protection (not X42).
 * All four nodes must use the same firmware and unique addresses.
 * See ../ZDT电机接入与联调.md before changing either switch. */
#ifndef ZDT_MOTOR_PROTOCOL
#define ZDT_MOTOR_PROTOCOL ZDT_EMM_CAN
#endif
#ifndef ZDT_MODBUS_COMMISSIONED
#define ZDT_MODBUS_COMMISSIONED 0
#endif
#ifndef ZDT_CAN_COMMISSIONED
#define ZDT_CAN_COMMISSIONED 0
#endif
#ifndef ZDT_MAX_RPM
#define ZDT_MAX_RPM 3000
#endif
#ifndef ZDT_EMM_ACCEL
#define ZDT_EMM_ACCEL 0U /* Preserve legacy direct-speed behavior; 1..255 are gears. */
#endif
#ifndef ZDT_X_ACCEL_RPM_S
#define ZDT_X_ACCEL_RPM_S 1000U
#endif
#define ZDT_HEARTBEAT_MS 300U
#define ZDT_TX_TIMEOUT_MS 5U
#define ZDT_RX_TIMEOUT_MS 10U
#define ZDT_CAN_REPLY_TIMEOUT_MS 10U

/* Physical mapping reported on 2026-09-01 (addresses 6..8 independently
 * read back; address 5 still needs its individual TTL readback):
 * SpeedTarget[0] = right-front / address 5
 * SpeedTarget[1] = left-front  / address 6
 * SpeedTarget[2] = right-rear  / address 7
 * SpeedTarget[3] = left-rear   / address 8
 * Addresses 3 and 4 would collide with the existing tower extended IDs
 * 0x300 and 0x400. Wheel direction signs still require a single-wheel test. */
#define ZDT_CAN_WHEEL_1_ADDRESS 5U
#define ZDT_CAN_WHEEL_2_ADDRESS 6U
#define ZDT_CAN_WHEEL_3_ADDRESS 7U
#define ZDT_CAN_WHEEL_4_ADDRESS 8U

#if ZDT_MOTOR_PROTOCOL < ZDT_LEGACY_F6 || ZDT_MOTOR_PROTOCOL > ZDT_EMM_CAN
#error Invalid ZDT_MOTOR_PROTOCOL
#endif
#if ZDT_MAX_RPM < 1 || ZDT_MAX_RPM > 3000
#error ZDT_MAX_RPM must be in 1..3000
#endif
#if ZDT_EMM_ACCEL > 255 || ZDT_X_ACCEL_RPM_S > 65535
#error Invalid ZDT acceleration
#endif

typedef enum {
    ZDT_FAULT_NONE = 0,
    ZDT_FAULT_NOT_COMMISSIONED,
    ZDT_FAULT_TX,
    ZDT_FAULT_RX,
    ZDT_FAULT_REPLY,
    ZDT_FAULT_EXCEPTION,
    ZDT_FAULT_MOTOR
} ZdtMotorFault;

typedef struct {
    uint32_t extended_id;
    uint8_t length;
    uint8_t data[8];
} ZdtCanFrame;

/* Called once, in the chassis task after power settling. No auto recovery. */
void ZdtMotor_Init(void);
/* Signed whole RPM, in the existing wheel order. Single task owns UART4.
 * Returns 0 on latched failure. A successful write is NOT motion feedback. */
/* Optional guard is rechecked while staging and before commit. False cancels
 * this batch with a stop, without creating a motor fault. */
uint8_t ZdtMotor_SetSpeeds(const int rpm[4], uint8_t (*motion_valid)(void));
ZdtMotorFault ZdtMotor_GetFault(void);
uint8_t ZdtMotor_GetFaultAddress(void);
uint8_t ZdtMotor_GetException(void);

/* Wire helpers also used by host tests. Buffers need 17 / 8 bytes respectively. */
uint16_t ZdtMotor_Crc16(const uint8_t *data, uint16_t length);
uint16_t ZdtMotor_BuildSpeed(uint8_t *out, uint8_t address, int rpm, uint8_t sync);
uint16_t ZdtMotor_BuildStop(uint8_t *out, uint8_t address);

/* Emm classic-CAN helpers. The address is encoded in extended_id, not data. */
void ZdtMotor_BuildCanSpeed(ZdtCanFrame *out, uint8_t address, int rpm, uint8_t sync);
void ZdtMotor_BuildCanStop(ZdtCanFrame *out, uint8_t address, uint8_t sync);
void ZdtMotor_BuildCanSync(ZdtCanFrame *out, uint8_t address);

/* Feed extended CAN data frames from the CAN1 FIFO0 ISR. Other IDs are
 * ignored, so the tower and wheel protocols can share CAN1 safely. */
void ZdtMotor_OnCanFrame(uint32_t extended_id, uint8_t dlc, const uint8_t *data);

#endif
