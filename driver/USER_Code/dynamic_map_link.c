#include "dynamic_map_link.h"
#include "usart.h"
#include "chassis_control.h"

#define MAP_HEAD_0                 0xA5U
#define MAP_HEAD_1                 0x5AU
#define MAP_MESSAGE_VELOCITY       0x02U
#define MAP_MESSAGE_STOP           0x03U
#define MAP_MESSAGE_ACK            0x80U
#define MAP_MESSAGE_NACK           0x81U
#define MAP_RX_RING_SIZE           256U
#define MAP_MAX_PAYLOAD            16U
#define MAP_MIN_WATCHDOG_MS        50U
#define MAP_MAX_WATCHDOG_MS        1000U
#define MAP_FRAME_TIMEOUT_MS        100U

typedef enum
{
    MAP_PARSE_HEAD_0 = 0U,
    MAP_PARSE_HEAD_1,
    MAP_PARSE_VERSION,
    MAP_PARSE_TYPE,
    MAP_PARSE_SEQUENCE_LO,
    MAP_PARSE_SEQUENCE_HI,
    MAP_PARSE_LENGTH_LO,
    MAP_PARSE_LENGTH_HI,
    MAP_PARSE_PAYLOAD,
    MAP_PARSE_CRC_LO,
    MAP_PARSE_CRC_HI
} MapParserState;

typedef struct
{
    MapParserState state;
    uint8_t version;
    uint8_t type;
    uint16_t sequence;
    uint16_t length;
    uint16_t index;
    uint16_t crc;
    uint16_t received_crc;
    uint8_t payload[MAP_MAX_PAYLOAD];
} MapParser;

static volatile uint8_t map_rx_ring[MAP_RX_RING_SIZE];
static volatile uint16_t map_rx_head = 0U;
static volatile uint16_t map_rx_tail = 0U;
static volatile uint8_t map_rx_overflow = 0U;
static MapParser map_parser;
static int16_t map_velocity_x = 0;
static int16_t map_velocity_y = 0;
static int16_t map_velocity_z = 0;
static uint16_t map_watchdog_ms = DYNAMIC_MAP_WATCHDOG_DEFAULT;
static uint32_t map_last_velocity_tick = 0U;
static uint16_t map_last_sequence = 0U;
static uint8_t map_has_sequence = 0U;
static uint8_t map_velocity_enabled = 0U;
static uint8_t map_fault_latched = 1U; /* A fresh STOP establishes each session. */
static uint32_t map_last_rx_byte_tick = 0U;

extern uint8_t control_mode;
static uint8_t map_motor_fault_latched;

#if DYNAMIC_MAP_LINK_UART5
static void DynamicMap_QueueByte(uint8_t value)
{
    uint16_t next = (uint16_t)((map_rx_head + 1U) % MAP_RX_RING_SIZE);
    if(next == map_rx_tail)
    {
        map_rx_overflow = 1U;
    }
    else
    {
        map_rx_ring[map_rx_head] = value;
        map_rx_head = next;
    }
}
#endif

static uint16_t DynamicMap_CrcUpdate(uint16_t crc, uint8_t value)
{
    uint8_t bit;
    crc ^= (uint16_t)value << 8U;
    for(bit = 0U; bit < 8U; bit++)
    {
        crc = ((crc & 0x8000U) != 0U) ?
            (uint16_t)((crc << 1U) ^ 0x1021U) : (uint16_t)(crc << 1U);
    }
    return crc;
}

static uint16_t DynamicMap_ReadU16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int16_t DynamicMap_ReadI16(const uint8_t *data)
{
    return (int16_t)DynamicMap_ReadU16(data);
}

static void DynamicMap_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void DynamicMap_ResetParser(void)
{
    map_parser.state = MAP_PARSE_HEAD_0;
    map_parser.length = 0U;
    map_parser.index = 0U;
    map_parser.crc = 0xFFFFU;
}

static void DynamicMap_SendResponse(uint8_t type, uint8_t code)
{
    uint8_t frame[12];
    uint16_t crc = 0xFFFFU;
    uint16_t index = 0U;
    uint16_t cursor;

    frame[index++] = MAP_HEAD_0;
    frame[index++] = MAP_HEAD_1;
    frame[index++] = DYNAMIC_MAP_PROTOCOL_VERSION;
    frame[index++] = type;
    DynamicMap_WriteU16(&frame[index], map_parser.sequence);
    index += 2U;
    DynamicMap_WriteU16(&frame[index], 2U);
    index += 2U;
    frame[index++] = code;
    frame[index++] = map_fault_latched ? 5U : (map_velocity_enabled ? 2U : 0U);

    for(cursor = 2U; cursor < index; cursor++)
    {
        crc = DynamicMap_CrcUpdate(crc, frame[cursor]);
    }
    DynamicMap_WriteU16(&frame[index], crc);
    index += 2U;
#if DYNAMIC_MAP_LINK_UART5
    (void)HAL_UART_Transmit(&huart5, frame, index, 20U);
#else
    (void)HAL_UART_Transmit(&huart6, frame, index, 20U);
#endif
}

static void DynamicMap_Stop(void)
{
    map_velocity_x = 0;
    map_velocity_y = 0;
    map_velocity_z = 0;
    map_velocity_enabled = 0U;
    control_mode = 2U;
    SpeedTarget_stop();
}

static void DynamicMap_Fault(void)
{
    DynamicMap_Stop();
    map_fault_latched = 1U;
}

void DynamicMap_Link_ReportMotorFault(void)
{
    map_motor_fault_latched = 1U;
    DynamicMap_Fault();
}

static void DynamicMap_Reject(uint8_t code)
{
    DynamicMap_Fault();
    DynamicMap_SendResponse(MAP_MESSAGE_NACK, code);
}

static void DynamicMap_HandleFrame(void)
{
    uint8_t duplicate = ((map_has_sequence != 0U) &&
                         (map_parser.sequence == map_last_sequence)) ? 1U : 0U;

    if(map_parser.received_crc != map_parser.crc)
    {
        DynamicMap_Reject(1U);
        return;
    }
    if(map_parser.version != DYNAMIC_MAP_PROTOCOL_VERSION)
    {
        DynamicMap_Reject(2U);
        return;
    }

    if(map_parser.type == MAP_MESSAGE_VELOCITY)
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint16_t watchdog;

        if(map_parser.length != 8U)
        {
            DynamicMap_Reject(4U);
            return;
        }
        x = DynamicMap_ReadI16(&map_parser.payload[0]);
        y = DynamicMap_ReadI16(&map_parser.payload[2]);
        z = DynamicMap_ReadI16(&map_parser.payload[4]);
        watchdog = DynamicMap_ReadU16(&map_parser.payload[6]);
        if((x < -100) || (x > 100) ||
           (y < -100) || (y > 100) ||
           (z < -100) || (z > 100) ||
           (watchdog < MAP_MIN_WATCHDOG_MS) ||
           (watchdog > MAP_MAX_WATCHDOG_MS))
        {
            DynamicMap_Reject(5U);
            return;
        }

        if(map_fault_latched != 0U || map_has_sequence == 0U)
        {
            DynamicMap_Reject(6U);
            return;
        }
        if(duplicate != 0U && map_velocity_enabled != 0U &&
           x == map_velocity_x && y == map_velocity_y && z == map_velocity_z &&
           watchdog == map_watchdog_ms)
        {
            /* Idempotent ACK only: replay cannot refresh the watchdog. */
            DynamicMap_SendResponse(MAP_MESSAGE_ACK, 1U);
            return;
        }
        if(duplicate != 0U || (uint16_t)(map_parser.sequence - map_last_sequence) >= 0x8000U)
        {
            DynamicMap_Reject(6U);
            return;
        }

        map_velocity_x = x;
        map_velocity_y = y;
        map_velocity_z = z;
        map_watchdog_ms = watchdog;
        map_last_velocity_tick = HAL_GetTick();
        map_velocity_enabled = 1U;
        control_mode = 2U;
        map_last_sequence = map_parser.sequence;
        map_has_sequence = 1U;
        DynamicMap_SendResponse(MAP_MESSAGE_ACK, duplicate);
    }
    else if(map_parser.type == MAP_MESSAGE_STOP)
    {
        if(map_parser.length != 0U)
        {
            DynamicMap_Reject(4U);
            return;
        }
        DynamicMap_Stop();
        map_fault_latched = map_motor_fault_latched;
        map_last_sequence = map_parser.sequence;
        map_has_sequence = 1U;
        DynamicMap_SendResponse(MAP_MESSAGE_ACK, duplicate);
    }
    else
    {
        DynamicMap_Reject(3U);
    }
}

static void DynamicMap_ProcessByte(uint8_t value)
{
    uint32_t now = HAL_GetTick();
    if((map_parser.state != MAP_PARSE_HEAD_0) &&
       ((uint32_t)(now - map_last_rx_byte_tick) > MAP_FRAME_TIMEOUT_MS))
    {
        DynamicMap_ResetParser();
    }
    map_last_rx_byte_tick = now;
    switch(map_parser.state)
    {
        case MAP_PARSE_HEAD_0:
            if(value == MAP_HEAD_0) map_parser.state = MAP_PARSE_HEAD_1;
            break;
        case MAP_PARSE_HEAD_1:
            map_parser.state = (value == MAP_HEAD_1) ?
                MAP_PARSE_VERSION : ((value == MAP_HEAD_0) ?
                MAP_PARSE_HEAD_1 : MAP_PARSE_HEAD_0);
            break;
        case MAP_PARSE_VERSION:
            map_parser.version = value;
            map_parser.crc = DynamicMap_CrcUpdate(0xFFFFU, value);
            map_parser.state = MAP_PARSE_TYPE;
            break;
        case MAP_PARSE_TYPE:
            map_parser.type = value;
            map_parser.crc = DynamicMap_CrcUpdate(map_parser.crc, value);
            map_parser.state = MAP_PARSE_SEQUENCE_LO;
            break;
        case MAP_PARSE_SEQUENCE_LO:
            map_parser.sequence = value;
            map_parser.crc = DynamicMap_CrcUpdate(map_parser.crc, value);
            map_parser.state = MAP_PARSE_SEQUENCE_HI;
            break;
        case MAP_PARSE_SEQUENCE_HI:
            map_parser.sequence |= (uint16_t)value << 8U;
            map_parser.crc = DynamicMap_CrcUpdate(map_parser.crc, value);
            map_parser.state = MAP_PARSE_LENGTH_LO;
            break;
        case MAP_PARSE_LENGTH_LO:
            map_parser.length = value;
            map_parser.crc = DynamicMap_CrcUpdate(map_parser.crc, value);
            map_parser.state = MAP_PARSE_LENGTH_HI;
            break;
        case MAP_PARSE_LENGTH_HI:
            map_parser.length |= (uint16_t)value << 8U;
            map_parser.crc = DynamicMap_CrcUpdate(map_parser.crc, value);
            map_parser.index = 0U;
            if(map_parser.length > MAP_MAX_PAYLOAD)
            {
                DynamicMap_Reject(4U);
                DynamicMap_ResetParser();
            }
            else
            {
                map_parser.state = (map_parser.length == 0U) ?
                    MAP_PARSE_CRC_LO : MAP_PARSE_PAYLOAD;
            }
            break;
        case MAP_PARSE_PAYLOAD:
            map_parser.payload[map_parser.index++] = value;
            map_parser.crc = DynamicMap_CrcUpdate(map_parser.crc, value);
            if(map_parser.index >= map_parser.length)
            {
                map_parser.state = MAP_PARSE_CRC_LO;
            }
            break;
        case MAP_PARSE_CRC_LO:
            map_parser.received_crc = value;
            map_parser.state = MAP_PARSE_CRC_HI;
            break;
        case MAP_PARSE_CRC_HI:
            map_parser.received_crc |= (uint16_t)value << 8U;
            DynamicMap_HandleFrame();
            DynamicMap_ResetParser();
            break;
        default:
            DynamicMap_ResetParser();
            break;
    }
}

void DynamicMap_Link_IRQHandler(void)
{
    if((USART6->SR & USART_SR_RXNE) != 0U)
    {
        uint8_t value = (uint8_t)USART6->DR;
        uint16_t next = (uint16_t)((map_rx_head + 1U) % MAP_RX_RING_SIZE);
        if(next == map_rx_tail)
        {
            map_rx_overflow = 1U;
        }
        else
        {
            map_rx_ring[map_rx_head] = value;
            map_rx_head = next;
        }
    }
}

#if DYNAMIC_MAP_LINK_UART5
void DynamicMap_Link_ReceiveBytes(const uint8_t *data, uint16_t length)
{
    uint16_t index;
    if(data == NULL)
    {
        return;
    }
    for(index = 0U; index < length; index++)
    {
        DynamicMap_QueueByte(data[index]);
    }
}
#endif

void DynamicMap_Link_Init(void)
{
    map_rx_head = 0U;
    map_rx_tail = 0U;
    map_rx_overflow = 0U;
    map_velocity_enabled = 0U;
    map_fault_latched = 1U;
    map_has_sequence = 0U;
    map_last_rx_byte_tick = HAL_GetTick();
    DynamicMap_ResetParser();
#if !DYNAMIC_MAP_LINK_UART5
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
#endif
}

void DynamicMap_Link_Process(void)
{
    /* Check before draining: a queued command must not hide an expired lease. */
    if((map_velocity_enabled != 0U) &&
       ((uint32_t)(HAL_GetTick() - map_last_velocity_tick) >= map_watchdog_ms))
    {
        DynamicMap_Fault();
    }
    if((map_parser.state != MAP_PARSE_HEAD_0) &&
       ((uint32_t)(HAL_GetTick() - map_last_rx_byte_tick) > MAP_FRAME_TIMEOUT_MS))
    {
        DynamicMap_Fault();
        DynamicMap_ResetParser();
    }
    if(map_rx_overflow != 0U)
    {
        map_rx_overflow = 0U;
        map_rx_tail = map_rx_head;
        DynamicMap_ResetParser();
        DynamicMap_Fault();
    }
    while(map_rx_tail != map_rx_head)
    {
        uint8_t value = map_rx_ring[map_rx_tail];
        map_rx_tail = (uint16_t)((map_rx_tail + 1U) % MAP_RX_RING_SIZE);
        DynamicMap_ProcessByte(value);
    }
    if((map_velocity_enabled != 0U) &&
       ((uint32_t)(HAL_GetTick() - map_last_velocity_tick) >= map_watchdog_ms))
    {
        DynamicMap_Fault();
    }
}

uint8_t DynamicMap_Link_GetVelocity(int16_t *x, int16_t *y, int16_t *z)
{
    if((x == NULL) || (y == NULL) || (z == NULL) ||
       (map_velocity_enabled == 0U))
    {
        return 0U;
    }
    *x = map_velocity_x;
    *y = map_velocity_y;
    *z = map_velocity_z;
    return 1U;
}
