#include "auto_aim_link.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "main.h"
#include "task.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define AUTO_AIM_FRAME_CAPACITY     96U
#define AUTO_AIM_TIMEOUT_MS         100U
#define AUTO_AIM_TASK_PERIOD_MS     10U
#define AUTO_AIM_TASK_STACK_WORDS   256U
#define AUTO_AIM_MAX_ABS_ANGLE_DEG  360.0F
#define AUTO_AIM_RX_RING_CAPACITY   256U

auto_aim_status_t g_auto_aim_status;

static char frame_buffer[AUTO_AIM_FRAME_CAPACITY];
static uint32_t frame_length;
static bool collecting;
static bool has_sequence;
static uint32_t previous_sequence;
static uint8_t rx_ring[AUTO_AIM_RX_RING_CAPACITY];
static volatile uint32_t rx_head;
static volatile uint32_t rx_tail;
static osThreadId_t task_handle;
static StaticTask_t task_control;
static StackType_t task_stack[AUTO_AIM_TASK_STACK_WORDS];

uint16_t auto_aim_crc16_modbus(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t index;
    uint32_t bit;

    if (data == NULL)
        return 0U;
    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
            crc = (crc & 1U) ? (uint16_t)((crc >> 1U) ^ 0xA001U) :
                               (uint16_t)(crc >> 1U);
    }
    return crc;
}

static bool parse_hex4(const char *text, uint16_t *value)
{
    uint16_t parsed = 0U;
    uint32_t index;

    if ((text == NULL) || (value == NULL))
        return false;
    for (index = 0U; index < 4U; ++index)
    {
        uint8_t digit;
        const char c = text[index];

        if ((c >= '0') && (c <= '9'))
            digit = (uint8_t)(c - '0');
        else if ((c >= 'a') && (c <= 'f'))
            digit = (uint8_t)(c - 'a' + 10);
        else if ((c >= 'A') && (c <= 'F'))
            digit = (uint8_t)(c - 'A' + 10);
        else
            return false;
        parsed = (uint16_t)((parsed << 4U) | digit);
    }
    *value = parsed;
    return true;
}

static bool parse_frame(char *frame, uint32_t length)
{
    char *pitch_end;
    char *yaw_end;
    char *fire_end;
    char *sequence_end;
    char *crc_marker;
    float pitch_deg;
    float yaw_deg;
    unsigned long fire;
    unsigned long sequence;
    uint16_t received_crc;
    uint16_t calculated_crc;
    uint32_t crc_offset;

    if ((frame == NULL) || (length < 18U) ||
        (frame[0] != 'A') || (frame[1] != 'A') || (frame[2] != 'P') ||
        (frame[length - 1U] != '%'))
        return false;

    frame[length] = '\0';
    pitch_deg = strtof(&frame[3], &pitch_end);
    if ((pitch_end == &frame[3]) || (*pitch_end != 'Y'))
        return false;
    yaw_deg = strtof(pitch_end + 1, &yaw_end);
    if ((yaw_end == pitch_end + 1) || (*yaw_end != 'F'))
        return false;
    fire = strtoul(yaw_end + 1, &fire_end, 10);
    if ((fire_end == yaw_end + 1) || (*fire_end != 'N') || (fire > 1UL))
        return false;
    if ((fire_end[1] < '0') || (fire_end[1] > '9'))
        return false;
    sequence = strtoul(fire_end + 1, &sequence_end, 10);
    if ((sequence_end == fire_end + 1) || (*sequence_end != 'C'))
        return false;
    crc_marker = sequence_end;
    if (((uint32_t)(crc_marker - frame) + 6U) != length ||
        !parse_hex4(crc_marker + 1, &received_crc) ||
        (crc_marker[5] != '%'))
        return false;
    if (!isfinite(pitch_deg) || !isfinite(yaw_deg) ||
        (fabsf(pitch_deg) > AUTO_AIM_MAX_ABS_ANGLE_DEG) ||
        (fabsf(yaw_deg) > AUTO_AIM_MAX_ABS_ANGLE_DEG))
        return false;

    crc_offset = (uint32_t)(crc_marker - frame);
    calculated_crc = auto_aim_crc16_modbus((const uint8_t *)frame, crc_offset);
    if (calculated_crc != received_crc)
    {
        g_auto_aim_status.crc_error_count++;
        return true;
    }

    if (has_sequence && ((uint32_t)sequence != previous_sequence + 1U))
        g_auto_aim_status.sequence_gap_count++;
    has_sequence = true;
    previous_sequence = (uint32_t)sequence;

    g_auto_aim_status.pitch_deg = pitch_deg;
    g_auto_aim_status.yaw_deg = yaw_deg;
    g_auto_aim_status.fire_requested = (fire != 0UL);
    g_auto_aim_status.target_present_heuristic =
        (fire != 0UL) || (fabsf(pitch_deg) > 0.0001F) ||
        (fabsf(yaw_deg) > 0.0001F);
    g_auto_aim_status.frame_number = (uint32_t)sequence;
    g_auto_aim_status.last_update_tick = HAL_GetTick();
    g_auto_aim_status.valid_frame_count++;
    g_auto_aim_status.frame_valid = true;
    g_auto_aim_status.online = true;
    return true;
}

static void finish_frame(void)
{
    g_auto_aim_status.frame_valid = false;
    if (!parse_frame(frame_buffer, frame_length))
        g_auto_aim_status.format_error_count++;
    frame_length = 0U;
    collecting = false;
}

static void process_bytes(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == NULL)
        return;
    for (index = 0U; index < length; ++index)
    {
        const char c = (char)data[index];

        if (!collecting)
        {
            if (c == 'A')
            {
                collecting = true;
                frame_length = 1U;
                frame_buffer[0] = c;
            }
            continue;
        }
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            finish_frame();
            continue;
        }
        if (frame_length >= AUTO_AIM_FRAME_CAPACITY - 1U)
        {
            g_auto_aim_status.overflow_count++;
            frame_length = 0U;
            collecting = false;
            continue;
        }
        frame_buffer[frame_length++] = c;
    }
}

void auto_aim_link_feed(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    if (data == NULL)
        return;
    for (index = 0U; index < length; ++index)
    {
        const uint32_t next = (rx_head + 1U) % AUTO_AIM_RX_RING_CAPACITY;

        if (next == rx_tail)
        {
            g_auto_aim_status.overflow_count++;
            break;
        }
        rx_ring[rx_head] = data[index];
        rx_head = next;
    }
}

static void link_task(void *argument)
{
    bool was_online = false;

    (void)argument;
    for (;;)
    {
        uint8_t byte;
        const uint32_t now = HAL_GetTick();

        while (rx_tail != rx_head)
        {
            byte = rx_ring[rx_tail];
            rx_tail = (rx_tail + 1U) % AUTO_AIM_RX_RING_CAPACITY;
            process_bytes(&byte, 1U);
        }
        const bool online = (g_auto_aim_status.valid_frame_count != 0U) &&
            ((uint32_t)(now - g_auto_aim_status.last_update_tick) <=
             AUTO_AIM_TIMEOUT_MS);

        if (was_online && !online)
        {
            g_auto_aim_status.timeout_count++;
            g_auto_aim_status.fire_requested = false;
            g_auto_aim_status.target_present_heuristic = false;
        }
        g_auto_aim_status.online = online;
        was_online = online;
        osDelay(AUTO_AIM_TASK_PERIOD_MS);
    }
}

void auto_aim_link_init(void)
{
    memset(&g_auto_aim_status, 0, sizeof(g_auto_aim_status));
    memset(frame_buffer, 0, sizeof(frame_buffer));
    frame_length = 0U;
    collecting = false;
    has_sequence = false;
    previous_sequence = 0U;
    rx_head = 0U;
    rx_tail = 0U;
    memset(rx_ring, 0, sizeof(rx_ring));
}

bool auto_aim_link_task_create(void)
{
    const osThreadAttr_t attributes = {
        .name = "autoAimLink",
        .cb_mem = &task_control,
        .cb_size = sizeof(task_control),
        .stack_mem = task_stack,
        .stack_size = sizeof(task_stack),
        .priority = osPriorityNormal,
    };

    if (task_handle != NULL)
        return true;
    task_handle = osThreadNew(link_task, NULL, &attributes);
    return task_handle != NULL;
}
