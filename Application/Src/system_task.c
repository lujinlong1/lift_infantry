#include "system_task.h"

#include "app_tasks.h"
#include "can_rx_service.h"
#include "cmsis_os2.h"
#include "main.h"
#include "system_init.h"
#include "tim.h"

#define BOARD_LED_VISIBLE_LEVEL 16384U

void board_led_set(uint16_t red, uint16_t green, uint16_t blue)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, red);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, green);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, blue);
}

void board_led_startup_self_test(void)
{
    uint32_t cycle;

    for (cycle = 0U; cycle < 2U; ++cycle)
    {
        board_led_set(BOARD_LED_VISIBLE_LEVEL, 0U, 0U);
        HAL_Delay(70U);
        board_led_set(0U, BOARD_LED_VISIBLE_LEVEL, 0U);
        HAL_Delay(70U);
        board_led_set(0U, 0U, BOARD_LED_VISIBLE_LEVEL);
        HAL_Delay(70U);
    }
    board_led_set(0U, 0U, 0U);
}

void board_led_show_error(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    gpio.Pin = LED_R_Pin | LED_G_Pin | LED_B_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &gpio);
    HAL_GPIO_WritePin(GPIOH, LED_G_Pin | LED_B_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, LED_R_Pin, GPIO_PIN_SET);
}

/**
 * @brief 系统启动任务，建立队列、创建业务任务并完成硬件初始化。
 * @note 创建阶段锁住调度器，防止部分任务在所有依赖尚未初始化时提前运行；
 *       完成后恢复调度并主动退出本启动任务。
 */
void StartSystemTask(void *argument)
{
    int32_t previous_lock_state;

    (void)argument;

    if (!can_rx_service_init())
    {
        Error_Handler();
    }

    previous_lock_state = osKernelLock();
    if (previous_lock_state < 0)
    {
        Error_Handler();
    }

    if (!app_tasks_create())
    {
        Error_Handler();
    }

    if (!system_init())
    {
        Error_Handler();
    }

    board_led_set(0U, BOARD_LED_VISIBLE_LEVEL, 0U);

    if (osKernelRestoreLock(previous_lock_state) < 0)
    {
        Error_Handler();
    }

    osThreadExit();

    for (;;)
    {
    }
}
