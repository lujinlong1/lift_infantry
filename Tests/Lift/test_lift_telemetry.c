#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "../../Diagnostics/Telemetry/Src/telemetry_task.c"
static DMA_HandleTypeDef dma={HAL_DMA_STATE_READY};
UART_HandleTypeDef huart1={HAL_UART_STATE_READY,&dma};
lift_jog_status_t g_lift_jog_status;
pitch_command_t g_pitch_control_command;
pitch_status_t g_pitch_control_status;
remote_mapping_t g_remote_mapping_status;
static jmp_buf stopped;
static unsigned calls;
static uint16_t length;
static uint8_t *active;
static uint8_t immutable[TELEMETRY_FRAME_SIZE];
static HAL_StatusTypeDef result=HAL_OK;
uint32_t osKernelGetTickCount(void){return 100;}
TickType_t xTaskGetTickCount(void){return 100;}
osThreadId_t osThreadNew(void (*f)(void *),void *a,const osThreadAttr_t *attr)
{(void)f;(void)a;(void)attr;return (void *)1;}
void vTaskDelayUntil(TickType_t *t,TickType_t inc){*t+=inc;longjmp(stopped,1);}
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *h){return h->State;}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *data,uint16_t len)
{
    ++calls;
    assert(h->gState==HAL_UART_STATE_READY && h->hdmatx->State==HAL_DMA_STATE_READY);
    if(result!=HAL_OK)return result;
    assert(len==36 || len==116);
    assert(memcmp(data+len-4,telemetry_tail,4)==0);
    active=data;length=len;memcpy(immutable,data,len);
    h->gState=HAL_UART_STATE_BUSY_TX;h->hdmatx->State=HAL_DMA_STATE_BUSY;
    return HAL_OK;
}
static void run(void){if(setjmp(stopped)==0)telemetry_task(NULL);}
static void finish(void){dma.State=HAL_DMA_STATE_READY;huart1.gState=HAL_UART_STATE_READY;}
static float value(unsigned ch){float f;memcpy(&f,active+ch*4,4);return f;}
int main(void)
{
    lift_diag_frame_t f={0};
    telemetry_init();lift_diag_init();
    g_remote_mapping_status=(remote_mapping_t){1,2,3};
    g_pitch_control_command.target_angle_deg=4;
    g_pitch_control_status=(pitch_status_t){5,6,7,8};
    run();assert(calls==1 && length==36);
    for(unsigned i=0;i<8;i++)assert(value(i)==(float)(i+1));
    for(unsigned n=0;n<60;n++) {
        g_telemetry_status.source=(uint8_t)(n%4);
        g_pitch_control_command.target_angle_deg+=1;
        run();assert(calls==1 && memcmp(active,immutable,length)==0);
    }
    dma.State=HAL_DMA_STATE_READY;run();assert(calls==1); /* DMA done, UART TC pending */
    huart1.gState=HAL_UART_STATE_READY;dma.State=HAL_DMA_STATE_BUSY;
    run();assert(calls==1 && !memcmp(active,immutable,length));
    finish();g_telemetry_status.source=1;
    for(unsigned ch=0;ch<4;ch++) {
        g_lift_jog_status.axis_offset_ecd[ch]=(int32_t)(ch*7);
        g_lift_jog_status.axis_target_offset[ch]=(int32_t)(ch*7+1);
        g_lift_jog_status.axis_output_current[ch]=(int16_t)(ch*7+2);
        g_lift_jog_status.axis_speed_ecd_s[ch]=(float)(ch*7+3);
        g_lift_jog_status.axis_target_speed_ecd_s[ch]=(float)(ch*7+4);
        g_lift_jog_status.axis_limit_now[ch]=(float)(ch*7+5);
        g_lift_jog_status.axis_given_current[ch]=(int16_t)(ch*7+6);
    }
    run();assert(length==116);for(unsigned i=0;i<28;i++)assert(value(i)==(float)i);
    f.cycle_id=7;f.transport_valid=true;f.parameter_version=9;f.source=LIFT_DIAG_SOURCE_SEQUENCE;
    for(unsigned ch=0;ch<4;ch++) {
        f.axis[ch].direction=-1;f.axis[ch].position_ecd=100+(int32_t)ch;
        f.axis[ch].target_ecd=200+(int32_t)ch;f.axis[ch].submitted_raw=-123;
        f.axis[ch].feedback_current=456;
    }
    lift_diag_publish(&f);finish();g_telemetry_status.source=2;run();
    for(unsigned ch=0;ch<4;ch++) {
        assert(value(ch*7)==100+(float)ch && value(ch*7+1)==200+(float)ch);
        assert(value(ch*7+2)==123 && value(ch*7+6)==456);
    }
    finish();g_telemetry_status.source=3;g_telemetry_status.diagnostic_axis=2;run();
    assert(value(0)==7 && value(1)==9 && value(2)==2 && value(3)==202);
    assert(value(14)==123 && value(17)==456 && value(27)==LIFT_DIAG_SOURCE_SEQUENCE);
    for(unsigned i=0;i<200;i++) {finish();g_telemetry_status.source=(uint8_t)(i%4);run();}
    finish();result=HAL_BUSY;unsigned old=calls;run();assert(calls==old+1);
    result=HAL_ERROR;run();assert(calls==old+2);
    result=HAL_OK;run();assert(calls==old+3);
    for(unsigned i=0;i<5;i++){run();assert(calls==old+3 && !memcmp(active,immutable,length));}
    finish();g_telemetry_status.enabled=false;run();assert(calls==old+3);
    g_telemetry_status.enabled=true;g_telemetry_status.source=2;
    lift_diag_init();run();assert(calls==old+3 && g_telemetry_status.snapshot_drop_count==1);
    puts("PASS: immutable DMA buffer, pending UART TC, busy/error/recovery, source 0/1/2/3 layout, 200 continuous frames");
    return 0;
}
