/*
 * 小车控制程序
 * TIMG6 50Hz IMU | SysTick 100Hz 控制 | TIMG7 PWM 步进电机
 */
#include "ti_msp_dl_config.h"
#include "UART/uart_comm.h"
#include "IMU/IMU.h"
#include "OLED/OLED.h"
#include "Motor/Motor.h"
#include "Track/Track.h"
#include "Control/Control.h"
#include "Button/Button.h"
#include "Motor_stp/StepMotor.h"
#include "Motor_stp/Balance.h"
#include <stdio.h>
#include <string.h>

float ypr[3];
volatile uint32_t g_imu_cnt = 0;
extern float g_bal_kp, g_bal_ki, g_bal_kd;

void delay_ms(uint32_t ms) { while(ms--) delay_cycles(CPUCLK_FREQ/1000); }
void TIMER_0_INST_IRQHandler(void) {}

void SysTick_Handler(void)
{
    Button_Poll();
    Control_Update();
    StepMotor_Update();
    Balance_Update();
    Balance_ChallengeUpdate();
}

static void ParseCommand(const char *cmd)
{
    float val; char pid[8], param[4];
    if(sscanf(cmd,"%7[^.].%3[^:]:%f",pid,param,&val)==3){
        if(!strcmp(pid,"speed")&&!strcmp(param,"kp")) g_speed_kp=val;
        else if(!strcmp(pid,"speed")&&!strcmp(param,"ki")) g_speed_ki=val;
        else if(!strcmp(pid,"speed")&&!strcmp(param,"kd")) g_speed_kd=val;
        else if(!strcmp(pid,"track")&&!strcmp(param,"kp")) g_track_kp=val;
        else if(!strcmp(pid,"track")&&!strcmp(param,"ki")) g_track_ki=val;
        else if(!strcmp(pid,"track")&&!strcmp(param,"kd")) g_track_kd=val;
        else if(!strcmp(pid,"ball")&&!strcmp(param,"kp")) g_bal_kp=val;
        else if(!strcmp(pid,"ball")&&!strcmp(param,"ki")) g_bal_ki=val;
        else if(!strcmp(pid,"ball")&&!strcmp(param,"kd")) g_bal_kd=val;
        else if(!strcmp(pid,"balpos")) Balance_SetTarget(val);
        else if(!strcmp(pid,"bal")&&!strcmp(param,"kp")) g_bal_kp=val;
        else if(!strcmp(pid,"bal")&&!strcmp(param,"ki")) g_bal_ki=val;
        else if(!strcmp(pid,"bal")&&!strcmp(param,"kd")) g_bal_kd=val;
    }
}

static void Display_Update(void)
{
    motor_speed_t spd; float trk_err; const char *mode_str;
    Motor_GetSpeed(&spd); trk_err=Track_Err(0);
    if(g_steer_mode==STEER_MODE_IDLE) mode_str="IDLE";
    else mode_str="TRACK";
    OLED_Clear();
    OLED_Printf(0,0,OLED_6X8,"%-5s %3lu.%02lus %c",
        mode_str,g_lap_ticks/100,g_lap_ticks%100,g_lap_active?'*':' ');
    OLED_Printf(0,8,OLED_6X8,"Trk:%-5.1f L:%-2ld R:%-2ld",
        trk_err,(long)spd.left_delta,(long)spd.right_delta);
    OLED_Printf(0,16,OLED_6X8,"Avg:%5.1f Diff:%5.1f",spd.avg,spd.diff);
    OLED_Printf(0,24,OLED_6X8,"Sp Kp:%-4.2f Ki:%-4.2f Kd:%-4.2f",
        g_speed_kp,g_speed_ki,g_speed_kd);
    OLED_Printf(0,32,OLED_6X8,"Tr Kp:%-4.2f Ki:%-4.2f Kd:%-4.2f",
        g_track_kp,g_track_ki,g_track_kd);
    OLED_Printf(0,40,OLED_6X8,"ba Kp:%-4.2f Ki:%-4.2f Kd:%-4.2f",
        g_bal_kp,g_bal_ki,g_bal_kd);
    OLED_Printf(0,48,OLED_6X8,"Bal A:%-4.1f S:%-4ld K:%-3.1f",
        Balance_GetPosition(),(long)Balance_GetMotorSteps(),g_bal_kp);
    OLED_Printf(0,56,OLED_6X8,"M%d %s %3lu.%02lus",
        g_bal_mode+1,
        g_bal_mode==BAL_MODE1_NORMAL?"NORM":
        g_bal_mode==BAL_MODE2_TRACK?"TRCK":
        g_chal_state==CHAL_STATE_RUN?"RUN ":
        g_chal_state==CHAL_STATE_DONE?"OK  ":"----",
        g_chal_tick/100,g_chal_tick%100);
    OLED_Update();
}

int main(void)
{
    SYSCFG_DL_init();
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    OLED_Init(); delay_ms(50);
    Motor_Init();
    Control_Init();
    StepMotor_Init();
    Balance_Init();
    Balance_SetTarget(25.0f);         /* 初始目标: 管道中心 */
    SysTick_Config(CPUCLK_FREQ/CTRL_LOOP_FREQ_HZ);
    NVIC_SetPriority(SysTick_IRQn,2);

    OLED_Printf(0,0,OLED_6X8,"CAR CONTROL");
    OLED_Printf(0,16,OLED_6X8,"STEP PWM TEST");
    OLED_Update();

    uart_printf("OK\r\n");
    while(1){
        Motor_UpdateLeftEncoder();
        Motor_UpdateRightEncoder();
        /* PB21: 模式切换 MODE1→MODE2→MODE3→MODE1 */
        if(Button_IsPressed()){
            Balance_NextMode();
            if(g_bal_mode == BAL_MODE1_NORMAL)
                Control_SetMode(STEER_MODE_IDLE);
            else
                Control_SetMode(STEER_MODE_TRACK);
        }

        /* 蓝牙PID调参 */
        if(g_usart_rx_sta&0x8000){uint16_t l=g_usart_rx_sta&0x3FFF;g_usart_rx_buf[l]=0;
            ParseCommand((const char*)g_usart_rx_buf);g_usart_rx_sta=0;}
        /* OLED刷新 */
        { static uint32_t tick=0; if(++tick>=250000){tick=0;Display_Update();} }
    }
}
