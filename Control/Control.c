/*
 * Control.c — 纯PID调试: Track_Err → PID → DiffPWM
 */
#include "ti_msp_dl_config.h"
#include "Control/Control.h"
#include "PID/PID.h"
#include "Motor/Motor.h"
#include "Track/Track.h"
#include <math.h>

steer_mode_t g_steer_mode=STEER_MODE_IDLE;
float g_target_speed=0, g_target_yaw=CTRL_TARGET_YAW;
float g_speed_kp=1,g_speed_ki=0,g_speed_kd=0;
float g_track_kp=15,g_track_ki=0.1f,g_track_kd=5;
float g_angle_kp=10,g_angle_ki=0,g_angle_kd=0;

static PID_t speed_pid, track_pid, angle_pid;
extern float ypr[3];

void Control_Init(void)
{
    PID_Init(&speed_pid);
    speed_pid.OutMax=CTRL_PWM_PERIOD; speed_pid.OutMin=0;
    speed_pid.ErrorIntMax=200; speed_pid.ErrorIntMin=-200;

    PID_Init(&track_pid);
    track_pid.OutMax=500; track_pid.OutMin=-500;
    track_pid.ErrorIntMax=100; track_pid.ErrorIntMin=-100;

    PID_Init(&angle_pid);
    angle_pid.OutMax=500; angle_pid.OutMin=-500;
    angle_pid.ErrorIntMax=100; angle_pid.ErrorIntMin=-100;
    g_steer_mode=STEER_MODE_IDLE;
}

static void Sync(void){
    speed_pid.Kp=g_speed_kp;speed_pid.Ki=g_speed_ki;speed_pid.Kd=g_speed_kd;
    track_pid.Kp=g_track_kp;track_pid.Ki=g_track_ki;track_pid.Kd=g_track_kd;
    angle_pid.Kp=g_angle_kp;angle_pid.Ki=g_angle_ki;angle_pid.Kd=g_angle_kd;
}

void Control_SetMode(steer_mode_t m){
    g_steer_mode=m;
    if(m==STEER_MODE_ANGLE)angle_pid.ErrorInt=0;
    else{Motor_SetLeftSpeed(0);Motor_SetRightSpeed(0);}
}
void Control_SetTargetSpeed(float s){
    if(s<0)s=0; if(s>CTRL_PWM_PERIOD)s=CTRL_PWM_PERIOD; g_target_speed=s;
}
void Control_SetTargetYaw(float y){g_target_yaw=y;}

static float clampf(float v,float lo,float hi){
    if(v<lo)return lo; if(v>hi)return hi; return v;
}

void Control_Update(void)
{
    motor_speed_t spd; float err,target,avg,diff; int16_t L,R;
    Motor_UpdateSpeed(); Motor_GetSpeed(&spd);
    Read_Track_DATA(&TrackN);
    err=Track_Err(0);
    if(g_steer_mode==STEER_MODE_IDLE)return;
    Sync();

    if(g_steer_mode==STEER_MODE_TRACK){
        float r=1-fabsf(err)/CTRL_MAX_OFFSET_MM;
        if(r<0)r=0;
        target=CTRL_MIN_SPEED+(CTRL_BASE_SPEED-CTRL_MIN_SPEED)*r;
    }else target=g_target_speed;

    speed_pid.Target=target;
    speed_pid.Actual=(fabsf((float)spd.left_delta)+fabsf((float)spd.right_delta))*0.5f;
    PID_Update(&speed_pid); avg=speed_pid.Out;

    if(g_steer_mode==STEER_MODE_TRACK){
        track_pid.Target=0; track_pid.Actual=-err;
        PID_Update(&track_pid); diff=track_pid.Out;
    }else{
        angle_pid.Target=g_target_yaw; angle_pid.Actual=ypr[0];
        PID_Update(&angle_pid); diff=angle_pid.Out;
    }

    float left=avg+diff, right=avg-diff;
    left=clampf(left,0,CTRL_PWM_PERIOD); right=clampf(right,0,CTRL_PWM_PERIOD);
    L=(int16_t)left; R=(int16_t)right;
    Motor_SetLeftSpeed(L); Motor_SetRightSpeed(R);
}
