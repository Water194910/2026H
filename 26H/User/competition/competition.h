#ifndef COMPETITION_H
#define COMPETITION_H

#include <stdint.h>

typedef enum
{
    COMP_CHASSIS_STOP = 0,
    COMP_CHASSIS_TRACK,
    COMP_CHASSIS_DIRECT
} Competition_ChassisMode;

void Competition_Init(void);
void Competition_Service(void);
void Competition_MarkerTick5ms(void);

Competition_ChassisMode Competition_ChassisTick20ms(float left_rpm,
                                                     float right_rpm,
                                                     int32_t left_encoder_delta,
                                                     int32_t right_encoder_delta,
                                                     int16_t *left_pwm,
                                                     int16_t *right_pwm);
float Competition_GetTrackSpeedRPM(void);
uint8_t Competition_CommandFeedforwardActive(void);
float Competition_GetCommandFeedforwardGain(void);
float Competition_AdjustCommandFeedforwardDeg(float planned_deg);
uint8_t Competition_BallTrackProfileActive(void);
uint8_t Competition_PeriodicDebugEnabled(void);
void Competition_TrackLost(void);

#endif
