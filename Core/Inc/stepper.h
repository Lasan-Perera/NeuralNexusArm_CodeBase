#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ---------------------------------------------------------------------
 * 6-axis stepper motion control
 *
 * Architecture:
 *  - TIM6 @ 1 kHz  -> Stepper_RampUpdate_1kHz()   (trapezoidal velocity ramp)
 *  - TIM7 @ 20 kHz -> Stepper_PulseUpdate_20kHz()  (per-axis DDA step pulses)
 *
 * Wire both calls into HAL_TIM_PeriodElapsedCallback() in stm32h7xx_it.c
 * or main.c (see bottom of stepper.c for the exact snippet).
 * ------------------------------------------------------------------- */

#define NUM_AXES 6

typedef enum {
    AXIS_M1 = 0,
    AXIS_M2,
    AXIS_M3,
    AXIS_M4,
    AXIS_M5,
    AXIS_M6
} AxisId;

typedef struct {
    /* Pins - filled from CubeMX-generated names in Stepper_Init() */
    GPIO_TypeDef *step_port;
    uint16_t      step_pin;
    GPIO_TypeDef *dir_port;
    uint16_t      dir_pin;
    GPIO_TypeDef *en_port;
    uint16_t      en_pin;

    /* Motion state */
    volatile int32_t current_pos;   /* steps */
    volatile int32_t target_pos;    /* steps */
    volatile float   current_speed; /* steps/sec, always >= 0 */
    volatile bool    dir_positive;  /* current direction */
    volatile float   accumulator;   /* DDA fractional-step accumulator */
    volatile bool    pulse_pending; /* step pin currently high, needs clearing */

    /* Per-axis tunables - set these to match each motor/driver */
    float max_speed;   /* steps/sec */
    float accel;        /* steps/sec^2 */
    float min_speed;    /* steps/sec, speed used at start/end of a move */
} StepperAxis;

extern StepperAxis axis[NUM_AXES];

void Stepper_Init(void);
void Stepper_EnableAll(bool enable);
void Stepper_Enable(AxisId a, bool enable);

void Stepper_SetMaxSpeed(AxisId a, float steps_per_sec);
void Stepper_SetAcceleration(AxisId a, float steps_per_sec2);

void Stepper_MoveTo(AxisId a, int32_t target_steps);
void Stepper_MoveRelative(AxisId a, int32_t delta_steps);
void Stepper_Stop(AxisId a);          /* decelerate to stop, keep position target */
void Stepper_EmergencyStopAll(void);  /* halt instantly, no ramp */

bool Stepper_IsMoving(AxisId a);
bool Stepper_AllIdle(void);
int32_t Stepper_GetPosition(AxisId a);
void Stepper_SetPosition(AxisId a, int32_t steps); /* for homing: define "here" as this value */

/* Call from timer ISRs - see stepper.c footer for wiring example */
void Stepper_RampUpdate_1kHz(void);
void Stepper_PulseUpdate_20kHz(void);

#endif /* STEPPER_H */
