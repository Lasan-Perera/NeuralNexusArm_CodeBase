#include "stepper.h"
#include <math.h>

/* ---------------------------------------------------------------------
 * IMPORTANT: The GPIO port/pin names below (M1_Step_Pin, M1_Dir_Pin, ...)
 * are the macros CubeMX auto-generates in main.h from the user labels you
 * set on the pinout (e.g. "M1_Step" -> M1_Step_Pin / M1_Step_GPIO_Port).
 *
 * Double-check these against your generated main.h - rename here if
 * your CubeMX labels differ slightly (e.g. underscores/casing).
 * ------------------------------------------------------------------- */

StepperAxis axis[NUM_AXES];

#define RAMP_DT     (1.0f / 1000.0f)   /* TIM6 period: 1 kHz */
#define PULSE_DT    (1.0f / 20000.0f)  /* TIM7 period: 20 kHz -> max 20k steps/sec/axis */

static void axis_assign_pins(void)
{
    axis[AXIS_M1].step_port = M1_Step_GPIO_Port; axis[AXIS_M1].step_pin = M1_Step_Pin;
    axis[AXIS_M1].dir_port  = M1_Dir_GPIO_Port;  axis[AXIS_M1].dir_pin  = M1_Dir_Pin;
    axis[AXIS_M1].en_port   = M1_EN_GPIO_Port;   axis[AXIS_M1].en_pin   = M1_EN_Pin;

    axis[AXIS_M2].step_port = M2_Step_GPIO_Port; axis[AXIS_M2].step_pin = M2_Step_Pin;
    axis[AXIS_M2].dir_port  = M2_Dir_GPIO_Port;  axis[AXIS_M2].dir_pin  = M2_Dir_Pin;
    axis[AXIS_M2].en_port   = M2_EN_GPIO_Port;   axis[AXIS_M2].en_pin   = M2_EN_Pin;

    axis[AXIS_M3].step_port = M3_Step_GPIO_Port; axis[AXIS_M3].step_pin = M3_Step_Pin;
    axis[AXIS_M3].dir_port  = M3_Dir_GPIO_Port;  axis[AXIS_M3].dir_pin  = M3_Dir_Pin;
    axis[AXIS_M3].en_port   = M3_EN_GPIO_Port;   axis[AXIS_M3].en_pin   = M3_EN_Pin;

    axis[AXIS_M4].step_port = M4_Step_GPIO_Port; axis[AXIS_M4].step_pin = M4_Step_Pin;
    axis[AXIS_M4].dir_port  = M4_Dir_GPIO_Port;  axis[AXIS_M4].dir_pin  = M4_Dir_Pin;
    axis[AXIS_M4].en_port   = M4_EN_GPIO_Port;   axis[AXIS_M4].en_pin   = M4_EN_Pin;

    axis[AXIS_M5].step_port = M5_Step_GPIO_Port; axis[AXIS_M5].step_pin = M5_Step_Pin;
    axis[AXIS_M5].dir_port  = M5_Dir_GPIO_Port;  axis[AXIS_M5].dir_pin  = M5_Dir_Pin;
    axis[AXIS_M5].en_port   = M5_EN_GPIO_Port;   axis[AXIS_M5].en_pin   = M5_EN_Pin;

    axis[AXIS_M6].step_port = M6_Step_GPIO_Port; axis[AXIS_M6].step_pin = M6_Step_Pin;
    axis[AXIS_M6].dir_port  = M6_Dir_GPIO_Port;  axis[AXIS_M6].dir_pin  = M6_Dir_Pin;
    axis[AXIS_M6].en_port   = M6_EN_GPIO_Port;   axis[AXIS_M6].en_pin   = M6_EN_Pin;
}

void Stepper_Init(void)
{
    axis_assign_pins();

    for (int i = 0; i < NUM_AXES; i++) {
        axis[i].current_pos   = 0;
        axis[i].target_pos    = 0;
        axis[i].current_speed = 0.0f;
        axis[i].dir_positive  = true;
        axis[i].accumulator   = 0.0f;
        axis[i].pulse_pending = false;

        /* Sensible defaults - OVERRIDE per motor after Stepper_Init(),
         * NEMA24 (M1,M2 via CL57T) can usually handle more torque at
         * lower top speed; NEMA17 (M4-M6 via TMC2209) can spin faster.
         * These are placeholders - tune to your microstepping + gearing. */
        axis[i].max_speed = 4000.0f;   /* steps/sec */
        axis[i].accel     = 8000.0f;   /* steps/sec^2 */
        axis[i].min_speed = 100.0f;    /* steps/sec, starting speed */

        HAL_GPIO_WritePin(axis[i].step_port, axis[i].step_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(axis[i].dir_port,  axis[i].dir_pin,  GPIO_PIN_RESET);
    }

    Stepper_EnableAll(false); /* start disabled/safe */
}

void Stepper_Enable(AxisId a, bool enable)
{
    /* Most STEP/DIR drivers (CL57T, TB6600, TMC2209 in legacy mode) use
     * active-LOW enable. Flip this if yours is active-high. */
    HAL_GPIO_WritePin(axis[a].en_port, axis[a].en_pin,
                       enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Stepper_EnableAll(bool enable)
{
    for (int i = 0; i < NUM_AXES; i++) Stepper_Enable((AxisId)i, enable);
}

void Stepper_SetMaxSpeed(AxisId a, float steps_per_sec)
{
    axis[a].max_speed = steps_per_sec;
}

void Stepper_SetAcceleration(AxisId a, float steps_per_sec2)
{
    axis[a].accel = steps_per_sec2;
}

void Stepper_MoveTo(AxisId a, int32_t target_steps)
{
    __disable_irq();
    axis[a].target_pos = target_steps;
    __enable_irq();
}

void Stepper_MoveRelative(AxisId a, int32_t delta_steps)
{
    __disable_irq();
    axis[a].target_pos = axis[a].current_pos + delta_steps;
    __enable_irq();
}

void Stepper_Stop(AxisId a)
{
    __disable_irq();
    axis[a].target_pos = axis[a].current_pos; /* ramp will decelerate to 0 */
    __enable_irq();
}

void Stepper_EmergencyStopAll(void)
{
    __disable_irq();
    for (int i = 0; i < NUM_AXES; i++) {
        axis[i].current_speed = 0.0f;
        axis[i].accumulator   = 0.0f;
        axis[i].target_pos    = axis[i].current_pos;
    }
    __enable_irq();
}

bool Stepper_IsMoving(AxisId a)
{
    return (axis[a].current_pos != axis[a].target_pos) || (axis[a].current_speed > 0.5f);
}

bool Stepper_AllIdle(void)
{
    for (int i = 0; i < NUM_AXES; i++)
        if (Stepper_IsMoving((AxisId)i)) return false;
    return true;
}

int32_t Stepper_GetPosition(AxisId a)
{
    return axis[a].current_pos;
}

void Stepper_SetPosition(AxisId a, int32_t steps)
{
    __disable_irq();
    axis[a].current_pos = steps;
    axis[a].target_pos  = steps;
    axis[a].current_speed = 0.0f;
    __enable_irq();
}

/* =====================================================================
 * TIM6 @ 1 kHz - trapezoidal velocity ramp
 * For each axis: accelerate while there's room to stop in time,
 * otherwise decelerate. Classic "stopping distance" trapezoidal profile.
 * ===================================================================== */
void Stepper_RampUpdate_1kHz(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        StepperAxis *ax = &axis[i];

        int32_t delta = ax->target_pos - ax->current_pos;

        if (delta == 0) {
            /* No distance left: decelerate to a stop */
            ax->current_speed -= ax->accel * RAMP_DT;
            if (ax->current_speed < 0.0f) ax->current_speed = 0.0f;
            continue;
        }

        bool want_positive = (delta > 0);
        if (want_positive != ax->dir_positive) {
            /* Direction changed: must fully stop first before reversing */
            if (ax->current_speed > 0.0f) {
                ax->current_speed -= ax->accel * RAMP_DT;
                if (ax->current_speed < 0.0f) ax->current_speed = 0.0f;
                continue;
            }
            ax->dir_positive = want_positive;
            HAL_GPIO_WritePin(ax->dir_port, ax->dir_pin,
                               want_positive ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        uint32_t distance = (uint32_t)(want_positive ? delta : -delta);

        /* Distance needed to decelerate from current speed to 0 */
        float stopping_dist = (ax->current_speed * ax->current_speed) / (2.0f * ax->accel);

        if ((float)distance <= stopping_dist) {
            /* Decelerate */
            ax->current_speed -= ax->accel * RAMP_DT;
        } else {
            /* Accelerate (or cruise once max_speed reached) */
            ax->current_speed += ax->accel * RAMP_DT;
        }

        if (ax->current_speed > ax->max_speed) ax->current_speed = ax->max_speed;
        if (ax->current_speed < ax->min_speed) ax->current_speed = ax->min_speed;
    }
}

/* =====================================================================
 * TIM7 @ 20 kHz - per-axis DDA step pulse generation
 * Runs independently of the ramp timer so each axis can step at its own
 * instantaneous rate. Pin is dropped LOW at the start of each tick
 * (finishing the previous pulse) before deciding whether to issue a new
 * pulse, guaranteeing a clean HIGH pulse close to one tick wide (~50us).
 * ===================================================================== */
void Stepper_PulseUpdate_20kHz(void)
{
    for (int i = 0; i < NUM_AXES; i++) {
        StepperAxis *ax = &axis[i];

        if (ax->pulse_pending) {
            HAL_GPIO_WritePin(ax->step_port, ax->step_pin, GPIO_PIN_RESET);
            ax->pulse_pending = false;
        }

        if (ax->current_pos == ax->target_pos && ax->current_speed <= 0.5f) {
            ax->accumulator = 0.0f;
            continue;
        }

        ax->accumulator += ax->current_speed * PULSE_DT;

        if (ax->accumulator >= 1.0f) {
            ax->accumulator -= 1.0f;
            HAL_GPIO_WritePin(ax->step_port, ax->step_pin, GPIO_PIN_SET);
            ax->pulse_pending = true;
            ax->current_pos += ax->dir_positive ? 1 : -1;
        }
    }
}

/* =====================================================================
 * WIRING - add to stm32h7xx_it.c (or wherever HAL_TIM_PeriodElapsedCallback
 * lives). Make sure both TIM6 and TIM7 are started with interrupts, e.g.
 * in main() after MX_TIM6_Init()/MX_TIM7_Init():
 *
 *   HAL_TIM_Base_Start_IT(&htim6);
 *   HAL_TIM_Base_Start_IT(&htim7);
 *
 * void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
 * {
 *     if (htim->Instance == TIM6) {
 *         Stepper_RampUpdate_1kHz();
 *     } else if (htim->Instance == TIM7) {
 *         Stepper_PulseUpdate_20kHz();
 *     }
 * }
 *
 * TIM7 has no channels configured yet in your CubeMX project - add it as
 * a basic timer (like TIM6), targeting 20 kHz:
 *   If TIM7 clock = 200 MHz: PSC = 99, ARR = 99  -> 200e6/(100*100) = 20 kHz
 * ===================================================================== */
