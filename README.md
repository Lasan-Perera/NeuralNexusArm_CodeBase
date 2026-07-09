# Neural Nexus — 6-DOF Robotic Arm

Firmware and hardware documentation for a six-degree-of-freedom robotic arm driven by six stepper motors and controlled by an STM32H743VITx microcontroller.

---

## Overview

This repository contains the embedded firmware for a 6-axis robotic arm. Each joint is actuated by a stepper motor, with driver selection matched to the torque and precision requirements of that axis. Step generation is handled by a single hardware timer interrupt, allowing coordinated multi-axis motion.

## Hardware

### Microcontroller

| Item | Value |
|------|-------|
| MCU | STM32H743VITx |
| Core clock | 420 MHz |
| APB1 timer clock | 210 MHz |
| Configuration tool | STM32CubeMX |

### Motors and Drivers

| Joint | Motor | Driver | Notes |
|-------|-------|--------|-------|
| M1 | NEMA 24 | CL57T | Closed-loop, common-anode wiring |
| M2 | NEMA 24 | CL57T | Closed-loop, common-anode wiring |
| M3 | NEMA 23 | DM542 | Replaced original TB6600 |
| M4 | NEMA 17 | TMC2209 | Silent, 1/8 microstepping default |
| M5 | NEMA 17 | TMC2209 | Silent, 1/8 microstepping default |
| M6 | NEMA 17 | TMC2209 | Silent, 1/8 microstepping default |

### Timer Allocation

| Timer | Function |
|-------|----------|
| TIM1 | PWM output for RC servos (3 channels) |
| TIM2 | PWM output for status LED and buzzer |
| TIM6 | 50 kHz step pulse generation (Prescaler = 0, Period = 4199) |

---

## Wiring Notes

Stepper driver signal inputs are **opto-isolated**. The `+` and `-` terminals of each signal pair are the two legs of an internal optocoupler LED, not a differential pair. Correct wiring depends on driving current through that LED.

### CL57T (M1, M2) — Common Anode

The CL57T does not reliably trigger on 3.3V logic. It must be wired common-anode:

- Tie `PUL+`, `DIR+`, `ENA+` to an external **5V** rail
- Configure the corresponding MCU GPIOs as **open-drain** in CubeMX
- GPIOs sink current on the `-` pins to activate each signal
- Set the CL57T `S3` switch to the **5V** position

Note that this inverts the logic polarity for these two axes relative to common-cathode drivers.

### DM542 (M3) — Common Cathode

Works with 3.3V logic in common-cathode wiring at moderate step rates. If missed steps appear at higher speeds, rewire common-anode with open-drain GPIOs as above.

**SW4 (standstill current):**
- `OFF` — half current at idle. Runs cooler, lower holding torque.
- `ON` — full current at idle. Maximum holding torque; required if the joint is back-driven by gravity.

Minimum pulse width: **2.5 µs**.

### TMC2209 (M4–M6) — Standalone Mode

With `MS1` and `MS2` left unconnected, internal pull-downs select **1/8 microstepping** by default.

| MS2 | MS1 | Microstepping |
|-----|-----|---------------|
| GND | GND | 1/8 (default) |
| GND | VIO | 1/32 |
| VIO | GND | 1/64 |
| VIO | VIO | 1/16 |

MicroPlyer interpolation is active regardless of setting, so motion remains smooth at any resolution.

> Ensure all three TMC2209 boards share the same MS pin configuration. Mismatched settings cause identical step counts to produce different joint angles.

### General

- A **common ground** between the MCU and the logic side of every driver is mandatory.
- Verify motor coil pairing with a multimeter (continuity within a coil) before first power-up. Swapped coil pairs produce audible motor noise without rotation.
- Confirm the VMOT supply meets each driver's minimum voltage requirement.

---

## Firmware Architecture

Step pulses are generated from a single **TIM6 interrupt at 50 kHz** (20 µs per tick). All six axes are serviced from this one ISR.

### Pulse Timing

Blocking busy-wait delays are **not** used for pulse width. Instead, each `Stepper_t` holds a `pulsePending` flag:

1. On the tick where a step is due, the STEP pin is driven high and `pulsePending` is set.
2. On the following tick, the STEP pin is cleared and the flag reset.

This guarantees a pulse width of one full timer tick (20 µs), comfortably above the 2.5 µs minimum required by the DM542 and the CL57T threshold. It also keeps the ISR non-blocking.

The resulting maximum step rate is **25,000 steps/sec per axis** (two ticks per step).

### Key Functions

```c
void Stepper_MoveAll(int32_t steps[6]);   // Queue a coordinated move across all six axes
bool Stepper_AnyMoving(void);             // Returns true while any axis has steps remaining
```

### Enable Pins

`EN` pins are explicitly driven after initialization rather than left floating. Polarity depends on the wiring scheme of each driver — see the wiring notes above.

---

## Build and Flash

1. Open the `.ioc` file in **STM32CubeMX** and generate code.
2. Build with STM32CubeIDE (or your configured toolchain).
3. Flash via ST-Link.

> **Important:** all timer and pin configuration changes must be made in the `.ioc` file. Manual edits to generated files are destroyed on the next CubeMX regeneration.

Ensure the TIM6 global interrupt is enabled in the NVIC settings and that `HAL_TIM_Base_Start_IT(&htim6)` is called during initialization in `main.c`.

---

## Troubleshooting

| Symptom | Likely Cause |
|---------|--------------|
| Motor hums or whines but does not rotate | Swapped coil pairs, or step pulse too short |
| No motion at all, no sound | EN pin holding driver disabled, or missing common ground |
| Motion at low speed only | 3.3V drive marginal on opto inputs — move to common-anode |
| One TMC axis moves further than the others | Mismatched MS1/MS2 configuration |
| Joint sags when stopped | DM542 SW4 set to half current |

---

## Repository Structure

```
├── Core/
│   ├── Inc/          # Headers
│   └── Src/          # Application source, stepper driver, main.c
├── Drivers/          # STM32 HAL and CMSIS
├── *.ioc             # CubeMX project configuration
└── README.md
```

---

## License

<!-- Add your license here, e.g. MIT -->

## Author

Lasan Perera