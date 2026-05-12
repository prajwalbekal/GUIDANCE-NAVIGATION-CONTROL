# Project 3: Embedded Closed-Loop PID Control

## Goal

Build a real embedded control loop that reads a sensor, compares it with a desired setpoint, computes PID feedback, and drives an actuator.

This project includes:

- An Arduino sketch for real hardware.
- A Python desktop simulation for tuning before flashing.

## Suggested Hardware

- Arduino Uno, Nano, or compatible board.
- Analog sensor connected to `A0`.
- PWM-driven actuator connected to pin `9`.
- Actuator driver appropriate for your hardware: MOSFET, motor driver, or LED driver.
- External supply if the actuator draws more current than the Arduino can provide.

## Wiring

```text
Sensor signal -> A0
Sensor VCC    -> 5V or 3.3V as required
Sensor GND    -> GND
PWM output    -> D9 -> driver input
Driver GND    -> Arduino GND
Driver supply -> actuator supply
```

Do not power motors, heaters, or other high-current loads directly from an Arduino pin.

## Arduino Run

Open `arduino_pid_controller/arduino_pid_controller.ino` in the Arduino IDE, select your board and port, then upload.

Serial commands:

```text
S 70.0   set target setpoint
P 3.0    set proportional gain
I 0.5    set integral gain
D 0.1    set derivative gain
```

The serial monitor prints:

```text
time_ms,setpoint,measurement,control,error
```

## Desktop Simulation

```powershell
python -m projects.embedded_controls.simulation.pid_plant_sim
```

Outputs:

- `results/embedded_pid/pid_history.csv`
- `results/embedded_pid/pid_response.png`
- `results/embedded_pid/pid_control.png`

## Tuning Procedure

1. Set `Ki = 0` and `Kd = 0`.
2. Increase `Kp` until the system responds quickly but does not oscillate too much.
3. Add `Kd` to reduce overshoot and damp oscillation.
4. Add a small `Ki` to remove steady-state error.
5. Keep integral limits enabled to prevent windup.

## Resume Bullet

Built an embedded closed-loop PID control system on Arduino with serial gain tuning, anti-windup, PWM actuator output, and a matching Python plant simulation for response validation.

