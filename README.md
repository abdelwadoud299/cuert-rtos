# Real-Time Vehicle Actuation & Fail-Safe Supervisor

A deterministic, multi-tasking FreeRTOS firmware implementing a safety-critical command ingest pipeline, hardware PWM actuator control, emergency brake dominance, and an autonomous 500 ms link-loss watchdog for embedded automotive nodes.

Targeted and validated on the **ESP32** (AI Thinker ESP32-CAM + CAM-MB baseboard).

---

## 1. System Overview

Modern automotive architectures separate high-level perception and telemetry from low-level safety-critical actuation. This firmware models an electronic actuation sub-node that:
- Ingests command frames over serial (serving as a gateway/CAN bus proxy) at 115200 baud.
- Controls an actuator (hardware PWM on GPIO 33) representing vehicle traction/throttle drive.
- Enforces an immediate **Brake System Plausibility Device (BSPD)** interlock: any non-zero brake signal clamps actuator drive to 0%.
- Executes an autonomous **500 ms communication watchdog** that trips into an active fail-safe state upon signal loss.
- Broadcasts 1 Hz diagnostic telemetry without jitter or deadline interference.

---

## 2. Hardware Configuration

| Parameter | Specification |
| :--- | :--- |
| **Microcontroller** | ESP32-D0WD (Dual-core Xtensa LX6 @ 240 MHz) |
| **Evaluation Board** | AI Thinker ESP32-CAM with CAM-MB USB Programmer |
| **Actuator Pin** | GPIO 33 (Onboard Red Indicator LED, Active-LOW) |
| **PWM Hardware** | LEDC Timer 0, Channel 0, 5 kHz, 8-bit resolution (0–255) |
| **Serial Bus** | UART0 (115200 baud, 8 data bits, no parity, 1 stop bit, `\n`) |

---

## 3. Architecture & Task Hierarchy

The application runs on FreeRTOS with tasks pinned to **Core 1**, ensuring real-time determinism isolated from wireless and background system interrupts on Core 0:

```
[Serial / Bus Ingest]
        |
        v
+-------------------+
|    COMMAND_RX     |  Priority 5 (Highest)
|  Drains UART FIFO |  Blocks on incoming characters; parses ASCII frames; updates vitality timestamp
+-------------------+
        |
        v  (g_cmd_queue: 10 slots, with BRAKE priority eviction)
+-------------------+
|      ACTUATE      |  Priority 4 (High)
|  Drives LED PWM   |  Blocks on queue; enforces Brake Interlock (Brake > 0 clamps drive to 0%)
+-------------------+

+-------------------+
|     WATCHDOG      |  Priority 3 (Medium, Autonomous)
|  500ms Supervisor |  Wakes every 50ms; trips active fail-safe (2 Hz visual beacon) on bus silence
+-------------------+

+-------------------+
|      STATUS       |  Priority 2 (Lowest)
| 1 Hz Telemetry    |  Uses vTaskDelayUntil() for drift-free background health reporting
+-------------------+
```

---

## 4. Engineering & Safety Design Rationale

### 4.1 Task Priority Allocation
Task priorities are assigned strictly based on deadline criticality, hardware buffer constraints, and CPU duty cycle:

1. **`COMMAND_RX` (Priority 5 — Highest)**: The UART hardware FIFO has limited buffer capacity and must be drained immediately upon byte arrival to prevent buffer overrun and dropped frames. Because it remains in the `Blocked` state until data arrives, its execution duty cycle is $<1\%$, eliminating any risk of starving lower-priority tasks.
2. **`ACTUATE` (Priority 4 — High)**: Once a verified setpoint is queued, the latency between reception and physical hardware PWM update must be minimal. It unblocks immediately, commits the duty cycle to the LEDC registers, and yields back to `Blocked`.
3. **`WATCHDOG` (Priority 3 — Medium)**: Supervises communication vitality against the 500 ms deadline. It wakes periodically every 50 ms to evaluate the monotonic timestamp delta (`pdMS_TO_TICKS`) and immediately sleeps, consuming negligible CPU cycles ($<0.1\%$).
4. **`STATUS` (Priority 2 — Lowest)**: String formatting (`snprintf`) and UART transmission for periodic monitoring are computationally expensive. Pinning telemetry to the lowest priority guarantees non-critical diagnostics never induce latency into safety-critical control or supervisor routines.

### 4.2 Hazard Mitigation & Active Fail-Safe Philosophy

#### Brake Dominance over Steering
In vehicle dynamics, a stale or delayed `STEER` command maintains the current heading, which can typically be trimmed by the driver or electronic stability control. In contrast, a stale or lost `BRAKE` command represents a catastrophic hazard: if the vehicle is accelerating toward an obstacle, a dropped brake command results in unmitigated forward motion. 

Therefore, **braking is safety-dominant**:
- **Queue Eviction**: If the 10-slot FreeRTOS queue is saturated, incoming `BRAKE` frames evict the oldest pending frame (`xQueueReceive`) and prepend to the head (`xQueueSendToFront`).
- **Hardware Interlock**: Whenever `brake > 0`, the actuator PWM is clamped strictly to 0%, overriding any active or queued throttle setpoint.

#### Active Fail-Safe vs. Silent Degradation
A common pitfall in embedded control is letting a timed-out node "go quiet" (silent stop). In high-speed environments, a silent node provides zero observability to operators or upstream controllers.

This supervisor enforces an **active fail-safe**:
- **Autonomous Detection**: The supervisor queries hardware tick counters independently. Severed communication or upstream master crashes trip the watchdog autonomously without requiring external fault signaling.
- **Active Annunciation**: Upon tripping ($>500\text{ ms}$ silence), the node logs `LINK LOST , failing safe`, disables actuator drive, and initiates a 2 Hz optical alert beacon.
- **Hot Recovery**: As soon as a valid packet arrives, normal operation resumes instantaneously without requiring a manual reset.

### 4.3 Production Roadmap & Next Steps
For automotive-grade deployment, the following enhancements are planned:
1. **TWAI / CAN 2.0B Integration**: Migrate from UART to the ESP32's native Two-Wire Automotive Interface (TWAI) for differential signaling, hardware message filtering, and automatic bus-off recovery.
2. **Dual-Tier Watchdog (Hardware WDT)**: Couple the software communication supervisor with the silicon Task Watchdog Timer (`esp_task_wdt`) to trigger an emergency hardware reset if an RTOS task deadlocks or encounters stack overflow.
3. **Slew-Rate Limiter (Current Inrush Protection)**: Implement a software ramp filter on throttle setpoint step changes (e.g., maximum 20% duty cycle change per 50 ms) to protect inverter switching stages from inductive back-EMF and excessive motor inrush current.

---

## 5. Command Interface & Verification Protocol

### Command Protocol
Commands are line-terminated ASCII strings:

| Command | Arguments | Description |
| :--- | :--- | :--- |
| `PING` | None | Heartbeat request. Responds: `PONG (ACK: PING)` |
| `THROTTLE` | `0` to `100` | Sets actuator drive percentage (PWM duty cycle). |
| `STEER` | `-100` to `100` | Updates steer setpoint. |
| `BRAKE` | `0` to `100` | Applies brake percentage; any value $>0$ clamps PWM to 0%. |

### Verification Test Matrix

| Step | Input Frame | Expected Response & Hardware Behavior |
| :---: | :--- | :--- |
| **1** | `PING` | `PONG (ACK: PING)` |
| **2** | `THROTTLE 40` | Actuator LED illuminates at ~40% brightness. |
| **3** | `STEER -60` | Telemetry records `S:-60`. |
| **4** | `THROTTLE 90` $\rightarrow$ `BRAKE 100` | Actuator drops immediately to 0% (Brake interlock active). |
| **5** | `BRAKE 0` $\rightarrow$ `THROTTLE 55` | Normal actuation resumes; LED illuminates at ~55%. |
| **6** | *(Silence $\ge 500\text{ ms}$)* | Watchdog trips: `LINK LOST , failing safe`, LED blinks at 2 Hz. |
| **7** | `THROTTLE 20` | Fail-safe clears; LED transitions to steady 20% duty cycle. |
| **8** | `THROTTLE abc` | Bounds check fails: `PARSER ERROR: Invalid number 'abc'`. |
| **9** | Burst: `BRAKE 100` & `THROTTLE 100` | Brake overrides throttle; actuator output remains 0%. |
| **Bkg** | *(Continuous)* | `[STATUS]` prints telemetry line once per second. |

---

## 6. Build & Deployment

### Option A: Arduino IDE
1. Open Arduino IDE.
2. Select Board: **AI Thinker ESP32-CAM** (or **ESP32 Dev Module**).
3. Connect the ESP32-CAM via the CAM-MB baseboard and select the appropriate COM port.
4. Open `arduino/cuert_rtos_controller/cuert_rtos_controller.ino`.
5. Click **Upload**.
6. Open **Serial Monitor** at **115200 baud** with **Newline** enabled.

### Option B: PlatformIO
```bash
# Build firmware
pio run

# Flash to target
pio run --target upload

# Monitor serial output
pio run --target monitor
```

---

## 7. Repository Layout

```text
cuert-rtos/
├── README.md                          # Architectural documentation and test protocol
├── platformio.ini                     # PlatformIO environment configuration
├── CMakeLists.txt                     # ESP-IDF CMake build definition
│
├── include/
│   ├── config.h                       # Hardware pins, task priorities, timings
│   ├── types.h                        # Command and state data structures
│   ├── sync.h                         # FreeRTOS mutexes and queue management
│   ├── parser.h                       # ASCII frame parsing interface
│   ├── actuator.h                     # Hardware PWM driver interface
│   └── tasks.h                        # FreeRTOS task function declarations
│
├── src/
│   ├── main.c                         # Core initialization and task scheduler launch
│   ├── sync.c                         # Queue management with priority brake eviction
│   ├── parser.c                       # Reentrant command parser
│   ├── actuator.c                     # ESP32 LEDC PWM driver
│   ├── task_command_rx.c              # Serial ingest task
│   ├── task_actuate.c                 # Actuator control task
│   ├── task_watchdog.c                # 500ms link supervisor task
│   └── task_status.c                  # Periodic telemetry task
│
└── arduino/
    └── cuert_rtos_controller/
        └── cuert_rtos_controller.ino  # All-in-one Arduino IDE sketch
```
