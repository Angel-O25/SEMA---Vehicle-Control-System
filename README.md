# 🏎️ VCS: Vehicle Control System (Shell Eco-marathon Edition)

The **Vehicle Control System (VCS)** is a high-reliability, RTOS-driven control architecture designed for the **Shell Eco-marathon (SEM)**. It serves as the deterministic bridge between high-level autonomous navigation (Raspberry Pi) and high-power execution (1500W BLDC Motor & Stepper Steering).

![3D Render of the VCS V1.4 Isolated PCB](assets/image_b42ac8.png)

## 🤖 AI Collaboration
This system was architected and refined in collaboration with **Gemini 3 Flash**. Key focus areas included:
* **Hardware Security & PCB Design:** Engineering a custom 100x100mm dual-layer PCB featuring strict galvanic isolation, dual ground planes, and an optocoupler "moat" to protect 3.3V logic from 60V motor noise.
* **Mbed OS Integration:** Transitioning to a multi-threaded architecture on the Arduino Nano 33 BLE.
* **Safety Engineering:** Implementing a Hardware Watchdog, CRC16 checksums, and prioritized interrupt-based overrides.
* **Digital Twin:** A HIL (Hardware-in-the-Loop) simulation module for lab-testing logic without high-voltage power.
* **Dual-Hand Deadman Logic (V1.5):** Implementing a software-debounced "AND" gate for driver readiness.
* **Organizer Relay Integration (V1.5):** Meeting SEM technical requirements for a physical vehicle state signal (Active-Low).
* **Software Speed Limiting (V1.5):** Replacing hardware optocouplers with dynamic throttle multipliers (30%/60%/100%) to protect the drivetrain.

---

## 🚀 Version History

### **v1.5 (Current - Safety Compliance & Logic Refactor)**
*The "Rules Compliance" release. Focuses on meeting explicit SEM autonomous safety requirements.*
* **Dual-DMS Logic:** Transitioned from a single button to a dual-grip system. Both must be held for 1.0s to engage Autonomous mode.
* **Organizer Interface:** Repurposed `D12` and `A6` to drive the mandatory State Relay and 12V Orange Strobe.
* **Software Governor:** Speed limits are enforced via a `getMaxThrottleMultiplier()` software clamp (30%, 60%, 100%).
* **Protocol Hardening:** Finalized the 14-byte CRC16 command/telemetry packet structure for high-noise environments.

### **v1.4 (Legacy - Custom PCB & Galvanic Isolation)**
*The "Hardware Security" release. Focuses on migrating from loose wire prototypes to a professional, noise-immune physical board.*
* **Custom PCB:** 100x80mm form factor with dedicated terminal blocks for perimeter harness wiring.
* **Analog Signal Conditioning:** On-board LM358 Op-Amp and RC filtering (`C1`, `C2`) for clean, hardware-smoothed throttle output.
* **5V Logic Translation:** Dedicated TXS0108E level-shifting IC for reliable communication with the stepper driver and peripherals.

### **v1.3 (Legacy - Mbed OS & Reliability)**
*The "RTOS" release. Focused on deterministic timing.*
* **Mbed OS Threads:** 1kHz Control Loop, 100Hz Comm/Safety Loop, and 20Hz UI/Telemetry Loop.
* **Hardware Watchdog:** Integrated nRF52840 watchdog (2s timeout) for SEM technical compliance.

### **v1.2 & v1.1 (Legacy - ESP32/STM32)**
*Focus on steering precision and initial modular isolation.*
* **Stepper Steering:** High-torque microstepping replaced legacy DC motor steering.

---

## 🛡️ SIDLAK Safety Architecture
The VCS enforces a strict priority hierarchy. Motor power is physically impossible unless the system evaluates all safety conditions as "Clear."

| State | SEM Requirement | Description |
| :--- | :--- | :--- |
| **INIT** | Self-Test | Boot sequence, optocoupler lockout, and hardware health check. |
| **IDLE** | Safety Standby | System alive; motor disabled; awaiting RPi heartbeat/handshake. |
| **MANUAL** | Human Control | Primary mode. Driver-operated via throttle pedal and steering. |
| **AUTONOMOUS**| Computer Control | Requires **Dual-Hand DMS hold** and verified 10Hz RPi heartbeat. |
| **FAULT** | Fail-Safe | Triggered by comms loss or sensor spikes. **Instant Motor Kill.** |
| **ESTOP** | Emergency Stop | Physical hardware lockout or pedal override (Rule 412c). |

---

## 🔌 Comprehensive Hardware & Wiring Guide (V1.5)

This section serves as the master reference for the V1.4/V1.5 PCB, detailing the exact signal path from the physical perimeter terminal blocks to the Arduino Nano 33 BLE software pins based on the official schematic.

Schematic Diagram of Vehicle Control System
![Schematic Diagram of Vehicle Control System](assets/image_3099c0.png)

PCB Footprint
![Close-up of final silkscreen labels for V1.4 perimeter terminal blocks](assets/image_3099c1.png)

### 📥 Inputs (Sensors & Driver Controls)
| Terminal Block | MCU Pin | Hardware Path / Conditioning | Function |
| :--- | :--- | :--- | :--- |
| **POWER** | `5V` | Direct to Nano `5V` via **C3 (1000µF)** | Main logic power (+5V input). Bulk smoothed. |
| **STEER** | `A0` | Direct trace to ADC | 10-turn Steering Potentiometer (+3.3V reference). |
| **THRT** | `A1` | Direct trace to ADC | Driver Throttle Pedal (0-3.3V). **Rule 412c Override.** |
| **ROTARY** | `A2, A3, A7`| Direct trace (`INPUT_PULLUP`) | Gear Selector. `A2` (Reverse), `A3` (Neutral), `A7` (Drive). |
| **DMSC_L** | `D2` | Direct trace (`INPUT_PULLUP`) | **Left** Dead Man's Switch Grip. Active-Low. |
| **DMSC_R** | `D4` | Direct trace (`INPUT_PULLUP`) | **Right** Dead Man's Switch Grip. Active-Low. |
| **BRKP** | `D8` | Hardware Debounce via **C2 (0.1µF)** | Physical Brake Pedal Switch. **Rule 412c Override.** |
| **SPS+** | `D10` | 5V to 3.3V via **TXB0108 Level Shifter** | Motor Hall-Effect Speed Sensor input. |
| **3SW_L** | `D11` | Level Shifted Input | 3-Speed Switch (LOW position). |
| **3SW_H** | `D13` | Level Shifted Input | 3-Speed Switch (HIGH position). |

### 📤 Outputs (Actuators & Motor Signals)
| Terminal Block | MCU Pin | Hardware Path / Security Barrier | Function |
| :--- | :--- | :--- | :--- |
| **BRKS+** | `D3` | **PC817 Optocoupler** | Galvanically isolated Brake Signal to Motor Controller. |
| **MISTECON** | `D5, D6, D7`| **TXB0108 Level Shifter** | Stepper control. **Wired Common Anode.** |
| **THRTS+** | `D9` | **RC Filter + LM358 Op-Amp** | Analog Throttle Signal to Motor Controller (0-5V). |
| **RELAY_ST** | `D12` | **TXB0108 -> Relay** | Organizer State Signal (State Relay). |
| **STROBE** | `A6` | **Direct/Transistor Drive** | 12V Orange Indicator Strobe. |

---

## 🔒 Hardware Security Architecture & Assembly Rules

TOP LAYER OF PCB
![V1.4 Dual Ground Planes demonstrating strict galvanic isolation (the central moat)](assets/image_b43b29.png)

BOTTOM LAYER OF PCB
![V1.4 Dual Ground Planes demonstrating strict galvanic isolation (the central moat)](assets/image_b41b29.png)

1. **The 5V Translation Bridge:** The **TXS0108E** high-speed bidirectional level shifter guarantees that 5V noise from the peripherals cannot back-feed into the 3.3V logic pins. 
2. **Common-Anode Stepper Wiring:** The Nano triggers steps by pulling the negative pins (`PUL-`, `DIR-`, `ENA-`) to ground via the level shifter. 
3. **The Analog Clean Room:** Routes the `D9` PWM through an RC low-pass filter (`C1`=10µF, `R1`=10kΩ) to create true DC, then pushes it through the **LM358 Op-Amp** to buffer the signal.
4. **Active-Low Fail-Safes:** Critical inputs (`DMSC`, `BRKP`) are wired active-low. If a wire is severed, the system defaults to a safe deactivated state.

---

## 📦 Bill of Materials (BOM) - V1.5 PCB

### Integrated Circuits & Core Modules
| Designator | Component | Function |
| :--- | :--- | :--- |
| **NANO1** | Arduino Nano 33 BLE | Primary MCU (3.3V Logic, RTOS). |
| **TXS0108E** | TXS0108E Module | Bidirectional Level Shifter. Translates 3.3V/5V signals. |
| **LM-358** | LM-358 | Dual Op-Amp. Buffers the hardware-filtered PWM throttle. |
| **PC817** | PC817 | Optocoupler. Galvanic barrier for the 60V Brake line. |

### Passive Components (Resistors & Capacitors)
| Designator | Value | Purpose |
| :--- | :--- | :--- |
| **C3** | `1000µF` | Bulk electrolytic capacitor. Absorbs voltage spikes on power input. |
| **C1** | `10µF` | Smoothing capacitor for the Op-Amp RC filter network. |
| **C2** | `0.1µF` | Hardware debounce capacitor for the Brake Pedal switch. |
| **R1, R3** | `10kΩ` | Throttle RC filter network and voltage dividers. |
| **R2** | `3.3kΩ` | Throttle RC filter network inline resistor. |
| **R4** | `220Ω` | Current-limiting resistor for the Optocoupler's internal LED. |

---

## 📡 Communication Protocol (V1.5 CRC16)

### **Uplink (Nano → Pi) - Telemetry [Type 0x02]**
**Format:** `[0xAA][0x55][0x02][0x07][RPM(2)][STEER(2)][STATE(1)][GEAR(1)][REV(1)][CRC(2)][0xFF]`
* **RPM:** 16-bit Signed Integer (High/Low Byte).
* **Steer:** 16-bit Unsigned (0-1000).
* **State:** SIDLAK State Index (3 = Autonomous).
* **Gear:** 3-speed position (0=Low, 1=Med, 2=High).

### **Downlink (Pi → Nano) - Command [Type 0x01]**
**Format:** `[0xAA][0x55][0x01][0x07][MODE][RPM(2)][STEER(2)][BRAKE(1)][REV(1)][CRC(2)][0xFF]`
* **Mode:** 1 = Request Autonomous, 0 = Manual.
* **RPM:** Target RPM (0 to 3000).
* **Steer:** Target Steering (500 = Center).
* **Rev:** 1 = Request Reverse (Gated by RPM < 5).

---

## 💻 Building and Testing (HIL Simulation)
1. Set `#define SIMULATION_MODE 1` in `vcs_constants.h`.
2. Compile and upload via **PlatformIO**.
3. Monitor Serial Monitor (**115200 baud**) for physics emulation.
4. Verify that **Rule 412c** triggers a state drop when the throttle pedal is touched during a simulated Auto-run.
