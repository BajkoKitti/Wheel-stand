# Automotive Suspension Test Stand - DAQ & Telemetry Suite

A real-time Data Acquisition (DAQ) and visualization environment engineered for an automotive suspension mechanical test rig at the University of Oradea. The platform measures, streams, visualizes, and logs applied mechanical loads and structural component displacements during dynamic compression and fatigue cycles.

---

## Key Features

- **High-Throughput Sampling:** Synchronized multi-sensor acquisition running at ~30 Hz (Δt &approx; 33 ms).
- **Industrial Modbus RTU Interfacing:** Differential RS-485 bus communication with industrial load cell transmitters at 115200 baud (8N1).
- **Dynamic I2C Addressing:** Sequential boot sequencing for 3x Time-of-Flight (ToF) laser sensors via dedicated XSHUT control lines (readdressed to `0x30`, `0x31`, `0x32`).
- **Hardware-Accelerated GUI:** 60 FPS real-time waveform rendering powered by PySide6 (Qt) and PyQtGraph with bounded circular buffers (`collections.deque`).
- **Interactive Inspection Tools:** MATLAB-style crosshair inspection with 30 px screen-pixel cursor snapping, persistent marker pinning, and single-click removal.
- **RFC 4180 CSV Data Export:** Complete time-series trial logging with microsecond/host time tracking for post-processing in MATLAB, Excel, or Python.
- **Built-In Virtual Simulator:** Synthetic mathematical suspension generator for offline software validation, demonstration, and student training without physical hardware.

---

## System Architecture

```text
+--------------------------------------------------------------+
|                   AUTOMOTIVE TEST RIG                        |
|                                                              |
|   [Load Cell 1]   [Load Cell 2]       [ToF 1] [ToF 2] [ToF 3]|
|         |               |                 |       |       |  |
|   (Modbus ID 1)   (Modbus ID 2)        (0x30)  (0x31)  (0x32)|
|         +-------+-------+                 +-------+-------+  |
|                 | RS-485                          | I2C +    |
|                 v                                 v XSHUT    |
|        [MAX485 Transceiver]                       |          |
+-----------------|---------------------------------|----------+
                  | UART (D0/D1)                    | (A4/A5, D2-D4)
                  +----------------+----------------+
                                   |
                                   v
                      [Arduino Uno R4 WiFi Node]
                                   |
                                   | USB Serial (115200 baud, JSON)
                                   v
                   [PySide6 / PyQtGraph Desktop App]
```

---

## Hardware Specifications

| Component | Interface | Description / Role |
| :--- | :--- | :--- |
| **Arduino Uno R4 WiFi** | USB-CDC / Logic | Main acquisition controller and JSON packet serializer |
| **2x ATO Load Cells** | RS-485 (Modbus RTU) | Compressive/tensile dynamic force logging (N) |
| **3x VL53L0X ToF Sensors** | I2C (Address Reassigned) | Damper travel and displacement logging (mm) |
| **MAX485 Module** | TTL to RS-485 | Differential bus transceiver |
| **External DC PSU** | 12V–24V DC | Isolated excitation power for load cell transmitters |

---

## Getting Started

### Option 1: Standalone Portable Binary (Recommended for Lab Workstations)

1. Download the latest `Suspension_DAQ.exe` from the **Releases** tab.
2. Connect the DAQ box via USB.
3. Launch `Suspension_DAQ.exe` (no Python or package installation required).
4. Select the corresponding serial COM port (or choose `Simulation Mode (Virtual)`) and click **Connect**.

### Option 2: Running from Source

Ensure Python 3.10+ is installed on the host system.

**1. Clone the repository:**
```bash
git clone https://github.com/BajkoKitti/Wheel-stand.git
cd Wheel-stand
```

**2. Install dependencies:**
```bash
pip install PySide6 pyqtgraph pyserial
```

**3. Run application:**
```bash
python wheel_stand.py
```

---

## Repository Structure

```text
├── firmware/
│   └── suspension_firmware.ino    # Arduino Uno R4 acquisition code
├── software/
│   ├── wheel_stand.py             # Desktop GUI & plotting suite
│   └── requirements.txt           # Python dependency specifications
├── docs/
│   ├── Documentation.docx         # Technical documentation & user manual
└── README.md
```

---
