# **lkmotor_mf4005_driver – Zephyr Driver for MF4005 Brushless Motor (CAN)**

`lkmotor_mf4005_driver` is an out-of-tree **Zephyr RTOS driver** for the **MF4005 brushless PMSM/BLDC motor** from LK Motor, communicating over **CAN bus**.
It provides a clean and extensible interface to command **torque**, **velocity**, and **position**, using the motor’s internal controller.

This driver is useful for **robotics**, **automation**, **mechatronics**, and any embedded system that requires deterministic motor control within Zephyr.

---

## 🌟 Features

* **Full CAN protocol support** for LK Motor MF4005
* **Closed-loop control modes:**

  * Torque control
  * Velocity control
  * Position control
* **Zephyr-native integration**

  * Kconfig options
  * Devicetree bindings
  * Standard Zephyr driver API structure
* **Shell-based control interface** (optional)

  * Send torque/velocity/position commands at runtime
* **Sample applications included**
* **Modular design** → easy integration with sensors, feedback loops, and higher-level robotics frameworks

---

## 📦 Repository Structure

```
lkmotor_mf4005_driver/
│
├── drivers/                      # Source code for the MF4005 driver
├── dts/                          # Devicetree bindings
├── include/zephyr/drivers/       # Public driver headers
├── samples/
│   └── mf4005_shell/             # Example: control motor via shell commands
│
├── Kconfig                       # Driver configuration options
├── CMakeLists.txt                # Build integration (out-of-tree module)
├── west.yml                      # Manifest for west-based integration
├── README.md                     # <—— You are here
└── LICENSE                       # Apache-2.0
```

---

# 🚀 Quick Start Guide

## 1. Add the module to your Zephyr project

### **Option A — Using `west` directly**

```bash
west init -m https://github.com/uLipe/lkmotor_mf4005_driver
west update
```

### **Option B — Adding to your project’s `west.yml`**

```yaml
remotes:
  - name: uLipe
    url-base: https://github.com/uLipe

projects:
  - name: lkmotor_mf4005_driver
    remote: uLipe
    revision: main
    path: modules/lib/lkmotor_mf4005_driver
```

Then run:

```bash
west update
```

---

## 2. Enable the driver in `prj.conf`

```conf
CONFIG_LKMOTOR_MF4005_DRIVER=y
```

Enable optional shell interface:

```conf
CONFIG_MF4005_DRIVER_SHELL=y
```

---

## 3. Configure devicetree

Example DTS snippet:

```dts
&can0 {
    status = "okay";
    bus-speed = <1000000>;

    mf4005: motor@1 {
        compatible = "lkmotor,mf4005";
        reg = <1>;                 // Motor CAN ID
        can = <&can0>;
        status = "okay";
    };
};
```

This exposes a Zephyr device:

```c
const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(mf4005));
```

---

## 4. Build & Flash

```bash
west build -b <your_board> path/to/app
west flash
```

If shell support is enabled, run:

```
mf4005 help
```

---

# 🎮 Using the Driver

## Acquire the device

```c
#include <zephyr/drivers/mf4005.h>

const struct device *motor = DEVICE_DT_GET_ONE(lkmotor_mf4005);

if (!device_is_ready(motor)) {
    printk("MF4005 motor not ready!\n");
}
```

---

## Control commands

### **Torque control**

```c
mf4005_set_torque(motor, 0.3f);   // 30% torque
```

### **Velocity control**

```c
mf4005_set_velocity(motor, 120.0f);   // 120 deg/s or RPM-equivalent (depends on motor spec)
```

### **Position control**

```c
mf4005_set_position(motor, 90.0f);    // Move to 90 degrees
```

### Read status

```c
struct mf4005_status st;
mf4005_get_status(motor, &st);

printk("Pos: %d\n", st.position);
printk("Vel: %d\n", st.velocity);
printk("Fault: %d\n", st.error_flags);
```

---

# 🐚 Shell Example (Interactive Control)

If `CONFIG_MF4005_DRIVER_SHELL=y`:

```
mf4005 torque 0.2
mf4005 velocity 100
mf4005 position 45
mf4005 status
```

This is extremely useful for:

* tuning
* verifying wiring
* validating CAN bus timing
* debugging motion profiles

---

# 🧪 Sample Application

Included sample:

```
samples/mf4005_shell/
```

Demonstrates:

* Devicetree motor declaration
* Initialization flow
* Shell command handling
* Basic control execution

Use this as a template for real projects.

---

# 🛠 Internal Design Overview

This driver follows Zephyr’s standard driver architecture:

### ✔ **Devicetree binding** (`lkmotor,mf4005`)

Defines:

* CAN bus handle
* Motor ID
* Driver instance

### ✔ **Driver API**

Functions exposed in `include/zephyr/drivers/mf4005.h`

### ✔ **Runtime data + config**

Split into:

* `struct mf4005_config`
* `struct mf4005_data`

### ✔ **CAN protocol layer**

Responsible for:

* Formatting and encoding motor commands
* Parsing returned frames
* Tracking state
* Abstraction of torque/velocity/position commands

### ✔ **Thread-safety**

Most operations are typically done inside:

* main thread
* motion control tasks
* interrupt callbacks (CAN RX)

### ✔ **Decoupling**

Allows integration with:

* estimators
* control loops
* ROS 2 nodes
* sensor fusion
* trajectory planners

---

# 📡 Hardware Requirements

* MF4005 brushless motor (CAN interface)
* CAN transceiver compatible with your board
* ESP32, STM32, NRF, or any Zephyr-supported MCU with CAN controller
* Stable power supply for the motor
* Proper bus termination (120Ω)

---

# 💡 Future Improvements

* Multi-motor support on shared CAN bus
* Better telemetry (temperature, torque feedback, faults)
* Trajectory generator integration
* Stronger configuration through Kconfig
* Optional ROS 2 interface layer
* More advanced diagnostics via shell

---

# 🤝 Contributing

Pull requests, bug reports, and suggestions are welcome!
Feel free to:

* Open issues
* Submit PRs
* Share test findings
* Request new features

---

# 📄 License

Licensed under the **Apache-2.0 License**.
See `LICENSE` for full details.
