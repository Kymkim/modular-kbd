# Firmware Architecture

The firmware operates under **three distinct modes** that define the device’s behavior:

```c
#define MODE_INACTIVE 0
#define MODE_MASTER   1
#define MODE_MODULE   2
```

These modes dictate how the system processes input and manages data flow between modules. Together, they establish the rules for communication and arbitration within the modular keyboard system.


## Master Mode

In **Master Mode**, the device behaves like a standard keyboard connected to a host via USB. However, it also takes on additional responsibilities:

* **USB Handling**: Sends keypress reports directly to the host.
* **Neighbor Listening**: Receives keypress messages from connected Module devices and integrates them into the USB report.
* **Device Queries**: Responds to discovery or status requests from devices currently in **Inactive Mode**.

👉 The system can only enter Master Mode if it is physically connected to a USB host.

## Module Mode

In **Module Mode**, the device functions similarly to Master Mode but with one key difference:

* Instead of sending reports to the USB host, the device forwards its keypress data upstream to its **parent device**.

This parent is determined during the discovery process in Inactive Mode. The parent may be either the Master itself or another Module closer to the Master.

## Inactive Mode

**Inactive Mode** is the initial state of every device upon power-up (unless it is directly connected to USB).

Behavior in this mode includes:

* **Ignoring Local Inputs**: The device does not process its own switch presses.
* **Discovery Queries**: The device broadcasts a query to its neighbors to determine its parent.
* **Awaiting Responses**: Active devices (Master or Modules) respond with a **depth value**—a numerical measure of distance (in hops) to the Master.

  * The Master has a depth of **0**.
  * Each Module reports its depth as `(parent depth + 1)`.

The querying device selects a parent based on the lowest reported depth. This ensures the shortest and most efficient communication path to the Master.

**Example:**
If device A reports depth = 2, and device B reports depth = 4, the querying device sets:

* **Parent = A**
* **Depth = 3**

After selecting a parent, the device transitions from **Inactive Mode → Module Mode**.

Devices already in Master or Module mode should respond to discovery queries with their current depth value.

# UART Queue with DMA

Data transmission in this system is handled via **UART**. However, standard UART operations are **blocking**:

* When sending or receiving data without DMA, the CPU must wait until the operation completes.
* Running UART directly in the main loop would **stall other critical processes**, such as scanning keypresses or handling module communication.

An alternative is using **interrupts**, which allow the CPU to continue running while UART signals completion.

**Problem:** Interrupts can occur at any moment, potentially interrupting time-sensitive operations like key scanning or other UART transactions.
* Frequent or nested interrupts could lead to **race conditions**, data corruption, or missed key events.


## DMA Solution

The system uses **DMA (Direct Memory Access)** to offload UART data handling:

* Each RX port operates via DMA, continuously collecting incoming data **without CPU intervention**.
* Received data is placed into a **queue**.
* During the main loop, the firmware checks the queue:

  * If the queue is not empty, it transmits the collected data to the parent/master device.

This system allows UART to behave like a **background subprocess**, leaving the CPU free for:

* Scanning the module’s own keys.
* Processing received messages.
* Managing the module-to-master communication efficiently.

## Queue Usage

The **same queue system** also handles the module’s own keypresses:

* Key events are pushed into the queue as they are detected.
* The main loop processes the queue, sending keypresses to the master device.

This unified queue system ensures **non-blocking, deterministic communication** between modules and the master, even under high data load.
---

# UART Message Structure  

Each UART message is **32 bits (4 bytes)** and structured as follows:  

| Byte 0 (MSB)        | Byte 1            | Byte 2      | Byte 3 (LSB) |
|---------------------|-----------------|------------|--------------|
| Sender’s Depth      | Message Type      | Extra Bits | Key Code     |

**Field Descriptions:**  

- **Sender’s Depth:** Indicates the module’s distance from the Master device. Useful for routing and arbitration.  
- **Message Type:** Determines the purpose of the message (see below).  
- **Extra Bits:** Reserved for additional flags or future extensions.  
- **Key Code:** Represents the key being pressed (or released).  


## Message Types  

| Value  | Name                   | Description                                                                 |
|--------|------------------------|-----------------------------------------------------------------------------|
| 0x0F   | Handshake Request       | Sent by modules in **Inactive Mode** to discover parent/master devices.    |
| 0xFF   | Handshake Confirmation  | Sent by active modules (or Master) in response to a Handshake Request.    |
| 0x01   | Keycode Message         | Generic keycode message sent from module to master.                        |


**Notes:**  

- All communication between modules follows this 4-byte structure.  
- Depth values help modules select the optimal parent device.  
- Reserved **Extra Bits** can later store flags like key release, special functions, or priority indicators.  

