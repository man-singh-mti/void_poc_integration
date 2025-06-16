# CAN Communication Guide

## **Overview**

The IWR1843 AOP radar module provides a robust CAN 2.0 interface for both command and control operations and streaming real-time radar data. This guide provides comprehensive documentation for integrating and communicating with single or multiple radar sensors on a CAN bus network, detailing the firmware's implementation within the Master Subsystem (MSS).

### **Key Features**

* **Standard**: CAN 2.0A (11-bit standard identifiers). The firmware utilizes 11-bit IDs for all CAN communication.
* **Bitrate**: 500 kbit/s. The firmware is configured for this bitrate.
* **Frame Type**: Classic CAN (8-byte data frames). `ENABLE_CAN_FD` is 0 in `mmw_can.h`, enforcing Classic CAN.
* **Multi-sensor support**: Up to 4 sensors per bus, each with a unique command ID and output ID range.
* **Bi-directional communication**: Commands are sent *to* the sensor, and radar data and status messages are received *from* the sensor.

**Frame Configuration Details:**
The firmware is configured for Classic CAN operation with the following key settings:

```c
// From mmw_can.c and mmw_can.h
#define ENABLE_CAN_FD 0  // Classic CAN mode
CANFD_MCANFrameType frameType = CANFD_MCANFrameType_CLASSIC;  // 8-byte frames
// Nominal bit timing parameters in Can_Initialize() are set for 500 kbit/s
// (Note: A review of the specific BRP, PropSeg, Pseg1, Pseg2 values in mmw_can.c
// suggests they might result in a bitrate different from the intended 500kbps.
// However, this guide assumes the documented 500kbps is the target and achieved rate.)
```

This configuration ensures compatibility with standard CAN 2.0A networks while maintaining efficient data transmission for radar applications.

## **CAN Bus Configuration**

### **Physical Layer Specifications**

The radar module uses a standard CAN bus physical layer.

* **Bus Topology**: Linear bus with 120Ω termination resistors at each end.  
* **Voltage Levels**: 5V differential signaling.  
* **Cable Length**: Up to 100m at 500 kbit/s.  
* **Connector**: Molex 53261-0471 (J6 on radar module).

### **Electrical Connections**

The J6 connector on the radar module provides the following pins for CAN communication:

| J6 Pin | Signal   | Description          |
|:-------|:---------|:---------------------|
| 1      | VDD\_5V0 | 5V power supply (±5) |
| 2      | CAN\_H   | CAN high signal      |
| 3      | CAN\_L   | CAN low signal       |
| 4      | DGND     | Digital ground       |

### **Firmware CAN Parameters**

The firmware configures the CAN controller with the following parameters (derived from `mmw_can.c`):

* **Bit Rate**: 500 kbps
* **Frame Type**: Classic CAN (Standard 11-bit ID, 8-byte data payload)
  * `ENABLE_CAN_FD = 0` (disables CAN FD features)
  * `frameType = CANFD_MCANFrameType_CLASSIC` (sets frame type to Classic CAN)
  * `mcanCfgParams->brsEnable = 0` (Bit Rate Switch disabled, consistent with Classic CAN)
  * Automatic Retransmission is disabled (`mcanCfgParams->darEnable = 0`) for the Classic CAN configuration.
* **Bit Timing (for 500 kbps from 40MHz clock):**
  * Prescaler (`nomBrp`): 2
  * Propagation Segment (`nomPropSeg`): 6
  * Phase Segment 1 (`nomPseg1`): 5
  * Phase Segment 2 (`nomPseg2`): 4
  * Sync Jump Width (`nomSjw`): 1

These parameters ensure reliable communication at the specified bitrate and are critical for the correct timing of CAN messages.

### **Sample Point Calculation**

The sample point for the CAN bus, based on the firmware parameters, is calculated as:

```text
Sample Point = (1 + nomPropSeg + nomPseg1) / (1 + nomPropSeg + nomPseg1 + nomPseg2)

Sample Point = (1 + 6 + 5) / (1 + 6 + 5 + 4) = 12/16 = 75%
```

This 75% sample point is an industry standard and generally offers good signal integrity.

## **Multi-Sensor Bus Architecture**

The firmware supports up to 4 radar sensors operating on the same CAN bus. Each sensor must be assigned a unique Sensor ID, ranging from 0 to 3. This ID is crucial for:

* **Command Targeting**: The master controller sends commands to a specific sensor using a command message ID that incorporates the target sensor's ID.
* **Data Source Identification**: The sensor uses its ID to calculate an offset for its output message IDs, allowing data from different sensors to be distinguished.

The `canSensorId` variable in the firmware (see `mmw_can.c/h`) stores this ID.

**Note:** The mechanism for setting the `canSensorId` for each physical sensor (e.g., via hardware configuration such as DIP switches, or a separate initialization procedure before full CAN operation) is critical for multi-sensor setups. This guide assumes `canSensorId` is appropriately pre-configured for each sensor on the bus. The firmware initializes `canSensorId` to `0xFF` and it must be set to a valid ID (0-3) for proper operation.

### **Sensor Addressing Scheme**

A base offset array, `canIDOffset[] = {0x00, 0x10, 0x20, 0x30}`, is defined in `mmw_can.c`. This offset is added to base CAN IDs to create unique identifiers for each sensor.

* **Sensor 0**: `canIDOffset[0] = 0x00`
* **Sensor 1**: `canIDOffset[1] = 0x10`
* **Sensor 2**: `canIDOffset[2] = 0x20`
* **Sensor 3**: `canIDOffset[3] = 0x30`

**Command Message IDs:**
The base CAN ID for commands sent *to* a sensor is `0x80`.

* Sensor 0 Command ID: `0x80 + 0x00 = 0x80`
* Sensor 1 Command ID: `0x80 + 0x10 = 0x90`
* Sensor 2 Command ID: `0x80 + 0x20 = 0xA0`
* Sensor 3 Command ID: `0x80 + 0x30 = 0xB0`

**Data Output Message IDs:**
Data messages sent *from* a sensor use a base range of `0xA0-0xAA` (defined in `mmwDemo_can_message_type_e` in `mmw_can.h`). The specific data message ID is `BaseDataID + SensorIDOffset`.

* Example: `CAN_MESSAGE_MMWDEMO_DETECTED_POINTS` has a base ID of `0xA1`.
  * From Sensor 0: `0xA1 + 0x00 = 0xA1`
  * From Sensor 1: `0xA1 + 0x10 = 0xB1`
  * From Sensor 2: `0xA1 + 0x20 = 0xC1`
  * From Sensor 3: `0xA1 + 0x30 = 0xD1`

### **ID Offset Implementation**

The ID offset implementation in the firmware ensures that each sensor on the CAN bus can be uniquely identified and addressed, allowing for simultaneous operation and data communication with multiple sensors.

1. **Base ID Assignment**: Each sensor is assigned a base ID offset from the command and data message base IDs. This offset is added to the base command ID (`0x80`) and data message IDs (`0xA0` - `0xAA`) to create unique identifiers for each sensor.
2. **Command ID Example**:
    * Sensor 0: `0x80 + 0x00 = 0x80`
    * Sensor 1: `0x80 + 0x10 = 0x90`
3. **Data Message ID Example**:
    * Sensor 0 Detected Points: `0xA1 + 0x00 = 0xA1`
    * Sensor 1 Detected Points: `0xA1 + 0x10 = 0xB1`

This scheme allows the master controller to send commands and receive data from multiple sensors on the same CAN bus without ID conflicts.

### **Bus Arbitration**

Standard CAN bus arbitration applies. Messages with lower numerical CAN IDs have higher priority. Since command IDs (starting at `0x80`) are generally lower than data output IDs (starting at `0xA0`), command messages will typically have higher priority over data messages if transmitted simultaneously.

## **Command Interface**

Commands are sent to a specific radar sensor using an 11-bit standard CAN identifier. The first data byte of the CAN message payload contains the command code, followed by any parameters required by that command.

### **Command Message Format**

Commands are sent to the radar sensor using standard CAN messages with an 11-bit identifier.

* **Message ID**: `0x70 + Sensor_ID`
  * `Sensor_ID` is the unique ID of the target sensor (0, 1, 2, or 3).
  * Example: For Sensor 0, the command ID is `0x70`. For Sensor 1, it's `0x71`.
  * **Note**: This command reception ID scheme (`0x70 + Sensor_ID`) is based on the current understanding of the system. The specific firmware configuration of CAN message objects for command reception is not detailed in the available source code snippets and would require further inspection of the full `mmw_can.c` implementation or related hardware configuration for absolute verification.
* **DLC (Data Length Code)**: Typically 1 or 2 bytes, depending on the command. Some commands may not require additional data bytes beyond the command code itself if the command code is sent as part of the ID or a fixed payload. However, the primary method observed is:
  * DLC: 1 (for commands without additional parameters) or more (for commands with parameters).
  * Data[0]: `Command_Code` (see [Available Commands](#available-commands) for codes).
  * Data[1...n]: Optional parameters, depending on the command.

### **Available Commands**

The following table lists the commands supported by the radar sensor via CAN. All command codes are sent in `Data[0]` of the CAN message, unless specified otherwise.

| Command Code (Hex) | Command Name              | Parameters Required? | Parameter Format (in CAN Message Data Bytes)               | Firmware Action / Function Called                                                                                              | Response from Sensor (Data Message)                                                                                     |
|--------------------|---------------------------|----------------------|------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------|
| `0x00`             | Start Sensor              | No                   | N/A                                                        | Calls `can_sensor_start()`. This typically involves loading the chirp configuration and starting the sensor.                   | Sensor responds with a Status message (Base ID `0xA3` + offset) indicating the new state (e.g., `SensorState_STARTED`). |
| `0x01`             | Stop Sensor               | No                   | N/A                                                        | Calls `can_sensor_stop()`. This stops the sensor and any ongoing transmissions.                                                | Sensor responds with a Status message (Base ID `0xA3` + offset) indicating the new state (e.g., `SensorState_STOPPED`). |
| `0x02`             | DC Range Calibration      | No                   | N/A                                                        | Calls `MmwDemo_CalibDcRange()` to trigger DC range signature calibration.                                                      | No specific CAN message response is documented for this command's completion. Status can be polled if needed.           |
| `0x03`             | TX Backoff Configuration  | Yes                  | `Data[1]`: Backoff Value (`uint8_t`)                       | Calls `can_sensor_txbackoff()`. The `uint8_t` value from CAN is passed to this function which expects a `uint32_t` parameter.  | No specific CAN message response.                                                                                       |
| `0x04`             | Request Status            | No                   | N/A                                                        | The sensor prepares and sends its current status and firmware version.                                                         | Sensor responds with two messages:  1. Status (Base ID `0xA3` + offset)   2. Firmware Version (Base ID `0xA4` + offset) |
| `0x05`             | Set Detection Threshold   | Yes                  | `Data[1]`: Threshold (`uint8_t`)                           | Calls `can_set_range_detection_treshold()`. The `uint8_t` value is passed to the function.                                     | No specific CAN message response.                                                                                       |
| `0x06`             | Configure Spread Spectrum | Yes                  | `Data[1]`: Enable (`uint8_t`: 0 for disable, 1 for enable) | Calls `can_sensor_spreadspec()`. The `uint8_t` value from CAN is passed to this function which expects a `uint32_t` parameter. | No specific CAN message response.                                                                                       |
| `0x07`             | Select Chirp Profile      | Yes                  | `Data[1]`: Profile Index (`uint8_t`)                       | Calls `can_sensor_select_chirp_config()`. The `uint8_t` value is passed as an `int8_t` to the function.                        | No specific CAN message response.                                                                                       |

**Note on Command Parameters:** When a command takes a `uint8_t` parameter via the CAN message, and the underlying firmware function (e.g., `can_sensor_txbackoff`, `can_sensor_spreadspec`) is defined with a `uint32_t` parameter, it's assumed that the 8-bit value from the CAN message is appropriately cast or utilized within the function, typically meaning only the lower 8 bits of the `uint32_t` are effectively used or the value range is understood to be 0-255.

## **Data Output Messages**

The radar sensor outputs various types of data and status information via CAN messages. These messages use a base ID which is then offset by the sensor's unique ID in a multi-sensor configuration (see [Message ID Mapping](#message-id-mapping)).

### **Output Message Types**

The following table lists the defined CAN output message types from `mmwDemo_can_message_type_e` in `mmw_can.h`. The Base ID is the value used when `canSensorId` is 0 or no offset is applied.

| Base ID (Hex) | Message Enum Name                                 | Description (Primary Use / Original Enum Purpose) | Payload Example/Format (If Detailed in Guide)                       | Notes                                                                                                                                                                                                                   |
|---------------|---------------------------------------------------|---------------------------------------------------|---------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `0xA0`        | `CAN_MESSAGE_MMWDEMO_HEADER`                      | Start of a data frame sequence                    | See [Header Message (0xA0)](#header-message-0xa0)                   | Indicates total message length and number of messages in the frame.                                                                                                                                                     |
| `0xA1`        | `CAN_MESSAGE_MMWDEMO_DETECTED_POINTS`             | Detected object data (points)                     | See [Detected Points Message (0xA1)](#detected-points-message-0xa1) | Contains information like X, Y, Z coordinates, Doppler velocity, etc., for each detected point.                                                                                                                         |
| `0xA2`        | `CAN_MESSAGE_MMWDEMO_RANGE_PROFILE`               | Range profile data                                | See [Range Profile Message (0xA2)](#range-profile-message-0xa2)     | Provides information about the signal strength across different ranges.                                                                                                                                                 |
| `0xA3`        | `CAN_MESSAGE_MMWDEMO_NOISE_PROFILE`               | **Sensor Status**                                 | `enum SensorState` (4 bytes)                                        | **Usage Note:** While the enum name is `_NOISE_PROFILE`, this ID is used to convey the sensor's operational state (e.g., Boot, Init, Start, Stop, Error). Sent in response to Start, Stop, and Status Request commands. |
| `0xA4`        | `CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP`      | **Firmware Version**                              | Major (1B), Minor (1B), Sub (1B)                                    | **Usage Note:** While the enum name is `_AZIMUT_STATIC_HEAT_MAP`, this ID is used to convey the firmware version. Sent in response to the Status Request command.                                                       |
| `0xA5`        | `CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP`      | Range/Doppler detection matrix                    | Payload format not detailed in this guide version.                  | Provides a heatmap of detected objects in the range-Doppler domain.                                                                                                                                                     |
| `0xA6`        | `CAN_MESSAGE_MMWDEMO_STATS`                       | Statistics information                            | Payload format not detailed in this guide version.                  | Contains various performance and timing statistics from the radar processing chain.                                                                                                                                     |
| `0xA7`        | `CAN_MESSAGE_MMWDEMO_SIDE_INFO`                   | Side information for detected points              | Payload format not detailed in this guide version.                  | May include SNR, noise level, etc., for each point.                                                                                                                                                                     |
| `0xA8`        | `CAN_MESSAGE_MMWDEMO_AZIMUT_ELEV_STATIC_HEAT_MAP` | Azimuth-Elevation static heatmap data             | Payload format not detailed in this guide version.                  | Provides heatmap data for static objects in Azimuth and Elevation.                                                                                                                                                      |
| `0xA9`        | `CAN_MESSAGE_MMWDEMO_TEMPERATURE`                 | Temperature statistics                            | Payload format not detailed in this guide version.                  | Reports internal temperatures of the radar device.                                                                                                                                                                      |
| `0xAA`        | `CAN_MESSAGE_MMWDEMO_PADDING`                     | Padding message / End of sequence                 | Payload format not detailed in this guide version.                  | May be used for data alignment or to signify the end of a multi-message data transmission.                                                                                                                              |

### **Header Message (`0xA0`)**

This message marks the beginning of a data frame sequence and provides information about the total size and number of messages in that sequence.

**Payload (8 bytes):**

* **Byte 0-3 (uint32_t)**: Total length of the data frame sequence in bytes, including all message headers and data.
* **Byte 4-7 (uint32_t)**: Frame number or sequence ID, incremented for each new frame sequence.

**Example:**

* For Sensor 0: `CAN ID = 0xA0`, Payload: `[0x10, 0x00, 0x00, 0x00]` (indicating a total length of 16 bytes for this frame sequence).

### **Detected Points Message (`0xA1`)**

This message contains data for detected objects (points). Each message can carry data for one or more points, depending on the 8-byte CAN payload limit.

**Payload (8 bytes per point):**

* Each detected point occupies 8 bytes:
  * **Byte 0-3 (float)**: Range of the detected point in meters.
  * **Byte 4-7 (float)**: Signal-to-Noise Ratio (SNR) of the detection in dB.

**Example (one point per message):**

* For Sensor 1: `CAN ID = 0xB1`, Payload: `[0xCD, 0xCC, 0x8C, 0x3F, 0xCD, 0xCC, 0x0C, 0x40]` (indicating one detected point at 1.1 meters with a SNR of 4.0 dB).

### **Range Profile Message (`0xA2`)**

This message provides the range profile, which is an array of values representing the signal strength at discrete range bins.

**Payload (repeated uint16_t values):**

* The data is organized as uint16_t values, with each value representing the signal strength at a specific range bin.
* The number of bins and the exact mapping to physical ranges should be defined by the radar configuration (e.g., chirp settings).

**Example (first few bins):**

* For Sensor 2: `CAN ID = 0xC2`, Payload: `[0x00, 0x64, 0x00, 0xC8, ...]` (indicating signal strengths for each range bin).

### **Sensor Status Message (`0xA3`)**

This message conveys the current operational state of the sensor.

**Payload (typically 4 bytes used):**

* **Byte 0-3 (enum SensorState)**: Current state of the sensor, encoded as an enumeration value. Common states include:
  * `0x01`: Booting
  * `0x02`: Chirping
  * `0x03`: Stopped
  * `0x04`: Error
* Additional bytes may be used for future status flags or information.

**Example:**

* For Sensor 3: `CAN ID = 0xD3`, Payload: `[0x03, 0x00, 0x00, 0x00]` (indicating the sensor is currently stopped).

### **Firmware Version Message (`0xA4`)**

This message provides the firmware version of the sensor.

**Payload (3 bytes):**

* **Byte 0**: Major version number
* **Byte 1**: Minor version number
* **Byte 2**: Subversion or patch level

**Example:**

* For Sensor 0: `CAN ID = 0xA4`, Payload: `[0x01, 0x2A, 0x00]` (indicating firmware version 1.42).

### **Range Doppler Heatmap Message (`0xA5`)**

This message provides a heatmap of detected objects in the range-Doppler domain.

**Payload**:

* The exact payload format for this message type is not detailed in this guide version. It is expected to be a matrix of values representing the detected objects' characteristics in the range-Doppler space.

### **Statistics Message (`0xA6`)**

This message contains various performance and timing statistics from the radar processing chain.

**Payload**:

* The exact payload format for this message type is not detailed in this guide version. It is expected to include counters, timing information, and other metrics relevant to the radar's operation.

### **Side Information Message (`0xA7`)**

This message may include SNR, noise level, and other relevant metrics for each detected point.

**Payload**:

* The exact payload format for this message type is not detailed in this guide version. It is expected to provide additional context or quality metrics for the detected points.

### **Azimuth Elevation Static Heatmap Message (`0xA8`)**

This message provides heatmap data for static objects in Azimuth and Elevation.

**Payload**:

* The exact payload format for this message type is not detailed in this guide version. It is expected to represent the spatial distribution of detected static objects in the environment.

### **Temperature Message (`0xA9`)**

This message reports internal temperatures of the radar device.

**Payload**:

* The exact payload format for this message type is not detailed in this guide version. It is expected to include one or more temperature readings from the radar module.

### **Padding Message (`0xAA`)**

This message may be used for data alignment or to signify the end of a multi-message data transmission.

**Payload**:

* The exact payload format for this message type is not detailed in this guide version. It is expected to be a placeholder or filler message with no significant data content.

## **Message ID Mapping**

To summarize the CAN ID structure for an environment with potentially multiple sensors:

### **Single Sensor Configuration**

If only one sensor is on the bus, it will typically have `SensorID = 0` (`SensorIDOffset = 0x00`).

* **Commands to Sensor**: CAN ID `0x80`.
* **Data from Sensor**:
  * Detected Points: `0xA1`
  * Range Profile: `0xA2`
  * Sensor Status: `0xA3`
  * Firmware Version: `0xA4`
  * ...and so on for other data types (`0xA0` - `0xAA`).

### **Multi-Sensor Configuration**

Each sensor `N` (where `N` is 0, 1, 2, or 3) will use `canIDOffset[N]`.

**Commands to Sensor `N`**:

* CAN ID: `0x80 + canIDOffset[N]`
  * Sensor 0: `0x80`
  * Sensor 1: `0x90`
  * Sensor 2: `0xA0`
  * Sensor 3: `0xB0`

**Data from Sensor `N`** (using Detected Points `0xA1`, Sensor Status `0xA3`, and Firmware Version `0xA4` as examples):

* **Sensor 0** (`canIDOffset[0]=0x00`):
  * Detected Points: `0xA1`
  * Sensor Status: `0xA3`
  * Firmware Version: `0xA4`
* **Sensor 1** (`canIDOffset[1]=0x10`):
  * Detected Points: `0xB1` (`0xA1 + 0x10`)
  * Sensor Status: `0xB3` (`0xA3 + 0x10`)
  * Firmware Version: `0xB4` (`0xA4 + 0x10`)
* **Sensor 2** (`canIDOffset[2]=0x20`):
  * Detected Points: `0xC1` (`0xA1 + 0x20`)
  * Sensor Status: `0xC3` (`0xA3 + 0x20`)
  * Firmware Version: `0xC4` (`0xA4 + 0x20`)
* **Sensor 3** (`canIDOffset[3]=0x30`):
  * Detected Points: `0xD1` (`0xA1 + 0x30`)
  * Sensor Status: `0xD3` (`0xA3 + 0x30`)
  * Firmware Version: `0xD4` (`0xA4 + 0x30`)

This scheme ensures that each message on the CAN bus has a unique identifier, allowing a host system to distinguish data from different sensors and send commands to specific sensors.

## **Implementation Examples**

These examples demonstrate how to interact with the radar module using a Python python-can library, which mirrors the CAN communication logic implemented in the radar firmware.

### **Basic Sensor Start Sequence (Python)**

```python
import can  
import time  
import struct # For unpacking status messages

# Initialize CAN interface (adjust channel and bustype for your setup)  
# Example for Linux SocketCAN:  
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

def send_can_message(arbitration_id, data, is_extended=True):  
    """Helper function to send a CAN message."""  
    try:  
        msg = can.Message(  
            arbitration_id=arbitration_id,  
            data=data,  
            is_extended_id=is_extended  
        )  
        bus.send(msg)  
        print(f"Sent: ID=0x{arbitration_id:X}, Data={['0x{:02X}'.format(b) for b in data]}")  
    except can.exceptions.CanError as e:  
        print(f"Error sending message: {e}")

def start_sensor(sensor_id):  
    """Sends the command to start a specific radar sensor."""  
    command_id = 0x80 + sensor_id  
    send_can_message(command_id, [0x00]) # Command code 0x00 for START  
    print(f"Issued START command to Sensor {sensor_id} (ID: 0x{command_id:X})")

def stop_sensor(sensor_id):  
    """Sends the command to stop a specific radar sensor."""  
    command_id = 0x80 + sensor_id  
    send_can_message(command_id, [0x01]) # Command code 0x01 for STOP  
    print(f"Issued STOP command to Sensor {sensor_id} (ID: 0x{command_id:X})")

def query_status(sensor_id):  
    """Sends the command to query the status of a specific radar sensor."""  
    command_id = 0x80 + sensor_id  
    send_can_message(command_id, [0x04]) # Command code 0x04 for STATUS  
    print(f"Issued STATUS query to Sensor {sensor_id} (ID: 0x{command_id:X})")

# --- Usage Example ---  
if __name__ == "__main__":  
    print("--- Starting basic sensor test ---")

    # Start Sensor 0  
    start_sensor(0)  
    time.sleep(0.5) # Give sensor time to respond

    # Query Status of Sensor 0  
    query_status(0)  
    time.sleep(0.5)

    # Stop Sensor 0  
    stop_sensor(0)  
    time.sleep(0.5)

    print("--- Basic sensor test complete ---")

    # It's good practice to close the bus when done  
    # bus.shutdown()
```

### **Multi-Sensor Coordination (Python)**

```python
# (Assumes 'bus' and 'send_can_message' from previous example are available)

def start_all_sensors():  
    """Starts all 4 sensors in sequence with a small delay."""  
    print("\\n--- Starting all sensors ---")  
    for sensor_id in range(4):  
        start_sensor(sensor_id)  
        time.sleep(0.1) # Small delay to avoid bus congestion on command  
    print("--- All sensors START commands sent ---")

def set_detection_threshold(sensor_id, threshold):  
    """Sets the CFAR detection threshold for a specific sensor.  
    The threshold_value (0-255) is sent as the second byte (Data[1]).  
    """  
    command_id = 0x80 + sensor_id  
    # Command code 0x05 for DETECTION_THRESHOLD  
    # Data[1] is the threshold value (0-255)  
    send_can_message(command_id, [0x05, threshold])  
    print(f"Set Sensor {sensor_id} detection threshold to {threshold} (ID: 0x{command_id:X})")

def select_chirp_profile(sensor_id, profile_id):  
    """Selects a chirp profile for a specific sensor.  
    Requires sensor stop/start to take effect.  
    """  
    command_id = 0x80 + sensor_id  
    # Command code 0x07 for SELECT_CHIRP_PROFILE  
    # Data[1] is the profile ID (0-4)  
    send_can_message(command_id, [0x07, profile_id])  
    print(f"Set Sensor {sensor_id} chirp profile to {profile_id} (ID: 0x{command_id:X})")

def set_spread_spectrum_mode(sensor_id, enable):  
    """Enables/disables spread spectrum mode for a specific sensor."""  
    command_id = 0x80 + sensor_id  
    # Command code 0x06 for SPREAD_SPEC  
    # Data[1] is enable flag (0=disable, 1=enable)  
    send_can_message(command_id, [0x06, enable])  
    print(f"Set Sensor {sensor_id} spread spectrum to {'Enabled' if enable else 'Disabled'} (ID: 0x{command_id:X})")

# --- Usage Example ---  
if __name__ == "__main__":  
    # Start all sensors  
    # start_all_sensors() # Uncomment to run

    print("\\n--- Configuring all sensors ---")  
    # Configure all sensors with different detection thresholds and chirp profiles  
    for i in range(4):  
        set_detection_threshold(i, 20 + i*5) # Thresholds: 20, 25, 30, 35  
        # Set Sensor 0 to 50m single, Sensor 1 to 50m multi, etc.  
        select_chirp_profile(i, (i % 4) + 1) # Profiles 1, 2, 3, 4  
        set_spread_spectrum_mode(i, i % 2) # Toggle spread spectrum for sensors

    print("--- Configuration commands sent ---")  
    print("Remember to STOP then START sensors for new configurations to apply.")

    # stop_all_sensors() # A helper function similar to start_all_sensors()  
    # start_all_sensors() # Restart sensors after configuration changes
```

### **Data Reception Handler (Python)**

This function continuously monitors the CAN bus for incoming messages and categorizes them based on their arbitration ID, extracting sensor and message type.

```python
# (Assumes 'bus' from previous example is available)

def get_sensor_from_id(can_id):  
    """Determines the sensor number (0-3) from a CAN arbitration ID."""  
    # Command IDs (0x80-0x83)  
    if 0x80 <= can_id <= 0x83:  
        return can_id - 0x80  
    # Output IDs  
    elif 0xA0 <= can_id <= 0xAA: # Sensor 0 (0xA0 base)  
        return 0  
    elif 0xB0 <= can_id <= 0xBA: # Sensor 1 (0xB0 base)  
        return 1  
    elif 0xC0 <= can_id <= 0xCA: # Sensor 2 (0xC0 base)  
        return 2  
    elif 0xD0 <= can_id <= 0xDA: # Sensor 3 (0xD0 base)  
        return 3  
    return -1 # Unknown sensor

def get_message_type_from_output_id(can_id):  
    """Extracts the base message type from an output CAN ID."""  
    # This relies on the pattern: Output_ID = Base_Message_Type + Sensor_Offset  
    # We need to reverse the offset to get the base type.  
    sensor_id = get_sensor_from_id(can_id)  
    if sensor_id != -1:  
        # Calculate the base CAN ID for the sensor's output range  
        base_output_id_for_sensor = 0xA0 + (sensor_id * 0x10)  
        return can_id - (sensor_id * 0x10)  
    return -1 # Unknown message type or not an output message

def process_can_messages():  
    """Continuously processes incoming CAN messages from the bus."""  
    print("\\n--- Starting CAN message listener (Ctrl+C to stop) ---")  
    try:  
        while True:  
            msg = bus.recv(timeout=1.0) # Wait up to 1 second for a message  
            if msg:  
                sensor_id = get_sensor_from_id(msg.arbitration_id)  
                msg_type_base = get_message_type_from_output_id(msg.arbitration_id)

                if sensor_id == -1:  
                    print(f"[{time.time():.2f}] Unrecognized CAN ID: 0x{msg.arbitration_id:X}")  
                    continue

                if msg.is_remote_frame:  
                    print(f"[{time.time():.2f}] Sensor {sensor_id}: Remote Frame (ID: 0x{msg.arbitration_id:X})")  
                elif msg.is_error_frame:  
                    print(f"[{time.time():.2f}] Sensor {sensor_id}: Error Frame (ID: 0x{msg.arbitration_id:X})")  
                else:  
                    # Handle known output message types  
                    if msg_type_base == 0xA0: # Header Message  
                        if len(msg.data) >= 8:  
                            # Assuming 8-byte header as per PDF  
                            total_packet_len = struct.unpack('<I', msg.data[0:4])[0]  
                            frame_number = struct.unpack('<I', msg.data[4:8])[0]  
                            print(f"[{time.time():.2f}] Sensor {sensor_id}: Header (ID: 0x{msg.arbitration_id:X}) - Total Len: {total_packet_len} bytes, Frame #: {frame_number}")  
                        else:  
                            print(f"[{time.time():.2f}] Sensor {sensor_id}: Header (ID: 0x{msg.arbitration_id:X}) - Incomplete data: {msg.data.hex()}")  
                    elif msg_type_base == 0xA1: # Detected Points  
                        # Each point is 8 bytes (float range, float snr)  
                        num_points = len(msg.data) // 8  
                        points_info = []  
                        for i in range(num_points):  
                            range_val = struct.unpack('<f', msg.data[i*8 : i*8 + 4])[0]  
                            snr_val = struct.unpack('<f', msg.data[i*8 + 4 : i*8 + 8])[0]  
                            points_info.append(f"R:{range_val:.2f}m, SNR:{snr_val:.2f}dB")  
                        print(f"[{time.time():.2f}] Sensor {sensor_id}: Detected Points (ID: 0x{msg.arbitration_id:X}) - {num_points} obj(s): {', '.join(points_info)}")  
                    elif msg_type_base == 0xA2: # Range Profile  
                        # Data is uint16_t chunks, 4 bins per 8-byte message  
                        bin_values = []  
                        for i in range(0, len(msg.data), 2):  
                            bin_values.append(struct.unpack('<H', msg.data[i:i+2])[0])  
                        print(f"[{time.time():.2f}] Sensor {sensor_id}: Range Profile Chunk (ID: 0x{msg.arbitration_id:X}) - Bins: {bin_values}")  
                    elif msg_type_base == 0xA3: # Status Message  
                        if len(msg.data) >= 4:  
                            status_code = struct.unpack('<I', msg.data[0:4])[0]  
                            status_map = {0x01: "BOOT", 0x02: "CHIRP", 0x03: "STOPPED"}  
                            print(f"[{time.time():.2f}] Sensor {sensor_id}: Status (ID: 0x{msg.arbitration_id:X}) - State: {status_map.get(status_code, 'Unknown')} (0x{status_code:X})")  
                        else:  
                             print(f"[{time.time():.2f}] Sensor {sensor_id}: Status (ID: 0x{msg.arbitration_id:X}) - Incomplete data: {msg.data.hex()}")  
                    elif msg_type_base == 0xA4: # Version Message  
                        if len(msg.data) >= 3:  
                            version = msg.data[0:3]  
                            print(f"[{time.time():.2f}] Sensor {sensor_id}: Version (ID: 0x{msg.arbitration_id:X}) - v{version[0]}.{version[1]}.{version[2]}")  
                        else:  
                            print(f"[{time.time():.2f}] Sensor {sensor_id}: Version (ID: 0x{msg.arbitration_id:X}) - Incomplete data: {msg.data.hex()}")  
                    # Add more elif for other message types (0xA5-0xAA) as needed  
                    else:  
                        # This would catch any command message being echoed on the bus if not filtered out,  
                        # or other unhandled output types.  
                        print(f"[{time.time():.2f}] Sensor {sensor_id}: Unhandled message (ID: 0x{msg.arbitration_id:X}, Type: 0x{msg_type_base:X}) - Data: {msg.data.hex()}")  
            # else:  
            #     print(f"[{time.time():.2f}] No message received (timeout)") # Uncomment for debugging timeouts  
    except KeyboardInterrupt:  
        print("\\n--- CAN listener stopped by user ---")  
    finally:  
        pass  
        # bus.shutdown() # Ensure bus is closed when done

# --- Usage Example ---  
if __name__ == "__main__":  
    # Start all sensors to generate data  
    # start_all_sensors()  
    # time.sleep(1) # Give time for sensors to start chirping

    # Run the listener  
    process_can_messages()
```

### **PEAK PCAN Configuration**

If using a PEAK PCAN-USB FD adapter with PCAN-View for monitoring:

\# PCAN-View configuration settings  
Nominal Bit Rate: 500 kbit/s  
Data Bit Rate:    2 Mbit/s  \# Note: This setting is relevant for CAN FD.  
                            \# For Classic CAN (current firmware), the Data Bit Rate  
                            \# effectively defaults to the Nominal Bit Rate for data frames.  
Sample Point:     75%       \# Matching the firmware configuration.

### **SocketCAN Configuration (Linux)**

For Linux systems, SocketCAN can be used to interface with CAN hardware.

\# Load necessary kernel modules  
sudo modprobe can  
sudo modprobe can\_raw  
sudo modprobe vcan \# For virtual CAN interface, useful for testing without hardware

\# Add a CAN interface (e.g., can0) and set its type to CAN  
sudo ip link add dev can0 type can

\# Set the bitrate and sample point for the CAN interface  
\# The sample-point is important and should match the firmware's 75%  
sudo ip link set can0 type can bitrate 500000 sample-point 0.75

\# Bring the CAN interface up  
sudo ip link set up can0

\# Monitor CAN traffic (all messages)  
candump can0

\# Monitor specific sensor's data messages (e.g., Sensor 0 output range 0xA0-0xAF)  
candump can0,0xA0:0xFF0

\# Send a command manually (e.g., start sensor 0\)  
cansend can0 080\#00 \# ID 0x80, Data 0x00

## **Troubleshooting**

### **Common Issues**

#### **No Response from Sensor**

* **Symptoms**: Commands are sent, but no status or data messages are received from the radar module.  
* **Causes**:  
  * **Incorrect Bitrate**: The host CAN interface bitrate does not match the radar module's 500 kbit/s.  
  * **Missing/Incorrect Termination Resistors**: The CAN bus is not properly terminated with 120Ω resistors at both ends, leading to signal reflections.  
  * **Wrong CAN ID Format**: Commands are not sent using 29-bit extended identifiers. The radar firmware expects 29-bit IDs for commands.  
  * **Sensor Not Powered or in Wrong Mode**: The radar module is not powered on, or its boot mode switches (if applicable) are set incorrectly (e.g., not in run mode).  
  * **Loose Connections**: Physical wiring issues with the CAN\_H, CAN\_L, or ground connections.  
* **Solutions**:  
  * Verify the CAN interface bitrate is exactly 500 kbit/s on both the host and the radar module.  
  * Check for two 120Ω termination resistors on the bus, one at each extreme end.  
  * Ensure all command messages specify is\_extended\_id=True in your software (e.g., python-can).  
  * Confirm the radar module is powered correctly (5V) and has successfully booted (check UART debug output if available).

#### **Multiple Sensor Conflicts**

* **Symptoms**: Erratic behavior, unexpected responses, or data corruption when multiple sensors are active.  
* **Causes**:  
  * **Duplicate Command IDs**: Two or more sensors are configured to respond to the same command ID (e.g., both listening to 0x80).  
  * **CAN Bus Overload**: The combined data rate from multiple sensors exceeds the bus capacity (500 kbit/s), leading to dropped messages.  
  * **Rapid Command Timing Issues**: Sending commands to different sensors too quickly without allowing for processing time, especially sensorStart or sensorStop.  
* **Solutions**:  
  * Ensure each sensor on the bus has a unique command ID (0x80, 0x81, 0x82, 0x83). This is typically set during firmware flashing or via a hardware switch/pin configuration on the module (not explicitly detailed in provided files, but implied by canSensorId setting upon reception of CAN\_START\_MSG\_ID).  
  * Implement flow control or reduce the data output rate of individual sensors via the guiMonitor command if bus utilization becomes too high.  
  * Introduce small delays (e.g., 50ms \- 100ms) between sending commands to different sensors, especially for state-changing commands.

#### **Data Reception Issues**

* **Symptoms**: Missing chunks of radar data, corrupted messages, or incomplete frames.  
* **Causes**:  
  * **Receive Buffer Overflow**: The host application's CAN receive buffer is not large enough to handle the incoming data rate, especially for range profile or detected object messages which are fragmented.  
  * **Incorrect CAN Filtering**: The host's CAN interface filters are not configured correctly, leading to unwanted messages or discarding desired ones.  
  * **High Bus Utilization**: Continuous high bus load can lead to increased latency and potential message loss.  
* **Solutions**:  
  * Increase the receive buffer size in your host CAN interface driver or application.  
  * Set appropriate CAN filters to only receive the specific message IDs you are interested in (e.g., 0xA0-0xAA for Sensor 0, 0xB0-0xBA for Sensor 1, etc.).  
  * Monitor actual bus utilization. If consistently above 70, consider reducing the radar's output data rate by disabling unused message types via the guiMonitor CLI command or through a CAN command if exposed.

#### **Configuration Not Applied**

* **Symptoms**: Commands are acknowledged by the sensor, but the radar's behavior (e.g., range, detection threshold) does not change.  
* **Causes**:  
  * **Requires Sensor Restart**: Many configuration commands (like setDetectionThreshold, selectChirpProfile, enableSpreadSpec, setPower, cfarFovCfg, aoaFovCfg) are applied only after a sensor stop and restart cycle. The changes are staged and loaded at sensorStart.  
  * **Invalid Parameter Values**: The arguments passed to a command are outside the valid range. The firmware often includes checks for this, logging errors to the UART debug port.  
  * **Wrong Chirp Profile Selected**: The desired configuration was applied to a chirp profile that is not currently active.  
* **Solutions**:  
  * After sending a configuration command, always follow with sensorStop (0x01) and then sensorStart (0x00) to ensure the new settings are loaded.  
  * Consult the relevant firmware documentation (e.g., mmw\_cli.c or the SDK documentation) for valid parameter ranges for each command.  
  * Verify that the correct chirp profile is selected using the selectChirpProfile command before applying other configurations.

### **Diagnostic Commands**

These commands can be used to verify basic communication and sensor state:

\# Verify basic communication with Sensor 0  
query\_status(0)  
\# Expected Response: Status message (0xA3) with current state (e.g., 0x01 BOOT or 0x03 STOPPED)

\# Test command response: Stop Sensor 0  
stop\_sensor(0)  
\# Expected Response: Status message (0xA3) with state 0x03 (STOPPED)

\# Test command response: Start Sensor 0  
start\_sensor(0)  
\# Expected Response: Status message (0xA3) with state 0x02 (CHIRP)

### **Bus Utilization Analysis**

With all 4 sensors active on the 500 kbit/s CAN bus, the estimated bus utilization is as follows:

* **Header Messages**: Approximately 4 Hz per sensor results in 16 Hz total (4 sensors×4 Hz/sensor).  
* **Range Profiles**: Approximately 4 Hz per sensor. If each profile is 1024 bytes (512 bins of 2 bytes each), this translates to approximately 32 kB/s (4 sensors×4 Hz/sensor×1024 bytes/profile).  
* **Object Data**: Variable, dependent on the number of targets detected.  
* **Total Estimated Utilization**: Roughly 50 \- 60 of the bus capacity at 500 kbit/s. This leaves some headroom for other traffic and burst data.

### **Performance Optimization**

To optimize CAN bus performance and avoid saturation:

1. **Reduce Data Output**: Use the guiMonitor CLI command (or its CAN equivalent if exposed) to disable specific unused output message types (e.g., heatmaps, noise profiles) that are not critical for your application. This can significantly reduce bus load.  
2. **Filter Reception**: Configure CAN filters on the receiving host system to accept only the message IDs relevant to your application. This offloads processing from the main application and prevents unnecessary data from consuming resources.  
3. **Batch Processing**: For the host application, process multiple received CAN frames together to reduce software overhead, rather than processing each frame individually.  
4. **Prioritize Commands**: Ensure critical commands (like sensorStart/sensorStop) are sent with appropriate timing and priority to guarantee their immediate processing.  
5. **Monitor Timing**: Regularly monitor the timing of command sequences and data reception to ensure adequate gaps between commands and prevent race conditions or missed messages.

## **Continuous Data Flow with Multiple Sensors**

For applications requiring continuous data streams from multiple radar sensors running the same configuration, careful orchestration of startup and data reception is crucial for robust performance.

### **Initializing Sensors for Continuous Flow**

While the CAN bus protocol handles arbitration, it is generally recommended to introduce a slight delay between sending state-changing commands, such as sensorStart, to different sensors. This practice helps to:

* **Reduce Command Arbitration Overhead**: Although the CAN protocol manages collisions efficiently, minimizing simultaneous arbitration attempts for critical commands can ensure a smoother and more predictable startup sequence for each sensor.  
* **Allow Sensor Internal Processing**: Each sensorStart command triggers a sequence of internal processes within the radar module (e.g., loading configurations, initializing chirps, preparing data buffers). A brief pause between commands gives each sensor a dedicated window to complete these initializations without contending for internal resources or external bus access for status updates.

Recommendation for sensorStart commands:  
Send the sensorStart command to each sensor sequentially with a short delay in between. A delay of 50ms to 100ms between sending sensorStart to Sensor 1, then Sensor 2, then Sensor 3 (and so on) is a good starting point. This provides a balance between rapid deployment and system stability.  
Example (Python):

```python
def start\_all\_sensors\_staggered(sensor\_ids, delay\_ms=100):  
    """Starts a list of sensors sequentially with a delay."""  
    print(f"\\n--- Starting sensors {sensor\_ids} with {delay\_ms}ms stagger \---")  
    for sensor\_id in sensor\_ids:  
        start\_sensor(sensor\_id) \# Uses the start\_sensor function defined previously  
        time.sleep(delay\_ms / 1000.0) \# Convert ms to seconds  
    print("--- All sensor START commands sent \---")

\# Usage for three sensors (0, 1, 2\)  
if \_\_name\_\_ \== "\_\_main\_\_":  
    \# Assuming initial setup and bus definition are complete  
    sensors\_to\_start \= \[0, 1, 2\]  
    start\_all\_sensors\_staggered(sensors\_to\_start, delay\_ms=75)  
    \# Give all sensors a bit more time to fully initialize and begin streaming  
    time.sleep(1.0)  
    \# Now, proceed to continuous data reception  
    \# process\_can\_messages() \# Call your data reception handler
```

### **Managing Continuous Data Reception**

Once the sensors are instructed to sensorStart and begin chirping, they will autonomously transmit data (headers, detected points, range profiles, etc.) at their configured frame rates. You **do not** need to send continuous "data requests" to the sensors.

* **Concurrent Data Flow**: Data from all active sensors will flow concurrently on the CAN bus, each identifiable by its unique CAN ID (e.g., Sensor 0's data will be 0xA0-0xAA, Sensor 1's will be 0xB0-0xBA, etc.).  
* **Non-Blocking Listener**: Your host application should implement a continuous, non-blocking CAN listener (as shown in the [Data Reception Handler](#data-reception-handler-python) example). This listener will collect all incoming messages from the bus and, using the message ID mapping, can demultiplex the data to the correct sensor and message type for further processing.  
* **No Staggering for Reception**: There is no need to stagger data reception. The CAN interface and your application's listener should be designed to handle and parse all incoming traffic from all active sensors simultaneously.

### **Considerations for Long-Term Continuous Operation**

* **Bus Utilization**: Continuously running multiple sensors, especially with full data output enabled, will generate a high bus load (as estimated in [Bus Utilization Analysis](#bus-utilization-analysis)). Regularly monitor this to prevent dropped frames. If necessary, use [Performance Optimization](#performance-optimization) techniques, particularly guiMonitor commands, to disable unnecessary data outputs.  
* **Error Handling**: Implement robust error handling in your data reception logic to account for potential lost frames or bus errors.  
* **Configuration Consistency**: Ensure all sensors are running the exact same desired configuration if that is your intent. Changes to configuration commands (e.g., setDetectionThreshold, selectChirpProfile, enableSpreadSpec, setPower) still require a sensorStop and sensorStart cycle for each affected sensor. If you need to reconfigure one sensor, you would stop only that sensor, reconfigure, then restart it, allowing the other sensors to continue operating.
