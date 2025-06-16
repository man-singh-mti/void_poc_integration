# CAN Communication Guide for IWR1843 AOP Radar Modules

## Overview

This document provides comprehensive technical documentation for CAN bus communication with IWR1843 AOP radar modules. The guide is based on actual firmware implementation analysis and serves as both an API reference and integration guide for systems using multiple radar sensors.

### System Architecture

The radar firmware implements a CAN 2.0A interface that supports:

- **Command and Control**: Send configuration and control commands to individual sensors
- **Real-time Data Streaming**: Receive processed radar data from multiple sensors  
- **Multi-sensor Operation**: Coordinate up to 4 sensors on a single CAN bus with unique addressing
- **Status Management**: Query sensor state and firmware version information

### Key Technical Specifications

- **CAN Standard**: CAN 2.0A with 11-bit standard identifiers
- **Bitrate**: 500 kbps (configured in firmware)
- **Frame Type**: Classic CAN with 8-byte maximum data payload
- **Multi-sensor Support**: Up to 4 sensors per bus (sensor IDs 0-3)
- **Message Types**: 11 different data message types (0xA0-0xAA) plus command/status messages

**Actual Firmware Configuration:**

```c
// From mmw_can.h and mmw_can.c (verified implementation)
#define ENABLE_CAN_FD 0                           // Classic CAN mode
#define MAX_CAN_FRAME_SIZE 8                      // 8-byte frames
#define MAX_NUM_SENSORS 4                         // Support 4 sensors max
CANFD_MCANFrameType frameType = CANFD_MCANFrameType_CLASSIC;

// Bit timing for 500 kbps (40MHz source clock)
mcanBitTimingParams.nomBrp      = 0x5U;          // Prescaler: 5
mcanBitTimingParams.nomPropSeg  = 0x6U;          // Prop Segment: 6
mcanBitTimingParams.nomPseg1    = 0x5U;          // Phase Seg 1: 5
mcanBitTimingParams.nomPseg2    = 0x4U;          // Phase Seg 2: 4
mcanBitTimingParams.nomSjw      = 0x1U;          // Sync Jump Width: 1

// Calculated bit rate: 40MHz / ((6+5+4+1) * 5) = 500 kbps
```

## Multi-Sensor Bus Architecture

The firmware supports up to 4 radar sensors operating on the same CAN bus. Each sensor must be assigned a unique Sensor ID, ranging from 0 to 3. This ID is crucial for:

- **Command Targeting**: The master controller sends commands to a specific sensor using a command message ID that incorporates the target sensor's ID
- **Data Source Identification**: The sensor uses its ID to calculate an offset for its output message IDs, allowing data from different sensors to be distinguished

**Note**: The firmware initializes `canSensorId` to `0xFF` and it must be set to a valid ID (0-3) for proper operation when the first command is received.

### Sensor Addressing Scheme

A base offset array is defined in the firmware: `uint8_t canIDOffset[MAX_NUM_SENSORS] = {0x00, 0X10, 0x20, 0x30};`

- **Sensor 0**: `canIDOffset[0] = 0x00`
- **Sensor 1**: `canIDOffset[1] = 0x10`
- **Sensor 2**: `canIDOffset[2] = 0x20`
- **Sensor 3**: `canIDOffset[3] = 0x30`

**Command Message IDs:**
The base CAN ID for commands sent TO a sensor is `0x80` (defined as `CAN_START_MSG_ID_BASE`).

- Sensor 0 Command ID: `0x80 + 0x00 = 0x80`
- Sensor 1 Command ID: `0x80 + 0x10 = 0x90`
- Sensor 2 Command ID: `0x80 + 0x20 = 0xA0`
- Sensor 3 Command ID: `0x80 + 0x30 = 0xB0`

**Data Output Message IDs:**
Data messages sent FROM a sensor use a base range of `0xA0-0xAA` (defined in `mmwDemo_can_message_type_e`). The sensor adds its offset to these base IDs.

Example: `CAN_MESSAGE_MMWDEMO_DETECTED_POINTS` has a base ID of `0xA1`:

- From Sensor 0: `0xA1 + 0x00 = 0xA1`
- From Sensor 1: `0xA1 + 0x10 = 0xB1`
- From Sensor 2: `0xA1 + 0x20 = 0xC1`
- From Sensor 3: `0xA1 + 0x30 = 0xD1`

## Command Interface

Commands are sent to a specific radar sensor using an 11-bit standard CAN identifier. The first data byte contains the command code, followed by any parameters required by that command.

### Command Message Format

- **CAN ID**: `0x80 + SensorOffset` (where SensorOffset is 0x00, 0x10, 0x20, or 0x30)
- **DLC**: Varies by command (typically 1-2 bytes)
- **Data[0]**: Command code (see Available Commands table)
- **Data[1...n]**: Optional parameters, depending on the command

### Available Commands

Based on the actual firmware implementation in `mmw_can.c`:

| Command Code | Command Name             | Parameters        | Description                    | Implementation Function              |
|:-------------|:-------------------------|:------------------|:-------------------------------|:-------------------------------------|
| 0x00         | CMD_START                | None              | Start radar operation          | `can_sensor_start()`                 |
| 0x01         | CMD_STOP                 | None              | Stop radar operation           | `can_sensor_stop()`                  |
| 0x02         | CMD_DC_CALIB             | None              | Perform DC calibration         | `MmwDemo_CalibDcRange()`             |
| 0x03         | CMD_TX_BACK_OFF          | uint8_t power     | Set TX power backoff           | `can_sensor_txbackoff()`             |
| 0x04         | CMD_STATUS               | None              | Query sensor status            | Sends status and version             |
| 0x05         | CMD_DETECTION_THRESHOLD  | uint8_t threshold | Set CFAR detection threshold   | `can_set_range_detection_treshold()` |
| 0x06         | CMD_SPREAD_SPEC          | uint8_t enable    | Enable/disable spread spectrum | `can_sensor_spreadspec()`            |
| 0x07         | CMD_SELECT_CHIRP_PROFILE | int8_t profile    | Select chirp configuration     | `can_sensor_select_chirp_config()`   |
| 0x08         | CMD_FOV                  | int8_t fov        | Set field of view              | `can_sensor_fov()`                   |

**Command Responses:**

- **CMD_START**: Responds with status message (0xA3) indicating CHIRP state (0x02)
- **CMD_STOP**: Responds with status message (0xA3) indicating STOPPED state (0x03)
- **CMD_STATUS**: Responds with status message (0xA3) and version message (0xA4)

## Data Output Messages

The radar sensor outputs various types of data via CAN messages. These messages use base IDs from the `mmwDemo_can_message_type_e` enumeration, offset by the sensor's unique ID.

### Output Message Types

Based on actual firmware enumeration in `mmw_can.h`:

| Base ID | Message Type                                    | Description                          |
|:--------|:------------------------------------------------|:-------------------------------------|
| 0xA0    | CAN_MESSAGE_MMWDEMO_HEADER                      | Start of data frame sequence         |
| 0xA1    | CAN_MESSAGE_MMWDEMO_DETECTED_POINTS             | Detected object data                 |
| 0xA2    | CAN_MESSAGE_MMWDEMO_RANGE_PROFILE               | Range profile data                   |
| 0xA3    | CAN_MESSAGE_MMWDEMO_NOISE_PROFILE               | **Sensor Status** (despite name)     |
| 0xA4    | CAN_MESSAGE_MMWDEMO_AZIMUT_STATIC_HEAT_MAP      | **Firmware Version** (despite name)  |
| 0xA5    | CAN_MESSAGE_MMWDEMO_RANGE_DOPPLER_HEAT_MAP      | Range/Doppler detection matrix       |
| 0xA6    | CAN_MESSAGE_MMWDEMO_STATS                       | Statistics information               |
| 0xA7    | CAN_MESSAGE_MMWDEMO_SIDE_INFO                   | Side information for detected points |
| 0xA8    | CAN_MESSAGE_MMWDEMO_AZIMUT_ELEV_STATIC_HEAT_MAP | Azimuth-Elevation static heatmap     |
| 0xA9    | CAN_MESSAGE_MMWDEMO_TEMPERATURE                 | Temperature statistics               |
| 0xAA    | CAN_MESSAGE_MMWDEMO_PADDING                     | Padding/End of sequence              |

**Important Note**: The firmware repurposes some message types:

- **0xA3** (NOISE_PROFILE): Actually used for sensor status messages
- **0xA4** (AZIMUT_STATIC_HEAT_MAP): Actually used for firmware version messages

### Sensor Status Message (0xA3)

Conveys the current operational state of the sensor using the `SensorState` enumeration:

```c
enum SensorState {
    BOOT    = 0x01,    // Sensor is booting
    CHIRP   = 0x02,    // Sensor is actively chirping
    STOPPED = 0x03     // Sensor is stopped
};
```

**Payload**: 4 bytes containing the current sensor state value.

### Firmware Version Message (0xA4)

Reports the firmware version using constants from `mmw_can.h`:

```c
#define MTI_VER_MAJOR 1
#define MTI_VER_MINOR 2
#define MTI_VER_SUB 0
```

**Payload**: 3 bytes [Major, Minor, Sub]

## Implementation Examples

### Basic Sensor Start Sequence (Python)

```python
import can
import time

# Initialize CAN interface
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

def send_can_message(arbitration_id, data):
    """Helper function to send a CAN message."""
    msg = can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=False  # Using 11-bit IDs
    )
    bus.send(msg)
    print(f"Sent: ID=0x{arbitration_id:X}, Data={[hex(b) for b in data]}")

def start_sensor(sensor_id):
    """Start a specific radar sensor."""
    command_id = 0x80 + (sensor_id * 0x10)  # Calculate command ID
    send_can_message(command_id, [0x00])    # CMD_START
    print(f"Started Sensor {sensor_id}")

def stop_sensor(sensor_id):
    """Stop a specific radar sensor."""
    command_id = 0x80 + (sensor_id * 0x10)
    send_can_message(command_id, [0x01])    # CMD_STOP
    print(f"Stopped Sensor {sensor_id}")

def query_status(sensor_id):
    """Query status of a specific radar sensor."""
    command_id = 0x80 + (sensor_id * 0x10)
    send_can_message(command_id, [0x04])    # CMD_STATUS
    print(f"Queried status of Sensor {sensor_id}")

# Example usage
if __name__ == "__main__":
    # Start Sensor 0
    start_sensor(0)
    time.sleep(0.5)
    
    # Query its status
    query_status(0)
    time.sleep(0.5)
    
    # Stop Sensor 0
    stop_sensor(0)
```

### Multi-Sensor Configuration (Python)

```python
def set_detection_threshold(sensor_id, threshold):
    """Set CFAR detection threshold for a specific sensor."""
    command_id = 0x80 + (sensor_id * 0x10)
    send_can_message(command_id, [0x05, threshold])  # CMD_DETECTION_THRESHOLD
    print(f"Set Sensor {sensor_id} threshold to {threshold}")

def select_chirp_profile(sensor_id, profile_id):
    """Select chirp profile for a specific sensor."""
    command_id = 0x80 + (sensor_id * 0x10)
    send_can_message(command_id, [0x07, profile_id])  # CMD_SELECT_CHIRP_PROFILE
    print(f"Set Sensor {sensor_id} to profile {profile_id}")

def enable_spread_spectrum(sensor_id, enable):
    """Enable/disable spread spectrum for a specific sensor."""
    command_id = 0x80 + (sensor_id * 0x10)
    send_can_message(command_id, [0x06, 1 if enable else 0])  # CMD_SPREAD_SPEC
    print(f"Spread spectrum {'enabled' if enable else 'disabled'} for Sensor {sensor_id}")

# Configure multiple sensors
for sensor_id in range(3):  # Configure sensors 0, 1, 2
    set_detection_threshold(sensor_id, 20 + sensor_id * 5)  # Different thresholds
    select_chirp_profile(sensor_id, sensor_id + 1)          # Different profiles
    enable_spread_spectrum(sensor_id, sensor_id % 2)        # Alternate spread spectrum
    time.sleep(0.1)  # Small delay between commands
```

### Data Reception Handler (Python)

```python
def process_can_messages():
    """Continuously process incoming CAN messages."""
    print("Starting CAN message listener...")
    
    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue
                
            # Determine sensor ID from CAN ID
            sensor_id = determine_sensor_id(msg.arbitration_id)
            msg_type = determine_message_type(msg.arbitration_id)
            
            if msg_type == 0xA3:  # Status message
                if len(msg.data) >= 4:
                    status = int.from_bytes(msg.data[0:4], byteorder='little')
                    status_map = {1: "BOOT", 2: "CHIRP", 3: "STOPPED"}
                    print(f"Sensor {sensor_id}: Status = {status_map.get(status, 'Unknown')} (0x{status:X})")
                    
            elif msg_type == 0xA4:  # Version message
                if len(msg.data) >= 3:
                    major, minor, sub = msg.data[0:3]
                    print(f"Sensor {sensor_id}: Version = {major}.{minor}.{sub}")
                    
            elif msg_type == 0xA1:  # Detected points
                print(f"Sensor {sensor_id}: Detected points data ({len(msg.data)} bytes)")
                
            elif msg_type == 0xA2:  # Range profile
                print(f"Sensor {sensor_id}: Range profile data ({len(msg.data)} bytes)")
                
            else:
                print(f"Sensor {sensor_id}: Message type 0x{msg_type:X} ({len(msg.data)} bytes)")
                
    except KeyboardInterrupt:
        print("\nStopped by user")

def determine_sensor_id(can_id):
    """Determine sensor ID from CAN arbitration ID."""
    if 0x80 <= can_id <= 0x83:  # Command IDs
        return can_id - 0x80
    elif 0xA0 <= can_id <= 0xAA:  # Sensor 0 data
        return 0
    elif 0xB0 <= can_id <= 0xBA:  # Sensor 1 data
        return 1
    elif 0xC0 <= can_id <= 0xCA:  # Sensor 2 data
        return 2
    elif 0xD0 <= can_id <= 0xDA:  # Sensor 3 data
        return 3
    else:
        return -1  # Unknown

def determine_message_type(can_id):
    """Determine base message type from CAN arbitration ID."""
    return can_id & 0x0F  # Get last 4 bits to determine message type

# Run the listener
process_can_messages()
```

## Troubleshooting

### Common Issues

#### Communication Problems

- **Symptoms**: No response to commands, missing data messages
- **Causes**:
  - Incorrect bitrate configuration (not 500 kbps)
  - Missing CAN bus termination resistors (120Ω at each end)
  - Wrong sensor command IDs
- **Solutions**:
  - Verify CAN bus hardware setup and termination
  - Use correct command IDs based on sensor addressing scheme
  - Check that sensor firmware is responding on UART debug output

#### Multiple Sensor Conflicts

- **Symptoms**: Erratic behavior when multiple sensors are active
- **Causes**:
  - Sensors responding to same command ID (not properly configured)
  - CAN bus overload from too much data output
- **Solutions**:
  - Ensure each sensor has unique addressing
  - Reduce data output using appropriate command configuration
  - Stagger sensor startup commands with 50-100ms delays

### Diagnostic Commands

Use these basic commands to verify communication:

```python
# Test basic communication with Sensor 0
query_status(0)
# Expected: Status message (0xA3) with current state

# Test command response
stop_sensor(0)
# Expected: Status message (0xA3) with STOPPED state (0x03)

start_sensor(0)
# Expected: Status message (0xA3) with CHIRP state (0x02)
```

## Message ID Reference

### Single Sensor (Sensor 0)

| Purpose            | CAN ID | Description                   |
|:-------------------|:-------|:------------------------------|
| Commands to sensor | 0x80   | All command codes (0x00-0x08) |
| Header             | 0xA0   | Data frame start              |
| Detected Points    | 0xA1   | Object detection data         |
| Range Profile      | 0xA2   | Range bin data                |
| Status             | 0xA3   | Sensor state                  |
| Version            | 0xA4   | Firmware version              |

### Multi-Sensor Configuration

**Commands TO sensors:**

- Sensor 0: 0x80
- Sensor 1: 0x90
- Sensor 2: 0xA0
- Sensor 3: 0xB0

**Data FROM sensors (example with Detected Points 0xA1):**

- Sensor 0: 0xA1
- Sensor 1: 0xB1
- Sensor 2: 0xC1
- Sensor 3: 0xD1

This addressing scheme allows up to 4 sensors to operate simultaneously on the same CAN bus without conflicts.

## Hardware Interface

### Physical Connections

The radar module uses connector J6 for CAN communication:

| J6 Pin | Signal  | Description     |
|:-------|:--------|:----------------|
| 1      | VDD_5V0 | 5V power supply |
| 2      | CAN_H   | CAN high signal |
| 3      | CAN_L   | CAN low signal  |
| 4      | DGND    | Digital ground  |

### Bus Configuration

- **Topology**: Linear bus with 120Ω termination at each end
- **Cable Length**: Up to 100m at 500 kbps
- **Voltage Levels**: Standard CAN differential signaling

## Notes on Firmware Implementation

Based on analysis of the actual firmware source code:

1. **Command Processing**: Commands are processed in the `MCANAppCallback` function in `mmw_can.c`
2. **Hard-coded Configurations**: The system uses pre-defined chirp profiles in `cli.c`
3. **State Management**: Sensor states are tracked using the `SensorState` enumeration
4. **Message ID Calculation**: Implemented in `Get_CanMessageIdentifier` function
5. **Classic CAN Only**: The firmware is configured for Classic CAN, not CAN FD

This guide represents the actual firmware implementation as found in the codebase and should be used as the authoritative reference for CAN communication with these radar modules.
