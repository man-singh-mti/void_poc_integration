# Developer's Guide

This guide provides essential information for developers working on the Downhole Commercial project. It covers development environment setup, common tasks, debugging strategies, and best practices for code modifications.

## Development Environment Setup

### Required Tools

1. **STM32CubeMX** - Used for initial code generation and configuration
   - Version: 6.5.0 or later
   - Configuration file: `config/Downhole.ioc`
   - Used for pin configuration and peripheral setup

2. **Keil MDK-ARM V5** - Primary IDE and compiler
   - Project file: `MDK-ARM/Downhole.uvprojx`
   - Target: STM32F722RETx (512KB Flash, 256KB RAM)
   - ARM Compiler V5.06 or later
   - Build targets: DEBUG and RELEASE configurations

3. **ST-Link Utility** - For flashing and programming
   - ST-Link V2/V3 compatible
   - Used for firmware download and memory operations

4. **Terminal Software** - For UART communication
   - PuTTY, TeraTerm, or similar
   - Multiple sessions for different UART channels
   - Baud rates: 57600 (uphole), 115200 (debug), 115200 (radar)

### Project Setup

1. **Clone and Setup**

   ```bash
   git clone <repository_url>
   cd void_poc_integration
   ```

2. **STM32CubeMX Configuration**
   - Open `config/Downhole.ioc` in STM32CubeMX
   - Review peripheral configurations:
     - Clock: 32MHz SYSCLK (PLL configuration)
     - SPI1/SPI2: Dual IMU sensors
     - USART1/USART2/USART3: Multi-channel communication
     - CAN1: Radar sensor communication
     - ADC1: Water and temperature sensors
   - Generate code only if configuration changes are needed

3. **Keil MDK Project**
   - Open `MDK-ARM/Downhole.uvprojx`
   - Select target configuration:
     - **DEBUG**: Full debug symbols, optimized for debugging
     - **RELEASE**: Optimized for performance and size
   - Verify include paths and source file organization

4. **Build System Structure**

   ```text
   Core/           # STM32 HAL and system files
   Device/         # Application layer (MTI/VMT modules)
   Drivers/        # STM32 peripheral drivers
   MDK-ARM/        # Keil project files
   config/         # STM32CubeMX configuration
   Docs/           # Documentation
   ```

### Hardware Requirements

For development and testing, you'll need:

1. **Target Hardware**
   - STM32F722RET6 microcontroller board
   - Dual ICM20948 IMU sensors (SPI interface)
   - Three AWR1843AOP radar sensors (CAN interface)
   - Water detection ADC sensors
   - Temperature monitoring sensor

2. **Development Interface**
   - ST-Link V2/V3 debugger/programmer
   - SWD debug connection (PA13/PA14)

3. **Communication Interfaces**
   - USB-TTL adapter for UART communication
   - CAN bus interface for radar sensor communication
   - RS485 interface for uphole communication

4. **Power Supply**
   - 3.3V regulated power supply
   - Current capacity: ~500mA minimum

## Building and Flashing

### Build Process

1. **Open Project**
   - Launch Keil MDK-ARM
   - Open `MDK-ARM/Downhole.uvprojx`

2. **Select Build Configuration**
   - **DEBUG Target**: Full debug information, unoptimized
     - Enables debug symbols and breakpoint support
     - Suitable for development and testing
   - **RELEASE Target**: Optimized for production
     - Size and speed optimization enabled
     - Minimal debug information

3. **Build Commands**
   - **Build** → **Build Target** (F7): Incremental build
   - **Build** → **Rebuild All Target Files**: Clean rebuild
   - **Build** → **Clean Target**: Remove build artifacts

4. **Build Output**
   - Binary files: `MDK-ARM/Downhole/Downhole.hex` and `.bin`
   - Object files: `MDK-ARM/Downhole/` directory
   - Map file: `MDK-ARM/Downhole/Downhole.map` (memory layout)

5. **Build Verification**
   - Check Build Output window for errors/warnings
   - Verify memory usage statistics
   - Flash: ~200KB used of 512KB available
   - RAM: ~50KB used of 256KB available

### Flash Procedure

1. **Using Keil MDK**
   - Connect ST-Link debugger to target board
   - Select **Flash** → **Download** (F8) to program target
   - Verify programming success in Output window
   - Use **Flash** → **Erase** to clear flash memory if needed

2. **Using ST-Link Utility**
   - Connect to target using ST-Link Utility
   - Open binary file: `MDK-ARM/Downhole/Downhole.hex`
   - Program flash memory using "Program & Verify"
   - Verify successful programming

3. **Programming Options**
   - **Reset and Run**: Program and start execution immediately
   - **Reset after Programming**: Program and reset target
   - **Run to main()**: Program and break at main function

4. **Flash Memory Layout**
   - Application: 0x08000000 - 0x0807FFFF (512KB)
   - Boot sector: 0x08000000 - 0x08007FFF (32KB)
   - Application sectors: 0x08008000+ (remaining space)

## Debugging

### Debug Interfaces

The system provides multiple debugging interfaces for different purposes:

1. **SWD Debug Port** - Hardware debugging
   - **Pins**: PA13 (SWDIO), PA14 (SWCLK)
   - **Capabilities**: Full debugging with breakpoints, variable inspection
   - **Usage**: Primary interface for Keil MDK debugging
   - **Connection**: ST-Link V2/V3 debugger

2. **UART Debug Channel** - Software debugging  
   - **Pins**: PC6 (TX), PC7 (RX)
   - **Baud Rate**: 115200 bps, 8N1
   - **Purpose**: Runtime debug messages and command interface
   - **Usage**: Connect terminal program for live monitoring

3. **Multi-Channel UART System**
   - **Uphole Channel**: PA9 (TX), PA10 (RX) - 57600 bps
   - **Debug Channel**: PC6 (TX), PC7 (RX) - 115200 bps  
   - **Radar Channel**: PB6 (TX), PB7 (RX) - 115200 bps
   - **Routing**: Automatic channel switching based on command context

4. **CAN Bus Interface** - Radar sensor communication
   - **Pins**: PB8 (RX), PB9 (TX)
   - **Baud Rate**: 500 kbps
   - **Purpose**: Communication with radar sensors
   - **Debug**: Monitor CAN traffic for sensor diagnostics

### Debug Output Format

Debug messages follow standardized formats for different module types:

**System Messages:**

```bash
[SYSTEM] Module init complete: IMU, RADAR, TEMP, WATER
[SYSTEM] State transition: INIT -> MEASURE
```

**Sensor Messages:**

```bash
[IMU] Sensor 0 init complete, ID: 0xEA
[IMU] Motion detected: bump threshold exceeded
[WATER] Calibration complete, threshold: 1750
[TEMP] Temperature: 23.5°C (threshold check: OK)
[RADAR] S0: Frame 1234 complete with 5 points
```

**Communication Messages:**

```bash
[CAN] S0: Status response sent
[UART] Channel switched to UPHOLE (57600)
[CMD] Processing: @sensor,raw,g
```

**Error Messages:**

```bash
[ERROR] IMU SPI communication timeout
[ERROR] CAN TX failed: sensor 1
[WARNING] Water sensor out of range
```

### Debugging Commands

Use these commands via the debug UART interface (115200 bps):

**System Control:**

- `@debug,on` - Enable verbose debug output
- `@debug,off` - Disable verbose debug output  
- `@debug,status` - Show comprehensive system status
- `@debug,mem` - Show memory usage statistics
- `@status` - Get current system state

**Sensor Testing:**

- `@sensor,raw` - Show raw sensor values from all sensors
- `@sensor,raw,g` - Show raw IMU data (gyro/accel)
- `@sensor,raw,w` - Show raw water sensor ADC values
- `@sensor,raw,t` - Show raw temperature sensor values

**Module-Specific:**

- `@init,g,1` - Initialize specific IMU sensor
- `@tp,get` - Get current temperature reading
- `@water,cal` - Calibrate water detection threshold
- `@vd,status` - Get void detection system status

## Code Modification Guidelines

### Architecture Overview

The codebase uses a layered architecture:

- **HAL Layer**: STM32 Hardware Abstraction Layer (Core/, Drivers/)
- **VMT Layer**: Vehicle Monitoring Technology layer (vmt_*.c/h files)
- **MTI Layer**: Measurement Technology Integration layer (mti_*.c/h files)

### Adding a New Command

The command system uses a table-driven approach in `vmt_command.c`:

1. **Define Command Handler**

   ```c
   void cmd_your_command(vmt_cmd_t* cmd)
   {
       // Parse parameters
       uint32_t param1 = vmt_cmd_get_param_u32(cmd, 1, 0);
       char* param2 = vmt_cmd_get_param_str(cmd, 2);
       
       // Implement functionality
       // ... your code here ...
       
       // Send response
       vmt_cmd_response_ok(cmd);  // or vmt_cmd_response_error()
   }
   ```

2. **Add to Command Table**

   Find the command table in `vmt_command.c`:

   ```c
   static const vmt_cmd_entry_t cmd_table[] = {
       // ... existing commands ...
       { "your_cmd", cmd_your_command },
   };
   ```

3. **Response Formats**
   - Success: `&your_cmd,result_data`
   - Error: `!your_cmd,error_message`
   - Channel routing handled automatically

### Modifying Sensor Processing

**IMU Processing** (`mti_imu.c`):

- **Motion Detection**: Modify `mti_imu_process()` for bump/tilt algorithms
- **Filter Parameters**: Adjust in `mti_imu_set_profile()`
- **Dual Sensor Setup**: Two ICM20948 sensors with automatic failover

**Water Detection** (`mti_water.c`):

- **Threshold Adaptation**: Modify `prv_update_adaptive_threshold()`
- **Detection Logic**: Adjust in `mti_water_process()`
- **Calibration**: Update `mti_water_calibrate()` for offset calculation

**Temperature Monitoring** (`mti_temp.c`):

- **Smoothing Algorithm**: Modify `prv_apply_smoothing()`
- **Alert Thresholds**: Configure via `temp_set_thresholds()`
- **Sampling Rate**: Adjust in `temp_set_config()`

**Void Detection** (`mti_void.c`):

- **Algorithm Selection**: Switch between simple threshold and circle fitting
- **Threshold Parameters**: Configure in `void_set_config()`
- **Circle Fitting**: Advanced 3-point circle fitting with automatic fallback

### Adding a New Sensor Module

To integrate a new sensor into the system:

1. **Hardware Configuration**
   - Add peripheral configuration in STM32CubeMX (`config/Downhole.ioc`)
   - Configure required pins (SPI, I2C, ADC, etc.)
   - Regenerate HAL code if needed

2. **Create Driver Files**

   **Header File** (`Device/Inc/mti_newsensor.h`):

   ```c
   #ifndef MTI_NEWSENSOR_H
   #define MTI_NEWSENSOR_H
   
   #include "stm32f7xx.h"
   #include <stdbool.h>
   
   typedef struct {
       uint16_t raw_value;
       bool     data_valid;
       uint32_t timestamp_ms;
   } newsensor_measurement_t;
   
   bool newsensor_init(void);
   bool newsensor_process(void);
   newsensor_measurement_t* newsensor_get_measurement(void);
   
   #endif
   ```

   **Implementation File** (`Device/Src/mti_newsensor.c`):

   ```c
   #include "mti_newsensor.h"
   #include "vmt_uart.h"
   
   static newsensor_measurement_t measurement = {0};
   
   bool newsensor_init(void) {
       // Initialize hardware interfaces
       // Configure sensor parameters
       debug_send("New sensor initialized");
       return true;
   }
   
   bool newsensor_process(void) {
       // Read sensor data
       // Process and validate
       // Update measurement structure
       return true;
   }
   ```

3. **System Integration**
   - Add initialization call to `mti_system.c` in appropriate init step
   - Add processing call to main loop in `vmt_device.c`
   - Register command handlers in `vmt_command.c`

4. **Command Interface**
   - Add sensor-specific commands following existing patterns
   - Implement configuration and data retrieval commands
   - Follow channel routing conventions

## Common Tasks

### Calibrating Water Sensors

To implement water sensor calibration:

1. Call `mti_water_calibrate()` function
2. The function will:
   - Sample current sensor values
   - Apply the configured offset (typically 500 units)
   - Store the resulting threshold for detection

### Changing Communication Parameters

To modify UART communication parameters:

1. Open Downhole.ioc in STM32CubeMX
2. Navigate to the appropriate UART peripheral
3. Modify the parameters (baud rate, parity, etc.)
4. Regenerate the code
5. Rebuild the project

### Optimizing Power Consumption

To optimize power consumption:

1. Modify sensor sampling rates in timer configurations
2. Implement low-power states in `mti_system.c`
3. Use HAL sleep functions when appropriate

## Testing Procedures

### Basic Functionality Test

1. Connect power to the device
2. Connect to the debug UART port
3. Verify startup messages appear
4. Send `@debug,status` and verify proper response

### Water Detection Test

1. Calibrate the water sensors using `@sensor,w`
2. Expose the sensor to air and verify the reading is above threshold
3. Expose the sensor to water and verify detection occurs
4. Check hysteresis by gradually exposing/removing from water

### IMU Sensor Test

1. Initialize the IMU using `@init,g,1`
2. Verify the IMU ID is correctly reported
3. Check raw sensor values using `@sensor,raw,g`
4. Rotate the device and verify orientation changes

## Troubleshooting

### Common Issues

1. **No UART Communication**
   - Check UART connections and baud rate settings
   - Verify power supply is stable
   - Check if TX/RX lines are reversed

2. **Erratic Sensor Readings**
   - Check sensor connections
   - Verify power supply is stable
   - Check for electrical interference

3. **IMU Initialization Failure**
   - Check SPI connections
   - Verify IMU power control pin is working
   - Check CS pin configuration

### Error Codes

The system may report error codes through the debug interface:

| Code | Description | Solution |
|------|-------------|----------|
| E01  | IMU communication error | Check SPI connections |
| E02  | Flash memory error | Verify flash connections |
| E03  | Calibration error | Retry calibration |
| E04  | Parameter out of range | Check command parameters |

## Performance Considerations

### Critical Sections

The following operations are time-critical and should not be interrupted:

1. IMU data acquisition in interrupt handlers
2. Flash memory operations
3. Water detection algorithm processing

### Memory Usage

The system has the following memory constraints:

1. Flash Memory: 512KB total
2. RAM: 256KB total
3. Critical buffer sizes:
   - UART buffers: 256 bytes each
   - IMU data buffers: 128 bytes per sensor
   - Flash FIFO buffer: 4KB

## Versioning and Release Process

### Version Numbering

The version numbering follows the format `major.minor.patch`:

- Major: Significant feature changes or API modifications
- Minor: New features with backward compatibility
- Patch: Bug fixes and minor improvements

### Release Process

1. Update the version number in `mti_system.h`
2. Update `releases.md` with changes
3. Compile a release build
4. Test all functionality
5. Create a release package with documentation

## Component-Specific Development Guidelines

### Temperature Sensing Development

When working with the enhanced temperature sensing functionality:

1. Use the `mti_temp.h`/`mti_temp.c` module for temperature-related code
2. The temperature command handler (`cmd_temp`) in `vmt_command.c` processes `@tp` commands
3. Temperature values are represented with 0.1°C precision as integers (e.g., 215 = 21.5°C)
4. Alert thresholds can be configured via the `@tp,t,<low>,<high>` command
5. Sampling rate can be adjusted using the `@tp,rate,<samples>` command

### Void Detection Development

This feature requires implementation of the void detection processing logic:

1. The CAN bus interface (`mti_can.h`/`mti_can.c`) is fully implemented
2. The `mti_void.c` file is currently empty and needs implementation
3. The `cmd_void` handler in `vmt_command.c` is a placeholder that needs complete implementation
4. When developing, use the protocol defined in `void_additions.md` for command structure
5. The feature requires hardware with `SENSOR_VD` and `PCB_CANBUS` defines enabled

## Additional Resources

### Reference Documentation

1. STM32F722RET6 Datasheet and Reference Manual
2. ICM20948 IMU Datasheet
3. STM32 HAL Documentation
4. Enhanced Temperature Sensor Datasheet
5. Void Detection Radar Sensor Specification
6. CAN Bus Protocol Documentation

### Support Contacts

For technical support or questions about the codebase, contact:

- Technical Support: <support@company.com>
- Lead Developer: <developer@company.com>
