man-singh-mti: @workspace

# codebase

time to analuse the entire codebase at it is at the moment and update the READme.MD, use #file:styleguide.instructions.md and #file:coding_style_guide.md to help format the readme,
use best practices and align to industry standards

Workspace:

man-singh-mti: #codebase

# Task: Analyse STM32 Codebase and Update README.md

**Role:**  
Act as a software/sysy abom design while providing clear, structured guidance suitable for both beginners and experienced engineers.

**Inputs:**  

- The entire STM32 project codebase (current state).  
- `#file:styleguide.instructions.md` (documentation style guide).  
- `#file:coding_style_guide.md` (coding conventions and guidelines).  

**Objectives:**  

1. Analyse the existing STM32 codebase structure and components.  
2. Create or update `README.md` to accurately describe:  
   - Project purpose and scope (generic).  
   - Directory and module layout.  
   - Build and flash instructions using common STM32 toolchains.  
   - Key components, dependencies and coding conventions.  
   - Best practices and industry-standard patterns.  
3. Use `styleguide.instructions.md` and `coding_style_guide.md` to guide formatting and content.  
4. Ensure the final `README.md` is:  
   - Well-formatted Markdown.  
   - Understandable by newcomers.  
   - Detailed enough for an experienced engineer to onboard quickly.  
   - Aligned with industry standards for embedded-system documentation.  

---

## 1. Preparation

1. **Load the Codebase**  
   - Clone or open the repository locally.  
   - Confirm that all files and folders are present.

2. **Inspect Reference Files**  
   - Review `#file:styleguide.instructions.md` to understand the expected Markdown layout, heading hierarchy, and formatting conventions (bullet points, code blocks, etc.).  
   - Review `#file:coding_style_guide.md` to identify naming conventions, indentation rules, comment styles and any required code formatting that should be referenced in the README.

3. **Adopt an Architect’s Mindset**  
   - Consider the system at a high level: MCU selection, peripheral usage, middleware (RTOS or bare-metal scheduler), and overall data flow.  
   - Explain concepts clearly for beginners, but include notes or references for deeper understanding.

---

## 2. High-Level Codebase Survey

1. **Identify Top-Level Directories**  
   - List each directory in the root (e.g. `Core/`, `Drivers/`, `BoardSupport/`, `Application/`, `Middleware/`, `Scripts/`, `Tests/`, `docs/`).  
   - For each directory, write a one-sentence summary of its purpose.

2. **Locate the Entry Point**  
   - Find `main.c` (often in `Core/Src/` or `Application/`).  
   - Document the startup sequence, including:  
     - System clock configuration (HSE/HSI, PLL settings).  
     - Peripheral initialisation (GPIO, ADC, UART, I²C, SPI, CAN, etc.).  
     - Scheduler or RTOS initialisation (if applicable).  
     - Main application loop or task creation.

3. **Examine Build Configuration**  
   - Locate IDE project files (e.g. Keil uVision `.uvprojx`/`.uvoptx`, STM32CubeIDE `.project`, or Makefiles/CMakeLists).  
   - Note how version numbers, feature flags and build targets (Debug/Release) are defined.  
   - Identify any manifest files (JSON, TOML, YAML) used for configuration or versioning.

4. **Catalogue Hardware-Specific Code**  
   - Identify where MCU-specific initialisation lives (e.g. `system_stm32xxx.c`, `startup_stm32xxx.s`).  
   - Locate modules for each peripheral (e.g. sensor drivers, communication interfaces).  
   - Note any board-support packages (BSP) or pin-mapping configurations.

5. **Scan for Middleware and Libraries**  
   - Check for RTOS (FreeRTOS, CMSIS-RTOS) under `Middleware/`.  
   - List third-party libraries (e.g. USB stacks, file systems, network stacks).  
   - Look for documentation or design diagrams in `docs/`.

6. **Inspect Utility Scripts**  
   - In `Scripts/`, find any Python, Bash or batch files (e.g. version stamping, post-build packaging, firmware upload scripts).  
   - Note how they integrate with the build process.  
   - Locate any test harnesses or validation tools under `Tests/`.

GitHub Copilot:

man-singh-mti:  # Task: Analyse the #codebase  and Update README.md

**Role:**
Act as a software/sysy abom design while providing clear, structured guidance suitable for both beginners and experienced engineers.

**Inputs:**

- The entire STM32 project codebase (current state).
- #file:styleguide.instructions.md  (documentation style guide).

**Objectives:**

1. Analyse the existing STM32 codebase structure and components.
2. Create or update `README.md` to accurately describe:
   - Project purpose and scope (generic).
   - Directory and module layout.
   - Build and flash instructions using common STM32 toolchains.
   - Key components, dependencies and coding conventions.
   - Best practices and industry-standard patterns.
3. Use `styleguide.instructions.md` and `coding_style_guide.md` to guide formatting and content.
4. Ensure the final `README.md` is:
   - Well-formatted Markdown.
   - Understandable by newcomers.
   - Detailed enough for an experienced engineer to onboard quickly.
   - Aligned with industry standards for embedded-system documentation.

GitHub Copilot:

man-singh-mti:  # Task: Analyse the #codebase  and Update README.md

**Role:**
Act as a software/sysy abom design while providing clear, structured guidance suitable for both beginners and experienced engineers.

**Inputs:**

- The entire STM32 project codebase (current state).
- #file:styleguide.instructions.md  (documentation style guide).

**Objectives:**

1. Analyse the existing STM32 codebase structure and components.
2. Create or update `README.md` to accurately describe:
   - Project purpose and scope (generic).
   - Directory and module layout.
   - Build and flash instructions using common STM32 toolchains.
   - Key components, dependencies and coding conventions.
   - Best practices and industry-standard patterns.
3. Use `styleguide.instructions.md` and `coding_style_guide.md` to guide formatting and content.
4. Ensure the final `README.md` is:
   - Well-formatted Markdown.
   - Understandable by newcomers.
   - Detailed enough for an experienced engineer to onboard quickly.
   - Aligned with industry standards for embedded-system documentation.

GitHub Copilot: I'll analyze the entire STM32 codebase and create a comprehensive README.md. Let me start by exploring the project structure and key components. I'll analyze the entire STM32 codebase and create a comprehensive README.md. Let me start by exploring the project structure and key components. I'll analyze the entire STM32 codebase and create a comprehensive README.md. Let me explore the project structure and key components. I'll analyze the entire STM32 codebase and create a comprehensive README.md. Let me explore more components to get a complete picture. Based on my comprehensive analysis of the STM32 codebase, I'll create a detailed README.md that accurately describes this sophisticated downhole sensor system. Let me examine the hardware configuration and additional details first. Based on my comprehensive analysis of the STM32 codebase, I'll create a detailed README.md that accurately documents this sophisticated downhole sensor system.

```markdown
# STM32F7 Downhole Sensor System

A real-time embedded sensor system designed for downhole monitoring applications, featuring dual IMU sensors, water detection, CAN bus communication, and comprehensive data logging capabilities.

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Platform](#hardware-platform)
- [Project Structure](#project-structure)
- [Key Features](#key-features)
- [Build Environment](#build-environment)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [API Reference](#api-reference)
- [Development Guidelines](#development-guidelines)
- [Troubleshooting](#troubleshooting)

## Overview

This project implements a sophisticated downhole sensor monitoring system built on the STM32F722RETx microcontroller. The system is designed for harsh environmental conditions and provides real-time monitoring of:

- **Inertial Measurement**: Dual ICM-20948 9-axis IMU sensors for motion detection and orientation tracking
- **Water Detection**: Multi-channel ADC-based water level sensing with adaptive thresholds
- **Temperature Monitoring**: Environmental temperature sensing for system diagnostics
- **Communication**: UART-based uphole/downhole communication and CAN bus for radar integration
- **Data Logging**: High-speed flash memory data logging with FIFO management

### Key Applications

- Downhole drilling monitoring
- Borehole orientation tracking
- Water intrusion detection
- Equipment impact detection
- Real-time telemetry systems

## System Architecture

The system follows a layered architecture pattern optimized for real-time embedded applications:

```

┌─────────────────────────────────────────────────────┐
│                Application Layer                    │
├─────────────────────────────────────────────────────┤
│  MTI System  │  Command Processor  │  Data Logger   │
├─────────────────────────────────────────────────────┤
│               Device Abstraction Layer              │
├─────────────────────────────────────────────────────┤
│  IMU Driver  │  ADC Manager  │  UART Interface      │
├─────────────────────────────────────────────────────┤
│                STM32 HAL Layer                      │
├─────────────────────────────────────────────────────┤
│              Hardware Abstraction                   │
└─────────────────────────────────────────────────────┘

```

### Data Flow

1. **Sensor Acquisition**: 1kHz IMU sampling with DMA-based ADC conversion
2. **Signal Processing**: Real-time motion analysis and water detection algorithms
3. **Event Detection**: Bottom impact, water intrusion, and orientation change detection
4. **Communication**: Bi-directional UART messaging with uphole systems
5. **Data Storage**: High-speed flash logging with wear leveling

## Hardware Platform

### Target MCU
- **STM32F722RETx** (ARM Cortex-M7, 216MHz)
- **Flash**: 512KB program memory
- **RAM**: 256KB SRAM
- **Clock**: External 8MHz HSE with PLL (32MHz system clock)

### Peripheral Configuration
- **ADC1**: Temperature sensor (PA5)
- **ADC2**: Dual water sensors (PB1, PA6)
- **SPI1**: IMU2 interface (PB3-PB5)
- **SPI2**: IMU1 interface (PB12-PB15)
- **USART1**: Uphole communication (PB6-PB7)
- **USART6**: Debug interface (PC6-PC7)
- **CAN1**: Radar module interface (optional)
- **TIM2/4/5/7**: ADC triggering and IMU sampling

### External Components
- **Dual ICM-20948**: 9-axis IMU sensors with SPI interface
- **Water Sensors**: Analog resistive sensors with configurable thresholds
- **External Flash**: SPI-based data logging storage
- **Power Management**: Low-power sleep modes with wake-on-event

## Project Structure

```

void_poc_integration/
├── Core/                           # STM32CubeMX generated core files
│   ├── Inc/                        # Core header files
│   │   ├── main.h                  # Main application header
│   │   ├── stm32f7xx_hal_conf.h    # HAL configuration
│   │   └── stm32f7xx_it.h          # Interrupt handlers
│   └── Src/                        # Core source files
│       ├── main.c                  # Application entry point
│       ├── system_stm32f7xx.c      # System initialization
│       ├── stm32f7xx_hal_msp.c     # HAL MSP functions
│       └── stm32f7xx_it.c          # Interrupt service routines
├── Device/                         # Custom device layer
│   ├── Inc/                        # Device headers
│   │   ├── mti_*.h                 # MTI system interfaces
│   │   └── vmt_*.h                 # VMT driver interfaces
│   └── Src/                        # Device implementation
│       ├── mti_can.c               # CAN bus communication
│       ├── mti_imu.c               # IMU processing algorithms
│       ├── mti_system.c            # System state management
│       ├── mti_water.c             # Water detection logic
│       ├── vmt_adc.c               # ADC management
│       ├── vmt_command.c           # Command parser
│       ├── vmt_device.c            # Main device controller
│       ├── vmt_flash.c             # Flash memory interface
│       ├── vmt_icm20948.c          # IMU driver
│       └── vmt_uart.c              # UART communications
├── Drivers/                        # STM32 peripheral drivers
│   ├── CMSIS/                      # ARM CMSIS headers
│   └── STM32F7xx_HAL_Driver/       # STM32F7 HAL library
├── MDK-ARM/                        # Keil μVision project files
│   ├── Downhole.uvprojx            # Project configuration
│   └── Downhole.uvoptx             # Project options
├── config/                         # Configuration files
│   └── Downhole.ioc                # STM32CubeMX configuration
├── Output/                         # Build output directory
├── coding_style_guide.md           # Coding standards
├── styleguide.instructions.md      # Documentation guidelines
└── README.md                       # This file

```

## Key Features

### Real-Time Motion Detection
- **Dual IMU System**: Redundant ICM-20948 sensors for fault tolerance
- **1kHz Sampling Rate**: High-frequency motion capture with DMA optimization
- **Motion Analysis**: Bottom impact detection, tilt monitoring, and jerk analysis
- **Calibration**: Automatic gravity vector calculation and sensor fusion

### Advanced Water Detection
- **Multi-Channel Sensing**: Dual water sensors with individual thresholds
- **Adaptive Algorithms**: Dynamic threshold adjustment based on environmental conditions
- **Configurable Parameters**: Adjustable sensitivity and multiplier settings
- **Event Logging**: Water intrusion detection with timestamp correlation

### Communication Protocols
- **UART Messaging**: ASCII-based command protocol for uphole communication
- **CAN Bus Interface**: Radar module integration with extended frame support
- **Debug Interface**: Dedicated UART for development and diagnostics
- **Command Parser**: Extensible text-based command processing

### Data Management
- **Flash FIFO**: Circular buffer implementation with wear leveling
- **High-Speed Logging**: DMA-based data transfer for minimal CPU overhead
- **Data Compression**: Optimized data structures for storage efficiency
- **Retrieval System**: Sequential and random access data reading

### Power Management
- **Sleep Modes**: STM32 STOP mode with wake-on-event capability
- **Power Sequencing**: Controlled sensor power management
- **Low-Power Design**: Optimized for battery-powered applications

## Build Environment

### Prerequisites

#### Keil μVision MDK-ARM (Recommended)
- **Keil MDK-ARM**: Version 5.29 or later
- **ARM Compiler**: ARMCC v5.06 or ARM Compiler 6
- **STM32F7xx DFP**: Device family pack v2.15.0+
- **CMSIS**: v5.7.0+

#### Alternative Toolchains
- **STM32CubeIDE**: Eclipse-based IDE with GCC
- **System Workbench**: OpenSTM32 with GCC toolchain
- **Command Line**: ARM GCC with Makefile

### Compiler Configuration
```c
// Optimization settings
#define ARM_MATH_CM7                    // Enable ARM DSP optimizations
#define __FPU_PRESENT           1       // Enable FPU support
#define HSE_VALUE               8000000 // 8MHz external crystal
#define VECT_TAB_OFFSET         0x0     // Vector table offset
```

### Memory Configuration

```
Flash:  0x08000000 - 0x0807FFFF (512KB)
SRAM:   0x20000000 - 0x2003FFFF (256KB)
DTCM:   0x20000000 - 0x2000FFFF (64KB)
ITCM:   0x00000000 - 0x00003FFF (16KB)
```

## Getting Started

### 1. Environment Setup

#### Using Keil μVision

```bash
# Open project file
File → Open Project → MDK-ARM/Downhole.uvprojx

# Configure debug interface
Project → Options → Debug → ST-Link Debugger

# Set target device
Project → Options → Device → STM32F722RETx
```

#### Using STM32CubeIDE

```bash
# Import existing project
File → Import → Existing Projects into Workspace
# Browse to project root directory
```

### 2. Build Process

#### Keil MDK-ARM

```bash
# Build project
Project → Build Target (F7)

# Clean build
Project → Clean (Ctrl+Alt+F7)

# Build output
Output/Listings/Downhole.hex
Output/Listings/Downhole.axf
```

#### Command Line (ARM GCC)

```bash
# Install ARM GCC toolchain
sudo apt-get install gcc-arm-none-eabi

# Build with make
make clean
make all

# Program device
make flash
```

### 3. Hardware Connections

#### Debug Interface (ST-Link V2)

```
ST-Link    │ STM32F722
-----------│-----------
SWDIO      │ PA13
SWCLK      │ PA14
GND        │ GND
3V3        │ 3V3
NRST       │ NRST
```

#### UART Debug Console

```
UART Config: 115200-8-N-1
Debug Port:  USART6 (PC6-TX, PC7-RX)
Uphole Port: USART1 (PB6-TX, PB7-RX)
```

### 4. Initial Programming

#### Using ST-Link Utility

```bash
# Connect ST-Link programmer
# Load hex file: Output/Listings/Downhole.hex
# Program and verify flash
# Start execution
```

#### Using OpenOCD

```bash
# Connect and program
openocd -f interface/stlink.cfg -f target/stm32f7x.cfg \
        -c "program Output/Listings/Downhole.hex verify reset exit"
```

### 5. System Verification

#### Debug Console Commands

```bash
# Connect to debug UART (115200 baud)
# System should output initialization messages

@init,down,ok              # System initialized
@status,down,1,ver,1,2,4   # Version information
@connect                   # Connection established
```

#### Sensor Testing

```bash
# IMU sensor test
@sensor g 1                # Set IMU profile
@debug imu 1              # Enable IMU debug output

# Water sensor calibration
@sensor w                 # Read current water values
@sensor w 1200           # Set water threshold
```

## Configuration

### System Parameters

#### IMU Configuration

```c
// vmt_device.h
#define DEV_IMU_SAMPLE_RATE     (1000.0)    // 1kHz sampling
#define DEV_IMU_AVG_TIME        (200.0)     // 200ms averaging
#define DEV_IMU_NUM             (2)         // Dual IMU setup

// IMU thresholds (configurable via commands)
uint16_t threshold_air = 4000;              // Air threshold
uint16_t threshold_water = 3000;            // Water threshold
```

#### ADC Configuration

```c
// ADC sampling rates
#define DEV_ADC_SAMPLE_RATE_DEFAULT (400)   // 400Hz default

// Water detection thresholds
float thre_h = 1.0;                         // High threshold (volts)
float thre_l = 0.8;                         // Low threshold (volts)
```

#### Communication Settings

```c
// UART configuration
#define DEBUG_BAUD_RATE         115200      // Debug interface
#define UPHOLE_BAUD_RATE        115200      // Uphole communication

// CAN bus configuration
#define CAN_BITRATE             125000      // 125kbps CAN
```

### Firmware Parameters

#### Build Configuration

```c
// Version information
#define FW_VER_MAJOR            (1)
#define FW_VER_MINOR            (2)
#define FW_VER_SUB              (4)

// Hardware version
#define HW_VER                  3

// Feature flags
#define PCB_CANBUS                          // Enable CAN interface
// #define SENSOR_RADAR                     // Enable radar sensor
```

### Runtime Configuration

#### Command Interface

The system supports runtime configuration through ASCII commands:

```bash
# System commands
@connect                    # Establish connection
@start                      # Begin measurement mode
@finish                     # Stop measurement mode
@sleep                      # Enter low-power mode

# Sensor configuration
@sensor w <threshold>       # Set water threshold
@sensor g <profile>         # Set IMU profile

# Debug commands
@debug imu <dataset>        # Enable IMU debug output
@debug water              # Enable water debug output
@debug radar <command>     # Control radar module

# Data logging
@log save 1                # Enable data logging
@log report 1              # Enable log reporting
@flash erase               # Erase all stored data
```

## API Reference

### Core Device Interface

#### Initialization

```c
/**
 * @brief Initialize device subsystems
 * @note Called automatically during startup
 */
void device_init_start(void);
bool device_init_finish_get(void);
```

#### Main Processing Loop

```c
/**
 * @brief Main device processing function
 * @note Call continuously in main loop
 */
void device_process(void);
```

### IMU Interface

#### Data Structures

```c
typedef struct h_imu_ {
    uint8_t id;                     // IMU identifier (0 or 1)
    int16_t accel_x, accel_y, accel_z;  // Acceleration data
    int16_t gyro_x, gyro_y, gyro_z;     // Gyroscope data
    int16_t mag_x, mag_y, mag_z;        // Magnetometer data
    h_vector_t h_accel;             // Processed acceleration vector
    h_vector_t h_gyro;              // Processed gyroscope vector
    bool b_update;                  // New data available flag
    bool b_overflow;                // Data overflow flag
} h_imu_t;
```

#### Functions

```c
/**
 * @brief Get IMU overflow status
 * @param imu_select IMU channel (0 or 1)
 * @return true if overflow occurred
 */
bool imu_overflow_get(uint8_t imu_select);

/**
 * @brief Clear IMU overflow flag
 * @param imu_select IMU channel (0 or 1)
 */
void imu_overflow_clear(uint8_t imu_select);
```

### Water Detection Interface

#### Functions

```c
/**
 * @brief Perform water detection
 * @return true if water detected
 */
bool water_detect(void);

/**
 * @brief Set water detection threshold
 * @param reserve Threshold value (ADC counts)
 */
void reserve_set(uint16_t reserve);

/**
 * @brief Get current water threshold
 * @return Threshold value (ADC counts)
 */
uint16_t reserve_get(void);
```

### Communication Interface

#### UART Functions

```c
/**
 * @brief Set UART transmission channel
 * @param channel UART channel selector
 */
void uart_tx_channel_set(uart_select_t channel);

/**
 * @brief Restore previous UART channel
 */
void uart_tx_channel_undo(void);
```

#### CAN Interface

```c
/**
 * @brief Send CAN message
 * @param ID CAN identifier
 * @param message Single byte message
 * @return true if successful
 */
bool can_send(uint32_t ID, uint8_t message);

/**
 * @brief Send CAN array message
 * @param ID CAN identifier
 * @param message Message array
 * @param length Message length
 * @return true if successful
 */
bool can_send_array(uint32_t ID, uint8_t *message, size_t length);
```

### Data Logging Interface

#### Functions

```c
/**
 * @brief Enable/disable data logging
 * @param b_enable Enable flag
 * @return true if successfully enabled
 */
bool dev_log_save_enable_set(bool b_enable);

/**
 * @brief Check if logging is active
 * @return true if logging active
 */
bool dev_log_save_busy_get(void);

/**
 * @brief Erase all stored data
 * @return true if erase started successfully
 */
bool dev_log_erase_all_set(void);
```

## Development Guidelines

### Coding Standards

This project follows the **BARR-C Embedded C Coding Standard** with specific adaptations for STM32 development.

#### Naming Conventions

```c
// Variables: snake_case
uint16_t water_threshold;
bool sensor_active;

// Constants: UPPER_SNAKE_CASE
#define MAX_BUFFER_SIZE         256
#define DEFAULT_SAMPLE_RATE     1000

// Functions: snake_case, verb-based
void initialize_sensors(void);
bool process_imu_data(h_imu_t *p_imu);

// Types: snake_case_t
typedef struct sensor_config_t sensor_config_t;
typedef enum operation_mode_t operation_mode_t;
```

#### Memory Management

```c
// Use static allocation - no malloc/free
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static h_imu_t imu_data[DEV_IMU_NUM];

// Initialize at declaration when possible
static bool system_initialized = false;
static uint32_t sample_count = 0;
```

#### Error Handling

```c
// Always check return values
if (HAL_SPI_Transmit(&hspi1, tx_data, length, timeout) != HAL_OK)
{
    // Handle error appropriately
    error_count++;
    return DEVICE_ERROR;
}

// Use standardized error codes
typedef enum {
    DEVICE_OK = 0,
    DEVICE_ERROR,
    DEVICE_BUSY,
    DEVICE_TIMEOUT
} device_result_t;
```

### Hardware Abstraction

#### Register Access

```c
// Use HAL functions when possible
HAL_GPIO_WritePin(PWR_IMU_GPIO_Port, PWR_IMU_Pin, GPIO_PIN_SET);

// Document direct register access
// Enable GPIOA clock (RCC_AHB1ENR.GPIOAEN = 1)
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
```

#### Interrupt Handlers

```c
/**
 * @brief SPI transfer complete callback
 * @param hspi SPI handle pointer
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // Keep ISR minimal and fast
    if (hspi == &hspi1)
    {
        imu_data_ready_flag = true;
    }
}
```

### Testing and Validation

#### Unit Testing Framework

```c
// Test function example
bool test_water_detection(void)
{
    // Arrange
    reserve_set(1000);
    
    // Act
    bool result = water_detect();
    
    // Assert
    return (result == expected_result);
}
```

#### Hardware-in-the-Loop Testing

```c
// Simulation mode for HIL testing
#ifdef SIMULATION_MODE
    #define ADC_READ_SIMULATED(channel) simulate_adc_value(channel)
#else
    #define ADC_READ_SIMULATED(channel) HAL_ADC_GetValue(&hadc1)
#endif
```

### Debug and Profiling

#### Debug Output

```c
// Use debug channels appropriately
uart_tx_channel_set(UART_DEBUG);
printf("@db,IMU calibration complete: x=%d, y=%d, z=%d\r\n", 
       cal_x, cal_y, cal_z);
uart_tx_channel_undo();
```

#### Performance Monitoring

```c
// Measure execution time
uint32_t start_time = HAL_GetTick();
process_imu_data(&imu_data[0]);
uint32_t execution_time = HAL_GetTick() - start_time;

if (execution_time > MAX_PROCESSING_TIME)
{
    performance_warnings++;
}
```

### Version Control Guidelines

#### Commit Message Format

```
type(scope): brief description

Detailed explanation of changes made, including:
- What was changed and why
- Any breaking changes
- References to issues or requirements

Examples:
feat(imu): add dual sensor redundancy support
fix(water): correct threshold calculation overflow
docs(readme): update build instructions for STM32CubeIDE
```

#### Branch Strategy

```
main            # Production-ready code
develop         # Integration branch
feature/xxx     # Feature development
hotfix/xxx      # Critical bug fixes
release/vx.y.z  # Release preparation
```

## Troubleshooting

### Common Issues

#### Build Problems

**Error: "Device not found"**

```bash
# Solution: Check device configuration
Project → Options → Device → Select STM32F722RETx
# Verify correct device family pack is installed
```

**Error: "Undefined reference to HAL functions"**

```bash
# Solution: Enable required HAL modules
# Edit Core/Inc/stm32f7xx_hal_conf.h
#define HAL_SPI_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
```

#### Hardware Issues

**IMU sensors not responding**

```c
// Check power and SPI connections
// Verify chip select polarity
// Test with oscilloscope on SPI lines
// Check IMU power sequencing
```

**Water detection inconsistent**

```c
// Calibrate ADC references
// Check sensor connections
// Verify threshold settings
// Monitor ADC noise levels
```

#### Communication Problems

**UART data corruption**

```c
// Check baud rate settings
// Verify hardware flow control
// Monitor for electrical noise
// Check cable connections
```

**CAN bus issues**

```c
// Verify termination resistors (120Ω)
// Check CAN_H and CAN_L wiring
// Confirm bitrate settings
// Test with CAN analyzer
```

### Debug Techniques

#### Live Debugging

```c
// Enable real-time debug output
@debug imu 1              // IMU data stream
@debug water              // Water sensor values
@debug dev                // Device status

// Monitor system variables
printf("@val,%d:%d:%d\r\n", var1, var2, var3);
```

#### Memory Analysis

```c
// Check stack usage
extern uint32_t _estack;
extern uint32_t _Min_Stack_Size;

uint32_t stack_used = &_estack - (uint32_t*)__get_MSP();
if (stack_used > STACK_WARNING_THRESHOLD)
{
    printf("@db,Stack usage warning: %d bytes\r\n", stack_used);
}
```

#### Performance Profiling

```c
// Measure interrupt latency
void EXTI4_IRQHandler(void)
{
    uint32_t entry_time = DWT->CYCCNT;
    
    // Process interrupt
    HAL_GPIO_EXTI_IRQHandler(IMU1_INT_Pin);
    
    uint32_t exit_time = DWT->CYCCNT;
    interrupt_latency = exit_time - entry_time;
}
```

### Support Resources

#### Documentation

- [STM32F722 Reference Manual](https://www.st.com/resource/en/reference_manual/dm00305666.pdf)
- [ICM-20948 Datasheet](https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf)
- [HAL Driver Documentation](https://www.st.com/resource/en/user_manual/dm00189702.pdf)

# void_poc_integration Firmware

## Overview
Bare-metal STM32F7 firmware demonstrating integration of UART, SPI, ADC, DMA and CAN peripherals to drive dual IMU (ICM-20948) and water-level sensors. Optimized for real-time data acquisition and telemetry.

## Repository Layout
- Core/        : STM32CubeMX–generated system and peripheral code  
- Device/      : Custom device layer (IMU drivers, ADC manager, UART/CAN protocols)  
- Drivers/     : CMSIS and STM32F7 HAL libraries  
- MDK-ARM/     : Keil μVision project files (`Downhole.uvprojx`)  
- Config/      : CubeMX `.ioc` configuration  
- Scripts/     : Build/flash helper scripts  
- Docs/        : Coding and documentation style guides  

## Prerequisites
- Keil MDK-ARM v5.x with ARMCC5  
- STM32F7xx Device Family Pack (DFP)  
- CMSIS & HAL libraries  

## Build (Keil μVision)
1. Open `MDK-ARM/Downhole.uvprojx` in Keil μVision.  
2. Ensure target is set to STM32F722/723.  
3. Build: **Project → Build Target** (F7).  

## Flashing
**ST-Link Utility**  
- Load the generated `.hex` or `.axf` and program device.  

**OpenOCD**  
```bash
openocd -f interface/stlink.cfg \
        -f target/stm32f7x.cfg \
        -c "program MDK-ARM/Output/Downhole.hex verify reset exit"
```

## Coding Conventions
- Follow BARR-C / C99 style (no dynamic memory, static pools)  
- 4-space indentation, ≤168 chars/line  
- `snake_case` for variables/functions, `UPPER_SNAKE_CASE` for constants/macros  
- Doxygen comments on public APIs  

Refer to `Docs/coding_style_guide.md` and `styleguide.instructions.md` for full guidelines.

## License
[Add your license or copyright here]
