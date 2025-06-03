Collecting workspace information# Integration Plan: Adding POC Features to STM32 Void Detection System

## Overview

This plan integrates the POC modules (mti_can.c, `mti_void.c`, `mti_temp.c`) into the existing STM32F722 downhole sensor system while maintaining backward compatibility and consistent messaging formats.

## Current System Analysis

### Existing Architecture
- **Main Controller**: `vmt_device.c` - Core device management
- **CAN Interface**: `mti_can.c` - Basic CAN communication
- **Command Parser**: `vmt_command.c` - ASCII command processing
- **System Management**: `mti_system.c` - State machine and initialization

### Current Message Format
```c
// Existing command format from vmt_command.c
"@status,down,<status>,ver,<major>,<minor>,<sub>"
"@void,<detected>"
"@debug,<module>,<data>"
```

### Current CAN Implementation
```c
// From mti_can.c
#define CAN_CMD 0x80
#define CAN_STATUS 0x04
bool can_send(uint32_t ID, uint8_t message);
```

## Integration Strategy

### Phase 1: File Integration and Structure

#### 1.1 Add POC Headers to Project

Create symbolic links or copy files to maintain project structure:

```bash
# Add to Device/Inc/
mti_void.h     # Void detection interface
mti_temp.h     # Temperature monitoring interface

# Add to Device/Src/ 
mti_void.c     # Void detection implementation
mti_temp.c     # Temperature sensor implementation
```

#### 1.2 Update Include Paths

Modify `c_cpp_properties.json`:

```jsonc
"includePath": [
    // ... existing paths ...
    "${workspaceFolder}/poc/Inc",  // Add POC headers if needed
    "${workspaceFolder}/Device/Inc"
]
```

### Phase 2: Module Integration Points

#### 2.1 Initialize POC Modules in Device Init

Modify `vmt_device.c` `dev_init_process()`:

```c
// In STEP_COMMAND case, after cmd_init():
case STEP_COMMAND:
    cmd_init();
    
    // Initialize POC modules
    if (h_dev_debug.b_init) {
        printf("> void_detection_init\r\n");
    }
    void_detection_init();
    
    if (h_dev_debug.b_init) {
        printf("> temperature_sensor_init\r\n");
    }
    temperature_sensor_init();
    
    step = STEP_FLASH;
```

#### 2.2 Add Processing Calls to Main Loop

Modify `vmt_device.c` `device_process()`:

```c
// In device_process(), after existing sensor processing:
void device_process(void) {
    if (b_init_finish == false) {
        // ... existing init code ...
        return;
    }
    
    // ... existing processing ...
    
    // Add POC module processing
    void_detection_process();
    temperature_sensor_process();
    
    // ... rest of existing code ...
}
```

### Phase 3: Data Integration and Messaging

#### 3.1 Extend Command Parser

Add new commands to `vmt_command.c`:

```c
// Add to command string definitions:
static const char cmd_str_void[] = "@void";
static const char cmd_str_temp[] = "@temp";

// Add to cmd_callback_list:
{
    .ptr = (char*) cmd_str_void,
    .callback = cmd_void_control,
},
{
    .ptr = (char*) cmd_str_temp,
    .callback = cmd_temp_control,
},
```

#### 3.2 Implement New Command Handlers

```c
// Add new command handlers:

static void cmd_void_control(h_str_pointers_t *str_p) {
    uart_tx_channel_set(cmd_uart_ch);
    
    if (str_p->part[1] == NULL) {
        // Query void status
        void_status_t status = get_void_status();
        printf("@void,status,%d\n", status.detected);
        printf("@void,confidence,%d\n", status.confidence);
    }
    else if (strcmp(str_p->part[1], "calibrate") == 0) {
        void_calibration_start();
        printf("@void,calibration,started\n");
    }
    else if (strcmp(str_p->part[1], "threshold") == 0 && str_p->part[2] != NULL) {
        uint16_t threshold = atoi(str_p->part[2]);
        set_void_threshold(threshold);
        printf("@void,threshold,%d\n", threshold);
    }
    
    uart_tx_channel_undo();
}

static void cmd_temp_control(h_str_pointers_t *str_p) {
    uart_tx_channel_set(cmd_uart_ch);
    
    if (str_p->part[1] == NULL) {
        // Query temperature
        float temperature = get_current_temperature();
        printf("@temp,value,%.2f\n", temperature);
    }
    else if (strcmp(str_p->part[1], "calibrate") == 0) {
        temperature_calibration_start();
        printf("@temp,calibration,started\n");
    }
    
    uart_tx_channel_undo();
}
```

#### 3.3 Update System Status Reporting

Modify `mti_system.c` to include POC module status:

```c
// In module_init() function, add void detection test:
case STEP_VOID:
    #ifdef SENSOR_VOID
    if (void_sensor_status == 0) {
        if (retries_void < 3) {
            if (retries_void == 0) void_sensor_init();
            bool test_result = void_sensor_self_test();
            if (test_result) {
                void_sensor_status = 1;
                init_step = STEP_TEMP;
            } else {
                retries_void++;
            }
        } else {
            printf("@status,down,5\n"); // Void sensor error
        }
    }
    else init_step = STEP_TEMP;
    #else
    init_step = STEP_TEMP;
    #endif
    break;

case STEP_TEMP:
    #ifdef SENSOR_TEMP
    if (temp_sensor_status == 0) {
        if (retries_temp < 3) {
            if (retries_temp == 0) temp_sensor_init();
            bool test_result = temp_sensor_self_test();
            if (test_result) {
                temp_sensor_status = 1;
                init_step = STEP_FINISH;
            } else {
                retries_temp++;
            }
        } else {
            printf("@status,down,6\n"); // Temperature sensor error
        }
    }
    else init_step = STEP_FINISH;
    #else
    init_step = STEP_FINISH;
    #endif
    break;
```

### Phase 4: CAN Message Integration

#### 4.1 Extend CAN Message Definitions

Update `mti_can.h`:

```c
// Add new CAN message types:
#define CAN_VOID_DETECTION  0x05
#define CAN_TEMP_DATA       0x06
#define CAN_VOID_THRESHOLD  0x07

// Extended CAN IDs for POC data
#define CAN_ID_VOID_STATUS  0xA4
#define CAN_ID_TEMP_DATA    0xA5
```

#### 4.2 Implement POC Data Transmission

Add to `mti_can.c`:

```c
// Add POC-specific CAN transmission functions:

bool can_send_void_status(void_status_t *status) {
    uint8_t message[4] = {
        CAN_VOID_DETECTION,
        status->detected ? 1 : 0,
        status->confidence,
        status->threshold & 0xFF
    };
    return can_send_array(CAN_ID_VOID_STATUS, message, 4);
}

bool can_send_temperature_data(float temperature) {
    uint8_t message[5];
    message[0] = CAN_TEMP_DATA;
    
    // Convert float to bytes (IEEE 754)
    union {
        float f;
        uint8_t bytes[4];
    } temp_union;
    temp_union.f = temperature;
    
    memcpy(&message[1], temp_union.bytes, 4);
    return can_send_array(CAN_ID_TEMP_DATA, message, 5);
}
```

### Phase 5: Event-Driven Reporting

#### 5.1 Add Event Callbacks

Create event notification system in `vmt_device.c`:

```c
// Add callback functions for POC events:

void on_void_detected(void_status_t *status) {
    // Report via UART
    uart_tx_channel_set(UART_UPHOLE);
    printf("@void,%d,%d\n", status->detected ? 1 : 0, status->confidence);
    uart_tx_channel_undo();
    
    // Report via CAN if enabled
    #ifdef PCB_CANBUS
    can_send_void_status(status);
    #endif
    
    // Log for debugging
    uart_tx_channel_set(UART_DEBUG);
    printf("@db,Void detection event: detected=%d, confidence=%d\n", 
           status->detected, status->confidence);
    uart_tx_channel_undo();
}

void on_temperature_changed(float temperature, float previous) {
    // Only report significant changes (>1°C)
    if (fabs(temperature - previous) > 1.0f) {
        uart_tx_channel_set(UART_UPHOLE);
        printf("@temp,%.2f\n", temperature);
        uart_tx_channel_undo();
        
        #ifdef PCB_CANBUS
        can_send_temperature_data(temperature);
        #endif
    }
}
```

### Phase 6: Configuration and Testing

#### 6.1 Add Feature Flags

Update `mti_system.h`:

```c
// Add feature flags:
#if HW_VER >= 3
    #define PCB_CANBUS
    #define SENSOR_VOID     // Enable void detection
    #define SENSOR_TEMP     // Enable temperature monitoring
    // #define SENSOR_RADAR // Keep optional
#endif
```

#### 6.2 Update Debug Commands

Extend `vmt_command.c` debug interface:

```c
// In cmd_debug() function, add POC module debug:
else if (strstr(str_p->part[1], "void") != NULL) {
    if (str_p->part[2] != NULL) {
        bool enable = atoi(str_p->part[2]);
        void_debug_enable(enable);
        printf("@db,Void debug = %d\r\n", enable);
    }
}
else if (strstr(str_p->part[1], "temp") != NULL) {
    if (str_p->part[2] != NULL) {
        bool enable = atoi(str_p->part[2]);
        temp_debug_enable(enable);
        printf("@db,Temperature debug = %d\r\n", enable);
    }
}
```

## Implementation Timeline

### Week 1: Foundation
- [ ] Add POC source files to project structure
- [ ] Update include paths and project configuration
- [ ] Verify compilation without functionality

### Week 2: Basic Integration
- [ ] Implement initialization calls in device startup
- [ ] Add basic processing loops
- [ ] Test module loading and basic operation

### Week 3: Messaging Integration
- [ ] Implement command parser extensions
- [ ] Add UART message formatting
- [ ] Test command interface functionality

### Week 4: CAN Integration
- [ ] Implement CAN message extensions
- [ ] Add event-driven reporting
- [ ] Integration testing with existing CAN functionality

### Week 5: Testing and Validation
- [ ] System integration testing
- [ ] Performance validation
- [ ] Documentation updates

## Testing Strategy

### Unit Testing
```c
// Test void detection integration
bool test_void_integration(void) {
    // Initialize void detection
    void_detection_init();
    
    // Simulate void condition
    simulate_void_condition(true);
    
    // Process and verify response
    void_detection_process();
    
    // Check message was sent
    return verify_void_message_sent();
}
```

### Integration Testing
```bash
# Test command interface
@void                    # Should return current status
@void threshold 2000    # Should set threshold
@temp                   # Should return temperature
@debug void 1          # Should enable void debug output
```

## Backward Compatibility

### Preserved Interfaces
- All existing UART command formats remain unchanged
- Current CAN message IDs (0x80-0xA3) remain functional
- Existing sensor interfaces (IMU, water) unmodified
- Debug output format consistency maintained

### New Additions Only
- New CAN IDs (0xA4-0xA5) for POC data
- Additional command handlers for void/temp control
- Extended debug capabilities
- Additional status codes for new sensors

This integration plan ensures minimal disruption to the existing codebase while adding comprehensive void detection and temperature monitoring capabilities that align with the POC requirements.
