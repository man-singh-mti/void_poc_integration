---
applyTo: '**'
---
# Embedded Systems Coding Standards

## General Principles

- Follow the BARR-C guidelines for embedded C development
- Prioritize memory efficiency and hardware constraints
- Consider real-time system requirements in all implementations
- Always document hardware-software interfaces thoroughly
- Optimize for readability and maintainability first, performance second

## Code Structure and Organization

### Layered Architecture
- Maintain strict layering: Application → Driver/Middleware → Core/HAL → Hardware
- Higher layers may depend on lower layers, but not vice versa
- Avoid circular dependencies between modules

## File Structure and Naming

### Directories
- `Core/`: STM32CubeMX generated core files
- `Device/`: Custom device layer code
- `Drivers/`: Standard peripheral drivers
- `MDK-ARM/`: Keil IDE project files
- `Config/`: Configuration files
- `Docs/`: Documentation
- `Scripts/`: Build and utility scripts

### File Headers
- All files must include a standardized header comment block
- Include file name, brief description, author, date, and detailed description

## Naming Conventions

- **Variables**: snake_case (current_sensor_value)
- **Constants**: UPPER_SNAKE_CASE (MAX_BUFFER_SIZE)
- **Macros**: UPPER_SNAKE_CASE (CALCULATE_CHECKSUM(x))
- **Functions**: snake_case, verb-based (initialize_device())
- **Types**: snake_case_t (sensor_config_t)
- **Enumerations**: snake_case_t (operation_mode_t)
- **Enum Members**: UPPER_SNAKE_CASE (MODE_STANDBY)
- **Global Variables**: g_snake_case (g_system_state)
- **Private Functions**: static snake_case (static calculate_checksum())
- **Unions**: snake_case_u (data_converter_u)
- **Files**: snake_case.ext (temperature_sensor.c)

## Code Formatting

### Indentation and Spacing
- Use 4 spaces for indentation (not tabs)
- Limit line length to 169 characters
- Align related variable declarations and assignments
- Place spaces after keywords and around operators
- No space between function name and opening parenthesis
- One space after commas in parameter lists

### Braces
- Opening brace on a new line for functions and control structures
- Closing brace aligned with the start of the opening line
- Always use braces for control structures, even for single-line blocks

## Documentation

### Function Documentation
- Use Doxygen-style comments for all public functions
- Document parameters with [in], [out], or [in,out] indicators
- Include @brief, @param, @return, and @note as appropriate
- Explain the "why" not just the "what" in comments

### Hardware Interface Documentation
- Document register addresses and bit meanings
- Include references to hardware datasheets and section numbers
- Specify endianness and alignment requirements for bitfields

## Types and Variables

### Type Definitions
- Use fixed-width integer types from stdint.h (uint32_t, int16_t)
- Add suffixes to type names (_t, _e, _u, _cb)
- Document the purpose of each type definition
- Use enums to define related constants

### Variables
- Declare at the smallest possible scope
- Initialize at declaration when possible
- Use const qualifier aggressively
- Align variable declarations for readability
- Minimize the use of global variables
- Use volatile for variables shared with ISRs or hardware registers

## Hardware and Registers

### Register Access
- Use hardware abstraction when possible
- Create well-named macros or inline functions for direct register access
- Use bit field unions for register access with proper documentation
- Specify target endianness in register access code

### Interrupt Handling
- Keep ISRs short and fast
- Prefix ISR function names with `ISR_`
- Don't use recursive functions or dynamic memory in ISRs
- Use critical sections sparingly and briefly
- Always restore interrupt state when exiting a critical section

## Memory Management

- No dynamic memory allocation (malloc/free) in production code
- Use static memory pools where possible
- Always validate buffer sizes before operations
- Check for potential overflow before performing memory operations

## Error Handling

- Use standardized error codes throughout the project
- Always check return values from functions that can fail
- Handle all error conditions explicitly
- Use assertions for conditions that should never happen

## C99 Language Features

- Use designated initializers for structures
- Use inline functions instead of function-like macros
- Use restricted pointers when applicable to help compiler optimizations
- Use flexible array members for variable-length structures

## Example Implementations

### STM32 Register Access
```c
// Direct register access with proper documentation
#define LED_ON()  (GPIOA->BSRR = GPIO_PIN_5)
#define LED_OFF() (GPIOA->BSRR = (GPIO_PIN_5 << 16))

// Or with the HAL
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
```

### Interrupt Handler
```c
/**
 * @brief UART receive interrupt handler
 */
void ISR_UART_RX(void)
{
    // Clear interrupt flag first
    UART->SR &= ~UART_SR_RXNE;
    
    // Process data quickly
    volatile uint8_t data = UART->DR;
    
    // Store for later processing
    if (g_rx_buffer_count < RX_BUFFER_SIZE)
    {
        g_rx_buffer[g_rx_buffer_write_index] = data;
        g_rx_buffer_write_index = (g_rx_buffer_write_index + 1) % RX_BUFFER_SIZE;
        g_rx_buffer_count++;
    }
}
```

### Static Memory Pool Example
```c
// Memory pool for message buffers
static uint8_t message_pool[MAX_MESSAGES][MESSAGE_SIZE];
static bool message_used[MAX_MESSAGES] = {false};

// Allocate a message buffer
uint8_t* get_message_buffer(void)
{
    for (uint8_t i = 0; i < MAX_MESSAGES; i++)
    {
        if (!message_used[i])
        {
            message_used[i] = true;
            return message_pool[i];
        }
    }
    return NULL; // No free buffers
}

// Return a buffer to the pool
void release_message_buffer(uint8_t* buffer)
{
    if (buffer == NULL)
    {
        return;
    }
    
    for (uint8_t i = 0; i < MAX_MESSAGES; i++)
    {
        if (buffer == message_pool[i])
        {
            message_used[i] = false;
            break;
        }
    }
}
```

### Python Script Example
```python
#!/usr/bin/env python3
"""
Flash Programmer Script

This script programs STM32 devices over UART using STM32 bootloader protocol.
"""
import argparse
import serial
import sys
import time
from typing import List, Dict, Optional

# Constants
ACK = 0x79
NACK = 0x1F

class STM32Flasher:
    def __init__(self, port: str, baudrate: int = 115200):
        """Initialize the flasher with serial port settings.
        
        Args:
            port: Serial port name
            baudrate: Communication speed
        """
        self.serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
    def connect(self) -> bool:
        """Establish connection with the bootloader.
        
        Returns:
            True if connection successful, False otherwise
        """
        # Send initial byte to synchronize
        self.serial.write([0x7F])
        response = self.serial.read(1)
        
        if len(response) == 0 or response[0] != ACK:
            print("Error: Failed to connect to bootloader")
            return False
            
        return True
        
    def read_memory(self, address: int, length: int) -> Optional[bytes]:
        """Read data from device memory.
        
        Args:
            address: Starting memory address
            length: Number of bytes to read
            
        Returns:
            Bytes read or None if operation failed
        """
        if length > 256:
            print("Error: Can only read up to 256 bytes at once")
            return None
            
        # Send read memory command
        self.serial.write([0x11, 0xEE])  # Command + XOR
        if self._wait_for_ack() != ACK:
            return None
            
        # Send address
        addr_bytes = [(address >> 24) & 0xFF, 
                      (address >> 16) & 0xFF,
                      (address >> 8) & 0xFF,
                      address & 0xFF]
                      
        checksum = addr_bytes[0] ^ addr_bytes[1] ^ addr_bytes[2] ^ addr_bytes[3]
        self.serial.write(addr_bytes + [checksum])
        
        if self._wait_for_ack() != ACK:
            return None
            
        # Send length
        self.serial.write([length - 1, 0xFF ^ (length - 1)])
        
        if self._wait_for_ack() != ACK:
            return None
            
        # Read data
        data = self.serial.read(length)
        if len(data) != length:
            print(f"Error: Expected {length} bytes, got {len(data)}")
            return None
            
        return data
        
    def _wait_for_ack(self) -> int:
        """Wait for ACK or NACK from device.
        
        Returns:
            Received byte (ACK, NACK, or 0 for timeout)
        """
        response = self.serial.read(1)
        return response[0] if response else 0


def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(description="STM32 Flash Programmer")
    parser.add_argument("--port", required=True, help="Serial port")
    parser.add_argument("--baudrate", type=int, default=115200, help="Baud rate")
    parser.add_argument("--read", action="store_true", help="Read memory")
    parser.add_argument("--write", action="store_true", help="Write memory")
    parser.add_argument("--address", type=lambda x: int(x, 0), help="Memory address")
    parser.add_argument("--file", help="Binary file to read/write")
    
    args = parser.parse_args()
    
    flasher = STM32Flasher(args.port, args.baudrate)
    if not flasher.connect():
        sys.exit(1)
        
    print("Connected to bootloader!")
    
    if args.read and args.address is not None:
        # Implementation of read operation
        pass
        
    elif args.write and args.file and args.address is not None:
        # Implementation of write operation
        pass
        
    else:
        parser.print_help()
    
if __name__ == "__main__":
    main()
```

## Static Analysis Rules

### Function Size Limits
- Maximum 100 lines per function
- Maximum 50 statements per function
- Maximum 15 branches per function

### Bug Prevention Checks
- No assignments within if conditions
- No infinite loops
- No suspicious sizeof expressions
- No misplaced widening casts
- No suspicious memset usage
- No narrowing conversions

## Clang-Format Settings
- 4 spaces for indentation, no tabs
- Column limit of 169 characters
- Unix-style line endings (LF)
- Aligned assignments, declarations, macros, and trailing comments
- Braces on new lines for functions and control structures
- No short blocks, functions, if statements, or loops on single lines
