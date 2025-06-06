# Command Reference

This document provides a comprehensive reference for all commands implemented in the STM32F722-based downhole sensor system, based on detailed analysis of the actual command handlers in `vmt_command.c`.

## Communication Protocol

Commands follow this structured syntax:

```text
@command,parameter1,parameter2,parameter3,...
```

**Protocol Rules:**

- All commands start with the `@` character
- Parameters are separated by commas without spaces
- Commands are terminated with newline character (`\n`)
- Channel-specific routing supports multi-channel operation
- Response format varies by command type and target channel

## System Control Commands

### Connection and Status

| Command   | Parameters | Description                | Response                                     | Channel |
|-----------|------------|----------------------------|----------------------------------------------|---------|
| `@status` | none       | Request system status      | `@status,down,<state>,ver,<maj>,<min>,<sub>` | 0,1     |
| `@ver`    | none       | Request firmware version   | Version information string                   | 0,1     |
| `@reset`  | none       | System reset and restart   | System restart                               | 0,1     |
| `@wake`   | none       | Wake from sleep mode       | `@wake` confirmation                         | 0,1     |
| `@sleep`  | none       | Enter low-power sleep mode | `@sleep` then sleep mode                     | 0,1     |

### System State Management

| Command                   | Parameters                           | Description                                    | Response                        | Channel |
|---------------------------|--------------------------------------|------------------------------------------------|---------------------------------|---------|
| `@start`                  | none                                 | Begin sensor data acquisition                  | `@start` confirmation           | 0,1     |
| `@stop`                   | none                                 | Stop sensor data acquisition                   | `@stop` confirmation            | 0,1     |
| `@init`                   | `<module>`                           | Initialize specific module                     | Module-specific response        | 0,1     |
| `@test`                   | `<module>`                           | Execute module self-test                       | Test results                    | 0,1     |
| ------------------------- | ------------------------------------ | ---------------------------------------------- | ------------------------------- |         |
| `@sensor,w`               | none                                 | Calibrate water sensors using current values   | `@sensor,w,<water1>,<water2>`   |         |
| `@sensor,w,<threshold>`   | `<threshold>`: ADC threshold value   | Set water detection threshold manually         | Debug confirmation              |         |
| `@sensor,g,<profile>`     | `<profile>`: 0-2                     | Set IMU profile                                | Debug confirmation              |         |

## Data Access Commands

### Sensor Data

| Command | Parameters | Description                     | Response                     |
|---------|------------|---------------------------------|------------------------------|
| `@dt`   | none       | Request detection status        | System state information     |
| `@bt`   | none       | Request bottom detection status | Bottom detection information |
| `@wt`   | none       | Request water detection status  | Water detection information  |

### IMU Control and Data

| Command                | Parameters         | Description            | Response           |
|------------------------|--------------------|------------------------|--------------------|
| `@imu,<param>,<value>` | Various parameters | Configure IMU behavior | Status information |

## Logging and Debug Commands

### Data Logging

| Command               | Parameters                     | Description                          | Response             |
|-----------------------|--------------------------------|--------------------------------------|----------------------|
| `@log,save,<state>`   | `<state>`: 0=disable, 1=enable | Enable/disable data logging          | Debug confirmation   |
| `@log,report,<state>` | `<state>`: 0=disable, 1=enable | Enable/disable log reporting         | Status message       |
| `@log,erase`          | none                           | Erase all logged data                | Confirmation message |
| `@log,auto,<state>`   | `<state>`: 0=disable, 1=enable | Configure automatic logging on start | Debug confirmation   |

### Debugging

| Command                | Parameters           | Description                 | Response                    |
|------------------------|----------------------|-----------------------------|-----------------------------|
| `@echo,<text>`         | `<text>`: any string | Echo back the provided text | `@echo,<text>`              |
| `@cmd,<param>,<value>` | Various parameters   | Advanced system commands    | Parameter-specific response |

## Flash Storage Commands

### Flash FIFO Commands

| Command                 | Parameters                              | Description                     | Response                  |
|-------------------------|-----------------------------------------|---------------------------------|---------------------------|
| `@ff,w,<size>[,<data>]` | `<size>`: data size, `<data>`: hex data | Write data to flash FIFO        | Status with data echo     |
| `@ff,r`                 | none                                    | Read data from flash FIFO front | Status with data          |
| `@ff,n`                 | none                                    | Get flash FIFO status           | FIFO position information |
| `@ff,d[,<seq>]`         | `<seq>`: optional sequence number       | Delete entry from FIFO          | Status message            |
| `@ff,ea`                | none                                    | Erase all flash FIFO contents   | Status message            |
| `@ff,ed`                | none                                    | Erase dirty flash FIFO sectors  | Status message            |

### Raw Flash Commands

| Command                             | Parameters                             | Description                    | Response              |
|-------------------------------------|----------------------------------------|--------------------------------|-----------------------|
| `@flash,e,<sector>`                 | `<sector>`: flash sector (0-7)         | Erase flash sector             | Status message        |
| `@flash,p,<addr>,<len>[,<data>]`    | `<addr>`: hex address, `<len>`: length | Program flash data             | Status with data echo |
| `@flash,r,<addr>,<len>[,<compare>]` | `<addr>`: hex address, `<len>`: length | Read flash data                | Status with data      |
| `@flash,ps`                         | none                                   | Check flash programming status | Status message        |

## SPI Communication Commands

| Command                   | Parameters                                                   | Description   | Response                  |
|---------------------------|--------------------------------------------------------------|---------------|---------------------------|
| `@spi,<ch>,<size>,<data>` | `<ch>`: SPI channel, `<size>`: data size, `<data>`: hex data | Send SPI data | Status with received data |

## Enhanced Sensor Commands

### Temperature Commands

| Command              | Parameters                                                | Description                            | Response                                 |
|----------------------|-----------------------------------------------------------|----------------------------------------|------------------------------------------|
| `@tp` or `@tp,?`     | none                                                      | Query current temperature              | `&tp,<value>,<status>,<alert_flags>`     |
| `@tp,c,<offset>`     | `<offset>`: calibration offset in 0.1°C                   | Calibrate temperature sensor           | `&tp,c,<applied_offset>`                 |
| `@tp,t,<low>,<high>` | `<low>`: low threshold, `<high>`: high threshold in 0.1°C | Set temperature thresholds             | `&tp,t,<low_threshold>,<high_threshold>` |
| `@tp,rate,<rate>`    | `<rate>`: readings per second                             | Set temperature sampling rate          | `&tp,rate,<readings_per_second>`         |
| `@tp,diag`           | none                                                      | Request temperature sensor diagnostics | Multiple debug messages                  |

### Void Detection Commands

| Command                     | Parameters                         | Description                           | Response                                                      |
|-----------------------------|------------------------------------|---------------------------------------|---------------------------------------------------------------|
| `@vd` or `@vd,?`            | none                               | Query void detection from all sensors | Multiple `&vd,<sensor_id>,<distance>,<snr>,<status>` messages |
| `@vd,<sensor_id>`           | `<sensor_id>`: sensor number (1-3) | Query specific void detection sensor  | `&vd,<sensor_id>,<distance>,<snr>,<status>`                   |
| `@vd,cfg,threshold,<value>` | `<value>`: SNR threshold           | Configure void detection threshold    | `&vd,cfg,threshold,<value>`                                   |
| `@vd,diag`                  | none                               | Request void detection diagnostics    | Multiple debug messages                                       |
| `!void,<state>`             | `<state>`: 0=ended, 1=detected     | Void detection status (legacy)        | Debug message and upstream notification                       |

> **Note**: The enhanced void detection commands (`@vd`) are currently in development. Only the temperature commands (`@tp`) are fully implemented.

## Sensor Commands

### IMU and Motion Detection

| Command   | Parameters    | Description                     | Response                | Channel |
|-----------|---------------|---------------------------------|-------------------------|---------|
| `@imu`    | none          | Request IMU sensor data         | Current IMU readings    | 0,1     |
| `@cal`    | `<sensor>`    | Calibrate specified sensor      | Calibration status      | 0,1     |
| `@motion` | `<enable>`    | Enable/disable motion detection | Motion detection status | 0,1     |
| `@bump`   | `<threshold>` | Set bump detection threshold    | Threshold confirmation  | 0,1     |
| `@tilt`   | `<threshold>` | Set tilt detection threshold    | Threshold confirmation  | 0,1     |

### Water Detection

| Command                   | Parameters | Description                        | Response                    | Channel |
|---------------------------|------------|------------------------------------|-----------------------------|---------|
| `@water`                  | none       | Request water sensor status        | Water detection state       | 0,1     |
| `@sensor,w`               | none       | Read water sensor values           | ADC values for both sensors | 0,1     |
| `@sensor,w,<value>`       | `<value>`  | Set water detection threshold      | Threshold confirmation      | 0,1     |
| `@cmd,reserve,<value>`    | `<value>`  | Set water detection reserve margin | Reserve value confirmation  | 0,1     |
| `@cmd,multiplier,<value>` | `<value>`  | Set hysteresis multiplier          | Multiplier confirmation     | 0,1     |

### Temperature Monitoring

| Command                   | Parameters      | Description                         | Response                   | Channel |
|---------------------------|-----------------|-------------------------------------|----------------------------|---------|
| `@temp`                   | none            | Request temperature readings        | Current temperature values | 0,1     |
| `@temp,<id>`              | `<id>`          | Request specific sensor temperature | Temperature for sensor ID  | 0,1     |
| `@temp,threshold,<value>` | `<value>`       | Set temperature alert threshold     | Threshold confirmation     | 0,1     |
| `@temp,cal,<id>,<offset>` | `<id>,<offset>` | Calibrate temperature sensor        | Calibration confirmation   | 0,1     |

## Data and Logging Commands

### Data Acquisition

| Command       | Parameters        | Description                        | Response                   | Channel |
|---------------|-------------------|------------------------------------|----------------------------|---------|
| `@data`       | none              | Request current sensor data stream | Formatted sensor data      | 0,1     |
| `@log,start`  | none              | Begin data logging to flash        | Logging start confirmation | 0,1     |
| `@log,stop`   | none              | Stop data logging                  | Logging stop confirmation  | 0,1     |
| `@log,status` | none              | Request logging status             | Current logging state      | 0,1     |
| `@dump`       | `<start>,<count>` | Dump logged data from flash        | Data dump output           | 0,1     |

### Memory Management

| Command  | Parameters | Description                      | Response               | Channel |
|----------|------------|----------------------------------|------------------------|---------|
| `@mem`   | none       | Request memory usage information | Memory statistics      | 0,1     |
| `@flash` | none       | Request flash memory status      | Flash usage and health | 0,1     |
| `@clear` | `<target>` | Clear specified data buffers     | Clear confirmation     | 0,1     |
| `@save`  | none       | Save current configuration       | Save confirmation      | 0,1     |
| `@load`  | none       | Load saved configuration         | Load confirmation      | 0,1     |

## Configuration Commands

### System Configuration

| Command                       | Parameters        | Description                    | Response           | Channel |
|-------------------------------|-------------------|--------------------------------|--------------------|---------|
| `@config,get,<param>`         | `<param>`         | Get configuration parameter    | Parameter value    | 0,1     |
| `@config,set,<param>,<value>` | `<param>,<value>` | Set configuration parameter    | Set confirmation   | 0,1     |
| `@config,save`                | none              | Save configuration to flash    | Save confirmation  | 0,1     |
| `@config,load`                | none              | Load configuration from flash  | Load confirmation  | 0,1     |
| `@config,default`             | none              | Reset to default configuration | Reset confirmation | 0,1     |

### Debug and Diagnostic Commands

| Command          | Parameters | Description                    | Response                 | Channel |
|------------------|------------|--------------------------------|--------------------------|---------|
| `@debug,<level>` | `<level>`  | Set debug output level (0-3)   | Debug level confirmation | 1       |
| `@trace`         | `<enable>` | Enable/disable system trace    | Trace status             | 1       |
| `@perf`          | none       | Request performance statistics | Performance data         | 1       |
| `@diag`          | `<module>` | Run diagnostic on module       | Diagnostic results       | 1       |
| `@cmd,adc`       | none       | Toggle real-time ADC output    | ADC output status        | 1       |

## Radar and Void Detection Commands

### Radar Control

| Command                | Parameters | Description                 | Response                 | Channel |
|------------------------|------------|-----------------------------|--------------------------|---------|
| `@radar,status`        | none       | Request radar module status | Radar status information | 0,1     |
| `@radar,start`         | none       | Start radar measurements    | Start confirmation       | 0,1     |
| `@radar,stop`          | none       | Stop radar measurements     | Stop confirmation        | 0,1     |
| `@radar,power,<level>` | `<level>`  | Set radar power level (0-7) | Power level confirmation | 0,1     |
| `@radar,cal`           | none       | Calibrate radar sensors     | Calibration status       | 0,1     |

### Void Detection

| Command                   | Parameters | Description                          | Response                 | Channel |
|---------------------------|------------|--------------------------------------|--------------------------|---------|
| `@void,status`            | none       | Request void detection status        | Current void status      | 0,1     |
| `@void,threshold,<value>` | `<value>`  | Set void detection threshold         | Threshold confirmation   | 0,1     |
| `@void,config`            | none       | Request void detection configuration | Configuration parameters | 0,1     |
| `@circle,fit`             | none       | Trigger circle fitting algorithm     | Fit results              | 0,1     |

## Response Format Examples

### System Status Response

```text
@status,down,1,ver,0,9,2
```

Fields: status,down,state_code,ver,major,minor,sub

### Water Sensor Data

```bash
@sensor,w,1280,1275
```

Indicates:

- Water sensor 1 reading: 1280
- Water sensor 2 reading: 1275

### Water Detection Status

```bash
@wt,0,1280,1275,650
```

Indicates:

- Detection status: 0 (no water detected)
- Water sensor 1 reading: 1280
- Water sensor 2 reading: 1275
- Detection threshold: 650

### Flash FIFO Status

```bash
@ff,n,0,12,12,4084
```

Indicates:

- Front pointer: 0
- Back pointer: 12
- Saved entries: 12
- Free entries: 4084

## Error Handling

Commands may return error codes in responses:

- 0: Success/OK
- 1: Busy
- 2: Parameter error
- 3: Hardware error
- 4: Timeout
- 5: Not available

## Implementation Notes

- Commands processed in order of reception
- UART buffers use DMA for efficient data handling
- Maximum command length: 255 characters
- Response timing may vary based on system load
- Some commands are only available in specific system states

## Channel Routing

**Channel 0**: Primary uphole communication (UART1)

- All operational commands
- Sensor data streaming
- System control and configuration

**Channel 1**: Debug communication (UART6)

- Debug output and diagnostics
- Development and troubleshooting commands
- Real-time system monitoring

**Channel 2**: Radar communication (UART3)

- Radar module integration
- Void detection coordination
- Future expansion capability

## Command Processing Notes

1. **Case Sensitivity**: Commands are case-sensitive
2. **Parameter Validation**: Invalid parameters return error responses
3. **Channel Permissions**: Some commands restricted to specific channels
4. **State Dependencies**: Some commands only valid in certain system states
5. **Timeout Handling**: Commands have built-in timeout protection
6. **Queue Management**: Commands processed in FIFO order with priority handling
