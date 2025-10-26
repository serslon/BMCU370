# BMCU370 Copilot Instructions

## Project Overview
BMCU370 is an embedded firmware for CH32V203 microcontroller managing Bambu Lab AMS (Automatic Material System). It handles filament feeding, sensor monitoring, and communication with the printer via custom BambuBus protocol.

## Architecture
- **Main Loop**: `main.cpp` orchestrates initialization and runs `BambuBus_run()` + `Motion_control_run()` in a tight loop
- **Communication**: `BambuBus.cpp` parses UART packets with CRC validation, handles AMS registration and filament commands
- **Motion Control**: `Motion_control.cpp` implements PID motor control for filament feeding/retraction using AS5600 encoders
- **Data Persistence**: `Flash_saves.cpp` stores filament data with checksum validation at `0x0800F000`
- **Sensor Integration**: `ADC_DMA.cpp` provides sliding window filtered ADC readings for pressure/microswitch sensors

## Key Patterns
- **Packet Handling**: Use `package_check_crc16()` and `package_send_with_crc()` for all BambuBus communications
- **State Management**: Filament states stored in `data_save.filament[]` array, persisted via `Bambubus_save()`
- **Motor Control**: PID implemented in `MOTOR_PID` class with anti-windup and range limiting
- **RGB Indication**: Status LEDs updated via `Set_MC_RGB()` with brightness scaling in `RGB_Set_Brightness()`
- **Debugging**: Use `DEBUG_MY()` macros for UART logging, enabled via `#define Debug_log_on`

## Build & Development
- **PlatformIO**: Build with `pio run`, upload via `pio run -t upload`
- **Debug Logging**: Connect to USART3 (PB10 TX, PB11 RX) at 115200 baud for debug output
- **Flash Addresses**: AMS data at `0x0800F000`, motion config at `0x0800E000`
- **CRC Libraries**: Relies on `CRC16` and `CRC8` classes for protocol integrity

## Common Workflows
- **Add New Command**: Extend `BambuBus_package_type` enum and handle in `BambuBus_run()` switch
- **Modify Motor Behavior**: Update PID parameters in `MOTOR_PID::init_PID()` or adjust voltage thresholds in `Motion_control.cpp`
- **Sensor Calibration**: Tune ADC thresholds in `MC_PULL_ONLINE_read()` for pressure/online detection
- **Save Changes**: Call `Bambubus_set_need_to_save()` after modifying `data_save` to trigger flash write

## Integration Points
- **Printer Communication**: UART1 at 1.25Mbps with 9-bit protocol and even parity
- **Sensor Pins**: ADC channels 0-7 for pressure/online, GPIO for AS5600 I2C
- **LED Control**: NeoPixel strips on PA11/PA8/PB1/PB0/PD1 with brightness 15/35 respectively