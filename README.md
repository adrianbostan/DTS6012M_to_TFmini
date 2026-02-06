I built a small ESP32-C3 Super Mini bridge that converts the DTS6012M LiDAR output into a TFmini-compatible data stream so it can be used directly with ArduPilot.

The issue is that DTS6012M doesn’t have proper software support/integration for ArduPilot, even though the hardware is solid. Instead of replacing the sensor, the ESP32 reads the DTS6012M data over UART, validates/filters it, then retransmits it in the TFmini protocol/format. As a result, ArduPilot detects it as a supported TFmini rangefinder—no custom ArduPilot drivers or complex patches needed.

Hardware: DTS6012M + ESP32-C3 Super Mini
Interface: UART in → convert → UART out (TFmini)
Goal: plug-and-play ArduPilot rangefinder support
