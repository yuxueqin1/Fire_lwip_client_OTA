"""
Modbus Slave Simulator — PC-side simulator for STM32 RS485 slave modules.

Simulates all module types:
  - Standard Modbus (0x31-0x36, 0x47-0x4A) via function code 0x04
  - Custom curve protocol (0x37-0x39, 0x4B-0x4D) via custom serial protocol
  - Axle counter protocol (0x50) via axle counter custom protocol (0x03/0x10)
  - DC 3-wire protocol (0x51/0x52) via DC 3-wire custom protocol (0x03/0x10)

Usage:
    python -m modbus_slave_simulator -c config.json
"""
from .axle_counter_handler import AxleCounterHandler, AxlePulseMode
from .dc3wire_handler import DC3WireHandler, DC3WireCurveMode
