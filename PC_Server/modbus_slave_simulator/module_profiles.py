"""
Module profile definitions — exact mirror of the C firmware's
uart_module_profiles[] table in bsp_rs485.c.

Each profile contains the register counts, protocol type, and (for custom
curve modules) the byte offsets that describe the response frame layout.

All values are verified against:
    User/usart/bsp_rs485.h  — type codes and struct definition
    User/usart/bsp_rs485.c  — uart_module_profiles[] table (lines 163-197)
"""

from dataclasses import dataclass, field
from typing import ClassVar

# ---------------------------------------------------------------------------
# Module type code constants (matching bsp_rs485.h defines)
# ---------------------------------------------------------------------------
# --- 普通 Modbus 模块 ---
FLFXG_XHJ_KGL   = 0x31   # 信号机开关量
FLFXG_XHJ       = 0x32   # 信号机
FLFXG_GD_25_J   = 0x33   # 25Hz 轨道接收
FLFXG_DC_AC     = 0x34   # DC/AC 转换器
FLFXG_GD_25_F   = 0x35   # 25Hz 轨道发送
FLFXG_CL        = 0x36   # 采集模块

# --- 自定义曲线模块 ---
FLFXG_AC_ZZJ    = 0x37   # 交流转辙机 (4 分支曲线)
FLFXG_DC_ZZJ4   = 0x38   # 直流四线制道岔 (1 分支曲线)
FLFXG_DC_ZZJ6   = 0x39   # 直流六线制道岔 (2 分支曲线)

# --- 高频 Modbus 模块 ---
FLFXG_NEW_XHJ       = 0x47   # 高频信号机
FLFXG_GD_25_J_HIGH  = 0x48   # 高频 25Hz 轨道接收 (133 输入寄存器!)
FLFXG_GD_25_F_HIGH  = 0x49   # 高频 25Hz 轨道发送
FLFXG_CL_HIGH       = 0x4A   # 高频采集

# --- 高频自定义曲线模块 ---
FLFXG_AC_ZZJ_HIGH   = 0x4B   # 高频交流转辙机 (4 分支)
FLFXG_DC_ZZJ4_HIGH  = 0x4C   # 高频直流四线制 (1 分支)
FLFXG_DC_ZZJ6_HIGH  = 0x4D   # 高频直流六线制 (2 分支)

# --- 防雷分线柜模块 ---
FLFXG_AXLE_COUNTER        = 0x50   # 计轴采集传感器 WBTQ301JH-1B3
FLFXG_DC3WIRE_PLATFORM    = 0x51   # 站台门采集 WBTQ802JH-1B3
FLFXG_DC3WIRE_STATION     = 0x52   # 站场联采集 WBTQ402JH-1B3

# ---------------------------------------------------------------------------
# Profile data class
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ModuleProfile:
    """
    Static profile for one module type.

    Mirrors UART_ModuleProfile_t from bsp_rs485.h.  All offsets for custom
    curve modules are byte positions in the response data (relative to the
    first data byte after addr + 0x03 + byte_count, i.e. offset 3 in the
    full response frame).
    """
    module_type: int             # 0x31 .. 0x4D
    input_regs: int              # Modbus input registers to read
    holding_regs: int            # Holding registers to copy (data-only)
    has_alarm: int               # 1 = check alarm bit
    uses_custom_curve: int       # 1 = custom serial protocol + curve

    # --- Custom curve fields (0 when uses_custom_curve == 0) ---
    realtime_bytes: int = 0          # Bytes of real-time data per module
    curve_data_offset: int = 0       # Offset where curve data starts
    curve_len_offset: int = 0        # Per-branch curve length byte
    curve_flag_offset: int = 0       # Frame flag (0 / 0x55 / 0xFF)
    curve_seq_offset: int = 0        # Frame sequence byte
    curve_direction_offset: int = 0  # Rotation direction word (2 bytes)
    curve_branch_count: int = 0      # Number of curve branches (1 / 2 / 4)

    # --- Derived helpers ---
    @property
    def is_standard_modbus(self) -> bool:
        """True if this module uses standard Modbus 0x04 protocol."""
        return self.uses_custom_curve == 0

    @property
    def is_custom_curve(self) -> bool:
        """True if this module uses custom serial protocol with curve data."""
        return self.uses_custom_curve == 1

    @property
    def is_high_frequency(self) -> bool:
        """True if this is a high-frequency module (0x47-0x4D)."""
        return 0x47 <= self.module_type <= 0x4D

    @property
    def exceeds_125_regs(self) -> bool:
        """True if input_regs > standard Modbus 125-register limit."""
        return self.input_regs > 125

    @property
    def per_branch_stride(self) -> int:
        """Bytes per branch in a single curve response frame. Always 50."""
        return 50 if self.uses_custom_curve else 0

    @property
    def total_curve_data_bytes(self) -> int:
        """Total curve data bytes per frame (all branches)."""
        return self.curve_branch_count * 50

    @property
    def response_frame_size(self) -> int:
        """
        Total response frame size (excluding CRC) for custom curve modules.
        Returns 0 for standard Modbus modules (frame size is dynamic).
        """
        if not self.uses_custom_curve:
            return 0
        # Frame ends at curve_direction_offset + 2 (the direction word)
        return self.curve_direction_offset + 2

    def __repr__(self) -> str:
        typ = "CURVE" if self.uses_custom_curve else "MODBUS"
        freq = "HF" if self.is_high_frequency else "NF"
        return (f"ModuleProfile(0x{self.module_type:02X} {freq} {typ} "
                f"in={self.input_regs})")


# ---------------------------------------------------------------------------
# The complete profile table — one entry per module type.
# Order and values MUST match uart_module_profiles[] in bsp_rs485.c.
# ---------------------------------------------------------------------------

_PROFILES: list[ModuleProfile] = [
    # ====== 普通 Modbus 模块 (uses_custom_curve=0) ======
    #  type             in   hold alarm cust  rt   doff loff foff soff dircnt
    ModuleProfile(FLFXG_XHJ_KGL,  13, 11, 1, 0),
    ModuleProfile(FLFXG_XHJ,      13, 11, 1, 0),
    ModuleProfile(FLFXG_GD_25_J,  34, 32, 1, 0),
    ModuleProfile(FLFXG_DC_AC,    10,  8, 1, 0),
    ModuleProfile(FLFXG_GD_25_F,  30, 28, 1, 0),
    ModuleProfile(FLFXG_CL,        8,  6, 1, 0),

    # ====== 自定义曲线模块 (uses_custom_curve=1) ======
    #  type            in  hold alarm cust  rt   doff loff foff soff dircnt
    ModuleProfile(FLFXG_AC_ZZJ,    29, 27, 1, 1,
                 58,  82,  81,  73,  75,  282, 4),
    ModuleProfile(FLFXG_DC_ZZJ4,    8,  5, 1, 1,
                 16,  40,  39,  31,  33,   90, 1),
    ModuleProfile(FLFXG_DC_ZZJ6,   11,  5, 1, 1,
                 22,  46,  45,  37,  39,  146, 2),

    # ====== 高频 Modbus 模块 (uses_custom_curve=0) ======
    #  type                 in   hold alarm cust  rt   doff loff foff soff dircnt
    ModuleProfile(FLFXG_NEW_XHJ,       50, 48, 1, 0),
    ModuleProfile(FLFXG_GD_25_J_HIGH, 133,131, 1, 0),  # >125 registers!
    ModuleProfile(FLFXG_GD_25_F_HIGH, 117,115, 1, 0),
    ModuleProfile(FLFXG_CL_HIGH,       62, 60, 1, 0),

    # ====== 高频自定义曲线模块 (uses_custom_curve=1) ======
    #  type                 in  hold alarm cust  rt   doff loff foff soff dircnt
    ModuleProfile(FLFXG_AC_ZZJ_HIGH,   68, 66, 1, 1,
                 136, 160, 159, 151, 153, 360, 4),
    ModuleProfile(FLFXG_DC_ZZJ4_HIGH,  23, 21, 1, 1,
                  46,  70,  69,  61,  63, 120, 1),
    ModuleProfile(FLFXG_DC_ZZJ6_HIGH,  26, 24, 1, 1,
                  52,  76,  75,  67,  69, 176, 2),

    # ====== 防雷分线柜模块 ======
    #  type              in  hold alarm cust  rt
    ModuleProfile(FLFXG_AXLE_COUNTER,       14,  0, 1, 0, 28),
    ModuleProfile(FLFXG_DC3WIRE_PLATFORM,    12,  0, 1, 0, 24),
    ModuleProfile(FLFXG_DC3WIRE_STATION,     12,  0, 1, 0, 24),
]

# Sentinel / fallback entry (type = 0x00, matches first entry config)
# Used by uart_get_module_profile() when no match is found.
FALLBACK_PROFILE: ModuleProfile = ModuleProfile(
    0x00, 13, 11, 1, 0
)

# Build a fast lookup dict keyed by module_type
PROFILE_BY_TYPE: dict[int, ModuleProfile] = {
    p.module_type: p for p in _PROFILES
}


def get_profile(module_type: int) -> ModuleProfile:
    """
    Look up the module profile by type code.

    Args:
        module_type: Module type code (0x31 .. 0x4D).

    Returns:
        ModuleProfile for the given type, or FALLBACK_PROFILE if not found.
    """
    return PROFILE_BY_TYPE.get(module_type, FALLBACK_PROFILE)


def get_all_profiles() -> list[ModuleProfile]:
    """Return a copy of all defined profiles."""
    return list(_PROFILES)


def get_all_type_codes() -> list[int]:
    """Return all valid module type codes."""
    return [p.module_type for p in _PROFILES]


def get_standard_modules() -> list[ModuleProfile]:
    """Return profiles for standard Modbus modules (non-custom-curve)."""
    return [p for p in _PROFILES if p.is_standard_modbus]


def get_custom_curve_modules() -> list[ModuleProfile]:
    """Return profiles for custom curve modules."""
    return [p for p in _PROFILES if p.is_custom_curve]


# Module type name lookup (human-readable labels)
MODULE_TYPE_NAMES: dict[int, str] = {
    0x31: "信号机开关量 (XHJ_KGL)",
    0x32: "信号机 (XHJ)",
    0x33: "25Hz轨道接收 (GD_25_J)",
    0x34: "DC/AC转换器 (DC_AC)",
    0x35: "25Hz轨道发送 (GD_25_F)",
    0x36: "采集模块 (CL)",
    0x37: "交流转辙机 (AC_ZZJ)",
    0x38: "直流四线制 (DC_ZZJ4)",
    0x39: "直流六线制 (DC_ZZJ6)",
    0x47: "高频信号机 (NEW_XHJ)",
    0x48: "高频25Hz轨道接收 (GD_25_J_HIGH)",
    0x49: "高频25Hz轨道发送 (GD_25_F_HIGH)",
    0x4A: "高频采集 (CL_HIGH)",
    0x4B: "高频交流转辙机 (AC_ZZJ_HIGH)",
    0x4C: "高频直流四线制 (DC_ZZJ4_HIGH)",
    0x4D: "高频直流六线制 (DC_ZZJ6_HIGH)",
    0x50: "计轴采集传感器 (AXLE_COUNTER)",
    0x51: "站台门采集 (DC3W_PLATFORM)",
    0x52: "站场联采集 (DC3W_STATION)",
}


def get_module_name(module_type: int) -> str:
    """Return a human-readable name for a module type."""
    return MODULE_TYPE_NAMES.get(
        module_type, f"未知模块 (0x{module_type:02X})"
    )
