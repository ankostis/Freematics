#!/usr/bin/env python
"""
Decode node-info config value for `compile-flags`

SYNTAX:
    ./macromacroflags.py <hex-number>

EXAMPLE:
    ./macromacroflags.py 207
    MACROFLAGS:  0x207 --> 0000_0010_0000_0111
    0(   0x1): ENABLE_OBD
    1(   0x2): ENABLE_MEMS
    2(   0x4): ENABLE_ORIENTATION
    3( 0x200): USE_ESP_IDF_LOG
"""
## TODO: extract common script for this and `freestate.py`.
import sys

flags = {
    0x001: "ENABLE_OBD",
    0x002: "ENABLE_MEMS",
    0x004: "ENABLE_ORIENTATION",
    0x008: "ENABLE_OLED",
    0x010: "ENABLE_BUZZING_INIT",
    0x020: "ENABLE_OTA_UPDATE",
    0x040: "_NEED_SD",
    0x080: "_NEED_SPIFFS",
    0x100: "ENABLE_MULTILOG",
    0x200: "USE_ESP_IDF_LOG",
    0x400: "HIDE_SECRETS_IN_LOGS",
}


def state_flag_n_names(state: int):
    for i in range(16):
        flag = 1 << i
        if state & flag:
            yield flag, flags.get(flag, "-")


try:
    from platformio.commands.device import DeviceMonitorFilter
    import re

    state_re = re.compile('(?i)(?<="macroflags": )(?:(?:"0x([0-9a-f]+)")|([0-9]+))')

    def expand_state(match):
        hex_state = match.group(1)
        dec_state = match.group(2)
        matched_state = hex_state or dec_state
        state = int(hex_state, 16) if hex_state else int(dec_state)
        names = "|".join(t[1] for t in state_flag_n_names(state))
        return f"({matched_state} --> {state:#x}: {names})"

    class MacroflagsMonFilter(DeviceMonitorFilter):
        NAME = "macroflags"

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            print("f{NAME} filter is loaded.")

        def rx(self, text):
            return state_re.sub(expand_state, text)

except ImportError:
    pass


def main(str_num: str):
    is_hex = str_num.strip().startswith("0x") or (set(str_num) & set("abcdef"))
    state = int(str_num, (16 if is_hex else 10))

    print(f"MACROFLAGS: {state}/{state:#x} --> {state:019_b}")
    for i, (flag, name) in enumerate(state_flag_n_names(state)):
        print(f"{i}({flag:#6x}): {name}")


if __name__ == "__main__":
    main(sys.argv[1])
