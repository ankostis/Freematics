#!/usr/bin/env python
"""
Decode firmware hex-code states

SYNTAX:
    ./freestate.py <hex-number>

EXAMPLE:
    ./freestate.py 0x23c
    FREESTATE:  0x23c --> 0000_0010_0011_1100
    0(   0x4): GPS_READY
    1(   0x8): MEMS_READY
    2(  0x10): NET_READY
    3(  0x20): NET_CONNECTED
    4( 0x200): GET_OBFCM
"""

import sys

flags = {
    0x1: "STORAGE_READY",
    0x2: "OBD_READY",
    0x4: "GPS_READY",
    0x8: "MEMS_READY",
    0x10: "NET_READY",
    0x20: "NET_CONNECTED",
    0x40: "WORKING",
    0x100: "STANDBY",
    0x200: "GET_OBFCM",
    0x800: "SEND_ITID17",
    0x1000: "SEND_ITID1A",
    0x2000: "SEND_ITID1B",
    0x4000: "SEND_ITID1C",
}


def state_flag_n_names(state: int):
    for i in range(16):
        flag = 1 << i
        if state & flag:
            yield flag, flags.get(flag, "-")


try:
    from platformio.commands.device import DeviceMonitorFilter
    import re

    state_re = re.compile(r'(?:state: "?(?:0x)?)([0-9a-fA-F]+)\b')

    def expand_state(match):
        matched_state = match.group(1)
        state = int(matched_state, 16)
        names = "|".join(t[1] for t in state_flag_n_names(state))
        return f"state: ({state:#x}: {names})"

    class FreematicsStateMonFilter(DeviceMonitorFilter):
        NAME = "freestate"

        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            print(f"Monitor-filter {self.NAME} is loaded.")

        def rx(self, text):
            return state_re.sub(expand_state, text)

except ImportError:
    pass


def main(str_num: str):
    is_hex = str_num.strip().startswith("0x") or (set(str_num) & set("abcdef"))
    state = int(str_num, (16 if is_hex else 10))

    print(f"FREESTATE: {state}/{state:#x} --> {state:019_b}")
    for i, (flag, name) in enumerate(state_flag_n_names(state)):
        print(f"{i}({flag:#6x}): {name}")


if __name__ == "__main__":
    main(sys.argv[1])
