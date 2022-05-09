#!/usr/bin/env python
#
# SYNTAX: ./freestate.py <hex-number>
# Decode firmware hex-code states

import sys

flags = {
   0x1: "STATE_STORAGE_READY",
   0x2: "STATE_OBD_READY",
   0x4: "STATE_GPS_READY",
   0x8: "STATE_MEMS_READY",
   0x10: "STATE_NET_READY",
   0x20: "STATE_NET_CONNECTED",
   0x40: "STATE_WORKING",
   0x100: "STATE_STANDBY",
   0x200: "STATE_GET_OBFCM",
   0x800: "STATE_SEND_ITID17",
   0x1000: "STATE_SEND_ITID1A",
   0x2000: "STATE_SEND_ITID1B",
   0x4000: "STATE_SEND_ITID1C",
}
def status_names(status: int):
    for i in range(16):
        flag = (1 << i)
        if status & flag:
            yield f"{i}({flag:#6x}): {flags.get(flag, '-')}"


def main(hex_status: str):
    status = int(hex_status, 16)
    print(f"STATUS: {status:#6x} --> {status:019_b}")
    print("\n".join(status_names(status)))

if __name__ == "__main__":
    main(sys.argv[1])
