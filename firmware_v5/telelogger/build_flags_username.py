#!/usr/bin/env python3
#
# Produce a dynamic *build_flags* for `platformio.in`.

import getpass, socket

print(f"-DBUILD_USERNAME='\"{getpass.getuser()}\"'")
print(f"-DBUILD_HOSTNAME='\"{socket.gethostname()}\"'")
