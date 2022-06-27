#!/usr/bin/env python3
#
# Produce a *build_flag* defining a GIT_DESCRIBE macro
# containing something like `v1.1.0-dev0-48-g18650f3`.

import subprocess

try:
    revision = (
        subprocess.check_output("git describe --always --long".split())
        .strip()
        .decode("utf-8")
    )
except Exception as ex:
    revision = f"git-describe: {ex}"

print(f"-DGIT_DESCRIBE='\"{revision}\"'")
