#!/usr/bin/env python3
#
# SYNTAX: <cmd> [extra dir]...
# Where: <extra dir> will be appended as `-fmacro-prefix-map=<extra dir>=` build_flag.
#
# Produce gcc's `-fmacro-prefix-map` flags so that `__FILE__` keeps basename only
# as explained in this [SO solution](https://stackoverflow.com/a/53848526/548792).

import os, pathlib, sys


def produce_build_flags():
    mydir = pathlib.Path(__file__).parent.resolve()

    project_dirs = [
        mydir,
        mydir.parent / "sandbox",
        *[
            (mydir.parent.parent / "libraries" / p)
            for p in "FreematicsOLED  FreematicsPlus httpd  TinyGPS".split()
        ],
    ]

    platformio_dir = pathlib.Path("~/.platformio").expanduser().resolve()
    arduino_dir = platformio_dir / "packages" / "framework-arduinoespressif32"
    espressif_dirs = [
        platformio_dir,
        platformio_dir / "packages",
        arduino_dir,
        arduino_dir / "cores" / "esp32",
    ]
    argv_dirs = [pathlib.Path(a).resolve() for a in sys.argv[1:]]

    for p in [*project_dirs, *espressif_dirs, *argv_dirs]:
        print(f"-fmacro-prefix-map='{p}/='")


if __name__ == "__main__":
    produce_build_flags()
