# JRCMATICS

JRC.C4 telemetry project based on **freematics**` devices (mostly model ONE+ H)
for OBD, OBFCM and GNSS data from light & heavy vehicles.

## Reduce size of this git repo

Code from the underlying ["freematics" git repo](https://github.com/stanleyhuangyc/Freematics.git) takes considerable time & space when cloning this repo (~500mb in disk), while only few parts of it are utilized;
running the commands below to *sparsely* clone & checkout results
in a x10 reduction in space/bandwidth (~45mb) and time to clone:
```bash
git clone --sparse <repo-url>
cd <repo-dir>
git sparse-checkout set --cone \
    server\
    libraries/{FreematicsPlus,httpd,TinyGPS,Utils}\
    firmware_v5/{telelogger,sandbox}
```

Limiting in addition history depth while cloning, eg with `--depth=30`
would result in half the size above.

**Tip:** Even if you fully cloned, you may still choose to *sparsely checkout* later,
to limit distraction & speedup the IDE scanning your working dir,
by running the last cmd above.

To checkout fully and disable *sparseness*, type:
```bash
git sparse-checkout disable
```

## git notes

This repo uses *git notes* to annotate past commits with any regressions discovered.
Unfortunately [git's manual](https://git-scm.com/docs/git-notes) does not explain
how to push & fetch them:

```bash
git push <remote> refs/notes/*
git fetch <remote> refs/notes/*:refs/notes/*
```


--( original README follow )--

---

Directories
===========

firmware_v4 - Arduino sketches and libraries for ATmega328p based [Freematics ONE](https://freematics.com/products/freematics-one)

firmware_v5 - Arduino sketches for ESP32 based [Freematics ONE+](https://freematics.com/products/freematics-one-plus)

ESPRIT - Arduino library and example sketches for ESP32 development board [Freematics Esprit](https://freematics.com/products/freematics-esprit) and [devkits based on it](https://freematics.com/products/#kits)

libraries - Arduino libraries for ESP32 based Freematics ONE+ and Esprit

server - [Freematics Hub](https://freematics.com/hub/) server source code
