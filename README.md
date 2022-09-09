# JRCMATICS

JRC.C4 telemetry project based on **freematics**` devices (mostly model ONE+ H)
for OBD, OBFCM and GNSS data from light & heavy vehicles.

## Reduce size of this git repo

> **NOTE:** the feature described here requires a version of cmd-line `git >= 2.25`.

Code from the underlying ["freematics" git repo](https://github.com/stanleyhuangyc/Freematics.git) takes considerable time & space when cloning this repo (~500mb in disk)
when `firmware_v5/telelogger` is mostly used;  running the commands below
*sparsely* clone & checkout this dir, resulting in a x10 reduction in space/bandwidth(~45mb)
and time to clone:

```bash
git clone --sparse <repo-url>
cd <repo-dir>
git sparse-checkout set --cone \
    server\
    libraries/{FreematicsPlus,httpd,TinyGPS,Utils,NlohmannJSON}\
    firmware_v5/{telelogger,sandbox}
```

Limiting additionally history-depth while cloning with `--depth=30`
reduces the clone bandwidth even further in half (~20mb).

> **Tip-1:** Even if you fully cloned initially, you may *checkout sparsely* later,
> to limit distraction & speedup the IDE scanning your working dir,
> by running the last cmd above from the root of the repo,
> to leave just the directories in the "cone" in your working-dir.

> **Tip-2:** To checkout fully and disable *sparseness*, type:
>
> ```bash
> git sparse-checkout disable
> ```

## git notes

This repo uses *git notes* to annotate past commits with any regressions discovered.
Unfortunately [git's manual](https://git-scm.com/docs/git-notes) does not explain
how to push & fetch them:

```bash
git push <remote> refs/notes/*
git fetch <remote> refs/notes/*:refs/notes/*
```

## Skip commit noise from git-blame

Commits in the `firmware_v5/telelogger/.` file are ignored when `git blame` is run
because they have been deemed as containing style-changes, moving code around verbatim,
and other unimportant stuff when reviewing code;
you may permanently ignore them with this command:

```bash
git config blame.ignoreRevsFile firmware_v5/telelogger/.ignore-revs-file.txt
```

--( original README follow )--

---

Directories
===========

- **firmware_v4/** - Arduino sketches and libraries for ATmega328p based [Freematics ONE](https://freematics.com/products/freematics-one)

- **firmware_v5/** - Arduino sketches for ESP32 based [Freematics ONE+](https://freematics.com/products/freematics-one-plus)

- **ESPRIT** - Arduino library and example sketches for ESP32 development board
  [Freematics Esprit](https://freematics.com/products/freematics-esprit) and
  [devkits based on it](https://freematics.com/products/#kits)

- **libraries/** - Arduino libraries for ESP32 based Freematics ONE+ and Esprit

- **server/** - [Freematics Hub](https://freematics.com/hub/) server source code
