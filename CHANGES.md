# CHANGES

## firmware v5

### TODO:

#### Urgent

- complete re-config
- dynamic PID-list
- trip/timestamp uint32-->64 server fixes
- [doc fixes](https://hypothes.is/users/ankostis?q=graphtik#)

#### Other

- [ ] dump stats less frequently (configurable interval)
- [ ] stats/infos should include WiFis around & stats
- [ ] +configuration combining **Tx-frequency of PIDs + OBFCM with Stats-frequency.**
- [ ] report `uxTaskGetNumberOfTasks()` & `uxTaskGetSystemState()` in NodeInfo status
- Port fixes:
  - [ ] 9343dc1a: Stanley GSM fixes @ `FreematcisNetwork.cpp`

### jrc-v0.3.0 (03 Aug 2022) upgrade some devices for summer with OTA

- FIX(obd) VIN truncate bug
- FIX(standby) STUCK STATE(3C) after not `WORKING` but before `STANDBY` flags
- feat(buz) `ENABLE_BUZTICKS` flag to disable buzzer click-patterns
- feat(net/cfg) reconfigurable with MultiWiFi SSID/pwd pairs
- fea(data/cfg) reconfigurable stationary intervals
  - enh: logs report the active stationary interval stage
- upd(data) obfcm Tx interval from 120-->30sec
- refact: replace some Arduino `String`--> `std::string` to reduce cheesy heap
- feat/refact(build) `config.h` applies also for libs
- feat(ninfo) boot-ark structure to account stuff across reboots.
- feat(ninfo/cmds) `INFO*` sub-cmds (but still cannot read output from server API)
- enh(logs) reconfigurable stats frequency to unclutter logs while working
- DOC(CHANGES) collect all past releases in `CHANGES.md` file

### jrc-v0.2.1 (27 Jul 2022) wip reconfig

- feat(build): Lower required Python from 9-->8
- WIP/FEAT(cfg/ninfo) RECONFIG & dynamic values
  - Dump node-info into JSON
  - not reading & reconfig yet
- enh(obd): obd-pipe on boot sugarcoating
- feat(build) c++ excepts
- fix(cfg) hide leaked secrets in logs
- doc: improve & add new command descriptions

### jrc-v0.2.0 (6 Jul 2022) firmware OTA-upgrades

- FEAT: OTA fw updates
- ENH(data) Larger buffers
- FEAT(logs) write logs to SD/SPIFFS-flash
- feat(ninfo/cfg) preliminary JSON-CONFIG
- feat(build):
  - g++17
  - enabled `-fexceptions`
  - Bigger OTA partitions, from 1.4M-->1.6M
  - engrave firmware program-name & version on image filename
- feat(build) *platformio* filters to expand device state & enabled-macroflags
- enh(logs) better stats
- doc: list AT-cmds of ELM327 OBD co-proc

### jrc-v0.0.3 (6 May 2022) LOGS ovunque

- FEAT(logs): enable & augment LOGS in libraries and main files
- fix(data): frop chunked-VIN
- feat(buz): buzzing-tick patterns to detect connectivity & heat problems
- chore(deps): bump arduino-core to 2.0.3
- enh(build): debugging filter to print offending source line

### jrc-v0.0.2 (7 Apr 2022) secrets & devinfo

- FEAT(CFG) `secrets.h` per user/node .
- FEAT(ninfo) DevInfo struct with git-hash & username builder
  (precursor to node-info).
- FEAT(build) log-configurations for Arduino/IDF subsystems
- CHORE(BUILD) bump & pin 2022Apr ESP-libs (to fix pio)
- FEAT(build/test): sandbox TCs
- feat(net) WiFi APlist preliminary work

### jrc-v0.0.1-rawData2022 (3 Apr 2022) replay `dev_rawData` history

- CHORE(GIT) Re-written and demangled `dev_rawData`
- FIX(build) trivial fixes to make it compile with recent *platformio*

### jrc-v0.0.0 (25 Mar 2022) git unite

- CHORE(GIT) Initial JRC commit re-written on top of upstream (Stanley's) history
