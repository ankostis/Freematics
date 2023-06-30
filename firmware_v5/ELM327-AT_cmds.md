# ELM327 AT commands summary

It is assumed that STM32F103 co-processor hooked on the CAN-bus on *freematics*
is running *some* version of the [ubiquitous ELM327 firmware](https://en.wikipedia.org/wiki/ELM327).
The list of AT-commands below are provided as an aid to the developers,
to facilitate quick text-searches while understanding the code.

The commands were manually extracted from the 99-page PDF document on [elm327 IC, v2.3](https://www.elmelectronics.com/ic/elm327l/)
([waybacked](https://web.archive.org/web/20220324203957/https://www.elmelectronics.com/ic/elm327l/))
on May 2022 (a month before the closing of ELM company).

> **Note:**
> - Settings which are shown with an asterisk (*) are the default values.
> - Commands in **bold** have been tested in BOOT_PIPE and they worked ok.
> - Commands starting with `/`  did not work when tested.

## General Commands

- `<CR>`: repeat the last command (`\r`, or ASCII(0x0D))
- /ATBRD hh: try Baud Rate Divisor hh
- /ATBRT hh: set Baud Rate Timeout
- /ATD: set all to Defaults
- **ATE0**: Echo off
- **ATE1**: Echo on*
- /ATFE: Forget Events
- **ATI**: print the version ID
- **ATL0**: Linefeeds off
- **ATL1**: Linefeeds on
- **ATLP**: go to Low Power mode
- **ATM0**: ~~Memory off~~**Enable sniffing**
- **ATM1**: ~~Memory on~~**Disable sniffing**
- /ATRD: Read the stored Data
- /ATSD hh: Save Data byte hh
- /ATWS: Warm Start (quick software reset)
- **ATZ**: reset all
- **AT@1**: display the device description
- /AT@2: display the device identifier
- /AT@3 cccccccccccc store the @2 identifier


## Programmable Parameter Commands

- /ATPP xx OFF: disable Prog Parameter xx
- /ATPP FF OFF: all Prog Parameters disabled
- /ATPP xx ON: enable Prog Parameter xx
- /ATPP FF ON: all Prog Parameters enabled
- /ATPP xx SV yy: for PP xx, Set the Value to yy
- /ATPPS: print a PP Summary


## Voltage Reading Commands
- ATCV dddd: Calibrate the Voltage to dd.dd volts
- ATCV 0000: restore CV value to factory setting
- **ATRV**: Read the input Voltage

## Other

- /ATIGN: read the IgnMon input level


## OBD Commands

- /ATAL: Allow Long (>7 byte) messages
- /ATAMC: display Activity Monitor Count
- /ATAMT hh: set the Activity Mon Timeout to hh
- /ATAR: Automatically Receive
- /ATAT0, 1, 2: Adaptive Timing off, auto1*, auto2
- /ATBD: perform a Buffer Dump
- /ATBI: Bypass the Initialization sequence
- **ATDP**: Describe the current Protocol
- **ATDPN**: Describe the Protocol by Number
- /ATFT: Filter for Transmitter off*
- /ATFT hh: Filter for Transmitter = hh
- *ATH0*: (RECV) Headers off*
- *ATH1*: (RECV) Headers on
- /ATIA: Is the protocol Active?
- /ATMA: Monitor All
- /ATMR hh: Monitor for Receiver = hh
- /ATMT hh: Monitor for Transmitter = hh
- /ATNL: Normal Length messages*
- *ATPC*: Protocol Close
- /ATR0: Responses off
- /ATR1: Responses on*
- /ATRA hh: set the Receive Address to hh
- *ATS0*: printing of Spaces off
- *ATS1*: printing of Spaces on*
- ATSH xyz: Set Header to xyz
- ATSH xxyyzz: Set Header to xxyyzz
- ATSH wwxxyyzz: Set Header to wwxxyyzz
- *ATSP h*: Set Protocol to h and save it
- /ATSP Ah: Set Protocol to Auto, h and save it
- /ATSP 00: Erase stored protocol
- /ATSR hh: Set the Receive address to hh
- /ATSS: use Standard Search order (J1978)
- /ATST hh: Set Timeout to hh x 4 msec
- /ATTA hh: set Tester Address to hh
- /ATTP h: Try Protocol h
- /ATTP Ah: Try Protocol h with Auto search


## J1850 Specific Commands (protocols 1 and 2)

- ATIFR0, 1, 2: IFRs off, auto*, or on, if not monitoring
- ATIFR4, 5, 6: IFRs off, auto, or on, at all times
- ATIFR H, S: IFR value from Header* or Source


## ISO Specific Commands (protocols 3 to 5)

- /ATFI: perform a Fast Initiation
- ATIB10: set the ISO Baud rate to 10400*
- ATIB12: set the ISO Baud rate to 12500
- ATIB15: set the ISO Baud rate to 15625
- ATIB48: set the ISO Baud rate to 4800
- ATIB96: set the ISO Baud rate to 9600
- *ATIIA hh*: set ISO (slow) Init Address to hh
- /ATKW: display the Key Words
- /ATKW0: Key Word checking off
- /ATKW1: Key Word checking on*
- /ATSI: perform a Slow (5 baud) Initiation
- /ATSW hh: Set Wakeup interval to hh x 20 msec
- /ATSW 00: Stop sending Wakeup messages
- /ATWM [1 - 6 bytes]: set the Wakeup Message


## CAN Specific Commands (protocols 6 to C)

- /ATC0: send Confirmation off
- /ATC1: send Confirmation on*
- /ATCAF0: Automatic Formatting off
- /ATCAF1: Automatic Formatting on*
- /ATCEA: turn off CAN Extended Addressing
- /ATCEA hh: use CAN Extended Address hh
- ATCER hh: set CAN Extended Rx address to hh
- *ATCF hhh*: set the ID Filter to hhh
- *ATCF hhhhhhhh*: set the ID Filter to hhhhhhhh
- *ATCFC0*: Flow Controls off
- *ATCFC1*: Flow Controls on*
- *ATCM hhh*: set the ID Mask to hhh
- *ATCM hhhhhhhh*: set the ID Mask to hhhhhhhh
- **ATCP hh**: set CAN Priority to hh (29 bit)
- /ATCRA: reset the Receive Address filters
- /ATCRA hhh: set CAN Receive Address to hhh
- /ATCRA hhhhhhhh: set the Rx Address to hhhhhhhh
- /ATCS: show the CAN Status counts
- /ATCSM0: Silent Monitoring off
- /ATCSM1: Silent Monitoring on*
- /ATCTM1: set Timer Multiplier to 1*
- /ATCTM5: set Timer Multiplier to 5
- /ATD0: display of the DLC off*
- /ATD1: display of the DLC on
- /ATFC SM h: Flow Control, Set the Mode to h
- /ATFC SH hhh: FC, Set the Header to hhh
- /ATFC SH hhhhhhhh: Set the Header to hhhhhhhh
- /ATFC SD [1 - 5 bytes]: FC, Set Data to [...]
- /ATPB xx yy: Protocol B options and baud rate
- /ATRTR: send an RTR message
- /ATV0: use of Variable DLC off*
- /ATV1: use of Variable DLC on


## J1939 CAN Specific Commands (protocols A to C)

- ATDM1: monitor for DM1 messages
- ATJE: use J1939 Elm data format*
- ATJHF0, JHF1: Header Formatting off, or on*
- ATJS: use J1939 SAE data format
- ATJTM1: set Timer Multiplier to 1*
- ATJTM5: set Timer Multiplier to 5
- ATMP hhhh: Monitor for PGN 0hhhh
- ATMP hhhh n: ^ ^ ^ and get n messages
- ATMP hhhhhh: Monitor for PGN hhhhhh
- ATMP hhhhhh n: ^ ^ ^ and get n messages


## Protocols

- 0: Automatic
- 1: SAE J1850 PWM (41.6 kbaud)
- 2: SAE J1850 VPW (10.4 kbaud)
- 3: ISO 9141-2 (5 baud init)
- 4: ISO 14230-4 KWP (5 baud init)
- 5: ISO 14230-4 KWP (fast init)
- 6: ISO 15765-4 CAN (11 bit ID, 500 kbaud)
- 7: ISO 15765-4 CAN (29 bit ID, 500 kbaud)
- 8: ISO 15765-4 CAN (11 bit ID, 250 kbaud)
- 9: ISO 15765-4 CAN (29 bit ID, 250 kbaud)
- A: SAE J1939 CAN (29 bit ID, 250* kbaud)
- B: User1 CAN (11* bit ID, 125* kbaud)
- C: User2 CAN (11* bit ID, 50* kbaud)


## Commands used in the code

### FreematicsOBD.cpp

- ATLP: go to Low Power mode (in `enterLowPowerMode()`)
- ATI: print the version ID (in `leaveLowPowerMode()`)
- ATRV: Read the input Voltage (in `COBD::getVoltage()`)
- ATE0: Echo off (in `COBD::init()`)
- ATH0: Headers off (in `COBD::init()`)
- ATZ: reset all (in `COBD::init()`)
- ATSP h: Set Protocol to h and save it (in `COBD::init()`)
- ATPC: Protocol Close (in `COBD::uninit()`)
- ATSH: set (TX) Header (in `COBD::setHeaderID()`)
- ATCP: set CAN Priority to hh (29 bit) (in `COBD::setHeaderID()`)
- ATM0/1: ~~Memory off/on~~**Enable sniffing** (in `COBD::sniff()`)
- ATCF: set the ID Filter (in `COBD::setHeaderFilter()`)
- ATCM: set the ID Mask (in `COBD::setHeaderMask()`)

#### extras

- ATR: REBOOT STM, in `COBD::reset()`

### FreematicsPlus.cpp

- ATI: in `getDeviceType()`, `reactivateLink()`

#### extras

- ATR: in `resetLink()`
- ATBR1: in `CLink_UART::changeBaudRate()`
- ATGPSOFF: in `FreematicsESP32::gpsEnd()/gpsBegin()`
- ATGPSON: in `FreematicsESP32::gpsBegin()`
- ATGPS: in `FreematicsESP32::_gpsGetData_linkUart`
- ATGRR: in `FreematicsESP32::gpsGetNMEA()`

See also [freematics announcement](https://blog.freematics.com/2014/freematics-obd-ii-adapter-v2-sample-completed/)


## Diffs of cmds in published PDF from [GPL sources](https://github.com/ankostis/elm327-allpro/)

```diff
$ colordiff --side-by-side  atcmds-pdf.txt  atcmds-sources.txt
AT@1                                                            AT@1
AT@2                                                          | AT#1
AT@3                                                          | AT#3
                                                              > AT#RSN
ATAL                                                            ATAL
ATAMC                                                         <
ATAMT                                                         <
ATAR                                                            ATAR
ATAT0                                                           ATAT0
                                                              > ATAT1
                                                              > ATAT2
ATBD                                                            ATBD
ATBI                                                            ATBI
ATBRD                                                           ATBRD
ATBRT                                                           ATBRT
ATC0                                                          <
ATC1                                                          <
ATCAF0                                                          ATCAF0
ATCAF1                                                          ATCAF1
ATCEA                                                           ATCEA
ATCEA                                                           ATCEA
ATCER                                                           ATCER
ATCF                                                            ATCF
ATCF                                                            ATCF
ATCFC0                                                          ATCFC0
ATCFC1                                                          ATCFC1
ATCM                                                            ATCM
ATCM                                                            ATCM
ATCP                                                            ATCP
ATCRA                                                           ATCRA
ATCRA                                                           ATCRA
ATCRA                                                           ATCRA
ATCS                                                            ATCS
ATCSM0                                                          ATCSM0
ATCSM1                                                          ATCSM1
ATCTM1                                                        | ATCTM
ATCTM5                                                        <
ATCV                                                          <
ATCV                                                            ATCV
ATD                                                             ATD
ATD0                                                            ATD0
ATD1                                                            ATD1
ATDM1                                                           ATDM1
ATDP                                                            ATDP
ATDPN                                                           ATDPN
ATE0                                                            ATE0
ATE1                                                            ATE1
ATFC                                                          | ATFCSD
ATFC                                                          | ATFCSH
ATFC                                                          | ATFCSH
ATFC                                                          | ATFCSM
ATFE                                                            ATFE
ATFI                                                            ATFI
ATFT                                                          <
ATFT                                                          <
ATH0                                                            ATH0
ATH1                                                            ATH1
ATI                                                             ATI
ATIA                                                          | ATIB
ATIB10                                                        <
ATIB12                                                        <
ATIB15                                                        <
ATIB48                                                        <
ATIB96                                                        <
ATIFR                                                           ATIFR
ATIFR0                                                        <
ATIFR4                                                        <
ATIGN                                                         <
ATIIA                                                           ATIIA
ATJE                                                            ATJE
ATJHF0                                                          ATJHF0
                                                              > ATJHF1
ATJS                                                            ATJS
ATJTM1                                                        | ATJTM
ATJTM5                                                        <
ATKW                                                            ATKW
ATKW0                                                           ATKW0
ATKW1                                                           ATKW1
ATL0                                                            ATL0
ATL1                                                            ATL1
ATLP                                                            ATLP
ATM0                                                            ATM0
ATM1                                                            ATM1
ATMA                                                          <
ATMP                                                          <
ATMP                                                          <
ATMP                                                          <
ATMP                                                            ATMP
ATMR                                                          <
ATMT                                                          <
ATNL                                                            ATNL
ATPB                                                            ATPB
ATPC                                                            ATPC
ATPP                                                          | ATPPFFOFF
ATPP                                                          | ATPPFFON
ATPP                                                          <
ATPP                                                          <
ATPP                                                          <
ATPPS                                                         <
ATR0                                                            ATR0
ATR1                                                            ATR1
ATRA                                                            ATRA
ATRD                                                          <
ATRTR                                                           ATRTR
ATRV                                                            ATRV
ATS0                                                            ATS0
ATS1                                                            ATS1
ATSD                                                          <
ATSH                                                            ATSH
ATSH                                                            ATSH
ATSH                                                            ATSH
ATSI                                                            ATSI
ATSP                                                            ATSP
ATSP                                                          <
ATSP                                                          <
ATSR                                                            ATSR
ATSS                                                            ATSS
ATST                                                            ATST
ATSW                                                            ATSW
ATSW                                                          <
ATTA                                                            ATTA
ATTP                                                            ATTP
ATTP                                                            ATTP
ATV0                                                            ATV0
ATV1                                                            ATV1
                                                              > ATVPW
ATWM                                                            ATWM
ATWS                                                            ATWS
ATZ                                                           / ATZ
```