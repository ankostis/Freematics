# ELM327 AT commands summary

It is assumed that STM32F103 co-processor hooked on the CAN-bus on *freematics*
is running *some* version of the [ubiquitous ELM327 firmware](https://en.wikipedia.org/wiki/ELM327).
The list of AT-commands below are provided as an aid to the developers,
to facilitate quick text-searches while understanding the code.

The commands were manually extracted from the 99-page PDF document on [elm327 IC, v2.3](https://www.elmelectronics.com/ic/elm327l/)
([waybacked](https://web.archive.org/web/20220324203957/https://www.elmelectronics.com/ic/elm327l/))
on May 2022 (a month before the closing of ELM company).

> **Note:** Settings which are shown with an asterisk (*) are the default values.

## General Commands

- `<CR>`: repeat the last command (`\r`, or ASCII(0x0D))
- ATBRD hh: try Baud Rate Divisor hh
- ATBRT hh: set Baud Rate Timeout
- ATD: set all to Defaults
- ATE0: Echo off
- ATE1: Echo on*
- ATFE: Forget Events
- ATI: print the version ID
- ATL0: Linefeeds off
- ATL1: Linefeeds on
- ATLP: go to Low Power mode
- ATM0: Memory off
- ATM1: Memory on
- ATRD: Read the stored Data
- ATSD hh: Save Data byte hh
- ATWS: Warm Start (quick software reset)
- ATZ: reset all
- AT@1: display the device description
- AT@2: display the device identifier
- AT@3 cccccccccccc store the @2 identifier


## Programmable Parameter Commands

- ATPP xx OFF: disable Prog Parameter xx
- ATPP FF OFF: all Prog Parameters disabled
- ATPP xx ON: enable Prog Parameter xx
- ATPP FF ON: all Prog Parameters enabled
- ATPP xx SV yy: for PP xx, Set the Value to yy
- ATPPS: print a PP Summary


## Voltage Reading Commands
- ATCV dddd: Calibrate the Voltage to dd.dd volts
- ATCV 0000: restore CV value to factory setting
- ATRV: Read the input Voltage

## Other

- ATIGN: read the IgnMon input level


## OBD Commands

- ATAL: Allow Long (>7 byte) messages
- ATAMC: display Activity Monitor Count
- ATAMT hh: set the Activity Mon Timeout to hh
- ATAR: Automatically Receive
- ATAT0, 1, 2: Adaptive Timing off, auto1*, auto2
- ATBD: perform a Buffer Dump
- ATBI: Bypass the Initialization sequence
- ATDP: Describe the current Protocol
- ATDPN: Describe the Protocol by Number
- ATFT: Filter for Transmitter off*
- ATFT hh: Filter for Transmitter = hh
- ATH0: Headers off*
- ATH1: Headers on
- ATIA: Is the protocol Active?
- ATMA: Monitor All
- ATMR hh: Monitor for Receiver = hh
- ATMT hh: Monitor for Transmitter = hh
- ATNL: Normal Length messages*
- ATPC: Protocol Close
- ATR0: Responses off
- ATR1: Responses on*
- ATRA hh: set the Receive Address to hh
- ATS0: printing of Spaces off
- ATS1: printing of Spaces on*
- ATSH xyz: Set Header to xyz
- ATSH xxyyzz: Set Header to xxyyzz
- ATSH wwxxyyzz: Set Header to wwxxyyzz
- ATSP h: Set Protocol to h and save it
- ATSP Ah: Set Protocol to Auto, h and save it
- ATSP 00: Erase stored protocol
- ATSR hh: Set the Receive address to hh
- ATSS: use Standard Search order (J1978)
- ATST hh: Set Timeout to hh x 4 msec
- ATTA hh: set Tester Address to hh
- ATTP h: Try Protocol h
- ATTP Ah: Try Protocol h with Auto search


## J1850 Specific Commands (protocols 1 and 2)

- ATIFR0, 1, 2: IFRs off, auto*, or on, if not monitoring
- ATIFR4, 5, 6: IFRs off, auto, or on, at all times
- ATIFR H, S: IFR value from Header* or Source


## ISO Specific Commands (protocols 3 to 5)

- ATFI: perform a Fast Initiation
- ATIB10: set the ISO Baud rate to 10400*
- ATIB12: set the ISO Baud rate to 12500
- ATIB15: set the ISO Baud rate to 15625
- ATIB48: set the ISO Baud rate to 4800
- ATIB96: set the ISO Baud rate to 9600
- ATIIA hh: set ISO (slow) Init Address to hh
- ATKW: display the Key Words
- ATKW0: Key Word checking off
- ATKW1: Key Word checking on*
- ATSI: perform a Slow (5 baud) Initiation
- ATSW hh: Set Wakeup interval to hh x 20 msec
- ATSW 00: Stop sending Wakeup messages
- ATWM [1 - 6 bytes]: set the Wakeup Message


## CAN Specific Commands (protocols 6 to C)

- ATC0: send Confirmation off
- ATC1: send Confirmation on*
- ATCAF0: Automatic Formatting off
- ATCAF1: Automatic Formatting on*
- ATCEA: turn off CAN Extended Addressing
- ATCEA hh: use CAN Extended Address hh
- ATCER hh: set CAN Extended Rx address to hh
- ATCF hhh: set the ID Filter to hhh
- ATCF hhhhhhhh: set the ID Filter to hhhhhhhh
- ATCFC0: Flow Controls off
- ATCFC1: Flow Controls on*
- ATCM hhh: set the ID Mask to hhh
- ATCM hhhhhhhh: set the ID Mask to hhhhhhhh
- ATCP hh: set CAN Priority to hh (29 bit)
- ATCRA: reset the Receive Address filters
- ATCRA hhh: set CAN Receive Address to hhh
- ATCRA hhhhhhhh: set the Rx Address to hhhhhhhh
- ATCS: show the CAN Status counts
- ATCSM0: Silent Monitoring off
- ATCSM1: Silent Monitoring on*
- ATCTM1: set Timer Multiplier to 1*
- ATCTM5: set Timer Multiplier to 5
- ATD0: display of the DLC off*
- ATD1: display of the DLC on
- ATFC SM h: Flow Control, Set the Mode to h
- ATFC SH hhh: FC, Set the Header to hhh
- ATFC SH hhhhhhhh: Set the Header to hhhhhhhh
- ATFC SD [1 - 5 bytes]: FC, Set Data to [...]
- ATPB xx yy: Protocol B options and baud rate
- ATRTR: send an RTR message
- ATV0: use of Variable DLC off*
- ATV1: use of Variable DLC on


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
