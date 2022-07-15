This Arduino sketch is designed for running on [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) to collect vehicle telemetry data from OBD-II, GPS and motion sensor and transmit the collected data to a remote server running [Freematics Hub](https://freematics.com/hub) software in realtime. It also has a mechansim for executing and responding to commands sent from serverside.

Data Collection
===============

The sketch collects following data.

* Vehicle OBD-II PIDs data (from OBD port)
* Battery voltage (from OBD port)
* Geolocation data (from cellular module's internal GNSS or external GNSS receiver)
* Acceleration and gyroscope data (from internal MEMS sensor)
* Device temperature (from MEMS sensor or ESP32 built-in sensor)

Data Transmissions
==================

Data transmission over UDP and HTTP protocols are implemented with following hardware.

* WiFi (ESP32 built-in)
* WiFi Mesh (ESP-MDF for ESP32)
* GSM/GPRS (SIM800)
* 3G WCDMA (SIM5360)
* 4G LTE (SIM7600)

There two ways of sending data:

1. UDP mode implements a full telemetry client for [Freematics Hub](https://freematics.com/hub/)
   and [Traccar](https://www.traccar.org) (sends more data, [protocol's API](https://freematics.com/pages/hub/api/), uses 5170 port).
2. HTTP/HTTPS mode implements a `osmand` protocol client for Traccar
   (sends only location data, [protocol's API](https://www.traccar.org/osmand/),
   uses 5055 port)

Data Storage
============

Following types of data storage are supported.

* MicroSD card storage
* ESP32 built-in Flash memory storage (SPIFFS)

Buzzer notes & patterns
=======================
While connected to OBD, the device may produce the following buzzing-sounds:

  - 1Hz tick: immediately after boot, while in `setup()` & before `initialize()`; hence while `BOOT_OBD_PIPE_TIMEOUT_SEC` mode is enabled.
  Occurs again if "reboot" from standby configured.
- 2Hz ta-tick: `initialize()`; always follows boot, and wake-up from standby
  (even if no "reboot" from standby configured).
- 1Hz ta-tick: while waiting to reconnect, after multiple attempts to login.

Remote Commands
===============

Commands can be sent to Freematics ONE+ for execution with results obtained, through serial terminal or by [Freematics Hub API](https://freematics.com/hub/api/) (remotely). Currently following commands are implemented.

* LED [0/1/2] - setting device LED status (0:auto 1:always off 2:always on)
* REBOOT - performing a reboot immediately
* STANDBY - entering standby mode immediately
* WAKEUP - waking up the device from standby mode
* SET SYNC [interval in ms] - setting server sync checking interval
* STATS - returning some stats
* OBD <AT-command> - send ELM327 command to to OBD-II co-proc and await result
* OTA [URL] - upgrade firmware from default or given url
* SET
  * SYNC [interval in ms] - setting server sync checking interval
  * OTABOOT 0|1 - set next OTA partition to boot

Example API calls
-----------------

Send OTA command through the *teleserver* with your browser like this:

  <teleserver-url>/api/command?id=DEVICEID&cmd=OTA

Switch which OTA partition to boot & reboot:

  <teleserver-url>/api/command?id=DEVICEID&cmd=SET%20OTABOOT%200
  <teleserver-url>/api/command?id=DEVICEID&cmd=REBOOT


Viewing Trip Data
=================

Once the sketch is running and data is being submitted to hub.freematics.com, you can open https://hub.freematics.com from any of your devices and enter your device ID (displayed in serial output) to view real-time data and history trip data.

![Freematics Hub Dashboard](https://freematics.com/pages/wp-content/uploads/2019/01/freematics_hub_dash-1024x576.png)

Prerequisites
=============

* [Freematics ONE+](https://freematics.com/products/freematics-one-plus/) or [Freematics ONE+ Model B](https://freematics.com/products/freematics-one-plus-model-b/)
* A micro SIM card if cellular network required
* [PlatformIO](http://platformio.org/), [Arduino IDE](https://github.com/espressif/arduino-esp32#installation-instructions) or [Freematics Arduino Builder](https://freematics.com/software/arduino-builder) for compiling and uploading code
