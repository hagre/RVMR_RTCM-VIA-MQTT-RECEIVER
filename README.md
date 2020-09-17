# RVMR (RTCM-VIA-MQTT-RECEIVER)
INFO: This Software is made for my private use. I like to share, but for further details read the LICENCE file! (GNU GPL V3)

Check:  

RVMT (RTCM VIA MQTT TRANSMITTER) https://github.com/hagre/RVMT_RTCM-VIA-MQTT-TRANSMITTER

RVMP (RTCM-VIA-MQTT-PROTOCOL) https://github.com/hagre/RVMP_RTCM-VIA-MQTT-PROTOCOL 

?(RVMC (RTCM-VIA-MQTT-CASTER) is on https://github.com/hagre/RVMC_RTCM-VIA-MQTT-CASTER)

RVMR (RTCM VIA MQTT RECEIVER) is using the MQTT protocol (as a secure and opensource alternative to NTRIP) to get RTK correction data to my rover GPS units.
As a target system i will use https://github.com/farmerbriantee/AgOpenGPS. AgOpenGPS is connecting to the RVMR as a simulated local NTRIP-Caster on Port 2101 and relay the RTCM to the rover (DualGPS) from https://github.com/mtz8302/AOG_GPS_ESP32.


Basic selection of features via #define in the top of this code

* Subscribing to MQTT Broker and MSM4 or MSM7 Base station Topics /NTRIP/Base/XYZ01/RTCM/...
* all received msg will arrive on TCP port (2101) and can be received as "NTRIP-Caster" with AgOpenGPS => and feed in to the f9p GPS (or somthing else) on the "normal" way

# General
All connections are currently done over wifi (planing to implement a Ethernet/LAN connection)

Nearly all parameters are adjusable in the first lines of code in the top #define section.
-Timings, IPs, BASE, RTCM Type,... adjust as required for your hardware (some hints or my last settings can be found behind the next // comment)

I am compiling witch VSCode and PlatformIO in the Arduino framwork.
Hardware in use:
-ESP32 noname board

Big thanks to the arduino community for making ths all possible.

I have to anounce that an external libarie is used: 
knolleary/pubsubclient  (licensed under the MIT License). 
Check https://github.com/knolleary/pubsubclient

During research for this project, i found some projects working on ths promissing concept.

initialy found
https://github.com/GeoscienceAustralia/gnss-mqtt

and
http://www.ignss2018.unsw.edu.au/sites/ignss2018/files/u80/Slides/D2-S3-ThC-Wang.pdf

and comercial
https://esprtk.com/

just found:
https://github.com/geerdkakes/mqtt-rtk

it would be nice if mqtt could be implemented to
https://github.com/nebkat/esp32-xbee 


# Compile:

require #include "PubSubClient.h" library https://github.com/knolleary/pubsubclient.git PlatforIO ID_89

require #include "CircularBuffer.h" library https://github.com/rlogiacco/CircularBuffer 

# see platformio.ini:

require #include "verysimpletimer.h"  https://github.com/hagre/VerySimpleTimer_Library.git

require #include "SyncWifiConnectionESP32.h"  https://github.com/hagre/SyncWiFIConnectionESP32_Library.git

require #include "SyncMQTTConnectionESP32.h" https://github.com/hagre/SyncMQTTConnectionESP32_Library.git

require #include "rtcmstreamsplitter.h" https://github.com/hagre/RTCM_Stream_Splitter_Library.git

and/as above                           rlogiacco/CircularBuffer@^1.3.3


ToDo:
* Find a good way to update of root_ca 
* Find a better and futureproof name for this new RTCM transmitting and hosting standard!
* Complete the design of using the MQTT broker as an correction data server according to an NTRIP Caster

by hagre 
2020 
