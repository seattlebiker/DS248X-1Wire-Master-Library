This is a library for interfacing an ESP8266 device (Wemos, NodeMCU, etc.) to a Maxim-Dallas DS2482/4 1-wire master.  It should also work with Arduino's as well, but I
haven't tested that.

I've been using 1-wire devices for years. Some time ago I adapted the Maxim-Dallas 1-Wire API for Java (OWAPI Version 1.10), and created a I2CBridgeAdapter class for the DS2482 1-Wire master based on other adapter classes. Interfacing an ESP8266 SoC with a DS2482 1-wire master in C++ firmware has been a learning (and re-learning) experience.  I've incorporated ideas from these many sources (some acknowledged below) and my J2ME_OneWire libary, for this DS248X 1-wire library. I changed some of the function names where it makes sense, i.e., diferentiating the DS2482 device reset from a 1-wire reset, e.g. OWReset. The library was built using the PlatformIO extension to VSCode.

Paul Stoffregan maintains the 1-wire library (originally by Jim Studt), the latest version of which may be found at: http://www.pjrc.com/teensy/td_libs_OneWire.html

I also got some ideas from these authors:

PaeaeTech: https://github.com/paeaetech/paeae/tree/master/Libraries/ds2482

CyberGibbons: https://github.com/cybergibbons/DS2482_OneWire

Also, credit is given to JBechter for DS2423 and DS2438 code:
    https://github.com/jbechter/arduino-onewire-DS2423
    https://github.com/jbechter/arduino-onewire-DS2438

The reason for resurrecting 1-wire code? I was on a quest to find 1-wire code that would find DS2423 counters on the Microlan, since work great for a remote MetOne Instruments rain gauge. The DS2423 is obsolete unfortunately, but I still have a few, hence the quest.  I ran across Cybergibbons' DS2482 ibrary, which works for finding DS1820 temperature devices, but it's based on Paeae's library which has the same issue as follows:

I had to fix the 1-wire search function because it failed to find other than DS1820 (family ID 0x10) devices on the MicroLan.  If a device with bit 64 in the ROM was a '1' was also on the bus, it didn't see it. I don't take credit for that search fix, but it's in my version. Here's a couple links; User "Garitron" on the Arduino forum found the fix for the 1-wire search detailed in the first link. https://forum.arduino.cc/t/introducing-maxim-ds2482-1-wire-master-library-2/51436 https://forum.arduino.cc/t/introducing-maxim-ds2482-1-wire-master-library/37430?

By running the example code, you should see something like the below.  I have the following on the Microlan (1-wire bus):

1) a singe DS2323 connected to a MetOne raingauge reed switch (over a 25 foot cable). 
2) a DS2438 Smart Battery monitor
3) two DS1820 temperature sensors

DS248X 1-wire library by SeattleBiker: a Demo

Checking for I2C devices...:
DS2482-100/800 present
        Checking for 1-Wire devices...
        There are devices present on 1-Wire bus
        Searching for { 0x1D, 0xB8, 0x87, 0x01, 0x00, 0x00, 0x00, 0x70 } using OWVerify()
                Found { 0x1D, 0xB8, 0x87, 0x01, 0x00, 0x00, 0x00, 0x70 }
                Found 1-wire counter using OWVerify()

        Searching for family 10
                Found device { 0x10, 0x61, 0x0E, 0x42, 0x00, 0x08, 0x00, 0x3C } using OWTargetSearch()

        Searching 1-Wire bus with OWSearch()...
        Found device: { 0x10, 0x61, 0x0E, 0x42, 0x00, 0x08, 0x00, 0x3C }
        Calculated CRC of device serial is 3C
        This device is a 1-wire thermometer
        No temperature error. Temperature = 22.6C or 72.6F
        Found device: { 0x10, 0x67, 0xBB, 0xE0, 0x02, 0x08, 0x00, 0xF2 }
        Calculated CRC of device serial is F2
        This device is a 1-wire thermometer
        No temperature error. Temperature = 21.1C or 69.9F
        Found device: { 0x26, 0x26, 0x14, 0x09, 0x00, 0x00, 0x00, 0x95 }
        Calculated CRC of device serial is 95
        This device is a 1-wire thermometer
        This device is a DS2438 ADC/thermometer
        No code here yet
        Found device: { 0x1D, 0xB8, 0x87, 0x01, 0x00, 0x00, 0x00, 0x70 }
        Calculated CRC of device serial is 70
        Found 1-wire counter at { 0x1D, 0xB8, 0x87, 0x01, 0x00, 0x00, 0x00, 0x70 }
        DS2423 returned: 61

Hopefully this library will help others, after hours and hours trying to find code that works for me. This library is posted in an effort to give back to the development community.

I welcome comments about the code and improvements; I'm not a software developer by trade (retired sysadmin), and I'm somewhat of a noob using Git, so be kind :-)