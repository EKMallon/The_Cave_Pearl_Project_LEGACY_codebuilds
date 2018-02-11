<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/CavePearlProjectBanner_130x850px.jpg">

# The_Cave_Pearl_Project_CURRENT_codebuilds

This repository contains current logger base code builds for the  "Modules &amp; Jumper Wires" data logger described in 

**Cave Pearl Data Logger: A Flexible Arduino-Based Logging Platform for Long-Term Monitoring in Harsh Environments**
Sensors 2018, 18(2), 530; http://www.mdpi.com/1424-8220/18/2/530  (open access journal)

and on the project's blog at  https://edwardmallon.wordpress.com/how-to-build-an-arduino-data-logger/ 

The overall data buffering/saving functions at the heart of the code have has been made generic by the use of
Mikal Hart's PString library  http://arduiniana.org/libraries/pstring/  which is used to concatenate sensor 
readings in different variable formats into a 30-byte char array that can be handled the wire library's 32-byte buffer.

Each version posted here is necessarily a working variant for some specific sensor combination, so check the comments at the start 
of each script for those hardware dependencies. I use #ifdef / #endif statements throughought the code to include or exclude 
blocks of sensor-specific code at compile time, depending on #define SensorName_Address statements at the start of the script.

It is also worth noting that these live versions are reasonably complex, with quite a bit of functionality. If you are new to programming in the Arduino IDE, then a much simpler working version of the logger's code can be found in the UNO-Breadboard repository:

https://github.com/EKMallon/UNO-Breadboard-Datalogger

That simpler code saves data to the SD card on every cycle, rather than buffering to an eeprom first, but you should still see about 2/3 of the operating lifespan that you'd get with the more advanced builds.
