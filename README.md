<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/CavePearlProjectBanner_130x850px.jpg">

# The_Cave_Pearl_Project_CURRENT_codebuilds

This repository contains current logger base code builds for the  "Modules &amp; Jumper Wires" data logger described in 

**Cave Pearl Data Logger: A Flexible Arduino-Based Logging Platform for Long-Term Monitoring in Harsh Environments**
Sensors 2018, 18(2), 530; http://www.mdpi.com/1424-8220/18/2/530  (open access - free PDF download)  and on the project's blog at  https://thecavepearlproject.org/ (note: step by step logger build instructions are on the blog)

<img src="https://github.com/EKMallon/The_Cave_Pearl_Project_CURRENT_codebuilds/blob/master/images/20180210_CavePearlLogger_ProMiniVariant_1240pix.png">

The overall data buffering/saving functions at the heart of the code have has been made generic by the use of
Mikal Hart's PString library  http://arduiniana.org/libraries/pstring/  which is used to concatenate sensor 
readings in different variable formats into a 30-byte char array that can be handled the wire library's 32-byte buffer.

Each version posted here is necessarily a working variant for some specific sensor combination, so check the comments at the start 
of each script for those hardware dependencies. I use #ifdef / #endif statements throughought the code to include or exclude 
blocks of sensor-specific code at compile time, depending on #define SensorName_Address statements at the start of the script.

It is also worth noting that these live versions are reasonably complex, with quite a bit of functionality. If you are new to programming in the Arduino IDE, then a much simpler working version of the logger's code can be found in the UNO-Breadboard repository:

https://github.com/EKMallon/UNO-Breadboard-Datalogger

That simpler UNO logger code saves data to the SD card on every cycle, rather than buffering to an eeprom first. Saving data to the SD card at every cycle uses less than 600mAs/day, while a logger that sleeps at 0.25 mA uses ~21,000 mAs during sleep. So you should still see at least 80% of the logger operating lifespan you'd get from the more complicated buffering code. (ie: >8 months on a full set of AA batteries)
