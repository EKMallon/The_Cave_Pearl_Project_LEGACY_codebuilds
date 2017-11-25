# The_Cave_Pearl_Project_CURRENT_codebuilds
Base code for the Arduino compatible "Modules &amp; Jumper Wires" data logger described on the project's blog
at  https://edwardmallon.wordpress.com/how-to-build-an-arduino-data-logger/ 

The overall data buffering/saving functions at the heart of the code have has been made generic by the use of
Mikal Hart's PString library  http://arduiniana.org/libraries/pstring/  which is used to concatenate sensor 
readings in different variable formats into a 30-byte char array that can be handled the wire library's 32-byte buffer.

Each version posted here is necessarily a working variant for some specific sensor combination, so check the comments at the start 
of each script for those hardware dependencies. I use #ifdef / #endif statements throughought the code to include or exclude 
blocks of sensor-specific code at compile time, depending on #define SensorName_Address statements at the start of the script.
