# The_Cave_Pearl_Project_CURRENT_codebuilds
Base code for the Arduino compatible "Modules &amp; Jumper Wires" data logger described on the project's blog.

The overall data buffering/saving functions at the heart of the code have has been made generic by the use of
Mikal Hart's PString library [ http://arduiniana.org/libraries/pstring/ ] which is used to concatenate sensor 
reading data into a 30-byte char array that can be handled the I2C wire library's limited 32-byte buffer.

Each version posted here is necessarily a variant for a specific sensor combination, so check the comments at the start 
of each script for those hardware dependencies. I use #ifdef / #endif statements extensively to include or exclude 
sensor specific blocks of code at compile time, depending on #define SensorName_Address statements at the start of the script.
