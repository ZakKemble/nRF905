#nRF905 AVR/Arduino Library/Driver

<a href="http://www.nordicsemi.com/eng/Products/Sub-1-GHz-RF/nRF905">Nordic nRF905</a> Radio IC library for Arduino and AVR microcontrollers.


##Documentation
<a href="http://zkemble.github.io/nRF905/">Doxygen pages</a>


##Installing
###Arduino
Copy the arduino/nRF905/ folder to your Arduino libraries folder.

If you are using Arduino 1.5 then you will need to move the .cpp and .h files into the src folder.

Add

    #include <nRF905.h>
	#include <SPI.h>

to the top of the sketches that use the library.

Also have a look at nRF905_config.h, that's where you can change which pins are used and stuff.

###AVR
Copy the nRF905 folder to your project and add

    #include "nRF905/nRF905.h"

to the source files that use the library.

Examples in the examples folder.

--------

http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/

--------

Zak Kemble

contact@zakkemble.co.uk
