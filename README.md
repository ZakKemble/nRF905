nRF905 AVR/Arduino Library/Driver
=================================

[Nordic nRF905](http://www.nordicsemi.com/eng/Products/Sub-1-GHz-RF/nRF905) Radio IC library for Arduino and AVR microcontrollers.

http://blog.zakkemble.co.uk/nrf905-avrarduino-librarydriver/

Documentation
-------------

[Doxygen pages](http://zkemble.github.io/nRF905/)

Installing
----------

### Arduino
Copy the `arduino/nRF905/` folder to your Arduino libraries folder.

Add

    #include <nRF905.h>

to the top of the sketches that use the library.

Also have a look at nRF905_config.h, that's where you can change which pins are used and stuff.

### AVR
Copy the `./nRF905` folder to your project and add

    #include "nRF905/nRF905.h"

to the source files that use the library.

Examples in the examples folder.

---

Zak Kemble

contact@zakkemble.co.uk
