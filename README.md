# EFIS-Remote-Module-AvionicsDuino
Software of the EFIS remote module, including magnetometer, temperature, and relative humidity sensors

See https://avionicsduino.com/index.php/en/digital-compass/

WARNING!

Version 2.0 or later of this software should only be used with the EFIS software version 3.0 or later and the AHRS software version 2.0 or later. 
This is due to major modifications to the CAN bus configuration required when removing the UART serial communication line that connected the EFIS to the AHRS and hooking the AHRS up to the CAN bus.
Version 1.0 should be used with EFIS software V 2.5, AHRS software V 1.3, and a UART serial communication line between AHRS and EFIS.
