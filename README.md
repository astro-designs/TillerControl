Arduino based radio-controlled Tiller Control
---------------------------------------------

Designed for Bruce Boats, part of the Kennet & Avon Canal Trust
Fully Accessible Canal Boating on the Kennet and Avon Canal

Designed by Bo,
Modifications by Mark

***
History
-------

Previous changes include, but are not limited to ('coz I may have missed something...)
* Added Pot-control;
* Added servo driver to provide a live / physical indication of the position of the tiller;
* Added Midship button to simulate the same action as putting the Pot into the midship position;
* Added batter level monitoring;
* Added status LED
    * Flashes every 5s when awake to provide some status
    * Status is binary-encoded to capture bits of information...
        * Single flash: All OK!
        * +1 flash: Radio communication fault
        * +2 flases: Low battery warning
        * +4 flashes: Critical battery warning
        * +8 flashes: Near to sleep
        * +16 flashes: Going to sleep!

09/07/23
* Changed timeout / standby trigger so that it doesn't time-out while the 'Goto' / Pot control switch is pressed;
* Modified the battery under-voltage warning & critical thresholds to suit the voltage regulator on the custom low-power Arduino board
    * Warning level set to 5.9V, was 4.8
    * Critical level set to 5.5V, was 4.4V
* Added constants to set valid Pot outputs for left-handed & right-handed use, the motor is disabled when the control is outside of these areas;
* Added Tiller position constants to define centre and range. The motor is prevented from going outside of this range;
* Added debug print of Tiller position received from radio (buf[3]);
* Disabled debug (Debug constant set to 0)
* Reorganised control priorities
    * If Pot control is enabled then this is the highest priority - no other controls work;
    * If Pot control is disabled then left, right or midship control work;
* setup radio function is repeated when coming out of sleep - this is necessary because the custom low-power Arduino board switches the radio off in standby to improve (reduce) standby power consumption;

22/07/23
* Reduced Pot deadband (now 2, was 5)
* Increased sensitivity to Pot change (now 4, was 5)

26/08/2025
* Changing motor controls to send speed instead of on/off
* work in progress...
* Moving NeoPixels pin from pin 4 to pin 9 (Optional, this or Servo)
* Moving Midship button from pin 1 to pin 4