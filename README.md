# Transistor-Curve-Tracer-MapleMini

Based on the super project "Transistor Curve Tracer" from Peter Balch : https://www.instructables.com/id/Transistor-Curve-Tracer/
I ported the Software from the 8-bit Arduino Platform to STM32duino .

For me was the MapleMini the best choice: cheap, quick, enough programm-space, more accessible port-pins as the Arduino Uno/Nano and more
flexibility :
more than 5 ADC's with 12-bit resolution, 2 SPI's/, more than 2 ISR-Pins and USB-socket for programming and supply. 
Also the 3.3 V port-voltage an was good for connection to the ILI9341-spi-tft-display.

Therefore I had the possibility to integrate other features:
- integration of two rotary encoders (on isr) for comfortable changing parameters, a duul Dac (MCP4822) with the same 12-bit resulotion as the ADC's.
- possibility to save the paramters in the internal EEPROM
- possibility compare similar or complementere devices on the same screen
- changing x-scale for diodes/zeners (0,1,2,3,4,6,12,24V) (0: starts with 24V and search the optimal scale)
- changing y-scale for MosFets/jFets automatic depending on max. IDssmax

For the hardware of the tracer is different from Peters:
I use switches to connect the Emitter/Source different respectively Collector/Drain to +/- supply-voltage while measuring.
At first tested with mechanical relais, I changed to  PhotoMosRelais because of space and supply-curren( 40ma -> 2-5mA.
The MapleMini-ports can drive the PhotoMosRelais direct.
As amplifier to drive the Emitter/Source ... Collector/Drain I use one amplifier of TCA0372 with gain = +3 .
As amplfier for the base-current/gate-volage I use a LT1301, because he can work with max. 44V.
The TCA0372 has a normal supply-voltgae from 12.4V, chaning to about 26V for measuring zeners with Vz > 12V.
his negative supply is near -3V, generated with a charge-pump.

The LT1301 has a pos. supply-voltage from around 26V, a negative supply from -12v via a ICL7662 charge-pump.
I need the negative supply for the gates from nJFET's and depletion-mode-nMOSFet's, the positive supply > 12V for the gates from 
p-JFets (I think it exist only one depletion-mode-pMOSFet-chip!).

The gain of the opamp's of the LT1301 can changed from +6 to -6 with two PhotoMosRelais, or better 
over the relais You can select the opamp with gain +6 or -6.

The current is limited most to max 50mA . only for jFet's/MosFet's it goes up to about 100mA.
