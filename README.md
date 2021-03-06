# Transistor-Curve-Tracer-MapleMini

This project based on the super project "Transistor Curve Tracer" from Peter Balch :
https://www.instructables.com/id/Transistor-Curve-Tracer/  

I ported the software from the 8-bit Arduino platform ( hardware: Arduino Pro Mini ) to 32 bit STM32duino (hw: MapleMini).




For me was the MapleMini the best choice: cheap, quick, enough program-space, more accessible port-pins as the Arduino Uno/Nano/... and more flexibility and accuracy :

more than 5 ADC's with 12-bit resolution, 2 SPI's/, more than 2 ISR-Pins (for the encoders) and USB-socket for programming and supply. 
Also the 3.3 V port-voltage was favourable for connections to the ILI9341-spi-tft-display/touch.

For more information about MapleMini look here: https://www.stm32duino.com/
or about this project:  https://www.stm32duino.com/viewtopic.php?t=4269



Therefore I had the possibility to integrate other features:

- integration of two rotary encoders (on isr) for a more comfortable changing of the parameters, a dual DAC (MCP4822) with the same 12-bit resolution as the ADC's on the MapleMini.
- possibility to save the parameters in the internal EEPROM
- possibility compare similar or complementary devices on the same screen
- changing x-scale for diodes/zeners (0,1,2,3,4,6,12,24V) (0: starts with 24V and search the optimal scale)
- changing y-sacle for diodes from 0...10mA  to 0...50mA
- changing y-scale for MosFETs/jFETs automatic depending on max. IDss from 50mA to 10mA or 50mA to 100mA
- changing x-scale for n-jFETs from 12V (default), 6V, 4V, 3V, 2V.

The schematic of the tracer is different from Peters version:

I use switches to connect the Emitter/Source/Cathode respectively Collector/Drain/Anode to +/- supply-voltage only while measuring - most time the DUT is only with the base connected, Emtter / Collector are isolated.
At first I tested it with a dual toggle switch, than with mechanical relais, than I changed to  PhotoMosRelais because of less space on the board and less supply-current( 40ma/5V -> 2-5mA/3.3V). So the MapleMini-ports can drive the PhotoMosRelais direct.

As amplifier to drive the Emitter/Source ... Collector/Drain I made another design.
Peter used a LM358 standard dual amplifier with emitter-followers, but I think it was difficult to make it stable and there was a big drop on the positive rail.
Therefore I searched and tested some other amplfiers with drive capacibility of more than 50mA, among others NE5532 , L2272 and TCA0372.
The TCA0372 was the best in relation to stability and to low drop on the supply (r-to-r) at 50mA.
I use only one amplifier of the TCA0372 with gain = +3 (default).
The output of the TCA0372 can manage more than 100mA, the negative supply-voltage I put down to about -3V, because the TCA0372 is not a real rail-to-rail-amplifier at this currents. The negative supply near -3V is generated with a charge-pump (from+3.3V).
With the U-DAC (MCP4822) I can generate 0...12V similar the version from Peter.
As amplifier for the base-current/gate-voltage I use a LT1013 dual amplifier, because it's a R to R and can work up to max. 44V.
The TCA0372 has a normal supply-voltage from 12.6V, changing to about 26V for measuring zeners with Vz > 12V and <= 24V. Therefore I have to change the gain of the TCA0372 to +6 (V-DAC-max  (=4.095V) * 6 >=24V).
For measuring nJFETS in the ohmic-region (< Vpinch) I can switch the x-scale at the display from 12V ... to 2V.
Between 2V and 4V the gain of the TCA0372 is switched from 3 to 2, the V-DAC MCP4822 works than from 0...2.047V with step = 0,5mV.  

The LT1013 has a positive supply-voltage from around 26V, a negative supply from -12v via a LM317L and ICL7662 charge-pump.
I need the negative supply for the gates from nJFET's and depletion-mode-nMOSFET's, the positive supply > 12V for the gates from 
p-JFET's (I think it exist only one depletion-mode-pMOSFET-chip!).

The gain of the opamp's of the LT1013 can be changed from +6 to -6 with two PhotoMosRelais, or better 
over the relais I can select the opamp with the gain +6 or -6.

For npn/pnp and MosFets the I-DAC here works from 0... 2.047 V to generate 0 ... + 12 V with gain = +6 or
with gain = -6  to generate 0 ... - 12 V gatevoltage for nJFET's,
for pJFET's the I-DAC works from about 2.048 ... 4.095 V to generate gatevoltage from +12 ... +24 V for
pJFET with gain +6.

The current is limited to max 50mA . only for JFET's/MOSFET's it can go in a second step up to about 100mA.

The software is developed with the ARDUINO IDE 1.8.4, STM32duino is about one year (2017) old, but I think the newest version would work to.

You have to load  Bootloader 2.0 (20k RAM, 120k Flash) in MapleMini.

The main components are:

- ILI9341 TFT-SPI-Display 2.8" with Touch ca 6.70 €
- MapleMini                                  3.60 €
- DACMCP4822                                 3.20 € 
- AQY212 6x                                  6.00 €
- LT1013                                     1.40 €
- TCA0372                                    1.10 €
- MT3608 3x                                  2.00 €
- Rotary Encoder 2x                          2.00 €
- nMOS switch    7x                          2.00 €    
- ICL7662 (-12V) + TSP490603(-5/-3.3V)       4.00 €
-                           SUM:            32.00 €
+ resistors, capacitors, diodes, test-sockets,                                          
+ pcb's, wire, zinn, sockets, battery, chassis ...     

The hardware is nearly finsihed, the software is growing ! 
