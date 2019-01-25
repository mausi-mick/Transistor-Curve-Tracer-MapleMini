# Transistor-Curve-Tracer-MapleMini

Based on the super project "Transistor Curve Tracer" from Peter Balch :
https://www.instructables.com/id/Transistor-Curve-Tracer/  based on an Arduino Mini Pro Mini 
I ported the software from the 8-bit Arduino Platform to 32 bit STM32duino / MapleMini.

For me was the MapleMini the best choice: cheap, quick, enough program-space, more accessible port-pins as the Arduino Uno/Nano/... and more flexibility and accuracy :
more than 5 ADC's with 12-bit resolution, 2 SPI's/, more than 2 ISR-Pins and USB-socket for programming and supply. 
Also the 3.3 V port-voltage an was favourable for connections to the ILI9341-spi-tft-display.

Therefore I had the possibility to integrate other features:
- integration of two rotary encoders (on isr) for comfortable changing parameters, a duul Dac (MCP4822) with the same 12-bit resulotion as the ADC's.
- possibility to save the paramters in the internal EEPROM
- possibility compare similar or complementary devices on the same screen
- changing x-scale for diodes/zeners (0,1,2,3,4,6,12,24V) (0: starts with 24V and search the optimal scale)
- changing y-sacle for diodes from 0...10mA  to 0...50mA
- changing y-scale for MosFets/jFets automatic depending on max. IDssmax

For the hardware of the tracer is different from Peters:
I use switches to connect the Emitter/Source respectively Collector/Drain to +/- supply-voltage only while measuring.
At first tested with mechanical relais, I changed to  PhotoMosRelais because of less space and supply-current( 40ma -> 2-5mA).
The MapleMini-ports can drive the PhotoMosRelais direct.
As amplifier to drive the Emitter/Source ... Collector/Drain I use one amplifier of TCA0372 with gain = +3 .
With the DAC I can genarate 0...12V similar the version from Peter.
As amplfier for the base-current/gate-voltage I use a LT1301 dual amplifier, because i's a r to R and can work up to max. 44V.
The TCA0372 has a normal supply-voltgae from 12.4V, changing to about 26V for measuring zeners with Vz > 12V.
his negative supply is near -3V, generated with a charge-pump.

The LT1301 has a pos. supply-voltage from around 26V, a negative supply from -12v via a ICL7662 charge-pump.
I need the negative supply for the gates from nJFET's and depletion-mode-nMOSFET's, the positive supply > 12V for the gates from 
p-JFET's (I think it exist only one depletion-mode-pMOSFET-chip!).

The gain of the opamp's of the LT1301 can be changed from +6 to -6 with two PhotoMosRelais, or better 
over the relais You can select the opamp with the gain +6 or -6.

For npn/pnp and MosFets the i-DAC here works from 0... 2.047 V to generate with gain = +6  0 ... + 12 V or with gain = -6
0... - 12 V for nJFET's,
for pJFET's the i-DAC works from 2.048 ... 4.095 V to generate with gain = +6 voltage from +12 ... +24 V for
pJFET gate.

The current is limited most to max 50mA . only for jFet's/MosFet's it goes in a second step up to about 100mA.

The software is developed with the ARDUINO IDE 1.8.4, STM32duino is about one year (2017) old, but I think the newest version would work to.

You have to load  Bootloader 2.0 (20k RAM, 120k Flash) in MapleMini.

The main components are:

ILI9341 TFT-SPI-Display 2.8" with Touch ca 6.70 €
MapleMini                                  3.60 €
DACMCP4822                                 3.20 € 
AQY212 6x                                  6.00 €
LT1013                                     1.40 €
TCA0372                                    1.10 €
MT3608 3x                                  2.00 €
Rotary Encoder 2x                          2.00 €
                          SUM:            26.00 €
                                          
+ pcb's, wire, zinn, sockets, battery, chassis ...     


