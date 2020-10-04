# TinySonar - Ultrasonic Range Finder based on ATtiny13 and HC-SR04
The ultrasonic ranging module HC-SR04 provides 2cm-400cm non-contact measurement function, the ranging accuracy can reach to 3mm. The module includes ultrasonic transmitter, receiver and control circuit.

The basic principle of work:
- start measurement by applying high signal on TRIG for at least 10us,
- module automatically sends eight 40 kHz pulses and detects echos,
- module applies high signal on ECHO with the same duration as the time it took from sending ultrasonic to returning.

For measuring the duration of the ECHO signal the timer0 is used with no prescaler, so it runs at MCU clock speed. Timer overflow interrupt is utilized to expand the 8-bit timer/counter to a virtual 16-bit counter. To calculate the distance in mm from the counter value, the following approximation formula was derived:
-  duration[s]  = counter / F_CPU = counter / 1200000[1/s]
-  distance[mm] = duration * speed of sound / 2 = duration[s] * 340260[mm/s] / 2
-  distance[mm] = counter * 340260[mm/s] / 1200000[1/s] / 2
-  distance[mm] ~ counter / 7

The distance in mm is shown on a 4-digit 7-segment display which is controlled via a MAX7219 in BCD decode mode. The SPI protocol is implemented with a simple bitbanging method. The MAX7219 is fast enough to be driven without delays even at the fastest clock speed of the ATtiny13. A transmission to the MAX7219 starts with pulling the CS line LOW. Afterwards two bytes are transmitted, the register address first and the register data second. The bytes are transmitted most significant bit first by setting the DIN line HIGH for a bit "1" or LOW for a bit "0" while the CLK line is LOW. The bit is shifted out on the rising edge of the CLK line. By setting the CS line HIGH again the end of the transmission is signified, the MAX7219 latches the two received bytes and writes the data byte into the register. 
