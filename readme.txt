Jan. 1, 2018
SDISK2 LCD mod by Ushiroda

avrdude -c usbasp -p m328p -B 5 -U flash:w:sdisk2.hex
avrdude -c usbasp -p m328p -B 5 -U hfuse:w:0xd9:m
avrdude -c usbasp -p m328p -B 5 -U lfuse:w:0xde:m
avrdude -c usbasp -p m328p -B 5 -U efuse:w:0x07:m

avrdode -c usbasp -p m328p -U lfuse:r:con:h