Jan. 1, 2018
SDISK2 LCD mod by Ushiroda

avrdude -c usbasp -p m328p -U fluse:w:0xde:m
avrdude -c usbasp -p m328p -U lfuse:w:0xd9:m
avrdude -c usbasp -p m328p -U efuse:w:0x07:m
avrdude -c usbasp -p m328p -U flash:w:sdisk2.hex
