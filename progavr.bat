cd avr
avrdude -c c232hm -p m8 -U flash:w:i2c.hex -v
cd ..
