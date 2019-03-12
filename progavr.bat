cd avr
avrdude -p m8 -c c232hm -U w:flash:i2c.hex -v
cd ..
