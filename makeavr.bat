@echo off
cd avr
IF [%1]==[] GOTO error

IF "%~1"=="-addr1" (
	echo on
	avr-gcc -mmcu=atmega8 -DSLAVE_ONE -Wall -Os i2c.c -o i2c.elf
	avr-objcopy -O ihex i2c.elf i2c.hex
	@echo off
) 
IF "%~1"=="-addr2" (
	echo on	
	avr-gcc -mmcu=atmega8 -DSLAVE_ONE -Wall -Os i2c.c -o i2c.elf
	avr-objcopy -O ihex i2c.elf i2c.hex
	@echo off
)
cd ..
exit /b


:error
echo No I2C address specified
cd ..
exit /b
