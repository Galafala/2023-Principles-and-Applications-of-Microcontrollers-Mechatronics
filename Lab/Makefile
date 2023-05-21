#Main application file name

#MAIN_APP = Lab
# MAIN_APP = lineFollower
MAIN_APP = yylun
FOLDER_NAME = Lab

#Main hex file path in windows format
MAIN_HEX_PATH = /Users/ben/Desktop/AVR-code-Microcontrollers-Mechatronics/$(FOLDER_NAME)/$(MAIN_APP).hex

# Compiler and other Section
CC = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-gcc
OBJCOPY = /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avr-objcopy
AVRDUDE := /Applications/Arduino.app/Contents/Java/hardware/tools/avr/bin/avrdude -C /Applications/Arduino.app/Contents/Java/hardware/tools/avr/etc/avrdude.conf

#Options for avr-gcc
CFLAGS = -g -Os -o

#Linking options for avr-gcc
LFLAGS = -Os -mmcu=atmega328p -o

#Options for HEX file generation
HFLAGS = -j .text -j .data -O ihex

#Options for avrdude to burn the hex file
#MMCU model here according to avrdude options
DUDEFLAGS = -c
DUDEFLAGS += avrisp
DUDEFLAGS += -p
DUDEFLAGS += m328p  
DUDEFLAGS += -P 
DUDEFLAGS += /dev/cu.usbmodem14201
DUDEFLAGS += -b 
DUDEFLAGS += 19200 
DUDEFLAGS += -U flash:w:$(MAIN_HEX_PATH):i

# Sources files needed for building the application 
SRC = $(MAIN_APP).cpp
SRC += 

# The headers files needed for building the application
INCLUDE = -I. 
INCLUDE += 

# commands Section
Burn : Build
	$(AVRDUDE) $(DUDEFLAGS)

Build : $(MAIN_APP).elf
	$(OBJCOPY) $(HFLAGS) $< $(MAIN_APP).hex
	
$(MAIN_APP).elf: $(MAIN_APP).o
	$(CC) $(SRC) $(INCLUDE) $(LFLAGS) $@
	
$(MAIN_APP).o:$(SRC)
	$(CC) $^ $(INCLUDE) $(CFLAGS) $@