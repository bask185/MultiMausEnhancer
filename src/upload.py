#!/usr/bin/env python
import os
os.system("python src/build.py")
print("UPLOADING")
os.system("arduino-cli upload -b arduino:avr:nano:cpu=atmega328old -p COM7 -i ./build/arduino.avr.nano/MultiMausEnhancer.ino.hex")
exit