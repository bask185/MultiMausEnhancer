#!/usr/bin/env python
import os
os.system("python src/build.py")
print("UPLOADING")
# os.system("arduino-cli upload -b arduino:avr:nano -p COM3 -i ./build/arduino.avr.nano/MultiMausEnhancer.ino.hex")
os.system("arduino-cli upload -b arduino:avr:uno -p COM4 -i ./build/arduino.avr.uno/MultiMausEnhancer.ino.hex")
exit