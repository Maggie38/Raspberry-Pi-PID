import RPi.GPIO as GPIO
import keyboard
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(27, GPIO.OUT)
GPIO.setup(22, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

try:
	while True:
		'''depending on key press, turn left/right/forward/back by sending signals 
		through GPIO pins (alter accordingly based on our raspberry pi wiring)'''
		if keyboard.is_pressed('w'):
			GPIO.output(13, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
			GPIO.output(27, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
		elif keyboard.is_pressed('a'):
			GPIO.output(13, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
			GPIO.output(27, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
		elif keyboard.is_pressed('s'):
			GPIO.output(13, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
			GPIO.output(27, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
		elif keyboard.is_pressed('d'):
			GPIO.output(13, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
			GPIO.output(27, GPIO.HIGH)
			GPIO.output(17, GPIO.HIGH)
except KeyboardInterrupt:
    GPIO.cleanup()