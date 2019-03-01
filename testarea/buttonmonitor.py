import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

while True:
    if GPIO.input(40)== GPIO.HIGH:
        print("Button was pushed!")
    else:
        print("Waiting! Also chloe is the best") 
