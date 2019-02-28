import RPi.GPIO as GPIO

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.IN, pull_up_down=GPIO.PUD_UP)

input("Press Enter when ready \n")
print("Thomas is dumb \n")
try:
    GPIO.wait_for_edge(40, GPIO.FALLING)
    print("\nFalling edge detected. Now your program can continue with")
    print("whatever was waiting for a button press.")
except KeyboardInterrupt:
    GPIO.cleanup()
GPIO.cleanup()

#while True:
#    if GPIO.input(40)== GPIO.HIGH:
#        print("Button was pushed!")
#    else:
#        print("Waiting! Also chloe is the best") 
