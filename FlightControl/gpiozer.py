from gpiozero import LED
from time import sleep

led = LED(17)

while True:
    input("Press enter to turn on")
    led.on()
    input("Press enter to turn off")
    led.off()


#while True:
#    print("on")
#    led.on()
#    sleep(1)
#    print("off")
#    led.off()
#    sleep(1)
