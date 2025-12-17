import RPi.GPIO as GPIO
import time

# Pins
IN1 = 15
IN2 = 18
ENA = 14

IN3 = 7
IN4 = 8
ENB = 25

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

pins = [IN1, IN2, ENA, IN3, IN4, ENB]
GPIO.setup(pins, GPIO.OUT)
GPIO.output(pins, GPIO.LOW)

pwm_a = GPIO.PWM(ENA, 1000)
pwm_b = GPIO.PWM(ENB, 1000)

pwm_a.start(0)
pwm_b.start(0)

SPEED = 80

try:
    print("--- TEST DIAGNOSTIQUE MOTEURS ---")
    
    print("\n1. Test Moteur A (Gauche ?)")
    print(f"Activation ENA (Pin {ENA}) et IN1/IN2...")
    pwm_a.ChangeDutyCycle(SPEED)
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    time.sleep(2)
    print("Arrêt Moteur A")
    GPIO.output(IN1, GPIO.LOW)
    pwm_a.ChangeDutyCycle(0)
    
    time.sleep(1)
    
    print("\n2. Test Moteur B (Droit ?)")
    print(f"Activation ENB (Pin {ENB}) et IN3/IN4...")
    pwm_b.ChangeDutyCycle(SPEED)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    time.sleep(2)
    print("Arrêt Moteur B")
    GPIO.output(IN3, GPIO.LOW)
    pwm_b.ChangeDutyCycle(0)
    
    print("\nFIN DU TEST")

except KeyboardInterrupt:
    print("\nArrêt manuel")

finally:
    pwm_a.stop()
    pwm_b.stop()
    GPIO.cleanup()
