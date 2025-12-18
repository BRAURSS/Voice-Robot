import speech_recognition as sr
import time
import RPi.GPIO as GPIO
import threading
import re
import random

LANGUAGE = 'both'
MOTOR_SPEED = 80
OBSTACLE_DISTANCE_THRESHOLD = 20

TURN_90_SEC = 1.0
TURN_45_SEC = 0.5 * TURN_90_SEC
TURN_135_SEC = 1.5 * TURN_90_SEC
TURN_180_SEC = 1.7 * TURN_90_SEC

GPIO_PINS = {
    'IN1': 15,
    'IN2': 18,
    'IN3': 7,
    'IN4': 8,
    'ENA': 14,
    'ENB': 25,
}

TRIG_PIN = 23
ECHO_PIN = 24

class DistanceSensor:
    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.5)

    def get_distance(self):

        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        timeout = time.time() + 0.04

        start_wait = time.time()
        while GPIO.input(self.echo) == 0:
            if time.time() - start_wait > 0.1:
                return 100

        start_time = time.time()

        while GPIO.input(self.echo) == 1:
            if time.time() - start_time > 0.1:
                 return 100

        end_time = time.time()

        duration = end_time - start_time
        distance = (duration * 34300) / 2
        return round(distance, 2)

class Robot:
    def __init__(self):
        print("Initialisation du Robot..." if LANGUAGE == 'fr-FR' else "Initializing Robot...")
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.IN1, self.IN2 = GPIO_PINS['IN1'], GPIO_PINS['IN2']
        self.IN3, self.IN4 = GPIO_PINS['IN3'], GPIO_PINS['IN4']
        self.ENA, self.ENB = GPIO_PINS['ENA'], GPIO_PINS['ENB']

        all_pins = [self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB]
        GPIO.setup(all_pins, GPIO.OUT)
        GPIO.output(all_pins, GPIO.LOW)

        self.pwm_A = GPIO.PWM(self.ENA, 1000)
        self.pwm_B = GPIO.PWM(self.ENB, 1000)
        self.pwm_A.start(0)
        self.pwm_B.start(0)

        self.current_speed = MOTOR_SPEED
        self.is_moving_forward = False
        self.is_moving = False

        self.mode = 'MANUAL'

        print(f"Robot prêt. Vitesse: {MOTOR_SPEED}%")

    def stop_motors(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_A.ChangeDutyCycle(0)
        self.pwm_B.ChangeDutyCycle(0)
        self.is_moving_forward = False
        self.is_moving = False

    def stop(self):
        self.stop_motors()
        self.mode = 'MANUAL'
        print("🛑 STOP (Mode Manuel)")

    def move_forward(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.is_moving_forward = True
        self.is_moving = True
        print("⬆️ AVANCER")

    def move_backward(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.is_moving_forward = False
        self.is_moving = True
        print("⬇️ RECULER")

    def move_left(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.is_moving_forward = False
        self.is_moving = True
        print("⬅️ GAUCHE")

    def move_right(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.is_moving_forward = False
        self.is_moving = True
        print("➡️ DROITE")

    def set_speed(self, speed):
        self.current_speed = max(40, min(100, speed))
        print(f"⚡ Vitesse: {self.current_speed}%")
        if self.is_moving:
            self.pwm_A.ChangeDutyCycle(self.current_speed)
            self.pwm_B.ChangeDutyCycle(self.current_speed)

    def cleanup(self):
        self.stop()
        self.pwm_A.stop()
        self.pwm_B.stop()

def extract_duration(text):
    match = re.search(r'(\d+)\s*(?:sec|s|seconde|second)', text)
    if match: return int(match.group(1))
    return None

def extract_speed_value(text):
    text = text.lower()
    if any(w in text for w in ["max", "maximum", "fond", "full"]): return 100
    if any(w in text for w in ["moyen", "moyenne", "medium", "average"]): return 50
    if any(w in text for w in ["min", "minimum", "lente", "slow", "low"]): return 40 # Updated to 40 as per min speed fix
    match = re.search(r'(?:vitesse|speed|velocity)\s*(?:de\s*)?(\d+)', text)
    if match: return int(match.group(1))
    return None

def execute_single_action(command, robot):
    command = command.lower()

    if any(w in command for w in ["off", "éteindre", "eteindre", "shutdown", "exit", "quit"]):
        print("🔴 Programme terminé.")
        robot.stop()
        return "EXIT"

    elif any(w in command for w in ["stop", "arrêt", "arrête", "arreter", "halt", "pause"]):
        robot.stop()
        return "STOP"

    if any(w in command for w in ["patrouille", "patrol", "automatique", "auto", "autonomous"]):
        print("\n🤖 ACTIVATION MODE PATROUILLE 🤖")
        robot.mode = 'PATROL'
        robot.move_forward()
        return "PATROL"

    speed_val = extract_speed_value(command)
    if speed_val is not None:
        robot.set_speed(speed_val)
        return "SPEED"

    if "demi tour" in command or "demi-tour" in command or "u-turn" in command or "turn around" in command:
        robot.move_left()
        return "TURN_180"

    if any(w in command for w in ["droite", "right"]):
        robot.move_right()
        if any(w in command for w in ["peu", "little", "bit", "small"]): return "TURN_RIGHT_45"
        elif any(w in command for w in ["beaucoup", "lot", "much", "grand"]): return "TURN_RIGHT_135"
        else: return "TURN_RIGHT_90"

    if any(w in command for w in ["gauche", "left"]):
        robot.move_left()
        if any(w in command for w in ["peu", "little", "bit", "small"]): return "TURN_LEFT_45"
        elif any(w in command for w in ["beaucoup", "lot", "much", "grand"]): return "TURN_LEFT_135"
        else: return "TURN_LEFT_90"

    elif any(w in command for w in ["avance", "avancer", "avant", "go", "forward", "move", "advance"]):
        robot.move_forward()
        return "MOVE"
    elif any(w in command for w in ["recule", "reculer", "arrière", "back", "backward", "reverse"]):
        robot.move_backward()
        return "MOVE"
    elif any(w in command for w in ["plus vite", "accélère", "faster"]):
        robot.set_speed(robot.current_speed + 20)
        return "SPEED"
    elif any(w in command for w in ["moins vite", "ralentir", "slower"]):
        robot.set_speed(robot.current_speed - 20)
        return "SPEED"
    else:
        return "UNKNOWN"

def process_command(full_command, robot):
    if not full_command: return True
    full_command = full_command.lower()

    segments = re.split(r'\s+(?:puis|ensuite|apres|then|and|et)\s+', full_command)

    print(f"Instruction : {segments}")

    for i, segment in enumerate(segments):
        duration = extract_duration(segment)
        result = execute_single_action(segment, robot)

        if result == "EXIT": return False
        if result == "UNKNOWN": continue

        if result == "PATROL":

            break

        is_last_step = (i == len(segments) - 1)

        if duration:
            print(f"   ⏳ Durée : {duration}s")
            time.sleep(duration)
            robot.stop()

        elif result == "TURN_180":
            time.sleep(TURN_180_SEC)
            robot.stop()
        elif "TURN" in result:

             sec = TURN_90_SEC
             if "45" in result: sec = TURN_45_SEC
             if "135" in result: sec = TURN_135_SEC
             time.sleep(sec)
             robot.stop()

        elif result == "MOVE" and not is_last_step:
            time.sleep(2.0)
            robot.stop()

    return True

def monitor_obstacles(robot, sensor, stop_event):
    print("👀 Surveillance d'obstacles activée...")
    while not stop_event.is_set():
        if robot.is_moving_forward:
            dist = sensor.get_distance()

            if dist < OBSTACLE_DISTANCE_THRESHOLD:
                print(f"\n🧱 OBSTACLE ({dist}cm) !")

                if robot.mode == 'MANUAL':

                    print("🛑 ARRÊT D'URGENCE.")
                    robot.stop()

                elif robot.mode == 'PATROL':

                    print("🔄 Évitement en cours...")

                    robot.stop_motors()
                    robot.is_moving_forward = False

                    robot.move_backward()

                    time.sleep(1.0)
                    if robot.mode != 'PATROL': break

                    robot.stop_motors()
                    time.sleep(0.5)

                    turn_time = TURN_90_SEC

                    print("   ↩️ Virage Droit")
                    robot.move_right()

                    if robot.mode != 'PATROL': break
                    time.sleep(turn_time)
                    if robot.mode != 'PATROL': break

                    print("   🚀 Reprise Patrouille")

                    if robot.mode == 'PATROL':
                        robot.move_forward()
                    else:
                        print("   ⚠️ Mode changé (STOP reçu ?), on reste à l'arrêt.")

        time.sleep(0.1)

def recognize_speech(recognizer, mic):
    with mic as source:

        print("🎤", end='', flush=True)
        try:
            audio = recognizer.listen(source, timeout=2, phrase_time_limit=2)
            print(".", end='', flush=True)
            lang_code = 'fr-FR'
            if LANGUAGE == 'en_EN':
                lang_code = 'en-US'
            return recognizer.recognize_google(audio, language=lang_code)

        except sr.UnknownValueError:
            return None
        except sr.RequestError:
            return None
        except sr.WaitTimeoutError:
            return None

def main():
    GPIO.setmode(GPIO.BCM)
    sensor = DistanceSensor(TRIG_PIN, ECHO_PIN)
    robot = Robot()

    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 3000
    mic = sr.Microphone()

    with mic as source:
        print("Calibration...")
        recognizer.adjust_for_ambient_noise(source, duration=1)

    stop_thread = threading.Event()
    obstacle_thread = threading.Thread(target=monitor_obstacles, args=(robot, sensor, stop_thread))
    obstacle_thread.start()

    print("\n--- ROBOT v3 (PATROL EDITION) ---")
    print("Dites 'PATROUILLE' pour lancer le mode autonome. 'STOP' pour arrêter.")

    try:
        while True:
            command = recognize_speech(recognizer, mic)
            if command:
                print(f"\n🗣️ '{command}'")
                running = process_command(command, robot)
                if not running:
                    break

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nArrêt manuel.")

    finally:
        stop_thread.set()
        obstacle_thread.join()
        robot.cleanup()
        GPIO.cleanup()
        print("Terminé.")

if __name__ == "__main__":
    main()
