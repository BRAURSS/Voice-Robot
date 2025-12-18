import speech_recognition as sr
import time
import RPi.GPIO as GPIO
import threading
import re

LANGUAGE = 'both'
MOTOR_SPEED = 80
OBSTACLE_DISTANCE_THRESHOLD = 20

TURN_90_SEC = 1.0
TURN_45_SEC = 0.5 * TURN_90_SEC
TURN_135_SEC = 1.5 * TURN_90_SEC
TURN_180_SEC = 2.0 * TURN_90_SEC

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

        print(f"Robot prêt. Vitesse: {MOTOR_SPEED}%")

    def stop(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_A.ChangeDutyCycle(0)
        self.pwm_B.ChangeDutyCycle(0)
        self.is_moving_forward = False
        print("🛑 STOP")

    def move_forward(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.is_moving_forward = True
        print("⬆️ AVANCER")

    def move_backward(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.is_moving_forward = False
        print("⬇️ RECULER")

    def move_left(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        self.is_moving_forward = False
        print("⬅️ GAUCHE")

    def move_right(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        self.is_moving_forward = False
        print("➡️ DROITE")

    def set_speed(self, speed):
        self.current_speed = max(0, min(100, speed))
        print(f"⚡ Vitesse: {self.current_speed}%")

    def cleanup(self):
        self.stop()
        self.pwm_A.stop()
        self.pwm_B.stop()

def extract_duration(text):

    match = re.search(r'(\d+)\s*(?:sec|s|seconde|second)', text)
    if match:
        return int(match.group(1))
    return None

def extract_speed_value(text):
    text = text.lower()

    if any(w in text for w in ["max", "maximum", "fond"]):
        return 100
    if any(w in text for w in ["moyen", "moyenne", "medium"]):
        return 50
    if any(w in text for w in ["min", "minimum", "lente"]):
        return 30

    match = re.search(r'(?:vitesse|speed)\s*(?:de\s*)?(\d+)', text)
    if match:
        return int(match.group(1))

    return None

def execute_single_action(command, robot):
    command = command.lower()

    if any(w in command for w in ["off", "éteindre", "eteindre"]):
        print("🔴 Programme terminé.")
        robot.stop()
        return "EXIT"

    elif any(w in command for w in ["stop", "arrêt", "arrête", "arreter"]):
        robot.stop()
        return "STOP"

    speed_val = extract_speed_value(command)
    if speed_val is not None:
        robot.set_speed(speed_val)
        return "SPEED"

    if "demi tour" in command or "demi-tour" in command:
        robot.move_left()
        return "TURN_180"

    if any(w in command for w in ["droite", "right"]):
        robot.move_right()
        if "peu" in command:
            return "TURN_RIGHT_45"
        elif "beaucoup" in command:
            return "TURN_RIGHT_135"
        else:
            return "TURN_RIGHT_90"

    if any(w in command for w in ["gauche", "left"]):
        robot.move_left()
        if "peu" in command:
            return "TURN_LEFT_45"
        elif "beaucoup" in command:
            return "TURN_LEFT_135"
        else:
            return "TURN_LEFT_90"

    elif any(w in command for w in ["avance", "avancer", "avant", "go"]):
        robot.move_forward()
        return "MOVE"

    elif any(w in command for w in ["recule", "reculer", "arrière", "back"]):
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

    print(f"Instruction décomposée : {segments}")

    for i, segment in enumerate(segments):
        print(f"👉 Étape {i+1}/{len(segments)} : '{segment.strip()}'")

        duration = extract_duration(segment)

        result = execute_single_action(segment, robot)

        if result == "EXIT":
            return False

        if result == "UNKNOWN":
            print("❓ Etape ignorée (non comprise).")
            continue

        is_last_step = (i == len(segments) - 1)

        if duration:
            print(f"   ⏳ Durée explicite : {duration} secondes...")
            time.sleep(duration)
            robot.stop()

        elif result == "TURN_180":
            print(f"   🔄 Demi-tour (180°) : {TURN_180_SEC}s")
            time.sleep(TURN_180_SEC)
            robot.stop()

        elif result == "TURN_RIGHT_45" or result == "TURN_LEFT_45":
            print(f"   🔄 Virage 45° : {TURN_45_SEC}s")
            time.sleep(TURN_45_SEC)
            robot.stop()

        elif result == "TURN_RIGHT_135" or result == "TURN_LEFT_135":
            print(f"   🔄 Virage 135° : {TURN_135_SEC}s")
            time.sleep(TURN_135_SEC)
            robot.stop()

        elif result == "TURN_RIGHT_90" or result == "TURN_LEFT_90":
            print(f"   🔄 Virage 90° : {TURN_90_SEC}s")

            time.sleep(TURN_90_SEC)
            robot.stop()

        elif result == "MOVE":
            if not is_last_step:
                default_move_duration = 2.0
                print(f"   ⏳ Mouvement intermédiaire : {default_move_duration}s...")
                time.sleep(default_move_duration)
                robot.stop()
            else:
                print("   🚀 Mouvement continu (dernière étape)")

    return True

def monitor_obstacles(robot, sensor, stop_event):
    print("👀 Surveillance d'obstacles activée...")
    while not stop_event.is_set():
        if robot.is_moving_forward:
            dist = sensor.get_distance()

            if dist < OBSTACLE_DISTANCE_THRESHOLD:
                print(f"\n🛑 OBSTACLE DÉTECTÉ ({dist}cm) ! ARRÊT D'URGENCE.")
                robot.stop()

        time.sleep(0.1)

def recognize_speech(recognizer, mic):
    with mic as source:
        print("🎤 En écoute...", end=' ', flush=True)
        try:
            audio = recognizer.listen(source, timeout=2, phrase_time_limit=2)
            print("✓ Traitement...")
            return recognizer.recognize_google(audio, language='fr-FR')
        except sr.WaitTimeoutError:
            print(".")
            return None
        except sr.UnknownValueError:
            print("?")
            return None
        except sr.RequestError:
            print("Erreur Connexion")
            return None

def main():
    GPIO.setmode(GPIO.BCM)

    sensor = DistanceSensor(TRIG_PIN, ECHO_PIN)
    robot = Robot()

    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 3000
    mic = sr.Microphone()

    with mic as source:
        print("Calibration bruit de fond...")
        recognizer.adjust_for_ambient_noise(source, duration=1)

    stop_thread = threading.Event()
    obstacle_thread = threading.Thread(target=monitor_obstacles, args=(robot, sensor, stop_thread))
    obstacle_thread.start()

    print("\n--- ROBOT AUTONOME DÉMARRÉ ---")
    print(f"Les obstacles à moins de {OBSTACLE_DISTANCE_THRESHOLD}cm arrêteront le robot.")

    try:
        while True:
            command = recognize_speech(recognizer, mic)
            if command:
                print(f"Reçu: '{command}'")
                running = process_command(command, robot)
                if not running:
                    break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArrêt manuel.")

    finally:
        print("Nettoyage et fermeture...")
        stop_thread.set()
        obstacle_thread.join()
        robot.cleanup()
        GPIO.cleanup()
        print("Terminé.")

if __name__ == "__main__":
    main()
