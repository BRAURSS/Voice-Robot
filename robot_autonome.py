import speech_recognition as sr
import time
import RPi.GPIO as GPIO
import threading
import re

# Configuration
LANGUAGE = 'both'
MOTOR_SPEED = 80
OBSTACLE_DISTANCE_THRESHOLD = 20  # cm

# Durées de rotation (estimations pour 80% de vitesse)
TURN_90_SEC = 1.0
TURN_45_SEC = 0.5 * TURN_90_SEC
TURN_135_SEC = 1.5 * TURN_90_SEC
TURN_180_SEC = 2.0 * TURN_90_SEC

# Pins
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
        # Laisser le capteur se stabiliser
        time.sleep(0.5)

    def get_distance(self):
        # Envoi impulsion
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        timeout = time.time() + 0.04 # 40ms timeout corresponds to ~6m (max range of HC-SR04 is 4m)
        
        # Attente debut impulsion (Echo = 0)
        start_wait = time.time()
        while GPIO.input(self.echo) == 0:
            if time.time() - start_wait > 0.1: # Timeout si pas de reponse
                return 100 # Retourne une distance "safe" si erreur lecture
        
        start_time = time.time()

        # Attente fin impulsion (Echo = 1)
        while GPIO.input(self.echo) == 1:
            if time.time() - start_time > 0.1: # Timeout
                 return 100
        
        end_time = time.time()

        # Calcul distance
        duration = end_time - start_time
        # Vitesse son = 34300 cm/s
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
        self.is_moving_forward = False  # Etat pour savoir si on doit verifier les obstacles

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
        self.is_moving_forward = False # On considere qu'on tourne sur place
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
        # Note: GPIO.cleanup is called in main/finally block usually, but good practice here too if needed uniquely


def extract_duration(text):
    """
    Extrait une durée en secondes d'un texte.
    Ex: "avancer 5 secondes" -> 5
    """
    # Recherche nombre suivi de 's', 'sec', 'seconde(s)', 'second(s)'
    match = re.search(r'(\d+)\s*(?:sec|s|seconde|second)', text)
    if match:
        return int(match.group(1))
    return None

def execute_single_action(command, robot):
    """
    Exécute une commande simple (sans durée explicite gérée ici).
    Retourne le TYPE d'action détaillé pour traitement ultérieur.
    """
    command = command.lower()

    if any(w in command for w in ["off", "éteindre", "eteindre"]):
        print("🔴 Programme terminé.")
        robot.stop()
        return "EXIT"

    elif any(w in command for w in ["stop", "arrêt", "arrête", "arreter"]):
        robot.stop()
        return "STOP"

    # --- Virages Spécifiques ---

    # Demi-tour (180°)
    if "demi tour" in command or "demi-tour" in command:
        robot.move_left() # Ou droite, peu importe pour un demi-tour sur place
        return "TURN_180"

    # Droite
    if any(w in command for w in ["droite", "right"]):
        robot.move_right()
        if "peu" in command:          # "Tourne un peu à droite"
            return "TURN_RIGHT_45"
        elif "beaucoup" in command:   # "Tourne beaucoup à droite"
            return "TURN_RIGHT_135"
        else:                         # Standard
            return "TURN_RIGHT_90"

    # Gauche
    if any(w in command for w in ["gauche", "left"]):
        robot.move_left()
        if "peu" in command:
            return "TURN_LEFT_45"
        elif "beaucoup" in command:
            return "TURN_LEFT_135"
        else:
            return "TURN_LEFT_90"

    # --- Mouvements ---

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
        # Commande non reconnue
        return "UNKNOWN"


def process_command(full_command, robot):
    if not full_command: return True
    full_command = full_command.lower()
    
    # 1. Découpage en segments (puis, ensuite, then, and, et)
    # On utilise re.split pour couper sur plusieurs séparateurs possibles
    segments = re.split(r'\s+(?:puis|ensuite|apres|then|and|et)\s+', full_command)
    
    print(f"Instruction décomposée : {segments}")

    for i, segment in enumerate(segments):
        print(f"👉 Étape {i+1}/{len(segments)} : '{segment.strip()}'")
        
        # 2. Extraction déla (durée)
        duration = extract_duration(segment)
        
        # 3. Exécution action
        result = execute_single_action(segment, robot)
        
        if result == "EXIT":
            return False
            
        if result == "UNKNOWN":
            print("❓ Etape ignorée (non comprise).")
            continue

        # 4. Gestion de la durée et de l'enchainement
        is_last_step = (i == len(segments) - 1)
        
        # PRIORITÉ 1: Durée explicite ("avancer pendant 5 secondes")
        if duration:
            print(f"   ⏳ Durée explicite : {duration} secondes...")
            time.sleep(duration)
            robot.stop()
        
        # PRIORITÉ 2: Virages Calibrés
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
             # Note: Si c'est la DERNIERE instruction et qu'on veut tourner indéfiniment, 
             # il faudrait dire "turn right forever" mais ici on a décidé que les virages étaient timés par défaut.
             # On garde donc le timing par défaut.
            time.sleep(TURN_90_SEC)
            robot.stop()

        # PRIORITÉ 3: Mouvements (Avancer/Reculer) sans durée explicite
        elif result == "MOVE":
            if not is_last_step:
                default_move_duration = 2.0
                print(f"   ⏳ Mouvement intermédiaire : {default_move_duration}s...")
                time.sleep(default_move_duration)
                robot.stop()
            else:
                print("   🚀 Mouvement continu (dernière étape)")
                # On ne stop pas, on laisse le robot avancer
            
    return True


def monitor_obstacles(robot, sensor, stop_event):
    """
    Fonction tournant dans un thread séparé pour surveiller la distance.
    S'arrête si stop_event est activé.
    """
    print("👀 Surveillance d'obstacles activée...")
    while not stop_event.is_set():
        if robot.is_moving_forward:
            dist = sensor.get_distance()
            # print(f"Dist: {dist}cm") # Debug
            
            if dist < OBSTACLE_DISTANCE_THRESHOLD:
                print(f"\n🛑 OBSTACLE DÉTECTÉ ({dist}cm) ! ARRÊT D'URGENCE.")
                robot.stop()
                # On peut ajouter ici un petit recul automatique si desiré
                
        time.sleep(0.1) # Verification tous les 100ms


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
    
    # Initialisation
    sensor = DistanceSensor(TRIG_PIN, ECHO_PIN)
    robot = Robot()
    
    recognizer = sr.Recognizer()
    recognizer.energy_threshold = 3000
    mic = sr.Microphone()
    
    # Calibration micro
    with mic as source:
        print("Calibration bruit de fond...")
        recognizer.adjust_for_ambient_noise(source, duration=1)
    
    # Thread obstacle
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
            
            # Petite pause pour pas surcharger CPU si boucle rapide
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nArrêt manuel.")

    finally:
        print("Nettoyage et fermeture...")
        stop_thread.set() # Arreter le thread surveillance
        obstacle_thread.join() # Attendre qu'il finisse
        robot.cleanup()
        GPIO.cleanup()
        print("Terminé.")

if __name__ == "__main__":
    main()
