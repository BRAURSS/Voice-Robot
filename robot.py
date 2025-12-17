import speech_recognition as sr
import sys
import time
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

TRIG = 16
ECHO = 18
i = 0

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

GPIO.output(TRIG, False)
print("Calibrating.....")
time.sleep(2)

print("Place the object......")

GPIO_PINS = {
    'IN1': 15,
    'IN2': 18,
    'IN3': 7,
    'IN4': 8,
    'ENA': 14,
    'ENB': 25,
}

MOTOR_SPEED = 60
LANGUAGE = 'both'


class Robot:
    def __init__(self):
        print("Initialisation du GPIO..." if LANGUAGE == 'fr-FR' else "Initializing GPIO...")
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

        print(f"GPIO initialisé. Vitesse par défaut: {MOTOR_SPEED}%" if LANGUAGE == 'fr-FR' 
              else f"GPIO initialized. Default motor speed: {MOTOR_SPEED}%")

    def stop(self):
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_A.ChangeDutyCycle(0)
        self.pwm_B.ChangeDutyCycle(0)
        print("🛑 Commande: ARRÊT" if LANGUAGE == 'fr-FR' else "🛑 Command: STOP")

    def move_forward(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        print("⬆️ Commande: AVANCER" if LANGUAGE == 'fr-FR' else "⬆️ Command: Moving FORWARD")

    def move_backward(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        print("⬇️ Commande: RECULER" if LANGUAGE == 'fr-FR' else "⬇️ Command: Moving BACKWARD")

    def move_left(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        print("⬅️ Commande: TOURNER À GAUCHE" if LANGUAGE == 'fr-FR' else "⬅️ Command: Turning LEFT")

    def move_right(self):
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        print("➡️ Commande: TOURNER À DROITE" if LANGUAGE == 'fr-FR' else "➡️ Command: Turning RIGHT")

    def set_speed(self, speed):
        self.current_speed = max(0, min(100, speed))
        print(f"⚡ Vitesse réglée à: {self.current_speed}%" if LANGUAGE == 'fr-FR' 
              else f"⚡ Speed set to: {self.current_speed}%")

    def cleanup(self):
        self.stop()
        self.pwm_A.stop()
        self.pwm_B.stop()
        GPIO.cleanup()


def process_command(command, robot):
    command = command.lower()

    if any(word in command for word in ["off", "éteindre", "eteindre"]):
        print("🔴 ARRÊT TOTAL DU PROGRAMME!" if LANGUAGE == 'fr-FR' else "🔴 SHUTTING DOWN PROGRAM!")
        robot.stop()
        return False

    elif any(word in command for word in ["stop", "arrêt", "arrête", "arreter"]):
        robot.stop()

    elif any(word in command for word in ["gauche", "left"]):
        robot.move_left()

    elif any(word in command for word in ["droite", "right"]):
        robot.move_right()

    elif any(word in command for word in ["avance", "avancer", "avant", "forward", "straight", "ahead", "go"]):
        robot.move_forward()

    elif any(word in command for word in ["recule", "reculer", "arrière", "back", "backward", "reverse"]):
        robot.move_backward()

    elif any(word in command for word in ["plus vite", "accélère", "accélérer", "faster", "speed up"]):
        robot.set_speed(robot.current_speed + 20)

    elif any(word in command for word in ["moins vite", "ralentir", "ralenti", "slower", "slow down"]):
        robot.set_speed(robot.current_speed - 20)

    else:
        if LANGUAGE == 'fr-FR':
            print("❌ Commande non reconnue. Dites: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite, ÉTEINDRE")
        elif LANGUAGE == 'en-US':
            print("❌ Command not recognized. Say: forward, back, left, right, stop, faster, slower, OFF")
        else:
            print("❌ Commande non reconnue / Command not recognized")
            print("FR: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite, ÉTEINDRE")
            print("EN: forward, back, left, right, stop, faster, slower, OFF")

    return True


def recognize_speech(recognizer, audio):
    if LANGUAGE == 'both':
        try:
            command = recognizer.recognize_google(audio, language='fr-FR')
            print(f"💬 Vous avez dit (FR): '{command}'")
            return command
        except (sr.UnknownValueError, sr.RequestError):
            try:
                command = recognizer.recognize_google(audio, language='en-US')
                print(f"💬 You said (EN): '{command}'")
                return command
            except:
                raise sr.UnknownValueError()

    elif LANGUAGE == 'fr-FR':
        command = recognizer.recognize_google(audio, language='fr-FR')
        print(f"💬 Vous avez dit: '{command}'")
        return command

    else:
        command = recognizer.recognize_google(audio, language='en-US')
        print(f"💬 You said: '{command}'")
        return command


def main():
    recognizer = sr.Recognizer()

    recognizer.energy_threshold = 3000
    recognizer.dynamic_energy_threshold = True
    recognizer.pause_threshold = 0.5
    recognizer.phrase_threshold = 0.2
    recognizer.non_speaking_duration = 0.3

    if LANGUAGE == 'fr-FR':
        print("\n--- Contrôle Vocal Démarré ---")
        print("Commandes: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite")
        print("🔴 Dites 'ÉTEINDRE' pour quitter le programme")
        print("Appuyez sur Ctrl+C pour quitter.\n")
    elif LANGUAGE == 'en-US':
        print("\n--- Voice Control Started ---")
        print("Commands: forward, back, left, right, stop, faster, slower")
        print("🔴 Say 'OFF' to exit the program")
        print("Press Ctrl+C to exit.\n")
    else:
        print("\n--- Contrôle Vocal Démarré / Voice Control Started ---")
        print("Commandes FR: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite")
        print("Commands EN: forward, back, left, right, stop, faster, slower")
        print("🔴 Dites 'ÉTEINDRE' ou 'OFF' pour quitter / Say 'OFF' or 'ÉTEINDRE' to exit")
        print("Appuyez sur Ctrl+C pour quitter / Press Ctrl+C to exit.\n")

    robot = None

    try:
        robot = Robot()

        mic = sr.Microphone()
        with mic as source:
            msg = "Calibration du microphone..." if LANGUAGE == 'fr-FR' else "Calibrating microphone..."
            print(msg)
            recognizer.adjust_for_ambient_noise(source, duration=0.5)
            msg = "✓ Microphone prêt!\n" if LANGUAGE == 'fr-FR' else "✓ Microphone ready!\n"
            print(msg)

        while True:
            with mic as source:
                msg = "🎤 En écoute..." if LANGUAGE == 'fr-FR' else "🎤 Listening..."
                print(msg, end=' ', flush=True)
                
                try:
                    audio = recognizer.listen(source, timeout=2, phrase_time_limit=2)
                    print("✓")
                    
                except sr.WaitTimeoutError:
                    print("⏱️")
                    continue

            try:
                command = recognize_speech(recognizer, audio)
                
                should_continue = process_command(command, robot)
                
                if not should_continue:
                    break

            except sr.UnknownValueError:
                msg = "❓ Non compris" if LANGUAGE == 'fr-FR' else "❓ Not understood"
                print(msg)
            except sr.RequestError as e:
                msg = f"❌ ERREUR: {e}" if LANGUAGE == 'fr-FR' else f"❌ ERROR: {e}"
                print(msg)

            time.sleep(0.05)

    except KeyboardInterrupt:
        msg = "\n\n⚠️ Contrôle vocal arrêté (Ctrl+C)." if LANGUAGE == 'fr-FR' else "\n\n⚠️ Voice control stopped (Ctrl+C)."
        print(msg)
    except Exception as e:
        msg = f"\n❌ Erreur inattendue: {e}" if LANGUAGE == 'fr-FR' else f"\n❌ Unexpected error: {e}"
        print(msg)
        import traceback
        traceback.print_exc()
    finally:
        if robot:
            msg = "Nettoyage..." if LANGUAGE == 'fr-FR' else "Cleaning up..."
            print(msg)
            robot.cleanup()
        msg = "✓ Programme terminé." if LANGUAGE == 'fr-FR' else "✓ Program exited cleanly."
        print(msg)


if __name__ == "__main__":
    main()