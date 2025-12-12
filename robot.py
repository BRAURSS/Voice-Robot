import speech_recognition as sr
import sys
import time
import RPi.GPIO as GPIO

GPIO_PINS = {
    'IN1': 17,
    'IN2': 18,
    'IN3': 27,
    'IN4': 22,
    'ENA': 12,
    'ENB': 13,
}

MOTOR_SPEED = 60

LANGUAGE = 'both'  # Change to 'fr-FR' or 'en--US'

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
        """Stop all motors"""
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.LOW)
        self.pwm_A.ChangeDutyCycle(0)
        self.pwm_B.ChangeDutyCycle(0)
        print("🛑 Commande: ARRÊT" if LANGUAGE == 'fr-FR' else "🛑 Command: STOP")

    def move_forward(self):
        """Move robot forward"""
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        print("⬆️ Commande: AVANCER" if LANGUAGE == 'fr-FR' else "⬆️ Command: Moving FORWARD")

    def move_backward(self):
        """Move robot backward"""
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        print("⬇️ Commande: RECULER" if LANGUAGE == 'fr-FR' else "⬇️ Command: Moving BACKWARD")

    def move_left(self):
        """Turn robot left"""
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.HIGH)
        GPIO.output(self.IN3, GPIO.HIGH)
        GPIO.output(self.IN4, GPIO.LOW)
        print("⬅️ Commande: TOURNER À GAUCHE" if LANGUAGE == 'fr-FR' else "⬅️ Command: Turning LEFT")

    def move_right(self):
        """Turn robot right"""
        self.pwm_A.ChangeDutyCycle(self.current_speed)
        self.pwm_B.ChangeDutyCycle(self.current_speed)
        
        GPIO.output(self.IN1, GPIO.HIGH)
        GPIO.output(self.IN2, GPIO.LOW)
        GPIO.output(self.IN3, GPIO.LOW)
        GPIO.output(self.IN4, GPIO.HIGH)
        print("➡️ Commande: TOURNER À DROITE" if LANGUAGE == 'fr-FR' else "➡️ Command: Turning RIGHT")
    
    def set_speed(self, speed):
        """Change motor speed (0-100)"""
        self.current_speed = max(0, min(100, speed))
        print(f"⚡ Vitesse réglée à: {self.current_speed}%" if LANGUAGE == 'fr-FR' 
              else f"⚡ Speed set to: {self.current_speed}%")

    def cleanup(self):
        """Clean shutdown of GPIO"""
        self.stop()
        self.pwm_A.stop()
        self.pwm_B.stop()
        GPIO.cleanup()

def process_command(command, robot):
    """Process voice command in French or English"""
    command = command.lower()
    
    if any(word in command for word in ["stop", "arrêt", "arrête", "arreter", "halt", "whoa"]):
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
            print("❌ Commande non reconnue. Dites: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite")
        elif LANGUAGE == 'en-US':
            print("❌ Command not recognized. Say: forward, back, left, right, stop, faster, slower")
        else:
            print("❌ Commande non reconnue / Command not recognized")
            print("FR: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite")
            print("EN: forward, back, left, right, stop, faster, slower")

def recognize_speech(recognizer, audio):
    """Try to recognize speech in multiple languages"""
    
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
    
    recognizer.energy_threshold = 4000
    recognizer.dynamic_energy_threshold = True
    recognizer.pause_threshold = 0.8
    
    if LANGUAGE == 'fr-FR':
        print("\n--- Contrôle Vocal Démarré ---")
        print("Commandes disponibles: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite")
        print("Appuyez sur Ctrl+C pour quitter.\n")
    elif LANGUAGE == 'en-US':
        print("\n--- Voice Control Started ---")
        print("Available Commands: forward, back, left, right, stop, faster, slower")
        print("Press Ctrl+C to exit.\n")
    else:  # both
        print("\n--- Contrôle Vocal Démarré / Voice Control Started ---")
        print("Commandes FR: avancer, reculer, gauche, droite, arrêt, plus vite, moins vite")
        print("Commands EN: forward, back, left, right, stop, faster, slower")
        print("Appuyez sur Ctrl+C pour quitter / Press Ctrl+C to exit.\n")
    
    robot = None
    
    try:
        robot = Robot()
        
        mic = sr.Microphone()
        with mic as source:
            msg = "Calibration du microphone..." if LANGUAGE == 'fr-FR' else "Calibrating microphone..."
            print(msg)
            recognizer.adjust_for_ambient_noise(source, duration=1)
            msg = "✓ Microphone prêt!\n" if LANGUAGE == 'fr-FR' else "✓ Microphone ready!\n"
            print(msg)
        
        while True:
            with mic as source:
                msg = "🎤 En écoute..." if LANGUAGE == 'fr-FR' else "🎤 Listening..."
                print(msg)
                try:
                    audio = recognizer.listen(source, timeout=3, phrase_time_limit=3)
                except sr.WaitTimeoutError:
                    msg = "⏱️ Aucun son détecté" if LANGUAGE == 'fr-FR' else "⏱️ No speech detected"
                    print(msg)
                    continue

            try:
                command = recognize_speech(recognizer, audio)
                process_command(command, robot)
                
            except sr.UnknownValueError:
                msg = "❓ Audio non compris. Parlez clairement." if LANGUAGE == 'fr-FR' else "❓ Could not understand audio."
                print(msg)
            except sr.RequestError as e:
                msg = f"❌ ERREUR de service de reconnaissance vocale: {e}" if LANGUAGE == 'fr-FR' else f"❌ ERROR: Speech recognition service error: {e}"
                print(msg)
                msg = "⚠️ Vérifiez votre connexion internet." if LANGUAGE == 'fr-FR' else "⚠️ Check your internet connection."
                print(msg)
            
            time.sleep(0.1)

    except KeyboardInterrupt:
        msg = "\n\n⚠️ Contrôle vocal arrêté." if LANGUAGE == 'fr-FR' else "\n\n⚠️ Voice control stopped."
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
