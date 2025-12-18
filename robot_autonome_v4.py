#!/usr/bin/env python3

from typing import Optional, Tuple, Literal
from dataclasses import dataclass
from enum import Enum
import speech_recognition as sr
import time
import RPi.GPIO as GPIO
import threading
import re
import random
import logging

@dataclass(frozen=True)
class RobotConfig:

    motor_speed: int = 80
    pwm_frequency: int = 1000

    obstacle_threshold_cm: int = 20
    sensor_timeout_sec: float = 0.1
    sensor_stabilization_sec: float = 0.5

    turn_90_sec: float = 1.0
    turn_45_sec: float = 0.5
    turn_135_sec: float = 1.5
    turn_180_sec: float = 2.0

    language: str = 'fr-FR'
    energy_threshold: int = 3000
    listen_timeout_sec: int = 2
    phrase_time_limit_sec: int = 2

    obstacle_check_interval_sec: float = 0.1
    main_loop_interval_sec: float = 0.05

@dataclass(frozen=True)
class PinConfig:

    in1: int = 15
    in2: int = 18
    in3: int = 7
    in4: int = 8
    ena: int = 14
    enb: int = 25

    trig: int = 23
    echo: int = 24

class RobotMode(Enum):
    MANUAL = "MANUAL"
    PATROL = "PATROL"

class ActionType(Enum):
    EXIT = "EXIT"
    STOP = "STOP"
    PATROL = "PATROL"
    SPEED = "SPEED"
    TURN_180 = "TURN_180"
    TURN_RIGHT_45 = "TURN_RIGHT_45"
    TURN_RIGHT_90 = "TURN_RIGHT_90"
    TURN_RIGHT_135 = "TURN_RIGHT_135"
    TURN_LEFT_45 = "TURN_LEFT_45"
    TURN_LEFT_90 = "TURN_LEFT_90"
    TURN_LEFT_135 = "TURN_LEFT_135"
    MOVE = "MOVE"
    UNKNOWN = "UNKNOWN"

def setup_logging() -> logging.Logger:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        datefmt='%H:%M:%S'
    )
    return logging.getLogger(__name__)

logger = setup_logging()

class DistanceSensor:

    def __init__(self, trig_pin: int, echo_pin: int, config: RobotConfig):
        self.trig = trig_pin
        self.echo = echo_pin
        self.config = config

        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)

        time.sleep(config.sensor_stabilization_sec)
        logger.info("✓ Distance sensor initialized")

    def get_distance(self) -> float:
        try:

            GPIO.output(self.trig, True)
            time.sleep(0.00001)
            GPIO.output(self.trig, False)

            start_wait = time.time()
            while GPIO.input(self.echo) == 0:
                if time.time() - start_wait > self.config.sensor_timeout_sec:
                    logger.warning("Sensor timeout waiting for echo start")
                    return 100.0

            pulse_start = time.time()

            while GPIO.input(self.echo) == 1:
                if time.time() - pulse_start > self.config.sensor_timeout_sec:
                    logger.warning("Sensor timeout waiting for echo end")
                    return 100.0

            pulse_end = time.time()

            duration = pulse_end - pulse_start
            distance = (duration * 34300) / 2

            return round(distance, 2)

        except Exception as e:
            logger.error(f"Error reading sensor: {e}")
            return 100.0

class Robot:

    def __init__(self, pin_config: PinConfig, config: RobotConfig):
        logger.info("Initializing robot controller...")

        self.pins = pin_config
        self.config = config
        self._mode_lock = threading.Lock()
        self._mode = RobotMode.MANUAL

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        all_pins = [
            self.pins.in1, self.pins.in2,
            self.pins.in3, self.pins.in4,
            self.pins.ena, self.pins.enb
        ]
        GPIO.setup(all_pins, GPIO.OUT)
        GPIO.output(all_pins, GPIO.LOW)

        self.pwm_a = GPIO.PWM(self.pins.ena, config.pwm_frequency)
        self.pwm_b = GPIO.PWM(self.pins.enb, config.pwm_frequency)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

        self.current_speed = config.motor_speed
        self.is_moving_forward = False

        logger.info(f"✓ Robot ready - Speed: {self.current_speed}%")

    @property
    def mode(self) -> RobotMode:
        with self._mode_lock:
            return self._mode

    @mode.setter
    def mode(self, value: RobotMode) -> None:
        with self._mode_lock:
            self._mode = value

    def stop(self) -> None:
        GPIO.output(self.pins.in1, GPIO.LOW)
        GPIO.output(self.pins.in2, GPIO.LOW)
        GPIO.output(self.pins.in3, GPIO.LOW)
        GPIO.output(self.pins.in4, GPIO.LOW)
        self.pwm_a.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)

        self.is_moving_forward = False
        self.mode = RobotMode.MANUAL

        logger.info("🛑 STOP (Mode Manuel)")

    def move_forward(self) -> None:
        self.pwm_a.ChangeDutyCycle(self.current_speed)
        self.pwm_b.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.pins.in1, GPIO.HIGH)
        GPIO.output(self.pins.in2, GPIO.LOW)
        GPIO.output(self.pins.in3, GPIO.HIGH)
        GPIO.output(self.pins.in4, GPIO.LOW)

        self.is_moving_forward = True
        logger.info("⬆️  AVANCER")

    def move_backward(self) -> None:
        self.pwm_a.ChangeDutyCycle(self.current_speed)
        self.pwm_b.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.pins.in1, GPIO.LOW)
        GPIO.output(self.pins.in2, GPIO.HIGH)
        GPIO.output(self.pins.in3, GPIO.LOW)
        GPIO.output(self.pins.in4, GPIO.HIGH)

        self.is_moving_forward = False
        logger.info("⬇️  RECULER")

    def move_left(self) -> None:
        self.pwm_a.ChangeDutyCycle(self.current_speed)
        self.pwm_b.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.pins.in1, GPIO.LOW)
        GPIO.output(self.pins.in2, GPIO.HIGH)
        GPIO.output(self.pins.in3, GPIO.HIGH)
        GPIO.output(self.pins.in4, GPIO.LOW)

        self.is_moving_forward = False
        logger.info("⬅️  GAUCHE")

    def move_right(self) -> None:
        self.pwm_a.ChangeDutyCycle(self.current_speed)
        self.pwm_b.ChangeDutyCycle(self.current_speed)

        GPIO.output(self.pins.in1, GPIO.HIGH)
        GPIO.output(self.pins.in2, GPIO.LOW)
        GPIO.output(self.pins.in3, GPIO.LOW)
        GPIO.output(self.pins.in4, GPIO.HIGH)

        self.is_moving_forward = False
        logger.info("➡️  DROITE")

    def set_speed(self, speed: int) -> None:
        self.current_speed = max(0, min(100, speed))
        logger.info(f"⚡ Vitesse: {self.current_speed}%")

    def cleanup(self) -> None:
        logger.info("Cleaning up robot resources...")
        self.stop()
        self.pwm_a.stop()
        self.pwm_b.stop()

class CommandParser:

    @staticmethod
    def extract_duration(text: str) -> Optional[int]:
        match = re.search(r'(\d+)\s*(?:sec|s|seconde|second)', text, re.IGNORECASE)
        return int(match.group(1)) if match else None

    @staticmethod
    def extract_speed_value(text: str) -> Optional[int]:
        text = text.lower()

        if any(word in text for word in ["max", "maximum", "fond"]):
            return 100
        if any(word in text for word in ["moyen", "moyenne", "medium"]):
            return 50
        if any(word in text for word in ["min", "minimum", "lente"]):
            return 30

        match = re.search(r'(?:vitesse|speed)\s*(?:de\s*)?(\d+)', text)
        return int(match.group(1)) if match else None

    @staticmethod
    def split_compound_command(command: str) -> list[str]:
        return re.split(
            r'\s+(?:puis|ensuite|apres|then|and|et)\s+',
            command.lower()
        )

class CommandExecutor:

    def __init__(self, robot: Robot):
        self.robot = robot
        self.parser = CommandParser()

    def execute_single_action(self, command: str) -> ActionType:
        command = command.lower()

        if any(word in command for word in ["off", "éteindre", "eteindre"]):
            logger.info("🔴 Programme terminé")
            self.robot.stop()
            return ActionType.EXIT

        if any(word in command for word in ["stop", "arrêt", "arrête", "arreter"]):
            self.robot.stop()
            return ActionType.STOP

        if any(word in command for word in ["patrouille", "patrol", "automatique", "auto"]):
            logger.info("\n🤖 ACTIVATION MODE PATROUILLE 🤖")
            self.robot.mode = RobotMode.PATROL
            self.robot.move_forward()
            return ActionType.PATROL

        speed_val = self.parser.extract_speed_value(command)
        if speed_val is not None:
            self.robot.set_speed(speed_val)
            return ActionType.SPEED

        if "demi tour" in command or "demi-tour" in command:
            self.robot.move_left()
            return ActionType.TURN_180

        if any(word in command for word in ["droite", "right"]):
            self.robot.move_right()
            if "peu" in command:
                return ActionType.TURN_RIGHT_45
            elif "beaucoup" in command:
                return ActionType.TURN_RIGHT_135
            else:
                return ActionType.TURN_RIGHT_90

        if any(word in command for word in ["gauche", "left"]):
            self.robot.move_left()
            if "peu" in command:
                return ActionType.TURN_LEFT_45
            elif "beaucoup" in command:
                return ActionType.TURN_LEFT_135
            else:
                return ActionType.TURN_LEFT_90

        if any(word in command for word in ["avance", "avancer", "avant", "go"]):
            self.robot.move_forward()
            return ActionType.MOVE

        if any(word in command for word in ["recule", "reculer", "arrière", "back"]):
            self.robot.move_backward()
            return ActionType.MOVE

        if any(word in command for word in ["plus vite", "accélère", "faster"]):
            self.robot.set_speed(self.robot.current_speed + 20)
            return ActionType.SPEED

        if any(word in command for word in ["moins vite", "ralentir", "slower"]):
            self.robot.set_speed(self.robot.current_speed - 20)
            return ActionType.SPEED

        return ActionType.UNKNOWN

    def process_command(self, full_command: str) -> bool:
        if not full_command:
            return True

        segments = self.parser.split_compound_command(full_command)
        logger.info(f"📝 Instruction: {segments}")

        for i, segment in enumerate(segments):
            duration = self.parser.extract_duration(segment)
            action = self.execute_single_action(segment)

            if action == ActionType.EXIT:
                return False

            if action == ActionType.UNKNOWN:
                continue

            if action == ActionType.PATROL:
                break

            is_last_step = (i == len(segments) - 1)

            self._handle_action_timing(action, duration, is_last_step)

        return True

    def _handle_action_timing(
        self,
        action: ActionType,
        duration: Optional[int],
        is_last_step: bool
    ) -> None:
        config = self.robot.config

        if duration:
            logger.info(f"   ⏳ Durée: {duration}s")
            time.sleep(duration)
            self.robot.stop()
            return

        turn_durations = {
            ActionType.TURN_180: config.turn_180_sec,
            ActionType.TURN_RIGHT_45: config.turn_45_sec,
            ActionType.TURN_LEFT_45: config.turn_45_sec,
            ActionType.TURN_RIGHT_90: config.turn_90_sec,
            ActionType.TURN_LEFT_90: config.turn_90_sec,
            ActionType.TURN_RIGHT_135: config.turn_135_sec,
            ActionType.TURN_LEFT_135: config.turn_135_sec,
        }

        if action in turn_durations:
            time.sleep(turn_durations[action])
            self.robot.stop()
            return

        if action == ActionType.MOVE and not is_last_step:
            default_duration = 2.0
            logger.info(f"   ⏳ Mouvement intermédiaire: {default_duration}s")
            time.sleep(default_duration)
            self.robot.stop()

class ObstacleMonitor:

    def __init__(
        self,
        robot: Robot,
        sensor: DistanceSensor,
        config: RobotConfig
    ):
        self.robot = robot
        self.sensor = sensor
        self.config = config
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        logger.info("👀 Démarrage surveillance d'obstacles...")
        self.thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        logger.info("Arrêt surveillance d'obstacles...")
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)

    def _monitoring_loop(self) -> None:
        while not self.stop_event.is_set():
            if self.robot.is_moving_forward:
                distance = self.sensor.get_distance()

                if distance < self.config.obstacle_threshold_cm:
                    self._handle_obstacle(distance)

            time.sleep(self.config.obstacle_check_interval_sec)

    def _handle_obstacle(self, distance: float) -> None:
        logger.warning(f"\n🧱 OBSTACLE ({distance}cm)!")

        if self.robot.mode == RobotMode.MANUAL:
            logger.warning("🛑 ARRÊT D'URGENCE")
            self.robot.stop()

        elif self.robot.mode == RobotMode.PATROL:
            self._perform_avoidance_maneuver()

    def _perform_avoidance_maneuver(self) -> None:
        logger.info("🔄 Évitement en cours...")

        self.robot.stop()
        self.robot.is_moving_forward = False

        self.robot.move_backward()
        time.sleep(0.5)

        if self.robot.mode != RobotMode.PATROL:
            logger.info("   ⚠️  Évitement interrompu (STOP reçu)")
            return

        turn_direction = random.choice(['left', 'right'])
        turn_duration = random.uniform(0.5, 1.0)

        if turn_direction == 'left':
            logger.info("   ↪️  Virage Gauche")
            self.robot.move_left()
        else:
            logger.info("   ↩️  Virage Droit")
            self.robot.move_right()

        if self.robot.mode != RobotMode.PATROL:
            logger.info("   ⚠️  Évitement interrompu (STOP reçu)")
            return

        time.sleep(turn_duration)

        if self.robot.mode != RobotMode.PATROL:
            logger.info("   ⚠️  Évitement interrompu (STOP reçu)")
            return

        logger.info("   🚀 Reprise Patrouille")
        if self.robot.mode == RobotMode.PATROL:
            self.robot.move_forward()
        else:
            logger.info("   ⚠️  Mode changé (STOP reçu?), on reste à l'arrêt")

class VoiceListener:

    def __init__(self, config: RobotConfig):
        self.config = config
        self.recognizer = sr.Recognizer()
        self.recognizer.energy_threshold = config.energy_threshold
        self.microphone = sr.Microphone()

        logger.info("🎙️  Calibration du microphone...")
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
        logger.info("✓ Microphone calibré")

    def listen(self) -> Optional[str]:
        with self.microphone as source:
            print("🎤", end='', flush=True)

            try:
                audio = self.recognizer.listen(
                    source,
                    timeout=self.config.listen_timeout_sec,
                    phrase_time_limit=self.config.phrase_time_limit_sec
                )
                print(".", end='', flush=True)

                text = self.recognizer.recognize_google(
                    audio,
                    language=self.config.language
                )
                return text

            except sr.WaitTimeoutError:

                return None

            except sr.UnknownValueError:

                print("?", end='', flush=True)
                return None

            except sr.RequestError as e:
                logger.error(f"Erreur de connexion au service de reconnaissance: {e}")
                return None

def main() -> None:

    config = RobotConfig()
    pin_config = PinConfig()

    GPIO.setmode(GPIO.BCM)

    sensor = DistanceSensor(pin_config.trig, pin_config.echo, config)
    robot = Robot(pin_config, config)
    executor = CommandExecutor(robot)
    monitor = ObstacleMonitor(robot, sensor, config)
    voice = VoiceListener(config)

    monitor.start()

    logger.info("\n" + "="*50)
    logger.info("🤖 ROBOT AUTONOME V3 - CLAUDE EDITION 🤖")
    logger.info("="*50)
    logger.info(f"Seuil obstacle: {config.obstacle_threshold_cm}cm")
    logger.info("Commandes:")
    logger.info("  - 'PATROUILLE' : Mode autonome")
    logger.info("  - 'STOP' : Arrêt")
    logger.info("  - 'OFF' : Quitter")
    logger.info("  - Ctrl+C : Arrêt d'urgence")
    logger.info("="*50 + "\n")

    try:

        while True:
            command = voice.listen()

            if command:
                logger.info(f"\n🗣️  '{command}'")
                should_continue = executor.process_command(command)

                if not should_continue:
                    break

            time.sleep(config.main_loop_interval_sec)

    except KeyboardInterrupt:
        logger.info("\n⚠️  Arrêt manuel via Ctrl+C")

    except Exception as e:
        logger.error(f"\n❌ Erreur inattendue: {e}", exc_info=True)

    finally:

        logger.info("\n🔧 Nettoyage et fermeture...")
        monitor.stop()
        robot.cleanup()
        GPIO.cleanup()
        logger.info("✓ Terminé proprement")

if __name__ == "__main__":
    main()
