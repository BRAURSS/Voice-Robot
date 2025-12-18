# GUIDE DES COMMANDES VOCALES - ROBOT AUTONOME

================================================================================
PARTIE 1 : COMMANDES COMMUNES (V2 et V3)
================================================================================

1. MOUVEMENTS DE BASE
---------------------
- "Avance" / "Avancer" / "Go"      : Fait avancer le robot (en continu ou 2s si enchaîné).
- "Recule" / "Reculer" / "Back"    : Fait reculer le robot.
- "Stop" / "Arrêt" / "Arrêter"     : Arrête tout mouvement immédiatement.

2. VIRAGES PRÉCIS
-----------------
- "Tourne à droite"                : Virage à 90° vers la droite.
- "Tourne un peu à droite"         : Virage léger à 45° vers la droite.
- "Tourne beaucoup à droite"       : Grand virage à 135° vers la droite.
- "Fais demi-tour"                 : Le robot se retourne complètement (180°).
*(Fonctionne à l'identique pour la "Gauche")*

3. CONTRÔLE DE VITESSE
----------------------
- "Vitesse 50"                     : Règle la vitesse à 50% précisément.
- "Vitesse max" / "À fond"         : Vitesse maximale (100%).
- "Vitesse moyenne"                : Vitesse 50%.
- "Vitesse lente" / "Min"          : Vitesse 30%.
- "Plus vite"                      : Augmente la vitesse actuelle de 20%.
- "Moins vite"                     : Diminue la vitesse actuelle de 20%.

4. MINUTEURS ET SÉQUENCES
-------------------------
- "Avance pendant 5 secondes"      : Avance pendant une durée précise puis s'arrête.
- "Avance puis tourne à droite"    : Exécute une action après l'autre.
- "Vitesse max ensuite recule"     : Change la vitesse d'abord, puis recule.

================================================================================
PARTIE 2 : COMMANDES EXCLUSIVES
================================================================================

1. MODE AUTOMATIQUE
-------------------
- "Patrouille"                     : Active le mode exploration autonome.
- "Mode automatique"               : (Synonyme)
- "Patrol"                         : (Synonyme)

> EN MODE PATROUILLE : Le robot avance tout seul. S'il voit un mur, il recule et change de direction sans votre aide.

2. ARRÊTER LA PATROUILLE
------------------------
- "Stop" / "Arrêt" / "Arrêter"       : Désactive le mode patrouille et remet le robot en attente d'ordres (Mode Manuel).

================================================================================
PART 3: ENGLISH COMMANDS
================================================================================
These commands work in 'robot_autonome_v3.py'.
*NOTE*: By default, the robot listens in French (unless configured otherwise). 
Speak English clearly or change the configuration.

1. BASIC MOVEMENT
-----------------
- "Forward" / "Move" / "Go"        : Move forward.
- "Backward" / "Back" / "Reverse"  : Move backward.
- "Stop" / "Halt" / "Pause"        : Stop all movement.

2. PRECISION TURNS
------------------
- "Turn right"                     : Turn 90° right.
- "Turn a little right"            : Turn 45° right.
- "Turn a lot right"               : Turn 135° right.
- "U-Turn" / "Turn around"         : Turn 180°.
* Same logic applies for "Left".

3. SPEED CONTROL
----------------
- "Speed 50"                       : Set speed to 50% precisely.
- "Max speed" / "Full speed"       : Set speed to 100%.
- "Medium speed" / "Average"       : Set speed to 50%.
- "Low speed" / "Slow"             : Set speed to 40% (minimum).
- "Faster"                         : Increase speed by 20%.
- "Slower"                         : Decrease speed by 20%.

4. AUTONOMOUS MODE
------------------
- "Patrol" / "Autonomous" / "Auto" : Activate obstacle avoidance mode.
- "Stop"                           : Exit patrol mode.

================================================================================
PART 4: CONFIGURATION
================================================================================
To change the language or speed settings, edit 'robot_autonome_v3.py':

- LANGUAGE = 'fr-FR'  : Robot listens in French only.
- LANGUAGE = 'en_EN'  : Robot listens in English (US).
- LANGUAGE = 'both'   : Robot listens in French (default), but understands English keywords.

