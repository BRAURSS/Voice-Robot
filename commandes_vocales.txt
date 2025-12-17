# GUIDE DES COMMANDES VOCALES - ROBOT AUTONOME

Ce fichier référence l'ensemble des commandes vocales supportées par le Robot v2 et v3.

================================================================================
PARTIE 1 : COMMANDES COMMUNES (V2 et V3)
================================================================================
Toutes ces commandes fonctionnent sur les deux versions.

1. MOUVEMENTS DE BASE
---------------------
- "Avance" / "Avancer" / "Go"      : Fait avancer le robot (en continu ou 2s si enchaîné).
- "Recule" / "Reculer" / "Back"    : Fait reculer le robot.
- "Stop" / "Arrêt"                 : Arrête tout mouvement immédiatement.

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
PARTIE 2 : COMMANDES EXCLUSIVES (V3 UNIQUEMENT)
================================================================================
Ces commandes ne fonctionnent que sur le script `robot_autonome_v3.py`.

1. MODE AUTOMATIQUE
-------------------
- "Patrouille"                     : Active le mode exploration autonome.
- "Mode automatique"               : (Synonyme)
- "Patrol"                         : (Synonyme)

> EN MODE PATROUILLE : Le robot avance tout seul. S'il voit un mur, il recule et change de direction sans votre aide.

2. ARRÊTER LA PATROUILLE
------------------------
- "Stop"                           : Désactive le mode patrouille et remet le robot en attente d'ordres (Mode Manuel).
