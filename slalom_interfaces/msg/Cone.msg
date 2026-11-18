# Eine erkannte Pylone. Gleicher Typ fuer OpenCV- und KI-Erkennung.

# Farben
uint8 YELLOW=0   # Slalom (Must-have)
uint8 ORANGE=1   # Slalom rechts umfahren (Stretch Goal)
uint8 PINK=2     # Stopp
uint8 RED=3      # linke Wand
uint8 BLUE=4     # rechte Wand

uint8 color
float32 x            # Meter nach vorne, vom Auto aus
float32 y            # Meter nach links (rechts = negativ)
float32 confidence   # 0..1
