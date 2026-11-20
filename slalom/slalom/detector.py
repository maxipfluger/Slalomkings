"""K1: OpenCV-Pylonenerkennung -> /cones/opencv
ROS:     ros2 run slalom detector
Offline: python3 detector.py ~/k1bilder   (zeigt die erkannten Pylonen auf den Bildern)
Werte und Hintergrund: Projekt-Doku k1-opencv-detektor.md
"""
import glob
import math
import os
import sys

import cv2
import numpy as np

# (H min, H max, S min, S max, V min, V max). Stand v6 (20.11.).
# H min > H max heisst: Bereich geht ueber 179 -> 0 (Rot).
# Farbnummern wie in Cone.msg: 0 yellow, 1 orange, 2 pink, 3 red, 4 blue
FARBEN = {
    0: ("yellow", (18, 38, 60, 255, 100, 255)),
    2: ("pink", (140, 171, 60, 255, 70, 255)),
    3: ("red", (172, 8, 120, 255, 70, 255)),
    4: ("blue", (85, 115, 120, 255, 60, 255)),
}
ZEICHENFARBE = {0: (0, 255, 255), 1: (0, 140, 255), 2: (203, 102, 255), 3: (0, 0, 255), 4: (255, 128, 0)}

HFOV = 69.0        # horizontaler Bildwinkel der Kamera in Grad (OAK-D, ggf. pruefen)
HOEHE = 0.17       # echte Pylonenhoehe in m (gemessen 20.11.)
HORIZONT = 0.20    # Unterkante oberhalb dieses Bildanteils (von oben) -> nicht auf dem Boden -> ignorieren
RAND = 3           # Box naeher als so viele Pixel am Bildrand -> abgeschnitten -> ignorieren
MINFLAECHE = 300.0 # kleinere Flecken ignorieren (Pixel im Vollbild 1280x800)
MAXDIST = 3.0      # weiter weg -> ignorieren


def maske(hsv, b):
    h1, h2, s1, s2, v1, v2 = b
    if h1 <= h2:
        m = cv2.inRange(hsv, (h1, s1, v1), (h2, s2, v2))
    else:
        m = cv2.inRange(hsv, (h1, s1, v1), (179, s2, v2)) | cv2.inRange(hsv, (0, s1, v1), (h2, s2, v2))
    kern = np.ones((5, 5), np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, kern)   # kleine Stoerpunkte weg
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kern)  # Loecher (Lichtreflexe) schliessen
    return m


def trennstellen(oben, unten):
    """Spalten, an denen zwei Pylonenspitzen durch ein Tal getrennt sind
    (oben = oberste, unten = unterste belegte Zeile je Spalte).
    Ein Tal zaehlt nur, wenn es tief und schmal genug ist
    (echte Pylonen haben steile Flanken, eine Delle im Klebeband nicht)."""
    # 1) alle Spitzen (Minima) und Taeler (Maxima) mit kleiner Schwelle sammeln
    spitzen, taeler = [], []
    spitzeX = 0
    imTal = False
    talX = 0
    for c in range(1, len(oben)):
        v = oben[c]
        if not imTal:
            if v < oben[spitzeX]:
                spitzeX = c
            elif v - oben[spitzeX] > 3:
                spitzen.append(spitzeX)
                imTal, talX = True, c
        else:
            if v > oben[talX]:
                talX = c
            elif oben[talX] - v > 3:
                taeler.append(talX)
                imTal, spitzeX = False, c
    if not imTal:
        spitzen.append(spitzeX)
    taeler = taeler[:len(spitzen) - 1]
    # 2) nur Taeler behalten, die im Verhaeltnis zum Abstand ihrer beiden Nachbarspitzen tief genug sind
    behalten = []
    for i, t in enumerate(taeler):
        l, r = spitzen[i], spitzen[i + 1]
        tiefe = min(oben[t] - oben[l], oben[t] - oben[r])
        # Breite des Tals auf halber Tiefe: echte Pylonen haben steile Flanken -> schmales, tiefes Tal.
        # Eine Delle im Klebeband ist dagegen flach und breit.
        halb = oben[t] - tiefe / 2
        lRand, rRand = t, t
        while lRand > l and oben[lRand] > halb:
            lRand -= 1
        while rRand < r and oben[rRand] > halb:
            rRand += 1
        # ausserdem muss das Tal im Verhaeltnis zur Pylonenhoehe nennenswert sein (kein Klebeband-Zipfel)
        pylHoehe = min(unten[l] - oben[l], unten[r] - oben[r])
        if tiefe >= max(3.0, 0.5 * (rRand - lRand), 0.08 * pylHoehe):
            behalten.append(t)
    taeler = behalten
    return taeler


def zerlege(m, x, y, w, h):
    """Ein Farbfleck kann mehrere ueberlappende Pylonen enthalten (Wandreihe).
    Zerlegt ihn an den Taelern zwischen den Spitzen. Gibt Boxen (x, y, w, h, flaeche) zurueck."""
    roi = m[y:y + h, x:x + w] > 0
    belegt = roi.any(axis=0)
    oben = np.where(belegt, roi.argmax(axis=0), h).astype(float)
    if w >= 5:
        oben = np.convolve(np.pad(oben, 2, mode="edge"), np.ones(5) / 5, mode="valid")
    unten = np.where(belegt, h - 1 - roi[::-1].argmax(axis=0), 0)
    schnitte = trennstellen(oben, unten)
    if not schnitte:
        return [(x, y, w, h, int(roi.sum()))]   # einzelne Pylone: ganze Box
    grenzen = [0] + schnitte + [w]
    boxen = []
    for a, b in zip(grenzen[:-1], grenzen[1:]):
        if b - a < 2 or not belegt[a:b].any():
            continue
        spalte = a + int(np.argmin(oben[a:b]))   # Spalte der Spitze
        spitze = int(roi[:, spalte].argmax())
        boden = int(unten[spalte])               # unterste Farbe unter der Spitze
        if boden <= spitze:
            continue
        zeilen = roi[spitze:boden + 1, a:b]
        spalten = np.where(zeilen.any(axis=0))[0]
        bx0, bx1 = a + spalten[0], a + spalten[-1]
        boxen.append((x + bx0, y + spitze, bx1 - bx0 + 1, boden - spitze + 1, int(zeilen.sum())))
    return boxen


def beruehrt(a, b, rand=0.4):
    """True, wenn Box a die um 'rand' vergroesserte Box b ueberlappt. Boxen: (farbe, x, y, w, h)."""
    _, ax, ay, aw, ah = a
    _, bx, by, bw, bh = b
    bx0, by0 = bx - rand * bw, by - rand * bh
    bx1, by1 = bx + bw + rand * bw, by + bh + rand * bh
    return ax < bx1 and ax + aw > bx0 and ay < by1 and ay + ah > by0


def finde(bild, minFlaeche=MINFLAECHE, horizont=HORIZONT):
    """Liste (farbe, x, y, w, h) in Pixeln."""
    bildHoehe, bildBreite = bild.shape[:2]
    hsv = cv2.cvtColor(bild, cv2.COLOR_BGR2HSV)
    treffer = []
    for farbe, (name, b) in FARBEN.items():
        m = maske(hsv, b)
        konturen = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        for k in konturen:
            if cv2.contourArea(k) < minFlaeche:
                continue
            teile = zerlege(m, *cv2.boundingRect(k))
            # einzelner Fleck: Pfosten/Tuerrahmen sind viel hoeher als breit (> 4x).
            # zerlegte Reihe: Teilstuecke sind schmaler (teils verdeckt), daher grosszuegiger
            maxVerhaeltnis = 4.0 if len(teile) == 1 else 6.0
            for x, y, w, h, flaeche in teile:
                if flaeche < minFlaeche:
                    continue
                if h < 0.6 * w:                    # flacher Fleck (Boden, Tape)
                    continue
                if h > maxVerhaeltnis * w:
                    continue
                if y + h < horizont * bildHoehe:   # steht nicht auf dem Boden
                    continue
                if x <= RAND or x + w >= bildBreite - RAND:
                    continue                       # links/rechts abgeschnitten -> Abstand falsch
                # unten abgeschnitten (Pylone direkt vor dem Auto) wird behalten: die echte Pylone ist
                # dann hoeher als sichtbar, der Abstand also hoechstens so gross wie berechnet.
                # Fuer den Stopp vor Pink / T0 ist das die sichere Seite.
                treffer.append((farbe, x, y, w, h))
    # Pink und Rot an derselben Stelle: das kleinere Stueck gehoert zum groesseren.
    #  - roter Fuss unter dem pinken Klebeband -> keine rote Wand-Pylone melden
    #  - rosa Lichtreflex auf einer roten Pylone -> keine pinke Stopp-Pylone melden
    def flaeche(t):
        return t[3] * t[4]
    pinke = [t for t in treffer if t[0] == 2]
    rote = [t for t in treffer if t[0] == 3]
    weg = set()
    for p in pinke:
        for r in rote:
            if beruehrt(r, p) or beruehrt(p, r):
                weg.add(r if flaeche(r) < flaeche(p) else p)
    return [t for t in treffer if t not in weg]


def position(x, w, h, bildBreite, hfovGrad, hoehe):
    """Pixel-Box -> (x nach vorne, y nach links) in m."""
    f = (bildBreite / 2) / math.tan(math.radians(hfovGrad) / 2)
    vorne = f * hoehe / h
    links = vorne * (bildBreite / 2 - (x + w / 2)) / f
    return vorne, links


def offline(ordner):
    bilder = sorted(glob.glob(os.path.join(os.path.expanduser(ordner), "*.jpg")))
    if not bilder:
        print("Keine Bilder gefunden in", ordner)
        return
    print(len(bilder), "Bilder. Ende: im Terminal Strg+C")
    fenster = "K1 Detektor"
    cv2.namedWindow(fenster, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(fenster, 960, 660)
    cv2.createTrackbar("Bild", fenster, 0, len(bilder) - 1, lambda v: None)
    letztesBild = None
    try:
        while True:
            i = cv2.getTrackbarPos("Bild", fenster)
            if i < 0:
                break
            bild = cv2.imread(bilder[i])
            treffer = []
            for t in finde(bild):
                px, py = position(t[1], t[3], t[4], bild.shape[1], HFOV, HOEHE)
                if px <= MAXDIST:
                    treffer.append((t, px, py))
            if i != letztesBild:
                print(f"Bild {i}: " + ", ".join(f"{FARBEN[t[0]][0]} ({px:.2f}, {py:+.2f})"
                                                for t, px, py in treffer), flush=True)
                letztesBild = i
            hy = int(HORIZONT * bild.shape[0])
            cv2.line(bild, (0, hy), (bild.shape[1], hy), (255, 255, 255), 1)
            for (farbe, x, y, w, h), px, py in treffer:
                c = ZEICHENFARBE[farbe]
                cv2.rectangle(bild, (x, y), (x + w, y + h), c, 3)
                cv2.putText(bild, f"{px:.2f}", (x, max(20, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, c, 2)
            cv2.putText(bild, f"Bild {i}", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
            cv2.imshow(fenster, cv2.resize(bild, (960, 600)))
            cv2.waitKey(50)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from slalom_interfaces.msg import Cone, ConeArray

    class Detector(Node):
        def __init__(self):
            super().__init__("detector")
            self.declare_parameter("image_topic", "/camera/color/image_raw")
            self.declare_parameter("cones_topic", "/cones/opencv")
            self.declare_parameter("scale", 0.5)        # Bild verkleinern -> schneller auf dem Pi
            self.declare_parameter("hfov_deg", HFOV)
            self.declare_parameter("cone_height", HOEHE)
            self.declare_parameter("horizon", HORIZONT)
            self.declare_parameter("max_dist", MAXDIST)
            self.pub = self.create_publisher(ConeArray, self.get_parameter("cones_topic").value, 10)
            self.create_subscription(Image, self.get_parameter("image_topic").value,
                                     self.neuesBild, qos_profile_sensor_data)
            self.zaehler = 0
            self.encodingGewarnt = False
            self.get_logger().info("K1 detector laeuft -> " + self.get_parameter("cones_topic").value)

        def neuesBild(self, msg):
            if msg.encoding not in ("bgr8", "rgb8"):
                if not self.encodingGewarnt:
                    self.get_logger().warn("Unbekanntes Encoding: " + msg.encoding)
                    self.encodingGewarnt = True
                return
            roh = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            bild = roh[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                bild = cv2.cvtColor(bild, cv2.COLOR_RGB2BGR)
            s = float(self.get_parameter("scale").value)
            if s != 1.0:
                bild = cv2.resize(bild, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            hfov = float(self.get_parameter("hfov_deg").value)
            hoehe = float(self.get_parameter("cone_height").value)
            horizont = float(self.get_parameter("horizon").value)
            maxDist = float(self.get_parameter("max_dist").value)

            aus = ConeArray()
            aus.header = msg.header
            for farbe, x, y, w, h in finde(bild, MINFLAECHE * s * s, horizont):
                px, py = position(x, w, h, bild.shape[1], hfov, hoehe)
                if px > maxDist:
                    continue
                c = Cone()
                c.color = farbe
                c.x = float(px)
                c.y = float(py)
                c.confidence = 1.0
                aus.cones.append(c)
            self.pub.publish(aus)

            self.zaehler += 1
            if self.zaehler % 50 == 0:
                text = ", ".join(f"{FARBEN[c.color][0]} ({c.x:.2f}, {c.y:+.2f})" for c in aus.cones)
                self.get_logger().info(f"{len(aus.cones)} Pylonen: {text}")

    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    if len(sys.argv) > 1 and not sys.argv[1].startswith("--"):
        offline(sys.argv[1])
    else:
        main()
