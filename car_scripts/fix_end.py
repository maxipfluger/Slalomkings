import sys
p = sys.argv[1]
s = open(p).read()
if "pink_lost_stop" in s:
    print("driver.py hat pink-stop schon"); sys.exit(0)
reps = [
("clear_frames=25, pink_hold=15):",
 "clear_frames=25, pink_hold=15, pink_lost_stop=1.0):"),
("        self.clear_frames = clear_frames",
 "        self.pink_lost_stop = pink_lost_stop  # m, pink seen closer than this and then lost (too close for the camera) -> stop\n        self.clear_frames = clear_frames"),
("        self.clear = 0\n        self.pink_recent = 0\n",
 "        self.clear = 0\n        self.pink_recent = 0\n        self.end_phase = False\n        self.pink_close = 0\n"),
("        self.pink_recent = 30 if pink else max(0, self.pink_recent - 1)\n",
 "        self.pink_recent = 30 if pink else max(0, self.pink_recent - 1)\n        if pink and min(pink)[0] < self.pink_lost_stop:\n            self.pink_close = 3             # pink is close ahead\n        elif not pink and self.pink_close > 0:\n            self.pink_close -= 1\n            if self.pink_close == 0 and self.end_phase:   # close pink vanished (below the camera view) -> we are there\n                self.state = 'DONE'\n                return None\n"),
("                self.last_x = None\n                self.clear = self.clear_frames\n",
 "                self.last_x = None\n                self.clear = self.clear_frames\n                self.end_phase = True\n"),
("                if self.pink_recent > 0:     # last yellow cone (pink ahead) is still beside us -> first drive straight past it\n                    self.clear = self.clear_frames\n",
 "                if self.pink_recent > 0:     # last yellow cone (pink ahead) is still beside us -> first drive straight past it\n                    self.clear = self.clear_frames\n                    self.end_phase = True\n"),
("            pink_hold=p('pink_hold', 15).value)",
 "            pink_hold=p('pink_hold', 15).value,\n            pink_lost_stop=p('pink_lost_stop', 1.0).value)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: pink-stop eingebaut")
