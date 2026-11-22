import sys
p = sys.argv[1]
s = open(p).read()
if "clear_frames" in s:
    print("driver.py hat clear schon"); sys.exit(0)
reps = [
("hold_frames=4, ignore_frames=12):",
 "hold_frames=4, ignore_frames=12, clear_frames=25, pink_hold=15):"),
("        self.hold_frames = hold_frames",
 "        self.clear_frames = clear_frames    # updates to drive straight past the last yellow cone before turning to pink\n        self.pink_hold = pink_hold          # updates to wait for a flickering yellow cone before turning to pink\n        self.hold_frames = hold_frames"),
("        self.missing = 0\n        self.last_steer = 0.0\n",
 "        self.missing = 0\n        self.last_steer = 0.0\n        self.clear = 0\n        self.pink_recent = 0\n"),
("        pink = [(x, y) for c, x, y in cones if c == Cone.PINK and x > 0.0]\n",
 "        pink = [(x, y) for c, x, y in cones if c == Cone.PINK and x > 0.0]\n        self.pink_recent = 30 if pink else max(0, self.pink_recent - 1)\n"),
("        if pink:                            # last yellow passed -> drive to the pink cone\n            px, py = min(pink)\n",
 "        if pink:                            # last yellow passed -> drive to the pink cone\n            if self.last_x is not None and self.last_x < self.pass_max_x:\n                # the last yellow cone just left the view -> count it and first drive straight past it\n                self.side = -self.side\n                self.passed += 1\n                self.since_pass = 0\n                self.last_x = None\n                self.clear = self.clear_frames\n            if self.last_x is not None and self.missing < self.pink_hold:\n                # yellow cone still ahead but flickering -> keep going for it, do not turn to pink yet\n                self.missing += 1\n                return self.last_steer\n            if self.clear > 0:\n                self.clear -= 1\n                self.state = 'APPROACH'\n                return 0.0\n            px, py = min(pink)\n"),
("                self.since_pass = 0\n                self.lost_dir = 0\n",
 "                self.since_pass = 0\n                self.lost_dir = 0\n                if self.pink_recent > 0:     # last yellow cone (pink ahead) is still beside us -> first drive straight past it\n                    self.clear = self.clear_frames\n"),
("        if self.lost_dir:                   # cone left the view before we passed it -> look for it again\n",
 "        if self.clear > 0:                  # just passed a cone that is still beside us -> straight, do not cut into it\n            self.clear -= 1\n            return 0.0\n        if self.lost_dir:                   # cone left the view before we passed it -> look for it again\n"),
("            ignore_frames=p('ignore_frames', 12).value)",
 "            ignore_frames=p('ignore_frames', 12).value,\n            clear_frames=p('clear_frames', 25).value,\n            pink_hold=p('pink_hold', 15).value)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: clear eingebaut")
