import sys
p = sys.argv[1]
s = open(p).read()
if "hold_frames" in s:
    print("driver.py hat hold schon"); sys.exit(0)
reps = [
("pass_max_x=1.0, pass_cooldown=8):",
 "pass_max_x=1.0, pass_cooldown=8, hold_frames=4):"),
("        self.pass_cooldown = pass_cooldown",
 "        self.hold_frames = hold_frames      # updates to keep steering if the yellow cone drops out briefly\n        self.pass_cooldown = pass_cooldown"),
("        self.since_pass = 999\n",
 "        self.since_pass = 999\n        self.missing = 0\n        self.last_steer = 0.0\n"),
("            self.last_x = x\n            self.state = 'APPROACH'\n            return self.pursuit(x, y + self.side * self.offset)\n",
 "            self.last_x = x\n            self.state = 'APPROACH'\n            self.missing = 0\n            self.last_steer = self.pursuit(x, y + self.side * self.offset)\n            return self.last_steer\n\n        # yellow cone dropped out for a moment (detection flicker) -> keep the last steering\n        if self.last_x is not None and self.missing < self.hold_frames:\n            self.missing += 1\n            return self.last_steer\n"),
("            pass_cooldown=p('pass_cooldown', 8).value)",
 "            pass_cooldown=p('pass_cooldown', 8).value,\n            hold_frames=p('hold_frames', 4).value)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: hold eingebaut")
