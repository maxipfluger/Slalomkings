import sys
p = sys.argv[1]
s = open(p).read()
if "ignore_frames" in s:
    print("driver.py hat confirm schon"); sys.exit(0)
reps = [
("pass_cooldown=8, hold_frames=4):",
 "pass_cooldown=8, hold_frames=4, ignore_frames=12):"),
("        self.hold_frames = hold_frames",
 "        self.ignore_frames = ignore_frames  # updates after a pass in which close cones (the passed one) are ignored\n        self.hold_frames = hold_frames"),
("        pink = [(x, y) for c, x, y in cones if c == Cone.PINK and x > 0.0]\n",
 "        pink = [(x, y) for c, x, y in cones if c == Cone.PINK and x > 0.0]\n        if self.since_pass < self.ignore_frames:   # just passed -> do not chase the passed cone again\n            yellow = [(x, y) for x, y in yellow if x > self.pass_max_x]\n"),
("            if can_pass and x > self.last_x + self.new_cone_jump:\n                self.side",
 "            if self.last_x is not None and x > self.last_x + self.new_cone_jump and self.missing < self.hold_frames:\n                # near cone vanished, only a far one is seen -> could be flicker, confirm over a few updates\n                self.missing += 1\n                return self.last_steer\n            if can_pass and x > self.last_x + self.new_cone_jump:\n                self.side"),
("            hold_frames=p('hold_frames', 4).value)",
 "            hold_frames=p('hold_frames', 4).value,\n            ignore_frames=p('ignore_frames', 12).value)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: confirm eingebaut")
