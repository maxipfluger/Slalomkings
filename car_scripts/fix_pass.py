import sys
p = sys.argv[1]
s = open(p).read()
if "pass_cooldown" in s:
    print("driver.py ist schon angepasst"); sys.exit(0)
reps = [
("                 search_steer=0.4, wall_dist=1.0):",
 "                 search_steer=0.4, wall_dist=1.0, pass_max_x=1.0, pass_cooldown=8):"),
("        self.wall_dist = wall_dist          # m, wall cones closer than this count\n",
 "        self.wall_dist = wall_dist          # m, wall cones closer than this count\n        self.pass_max_x = pass_max_x        # m, a cone only counts as passed if it was this close\n        self.pass_cooldown = pass_cooldown  # updates (~11 Hz) before the next pass can count\n"),
("        self.last_x = None\n\n    def start(self):",
 "        self.last_x = None\n        self.since_pass = 999\n\n    def start(self):"),
("        if self.state in ('IDLE', 'DONE'):\n            return None\n",
 "        if self.state in ('IDLE', 'DONE'):\n            return None\n        self.since_pass += 1\n        can_pass = (self.last_x is not None and self.last_x < self.pass_max_x\n                    and self.since_pass >= self.pass_cooldown)\n"),
("            if self.last_x is not None and x > self.last_x + self.new_cone_jump:\n                self.side = -self.side      # old cone passed -> other side for the next one\n                self.passed += 1\n",
 "            if can_pass and x > self.last_x + self.new_cone_jump:\n                self.side = -self.side      # old cone passed -> other side for the next one\n                self.passed += 1\n                self.since_pass = 0\n"),
("        if self.state == 'APPROACH' and self.last_x is not None:\n            self.side = -self.side          # the cone we followed has left the view = passed\n            self.passed += 1\n            self.last_x = None\n",
 "        if self.state == 'APPROACH' and self.last_x is not None:\n            if can_pass:                    # the cone we followed has left the view = passed\n                self.side = -self.side\n                self.passed += 1\n                self.since_pass = 0\n            self.last_x = None\n"),
("            wall_dist=p('wall_dist', 1.0).value)",
 "            wall_dist=p('wall_dist', 1.0).value,\n            pass_max_x=p('pass_max_x', 1.0).value,\n            pass_cooldown=p('pass_cooldown', 8).value)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py angepasst")
