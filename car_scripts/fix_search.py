import sys
p = sys.argv[1]
s = open(p).read()
if "lost_dir" in s:
    print("driver.py hat search-fix schon"); sys.exit(0)
reps = [
("        self.missing = 0\n        self.last_steer = 0.0\n",
 "        self.missing = 0\n        self.last_steer = 0.0\n        self.last_y = 0.0\n        self.lost_dir = 0              # +1/-1 = cone lost before passing it, it was left/right\n"),
("            self.last_x = x\n            self.state = 'APPROACH'\n            self.missing = 0\n",
 "            self.last_x = x\n            self.last_y = y\n            self.lost_dir = 0\n            self.state = 'APPROACH'\n            self.missing = 0\n"),
("                self.since_pass = 0\n            self.last_x = None",
 "                self.since_pass = 0\n                self.lost_dir = 0\n            else:                           # lost it before passing -> turn back towards it\n                self.lost_dir = 1 if self.last_y > 0 else -1\n            self.last_x = None"),
("        # we are on the -side of the line now -> steer back towards it\n",
 "        if self.lost_dir:                   # cone left the view before we passed it -> look for it again\n            return self.lost_dir * self.search_steer\n        # we are on the -side of the line now -> steer back towards it\n"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: search-fix eingebaut")
