import sys
p = sys.argv[1]
s = open(p).read()
if "min_lookahead" in s:
    print("driver.py hat smooth schon"); sys.exit(0)
reps = [
("pink_hold=15, pink_lost_stop=1.0):",
 "pink_hold=15, pink_lost_stop=1.0, steer_gain=1.0, min_lookahead=0.0):"),
("        self.pink_lost_stop = pink_lost_stop",
 "        self.steer_gain = steer_gain        # factor on the cone-following steering (human data: steer earlier)\n        self.min_lookahead = min_lookahead  # m, aim point at least this far ahead (human data: ease off close to the cone)\n        self.pink_lost_stop = pink_lost_stop"),
("            self.last_steer = self.pursuit(x, y + self.side * self.offset)\n",
 "            self.last_steer = max(-1.0, min(1.0, self.steer_gain * self.pursuit(max(x, self.min_lookahead), y + self.side * self.offset)))\n"),
("            pink_lost_stop=p('pink_lost_stop', 1.0).value)",
 "            pink_lost_stop=p('pink_lost_stop', 1.0).value,\n            steer_gain=p('steer_gain', 1.0).value,\n            min_lookahead=p('min_lookahead', 0.0).value)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: smooth eingebaut")
