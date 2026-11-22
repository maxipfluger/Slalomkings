import sys
p = sys.argv[1]
s = open(p).read()
if "clear_always" in s:
    print("driver.py hat clear_always schon"); sys.exit(0)
reps = [
("steer_gain=1.0, min_lookahead=0.0):",
 "steer_gain=1.0, min_lookahead=0.0, clear_always=False):"),
("        self.steer_gain = steer_gain",
 "        self.clear_always = clear_always    # drive straight past EVERY cone that left the view, not only the last one\n        self.steer_gain = steer_gain"),
("                if self.pink_recent > 0:     # last yellow cone (pink ahead) is still beside us -> first drive straight past it\n",
 "                if self.pink_recent > 0 or self.clear_always:   # passed cone is still beside us -> first drive straight past it\n"),
("            min_lookahead=p('min_lookahead', 0.0).value)",
 "            min_lookahead=p('min_lookahead', 0.0).value,\n            clear_always=p('clear_always', False).value)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: clear_always eingebaut")
