import sys
p = sys.argv[1]
s = open(p).read()
if "far cone flickers" in s:
    print("driver.py hat farhold schon"); sys.exit(0)
a = "        if self.last_x is not None and self.missing < self.hold_frames:\n"
b = ("        # far cone flickers out more often -> wait longer (pink_hold) before searching\n"
     "        hold = self.hold_frames if (self.last_x is None or self.last_x < self.pass_max_x) else self.pink_hold\n"
     "        if self.last_x is not None and self.missing < hold:\n")
if a not in s:
    print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
s = s.replace(a, b, 1)
open(p, "w").write(s)
print("driver.py: farhold eingebaut")
