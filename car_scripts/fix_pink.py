import sys
p = sys.argv[1]
s = open(p).read()
if "0.25 * flaeche(r)" in s:
    print("detector.py hat pink-fix schon"); sys.exit(0)
reps = [
('2: ("pink", (140, 171, 60, 255, 70, 255)),', '2: ("pink", (140, 175, 60, 255, 70, 255)),'),
('3: ("red", (172, 8, 120, 255, 70, 255)),', '3: ("red", (177, 8, 120, 255, 70, 255)),'),
("                weg.add(r if flaeche(r) < flaeche(p) else p)",
 "                # pink wins (sunlit pink cover looks reddish), unless the pink part is tiny (reflection on a red cone)\n                weg.add(p if flaeche(p) < 0.25 * flaeche(r) else r)"),
]
for a, b in reps:
    if a not in s:
        print("NICHT GEFUNDEN:", a[:60]); sys.exit(1)
    s = s.replace(a, b, 1)
open(p, "w").write(s)
print("detector.py: pink-fix eingebaut")
