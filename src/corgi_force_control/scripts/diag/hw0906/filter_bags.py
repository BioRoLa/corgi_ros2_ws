#!/usr/bin/env python3
"""Print only the bags that open and read cleanly. Some of today's were killed mid-write
by an Orin reset and are malformed (#46); they are skipped rather than allowed to abort a
whole-day sweep. Prints the usable paths on stdout, a report on stderr.
"""
import sys, sqlite3

good, bad = [], []
for p in sys.argv[1:]:
    try:
        con = sqlite3.connect("file:%s?mode=ro" % p, uri=True)
        T = {n: i for i, n, t in con.execute("SELECT id,name,type FROM topics")}
        if "/trigger" not in T or "/motor/state" not in T or "/motor/command" not in T:
            bad.append((p, "missing topics")); con.close(); continue
        con.execute("SELECT COUNT(*) FROM messages").fetchone()
        con.execute("SELECT timestamp,data FROM messages LIMIT 1").fetchone()
        con.close()
        good.append(p)
    except Exception as e:
        bad.append((p, type(e).__name__))

for p in good:
    print(p)
sys.stderr.write("usable %d, skipped %d\n" % (len(good), len(bad)))
for p, why in bad:
    sys.stderr.write("   skip %-28s %s\n" % (p.split("/")[-2], why))
