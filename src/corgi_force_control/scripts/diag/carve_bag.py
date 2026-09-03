#!/usr/bin/env python3
"""Carve rosbag2 rows out of a malformed SQLite file (page-level scan, no schema needed)
and rebuild a valid bag directory from a known-good bag's schema.

usage: carve_bag.py <broken.db3> <good_reference.db3> <out_dir> <out_basename>
"""
import sys, os, sqlite3, struct, datetime, collections

broken, good, out_dir, base = sys.argv[1:5]
buf = open(broken, "rb").read()
page_size = struct.unpack(">H", buf[16:18])[0]
if page_size == 1:
    page_size = 65536
reserved = buf[20]
U = page_size - reserved
n_pages = len(buf) // page_size
print(f"file {len(buf)} B, page_size {page_size}, reserved {reserved}, usable {U}, pages {n_pages}")


def varint(b, i):
    v = 0
    for k in range(8):
        c = b[i + k]
        v = (v << 7) | (c & 0x7F)
        if not (c & 0x80):
            return v, i + k + 1
    v = (v << 8) | b[i + 8]
    return v, i + 9


def page(p):  # 1-based
    return buf[(p - 1) * page_size: p * page_size]


def overflow_chain(first, need):
    out = bytearray()
    p = first
    seen = set()
    while p and need > 0 and 1 <= p <= n_pages and p not in seen:
        seen.add(p)
        pg = page(p)
        nxt = struct.unpack(">I", pg[:4])[0]
        take = min(U - 4, need)
        out += pg[4:4 + take]
        need -= take
        p = nxt
    return bytes(out), need


def record(payload):
    hlen, i = varint(payload, 0)
    types = []
    while i < hlen:
        t, i = varint(payload, i)
        types.append(t)
    vals = []
    i = hlen
    for t in types:
        if t == 0:
            vals.append(None)
        elif t in (1, 2, 3, 4, 5, 6):
            n = {1: 1, 2: 2, 3: 3, 4: 4, 5: 6, 6: 8}[t]
            vals.append(int.from_bytes(payload[i:i + n], "big", signed=True))
            i += n
        elif t == 7:
            vals.append(struct.unpack(">d", payload[i:i + 8])[0])
            i += 8
        elif t == 8:
            vals.append(0)
        elif t == 9:
            vals.append(1)
        elif t >= 12 and t % 2 == 0:
            n = (t - 12) // 2
            vals.append(bytes(payload[i:i + n]))
            i += n
        elif t >= 13:
            n = (t - 13) // 2
            vals.append(payload[i:i + n].decode("utf-8", "replace"))
            i += n
        else:
            raise ValueError(f"bad serial type {t}")
    return vals


X = U - 35
M = ((U - 12) * 32 // 255) - 23
rows_by_sig = collections.defaultdict(dict)   # signature -> {rowid: vals}
conflicts = collections.Counter()
leaf_pages = 0
bad_cells = 0
truncated = 0
for p in range(1, n_pages + 1):
    pg = page(p)
    hoff = 100 if p == 1 else 0
    if pg[hoff] != 0x0D:
        continue
    leaf_pages += 1
    ncell = struct.unpack(">H", pg[hoff + 3:hoff + 5])[0]
    ptrs = [struct.unpack(">H", pg[hoff + 8 + 2 * k:hoff + 10 + 2 * k])[0] for k in range(ncell)]
    for off in ptrs:
        try:
            if off < hoff + 8 or off >= page_size:
                bad_cells += 1
                continue
            P, i = varint(pg, off)
            rowid, i = varint(pg, i)
            if P <= X:
                local = P
                ovf = 0
            else:
                K = M + ((P - M) % (U - 4))
                local = K if K <= X else M
                ovf = struct.unpack(">I", pg[i + local:i + local + 4])[0]
            payload = bytes(pg[i:i + local])
            if ovf:
                extra, missing = overflow_chain(ovf, P - local)
                payload += extra
                if missing > 0:
                    truncated += 1
                    continue
            if len(payload) != P:
                bad_cells += 1
                continue
            vals = record(payload)
        except Exception:
            bad_cells += 1
            continue
        sig = tuple("N" if v is None else ("i" if isinstance(v, int) else "f" if isinstance(v, float)
                    else "b" if isinstance(v, bytes) else "t") for v in vals)
        d = rows_by_sig[sig]
        if rowid in d:
            if d[rowid] != vals:
                conflicts[sig] += 1
        else:
            d[rowid] = vals
print(f"leaf pages {leaf_pages}, bad cells {bad_cells}, truncated-overflow {truncated}")
for sig, d in rows_by_sig.items():
    print(f"  signature {''.join(sig)}: {len(d)} rows, conflicts {conflicts[sig]}")

msgs = rows_by_sig.get(("N", "i", "i", "b"), {})
topics = rows_by_sig.get(("N", "t", "t", "t", "t"), {})
print("carved topics:", [(rid, v[1], v[2]) for rid, v in sorted(topics.items())])

# ---- reference schema + topics (fallback) from the good bag
g = sqlite3.connect(f"file:{good}?mode=ro", uri=True)
ddl = [r[0] for r in g.execute("select sql from sqlite_master where sql is not null order by rowid")]
good_topics = g.execute("select * from topics").fetchall()
good_cols = [d[1] for d in g.execute("pragma table_info(topics)")]
extra_tables = {}
for (name,) in g.execute("select name from sqlite_master where type='table' and name not in ('topics','messages')"):
    extra_tables[name] = ([d[1] for d in g.execute(f"pragma table_info({name})")],
                          g.execute(f"select * from {name}").fetchall())
print("reference tables:", ['topics', 'messages'] + list(extra_tables))
print("reference topics:", [(r[0], r[1]) for r in good_topics])

if not topics:
    print("!! no topics rows carved -- falling back to the reference bag's topics table (ids assumed identical)")
    topic_rows = good_topics
else:
    topic_rows = [tuple([rid] + list(v[1:])) for rid, v in sorted(topics.items())]

# ---- rebuild
os.makedirs(out_dir, exist_ok=True)
out = os.path.join(out_dir, base + "_0.db3")
if os.path.exists(out):
    os.remove(out)
n = sqlite3.connect(out)
for s in ddl:
    n.execute(s)
n.executemany(f"insert into topics values ({','.join('?' * len(good_cols))})", topic_rows)
for name, (cols, rows) in extra_tables.items():
    if rows:
        n.executemany(f"insert into {name} values ({','.join('?' * len(cols))})", rows)
ordered = sorted(msgs.items(), key=lambda kv: (kv[1][2], kv[0]))
n.executemany("insert into messages(id, topic_id, timestamp, data) values (?,?,?,?)",
              [(rid, v[1], v[2], v[3]) for rid, v in ordered])
n.commit()


def f(ns):
    return datetime.datetime.fromtimestamp(ns / 1e9).strftime('%H:%M:%S.%f')[:-3]


lo, hi = n.execute("select min(timestamp),max(timestamp) from messages").fetchone()
print(f"rebuilt {out}: {len(ordered)} messages, span {f(lo)} -> {f(hi)} = {(hi - lo) / 1e9:.1f} s")
tid2name = {r[0]: r[1] for r in topic_rows}
for tid, cnt, a, b in n.execute("select topic_id, count(*), min(timestamp), max(timestamp) from messages group by topic_id order by topic_id"):
    print(f"  topic {tid} {tid2name.get(tid, '?'):18s} n={cnt:7d}  {f(a)} -> {f(b)}")
# per-topic gap check (largest gap between consecutive messages) -- corruption holes show up here
for tid in tid2name:
    ts = [r[0] for r in n.execute("select timestamp from messages where topic_id=? order by timestamp", (tid,))]
    if len(ts) > 2:
        gaps = [(ts[i + 1] - ts[i]) / 1e6 for i in range(len(ts) - 1)]
        gi = max(range(len(gaps)), key=gaps.__getitem__)
        print(f"  {tid2name[tid]:18s} median dt {sorted(gaps)[len(gaps) // 2]:.1f} ms, largest gap {gaps[gi]:.1f} ms at {f(ts[gi])}")
n.close()
