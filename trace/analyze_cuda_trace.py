import sys, collections, bisect
f = sys.argv[1]
A = collections.defaultdict(list)   # (pid,block) -> [t_alloc]
Sg = collections.defaultdict(list)  # (pid,block) -> [(t_assign, uid)]
I = []                              # (sub, pid, block, exp_uid, cur_uid, rc, t)
X = 0
for line in open(f):
    p = line.split()
    if not p: continue
    if p[0] == 'A': A[(int(p[1]), int(p[2]))].append(int(p[3]))
    elif p[0] == 'S': Sg[(int(p[1]), int(p[2]))].append((int(p[4]), int(p[3])))
    elif p[0] == 'I': I.append((int(p[1]), int(p[2]), int(p[3]), int(p[4]), int(p[5]), int(p[6]), int(p[7])))
    elif p[0] == 'X': X += 1
for k in A: A[k].sort()
for k in Sg: Sg[k].sort()
in_window = []; windows = []
for k, al in A.items():
    ss = Sg.get(k, [])
    st = [t for t, _ in ss]
    for ta in al:
        j = bisect.bisect_right(st, ta)
        if j < len(st): windows.append(st[j] - ta)
for (sub, pid, blk, exp, cur, rc, t) in I:
    k = (pid, blk)
    al = A.get(k, []); ss = Sg.get(k, [])
    if not al: continue
    i = bisect.bisect_right(al, t) - 1
    if i < 0: continue
    ta = al[i]
    st = [x for x, _ in ss]
    j = bisect.bisect_right(st, ta)
    next_assign = st[j] if j < len(st) else None
    if next_assign is None or next_assign > t:
        in_window.append((blk, ta, t, next_assign))
windows.sort()
def pct(q): return windows[int(q*(len(windows)-1))] if windows else None
print(f"imports passed={len(I)} stale_rejects(X)={X} allocs(reuse)={sum(len(v) for v in A.values())}")
print(f"alloc->assign window us: n={len(windows)} p50={pct(.5)} p95={pct(.95)} max={windows[-1] if windows else None}")
print(f"imports that passed UID check while the block was allocated but not yet reassigned: {len(in_window)}")
for blk, ta, t, na in in_window[:10]:
    print(f"   block {blk}: alloc@{ta} import@{t} (+{t-ta}us after alloc) next_assign@{na} ({'' if na is None else str(na-t)+'us after import'})")
