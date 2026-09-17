"""Pure-math check of curvature_loop_samples / curvature_calculate.

Runs without the addon: it compiles the curvature slice of extras.py directly,
so it also works with plain python3 (only needs mathutils via Blender).
"""
import sys, os, math
from mathutils import Vector

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "extras.py")
src = open(SRC).read()
ns = {"Vector": Vector}
# the shared loop-offset helpers live earlier in the file
h_start = src.index("def add_loop_offset")
h_end = src.index("def space_calculate")
exec(compile(src[h_start:h_end], "extras_helpers", "exec"), ns)
start = src.index("def curvature_loop_samples")
end = src.index("def evaluate_constraints")
exec(compile(src[start:end], "extras_slice", "exec"), ns)
curvature_loop_samples = ns["curvature_loop_samples"]
curvature_calculate = ns["curvature_calculate"]
same_turn_runs = ns["same_turn_runs"]
profile_targets = ns["profile_targets"]

class FakeVert:
    def __init__(self, co, normal, index):
        self.co = Vector(co); self.normal = Vector(normal); self.index = index

def circle_loop(radius, count, uneven=False, circular=True):
    verts = []
    for i in range(count):
        a = 2 * math.pi * i / count
        if uneven:
            a += 0.35 * math.sin(3 * a) * (2 * math.pi / count)
        verts.append(FakeVert((math.cos(a) * radius, math.sin(a) * radius, 0.0),
                              (math.cos(a), math.sin(a), 0.0), i))
    return [verts, circular]

fails = []
def check(name, got, want, tol=1e-3):
    ok = abs(got - want) <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got:.6f} want {want:.6f}")
    if not ok: fails.append(name)

for radius in (0.5, 2.0):
    loop = circle_loop(radius, 32)
    ks = [s["k"] for s in curvature_loop_samples(loop[0], True)]
    check(f"circle r={radius} curvature", sum(ks) / len(ks), 1.0 / radius, tol=2e-3)

loop = circle_loop(1.0, 32)
samples = curvature_loop_samples(loop[0], True)
print("PASS  sign convention" if samples[0]["k"] > 0 else "FAIL  sign")
if samples[0]["k"] <= 0: fails.append("sign")

loop = circle_loop(1.0, 40, uneven=True)
ks = [s["k"] for s in curvature_loop_samples(loop[0], True)]
spread = max(ks) - min(ks)
print(f"{'PASS' if spread < 0.02 else 'FAIL'}  uneven spacing spread: {spread:.6f}")
if spread >= 0.02: fails.append("uneven spread")

offs = curvature_calculate([circle_loop(1.0, 32)], mode="CONSTANT")
worst = max(o.length for o in offs.values())
print(f"{'PASS' if worst < 1e-6 else 'FAIL'}  perfect circle stable: {worst:.3e}")
if worst >= 1e-6: fails.append("circle stability")

loop = circle_loop(1.0, 32)
loop[0][8].co *= 0.8
def curvature_spread(loop):
    ks = [s["k"] for s in curvature_loop_samples(loop[0], loop[1])]
    return max(ks) - min(ks)
before = curvature_spread(loop)
for _ in range(60):
    offs = curvature_calculate([loop], mode="CONSTANT")
    for v in loop[0]:
        if v.index in offs: v.co += offs[v.index] * 0.3
    for v in loop[0]:
        v.normal = Vector((v.co.x, v.co.y, 0)).normalized()
after = curvature_spread(loop)
print(f"{'PASS' if after < before * 0.25 else 'FAIL'}  dent relaxes: {before:.4f} -> {after:.4f}")
if after >= before * 0.25: fails.append("convergence")

open_loop = circle_loop(1.0, 12, circular=False); open_loop[1] = False
offs = curvature_calculate([open_loop], mode="CONSTANT")
ends_fixed = (open_loop[0][0].index not in offs) and (open_loop[0][-1].index not in offs)
print(f"{'PASS' if ends_fixed else 'FAIL'}  open loop endpoints anchored")
if not ends_fixed: fails.append("endpoints")

offs = curvature_calculate([circle_loop(1.0, 32)], mode="LINEAR")
worst = max(o.length for o in offs.values())
print(f"{'PASS' if worst < 1e-6 else 'FAIL'}  LINEAR closed fallback: {worst:.3e}")
if worst >= 1e-6: fails.append("linear closed")

open_loop = circle_loop(1.0, 20, uneven=True, circular=False); open_loop[1] = False
open_loop[0][7].co *= 0.85
def linear_residual(loop):
    samples = curvature_loop_samples(loop[0], loop[1])
    ts = [s["t"] for s in samples]; ks = [s["k"] for s in samples]
    n = len(ts); mt, mk = sum(ts)/n, sum(ks)/n
    den = sum((t - mt) ** 2 for t in ts)
    b = sum((t - mt) * (k - mk) for t, k in zip(ts, ks)) / den if den else 0
    a = mk - b * mt
    return math.sqrt(sum((a + b * t - k) ** 2 for t, k in zip(ts, ks)) / n)
before = linear_residual(open_loop)
for _ in range(80):
    offs = curvature_calculate([open_loop], mode="LINEAR")
    for v in open_loop[0]:
        if v.index in offs: v.co += offs[v.index] * 0.3
after = linear_residual(open_loop)
print(f"{'PASS' if after < before else 'FAIL'}  LINEAR reduces residual: {before:.4f} -> {after:.4f}")
if after >= before: fails.append("linear open")

# --- Same Turn: same-turn bows keep their own curvature ---
def s_curve(noise=0.0, count_a=14, count_b=12):
    """Arc of radius 1 turning left, then tangentially an arc of radius 0.6
    turning right - an S. Normals are the consistent left-normals of the
    tangent, so the signed curvature flips at the inflection."""
    pts = []
    for i in range(count_a):
        a = -math.pi * 0.45 + math.pi * 0.9 * i / (count_a - 1)
        pts.append((math.sin(a), 1.0 - math.cos(a)))
    top = pts[-1]
    # the second arc's center sits on the tangent's normal at the join
    a_end = math.pi * 0.45
    tangent = (math.cos(a_end), math.sin(a_end))
    center = (top[0] + 0.6 * -(-tangent[1]), top[1] + 0.6 * -(tangent[0]))
    start = math.atan2(top[1] - center[1], top[0] - center[0])
    for i in range(1, count_b):
        a = start - math.pi * 0.8 * i / (count_b - 1)
        pts.append((center[0] + 0.6 * math.cos(a), center[1] + 0.6 * math.sin(a)))
    verts = []
    for i, (x, y) in enumerate(pts):
        verts.append(FakeVert((x, y, 0.0), (0, 0, 1), i))
    refresh_normals(verts)
    if noise:
        for i, v in enumerate(verts):
            v.co += v.normal * noise * math.sin(i * 2.7)
        refresh_normals(verts)
    return [verts, False], count_a

def refresh_normals(verts):
    for i, v in enumerate(verts):
        p = verts[max(i - 1, 0)].co; q = verts[min(i + 1, len(verts) - 1)].co
        t = (q - p).normalized()
        v.normal = Vector((-t.y, t.x, 0.0))

loop, count_a = s_curve()
samples = curvature_loop_samples(loop[0], False)
weights = [(s["h1"] + s["h2"]) * 0.5 for s in samples]
runs = same_turn_runs(samples, weights, False)
targets = [0.0] * len(samples)
for run in runs:
    for i, tgt in zip(run, profile_targets([samples[i] for i in run], [weights[i] for i in run], "CONSTANT", False)):
        targets[i] = tgt
distinct = sorted({round(t, 3) for t in targets})
print(f"{'PASS' if len(distinct) == 2 else 'FAIL'}  S-curve splits into two segments: targets {distinct}")
if len(distinct) != 2: fails.append("segment count")
mags = sorted(abs(t) for t in distinct)
ok = distinct[0] < 0 < distinct[1] and 0.8 < mags[0] < 1.2 and 1.4 < mags[1] < 1.9
print(f"{'PASS' if ok else 'FAIL'}  the two bows keep opposite signs and their own radii (|k| ~1 and ~1.67)")
if not ok: fails.append("segment signs")

def bow_means(loop, count_a):
    samples = curvature_loop_samples(loop[0], loop[1])
    a = [s["k"] for s in samples if s["index"] < count_a - 1]
    b = [s["k"] for s in samples if s["index"] > count_a - 1]
    return sum(a) / len(a), sum(b) / len(b)
def bow_spread(loop, count_a):
    samples = curvature_loop_samples(loop[0], loop[1])
    a = [s["k"] for s in samples if s["index"] < count_a - 1]
    b = [s["k"] for s in samples if s["index"] > count_a - 1]
    return max(max(a) - min(a), max(b) - min(b))

noisy, count_a = s_curve(noise=0.004)
spread0 = bow_spread(noisy, count_a)
for _ in range(120):
    offs = curvature_calculate([noisy], mode="CONSTANT", split_turns=True)
    for v in noisy[0]:
        if v.index in offs: v.co += offs[v.index] * 0.3
    refresh_normals(noisy[0])
spread1 = bow_spread(noisy, count_a)
ma, mb = bow_means(noisy, count_a)
print(f"{'PASS' if spread1 < spread0 * 0.35 else 'FAIL'}  Same Turn evens each bow: spread {spread0:.3f} -> {spread1:.3f}")
if spread1 >= spread0 * 0.35: fails.append("segments even")
ok = ma * mb < 0 and 0.6 < abs(ma) < 1.3 and 1.2 < abs(mb) < 2.1
print(f"{'PASS' if ok else 'FAIL'}  Same Turn keeps the S: bow means {ma:.2f} / {mb:.2f}")
if not ok: fails.append("segments keep S")

# CONSTANT on the same S pulls both bows toward one shared value
flat, count_a = s_curve(noise=0.004)
for _ in range(120):
    offs = curvature_calculate([flat], mode="CONSTANT")
    for v in flat[0]:
        if v.index in offs: v.co += offs[v.index] * 0.3
    refresh_normals(flat[0])
ca, cb = bow_means(flat, count_a)
ok = abs(ca - cb) < abs(ma - mb) * 0.5
print(f"{'PASS' if ok else 'FAIL'}  CONSTANT merges the bows instead: {ca:.2f} / {cb:.2f}")
if not ok: fails.append("constant merges")

# closed loop with a dent: the seam sits in the convex part, the two convex
# runs must join across it into one segment
def ring_with_dent(first, last, depth):
    ring = circle_loop(1.0, 32)
    for i in range(first, last + 1):
        ring[0][i].co *= 1.0 - depth * math.sin(math.pi * (i - first) / (last - first))
    for i in range(32):
        p = ring[0][(i - 1) % 32].co; q = ring[0][(i + 1) % 32].co
        t = (q - p).normalized()
        ring[0][i].normal = Vector((t.y, -t.x, 0.0))  # outward for a ccw circle
    return ring
def segment_count(loop):
    samples = curvature_loop_samples(loop[0], loop[1])
    weights = [(s["h1"] + s["h2"]) * 0.5 for s in samples]
    return len(same_turn_runs(samples, weights, loop[1]))
# a deep dent is a concave bow of its own; the convex rest joins across the
# seam (a shallow dent stays convex all the way and would be one segment)
n_seg = segment_count(ring_with_dent(10, 22, 0.5))
print(f"{'PASS' if n_seg == 2 else 'FAIL'}  dented ring: convex runs join across the seam, dent separate ({n_seg} segments)")
if n_seg != 2: fails.append("ring seam merge")
# a single vertex pushed in far enough to turn the other way defines a
# turn of its own - one vertex is enough in subdivision modelling
n_seg = segment_count(ring_with_dent(12, 14, 0.04))
print(f"{'PASS' if n_seg == 2 else 'FAIL'}  a single reversed vertex is its own segment ({n_seg} segments)")
if n_seg != 2: fails.append("single vertex turn")
# a shallow nudge that keeps bending the same way does not split
n_seg = segment_count(ring_with_dent(12, 14, 0.005))
print(f"{'PASS' if n_seg == 1 else 'FAIL'}  a same-direction nudge does not split ({n_seg} segments)")
if n_seg != 1: fails.append("same direction nudge")

# Same Turn combined with the LINEAR profile: each bow fits its own rate of
# change, so the S survives here too and both bows converge to a line fit
sweep, count_a = s_curve(noise=0.004)
for _ in range(120):
    offs = curvature_calculate([sweep], mode="LINEAR", split_turns=True)
    for v in sweep[0]:
        if v.index in offs: v.co += offs[v.index] * 0.3
    refresh_normals(sweep[0])
la, lb = bow_means(sweep, count_a)
samples = curvature_loop_samples(sweep[0], False)
weights = [(s["h1"] + s["h2"]) * 0.5 for s in samples]
worst_fit = 0.0
for run in same_turn_runs(samples, weights, False):
    if len(run) < 3: continue
    sub = [samples[i] for i in run]; sw = [weights[i] for i in run]
    fitted = profile_targets(sub, sw, "LINEAR", False)
    worst_fit = max(worst_fit, max(abs(f - s["k"]) for f, s in zip(fitted, sub)))
ok = la * lb < 0
print(f"{'PASS' if ok else 'FAIL'}  Same Turn + LINEAR keeps the S: bow means {la:.2f} / {lb:.2f}")
if not ok: fails.append("linear split keeps S")
ok = worst_fit < 0.05
print(f"{'PASS' if ok else 'FAIL'}  and each bow settles on its own linear curvature (worst residual {worst_fit:.3f})")
if not ok: fails.append("linear split fit")

# --- BLUR: local evening that flows through a pinned vertex ---
# two pins displaced opposite ways: together with the anchored ends they
# don't lie on any one circle, so a single constant curvature is impossible
def arc_with_pins(count=20):
    loop = circle_loop(1.0, count, circular=False); loop[1] = False
    pins = (count // 3, 2 * count // 3)
    loop[0][pins[0]].co *= 1.10
    loop[0][pins[1]].co *= 0.90
    for v in loop[0]:
        v.normal = Vector((v.co.x, v.co.y, 0)).normalized()
    return loop, set(pins)
# blur is a diffusion, it needs more iterations than the global constant -
# which converges fast but stalls with permanent kinks at the pins
def run_pinned(mode, iters=1500):
    loop, pins = arc_with_pins()
    for _ in range(iters):
        offs = curvature_calculate([loop], mode=mode)
        for v in loop[0]:
            if v.index in offs and v.index not in pins:   # pins never move
                v.co += offs[v.index] * 0.3
        for v in loop[0]:
            v.normal = Vector((v.co.x, v.co.y, 0)).normalized()
    ks = [s["k"] for s in curvature_loop_samples(loop[0], False)]
    jumps = [abs(ks[i + 1] - ks[i]) for i in range(len(ks) - 1)]
    return max(jumps)
kink_constant = run_pinned("CONSTANT")
kink_blur = run_pinned("BLUR")
ok = kink_blur < kink_constant * 0.5
print(f"{'PASS' if ok else 'FAIL'}  BLUR flows through pinned vertices, CONSTANT kinks around them (max curvature jump {kink_blur:.3f} vs {kink_constant:.3f})")
if not ok: fails.append("blur through pin")

# on a dented closed ring, blurring diffuses the dent away like CONSTANT does
ring = circle_loop(1.0, 32)
ring[0][8].co *= 0.8
before = curvature_spread(ring)
for _ in range(150):
    offs = curvature_calculate([ring], mode="BLUR")
    for v in ring[0]:
        if v.index in offs: v.co += offs[v.index] * 0.3
    for v in ring[0]:
        v.normal = Vector((v.co.x, v.co.y, 0)).normalized()
after = curvature_spread(ring)
print(f"{'PASS' if after < before * 0.25 else 'FAIL'}  BLUR relaxes a dent: {before:.4f} -> {after:.4f}")
if after >= before * 0.25: fails.append("blur dent")

# --- reference: surface normal vs loop ---
# a loop snaking sideways on a flat plane (normals all +Z): no bending against
# the normal at all, but plenty of bending as a space curve
snake = []
for i in range(14):
    snake.append(FakeVert((i * 0.3, 0.25 * math.sin(i * 1.1), 0.0), (0, 0, 1), i))
k_normal = [s["k"] for s in curvature_loop_samples(snake, False, reference="NORMAL")]
k_loop = [s["k"] for s in curvature_loop_samples(snake, False, reference="LOOP")]
ok = max(abs(k) for k in k_normal) < 1e-9 and max(abs(k) for k in k_loop) > 0.5
print(f"{'PASS' if ok else 'FAIL'}  sideways snake: normal reference sees nothing ({max(abs(k) for k in k_normal):.1e}), loop reference sees the bends ({max(abs(k) for k in k_loop):.2f})")
if not ok: fails.append("reference snake")
# on a circle with radial normals both references agree exactly
ring = circle_loop(1.0, 32)
kn = [s["k"] for s in curvature_loop_samples(ring[0], True, reference="NORMAL")]
kl = [s["k"] for s in curvature_loop_samples(ring[0], True, reference="LOOP")]
# mathutils vectors are float32, so "exactly radial" holds to ~1e-7
ok = max(abs(a - b) for a, b in zip(kn, kl)) < 1e-5
print(f"{'PASS' if ok else 'FAIL'}  circle: both references agree (1/r = {sum(kl)/len(kl):.4f})")
if not ok: fails.append("reference circle")

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
sys.exit(1 if fails else 0)
