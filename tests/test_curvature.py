"""Pure-math check of curvature_loop_samples / curvature_calculate.

Runs without the addon: it compiles the curvature slice of extras.py directly,
so it also works with plain python3 (only needs mathutils via Blender).
"""
import sys, os, math
from mathutils import Vector

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "extras.py")
src = open(SRC).read()
start = src.index("def curvature_loop_samples")
end = src.index("def evaluate_constraints")
ns = {"Vector": Vector}
exec(compile(src[start:end], "extras_slice", "exec"), ns)
curvature_loop_samples = ns["curvature_loop_samples"]
curvature_calculate = ns["curvature_calculate"]

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

print("\n" + ("ALL PASSED" if not fails else f"FAILURES: {fails}"))
sys.exit(1 if fails else 0)
