#


import bmesh

# this file has separate tools for experts in cad and automotive design
import bpy
from mathutils import Vector, kdtree, Matrix, Euler, geometry


# project into XY plane,
up = Vector((0, 0, 1))
from bisect import bisect_right
from math import atan2, cos, exp, log, pi, radians, sin, sqrt
from random import random

from bpy.props import (
    FloatProperty,
    IntProperty,
    BoolProperty,
    EnumProperty,
    FloatVectorProperty,
)
from bpy.types import Operator, GizmoGroup, Panel, PropertyGroup

from . import draw, gizmos, utils

DEBUG_DRAW = False


def kd_tree_from_bmesh(bm):
    from mathutils.kdtree import KDTree

    kd = KDTree(len(bm.verts))
    for i, v in enumerate(bm.verts):
        kd.insert(v.co, i)
    kd.balance()
    return kd


class NormalLoopAlign(Operator):
    bl_idname = "mesh.flatten_loop_normal"
    bl_label = "Loop to normal-plane"

    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return (
            context.active_object is not None and context.active_object.type == "MESH"
        )

    def execute(self, context):
        obj = context.active_object
        # this simple solution actually works pretty nice, but not for closed loops :)
        bpy.ops.transform.resize(
            value=(0, 1, 1),
            orient_type="NORMAL",
            orient_matrix_type="NORMAL",
            mirror=True,
            use_proportional_edit=False,
        )
        return {"FINISHED"}


def project_point_to_plane(point, plane_center, plane_normal):
    """Project a point onto a plane and return the projected point"""

    dist = geometry.distance_point_to_plane(point, plane_center, plane_normal)
    projected_point = point - plane_normal * dist
    if DEBUG_DRAW:
        draw.add_arrow(
            plane_center, plane_center + plane_normal, (1, 0, 0, 0.2), scale=1
        )
        draw.add_arrow(point, projected_point, (0.5, 0.5, 1, 0.2), scale=1)
    return projected_point


def project_point_to_curve(point, curve_bm, normal, world_matrix, kd):
    """Project a point onto a curve along a given normal direction and return the closest point on the curve"""
    # Project the point along the normal direction
    projected_point = point + normal

    # Transform the points to the curve's local space
    local_point = world_matrix.inverted() @ point
    local_projected_point = world_matrix.inverted() @ projected_point

    # Find the closest point on the curve to the projected line
    closest_point = None
    min_dist = float("inf")
    for edge in curve_bm.edges:
        p1 = edge.verts[0].co
        p2 = edge.verts[1].co
        intersection = line_line_intersection(
            local_point, local_projected_point, p1, p2
        )
        if intersection:
            world_intersection = world_matrix @ intersection
            dist = (world_intersection - point).length
            if dist < min_dist:
                min_dist = dist
                closest_point = world_intersection

    # If no intersection found, use KDTree to find the nearest point
    if closest_point is None:
        closest_point, _, _ = kd.find(point)

    return closest_point


def line_line_intersection(p1, p2, p3, p4):
    """Find the intersection point of two lines (if exists)"""
    # Line AB represented as a1x + b1y = c1
    a1 = p2.y - p1.y
    b1 = p1.x - p2.x
    c1 = a1 * p1.x + b1 * p1.y

    # Line CD represented as a2x + b2y = c2
    a2 = p4.y - p3.y
    b2 = p3.x - p4.x
    c2 = a2 * p3.x + b2 * p3.y

    determinant = a1 * b2 - a2 * b1

    if determinant == 0:
        # The lines are parallel
        return None
    else:
        x = (b2 * c1 - b1 * c2) / determinant
        y = (a1 * c2 - a2 * c1) / determinant
        return Vector((x, y, 0))


def eval_point(co, curve_snapping="3D", source_curve=None):
    if curve_snapping == "3D":
        reference_co = co
    elif curve_snapping == "PROJECT_PLANE":
        curve_plane_normal = source_curve.rotation_euler.to_matrix() @ Vector((0, 0, 1))
        reference_co = project_point_to_plane(
            co, source_curve.location, curve_plane_normal
        )
    return reference_co

def calculate_curve_length(source_curve, curve_snapping):
    # Calculate curve length from actual evaluated curve vertices

    depsgraph = bpy.context.evaluated_depsgraph_get()
    bmesh_curve = utils.get_evaluated_bm(source_curve, depsgraph)
    curve_world_matrix = source_curve.matrix_world
    curve_length = 0
    for i in range(len(bmesh_curve.verts)):
        if i == 0:
            continue
        start_co = eval_point(
            curve_world_matrix @ bmesh_curve.verts[i - 1].co,
            curve_snapping,
            source_curve,
        )
        end_co = eval_point(
            curve_world_matrix @ bmesh_curve.verts[i].co,
            curve_snapping,
            source_curve,
        )
        curve_length += (end_co - start_co).length
    return curve_length

def sample_curve_polyline(source_curve, curve_snapping, samples_per_segment=24):
    """Dense world space polyline of the curve's first spline.

    Bezier splines are sampled from their true segments - the display
    tessellation is too coarse, and walking it while scaling targets by the
    smooth spline length drifted vertices along the curve a little on every
    iteration. Other spline types fall back to the evaluated tessellation.

    Returns (points, cyclic), points already flattened for plane projection.
    """
    matrix = source_curve.matrix_world
    spline = source_curve.data.splines[0]
    cyclic = spline.use_cyclic_u
    points = []
    if spline.type == "BEZIER" and len(spline.bezier_points) >= 2:
        bez = spline.bezier_points
        count = len(bez)
        segments = count if cyclic else count - 1
        for s in range(segments):
            a = bez[s]
            b = bez[(s + 1) % count]
            segment = geometry.interpolate_bezier(
                a.co, a.handle_right, b.handle_left, b.co, samples_per_segment + 1
            )
            if s > 0:
                segment = segment[1:]
            points.extend(segment)
        if cyclic and len(points) > 1:
            # the last sample equals the first point
            points = points[:-1]
        points = [matrix @ p for p in points]
    else:
        depsgraph = bpy.context.evaluated_depsgraph_get()
        bm_curve = utils.get_evaluated_bm(source_curve, depsgraph)
        bm_curve.verts.ensure_lookup_table()
        # only the first spline: the evaluated mesh holds every spline's
        # tessellation concatenated, walk the chain its first vertex sits on
        adjacency = {v.index: [] for v in bm_curve.verts}
        for e in bm_curve.edges:
            adjacency[e.verts[0].index].append(e.verts[1].index)
            adjacency[e.verts[1].index].append(e.verts[0].index)
        chain = [0]
        visited = {0}
        while True:
            following = [n for n in adjacency.get(chain[-1], []) if n not in visited]
            if not following:
                break
            chain.append(following[0])
            visited.add(following[0])
        points = [matrix @ bm_curve.verts[i].co for i in chain]
    points = [eval_point(p, curve_snapping, source_curve) for p in points]
    return points, cyclic


def polyline_arc_table(points, cyclic):
    """Cumulative arc lengths per point and the total length."""
    table = [0.0]
    for i in range(1, len(points)):
        table.append(table[-1] + (points[i] - points[i - 1]).length)
    total = table[-1]
    if cyclic:
        total += (points[0] - points[-1]).length
    return table, total


def polyline_point_at(points, table, total, cyclic, distance):
    """Position at an arc length distance, wrapping on cyclic polylines."""
    if cyclic:
        distance = distance % total
    else:
        distance = max(0.0, min(distance, total))
    segment = bisect_right(table, distance) - 1
    segment = max(0, min(segment, len(points) - (1 if cyclic else 2)))
    a = points[segment]
    b = points[(segment + 1) % len(points)]
    span = (b - a).length
    t = 0.0 if span < 1e-12 else (distance - table[segment]) / span
    return a.lerp(b, min(t, 1.0))


def polyline_closest_arc(points, table, total, cyclic, co):
    """Arc length position of the point on the polyline closest to co.

    The exact spot on the neighbouring segments, not the nearest sample - a
    quantized anchor made the whole distribution jump as vertices moved.
    """
    best_index = 0
    best_sq = None
    for i, p in enumerate(points):
        d_sq = (p - co).length_squared
        if best_sq is None or d_sq < best_sq:
            best_sq = d_sq
            best_index = i
    best_arc = table[best_index]
    count = len(points)
    for segment in ((best_index - 1) % count, best_index):
        if not cyclic and (segment < 0 or segment >= count - 1):
            continue
        a = points[segment]
        b = points[(segment + 1) % count]
        ab = b - a
        span_sq = ab.length_squared
        if span_sq < 1e-18:
            continue
        t = max(0.0, min(1.0, (co - a).dot(ab) / span_sq))
        p = a + ab * t
        d_sq = (p - co).length_squared
        if d_sq <= best_sq:
            best_sq = d_sq
            best_arc = table[segment] + ab.length * t
    return best_arc


def to_curve_verts_calculate(
    loop,
    curve_snapping="PROJECT_PLANE",
    curve_distribution="EVEN",
    source_curve=None,
    kd=None,
    normal=None,
    edit_loop_for_endpoints=None,
):
    """Snap the loop vertices onto the curve, spread by arc length.

    Every vertex gets a fraction along the loop - its own arc position for
    the original distribution, an even split otherwise - and lands at that
    fraction of the curve's length. All lengths are measured on one densely
    sampled polyline of the true curve, so the placement stays consistent
    between iterations instead of slipping along the curve.
    """
    target_offsets = {}
    loop_verts = loop[0]
    loop_closed = loop[1]
    if source_curve is None or len(loop_verts) < 2:
        return target_offsets

    object_world_matrix = bpy.context.active_object.matrix_world
    points, curve_cyclic = sample_curve_polyline(source_curve, curve_snapping)
    if len(points) < 2:
        return target_offsets
    table, total = polyline_arc_table(points, curve_cyclic)
    if total < 1e-9:
        return target_offsets

    # reference positions in world space, flattened for plane projection;
    # open loop ends of a subdivision constraint anchor at the edit vertices
    references = []
    offset_keys = []
    for i, vert in enumerate(loop_verts):
        use_edit_vert = (
            edit_loop_for_endpoints is not None
            and not loop_closed
            and (i == 0 or i == len(loop_verts) - 1)
        )
        if use_edit_vert:
            edit_vert = edit_loop_for_endpoints[0][i]
            references.append(
                eval_point(object_world_matrix @ edit_vert.co, curve_snapping, source_curve)
            )
            offset_keys.append(edit_vert.index)
        else:
            references.append(
                eval_point(object_world_matrix @ vert.co, curve_snapping, source_curve)
            )
            offset_keys.append(vert.index)

    # each vertex's fraction of the way along the loop
    if curve_distribution == "ORIGINAL":
        cumulative = [0.0]
        for i in range(1, len(loop_verts)):
            cumulative.append(
                cumulative[-1] + (loop_verts[i].co - loop_verts[i - 1].co).length
            )
        loop_total = cumulative[-1]
        if loop_closed:
            loop_total += (loop_verts[0].co - loop_verts[-1].co).length
        if loop_total < 1e-9:
            return target_offsets
        fractions = [c / loop_total for c in cumulative]
    else:
        steps = len(loop_verts) - 1 + (1 if loop_closed else 0)
        fractions = [i / steps for i in range(len(loop_verts))]

    if loop_closed:
        # anchor at the exact closest curve position of the first vertex, and
        # follow the loop's own travel direction along the curve
        start_arc = polyline_closest_arc(points, table, total, curve_cyclic, references[0])
        direction = 1
        probe = max(total * 1e-3, 1e-9)
        tangent = polyline_point_at(
            points, table, total, curve_cyclic, start_arc + probe
        ) - polyline_point_at(points, table, total, curve_cyclic, start_arc)
        forward = references[1] - references[0]
        backward = references[-1] - references[0]
        if (
            tangent.length > 1e-12
            and forward.length > 1e-12
            and backward.length > 1e-12
            and forward.angle(tangent) > backward.angle(tangent)
        ):
            direction = -1
    else:
        # an open loop spans the whole curve, starting at the nearer end
        if (references[0] - points[0]).length <= (references[0] - points[-1]).length:
            start_arc, direction = 0.0, 1
        else:
            start_arc, direction = total, -1

    for fraction, reference, key in zip(fractions, references, offset_keys):
        distance = start_arc + direction * fraction * total
        co = polyline_point_at(points, table, total, curve_cyclic, distance)
        target_offsets[key] = co - reference

    return target_offsets


class FlattenSelectionOperator(Operator):
    bl_idname = "mesh.flatten_selection"
    bl_label = "Flatten Selection"
    bl_options = {"REGISTER", "UNDO"}

    method: bpy.props.EnumProperty(
        items=[
            (
                "best_fit",
                "Least Squares",
                "Use the least squares method to estimate the plane",
            ),
            (
                "mean_normal",
                "Mean Normal",
                "Use the mean normal of the vertices to estimate the plane",
            ),
        ],
        name="Method",
        default="best_fit",
    )

    slide: bpy.props.BoolProperty(
        name="Slide Along Edges",
        description="Slide vertices along edges towards the target plane instead of direct projection",
        default=True,
    )

    @classmethod
    def poll(cls, context):
        return (
            context.active_object is not None and context.active_object.type == "MESH"
        )

    def execute(self, context):
        obj = context.active_object
        # this simple solution actually works pretty nice, but not for closed loops :)
        # bpy.ops.transform.resize(value=(0, 1, 1), orient_type='NORMAL',
        #                          orient_matrix_type='NORMAL',  mirror=True,
        #                          use_proportional_edit=False)
        # return {'FINISHED'}

        bm = bmesh.from_edit_mesh(obj.data)

        # Get the selected vertices
        selected_verts = [v for v in bm.verts if v.select]

        utils.flatten_verts(selected_verts, self.method, self.slide)

        bmesh.update_edit_mesh(obj.data)

        return {"FINISHED"}


def edge_angle(e1, e2, face_normal):
    b = set(e1.verts).intersection(e2.verts).pop()
    a = e1.other_vert(b).co - b.co
    c = e2.other_vert(b).co - b.co
    a.negate()
    axis = a.cross(c).normalized()
    if axis.length < 1e-5:
        return pi

    if axis.dot(face_normal) < 0:
        axis.negate()
    M = axis.rotation_difference(up).to_matrix().to_4x4()

    a = (M @ a).xy.normalized()
    c = (M @ c).xy.normalized()

    return pi - atan2(a.cross(c), a.dot(c))


def circle_through_points(a, b, c):
    """Circumcenter of the circle through three points, None when collinear.

    The collinearity test is relative to the segment lengths: nearly straight
    triples put the circumcenter enormously far away, and rotating such a
    radius vector in single precision produced garbage positions - a straight
    stretch of a loop must fall back to its chord instead.
    """
    u = b - a
    v = c - a
    n = u.cross(v)
    n_length_sq = n.length_squared
    # sin of the angle at a below 1e-4 counts as a straight line
    if n_length_sq < u.length_squared * v.length_squared * 1e-8:
        return None
    return a + (
        v.length_squared * n.cross(u) + u.length_squared * v.cross(n)
    ) / (2.0 * n_length_sq)


def arc_point(a, b, center, t):
    """Point at fraction t along the circular arc from a to b around center."""
    ra = a - center
    rb = b - center
    axis = ra.cross(rb)
    # relative test - a tiny angle on a huge radius is numerically a chord
    if axis.length_squared < ra.length_squared * rb.length_squared * 1e-10:
        return a.lerp(b, t)
    angle = ra.angle(rb)
    return center + Matrix.Rotation(angle * t, 3, axis.normalized()) @ ra


def evaluate_arc(verts, tknots, m):
    """Position at arc length m, interpolated on circular arcs through the
    neighbouring vertices.

    A vertex slid this way follows the curve the loop describes instead of its
    chords: the method is exact on circles and keeps the local curvature on
    smooth loops, where resampling a fitted spline slowly flattens the shape a
    little more on every iteration.

    Each segment blends two arcs, one through the segment and its previous
    vertex, one through the segment and its next vertex, so neighbouring
    segments join smoothly.
    """
    if m <= tknots[0]:
        return verts[0].co.copy()
    if m >= tknots[-1]:
        return verts[-1].co.copy()

    segment = bisect_right(tknots, m) - 1
    segment = max(0, min(segment, len(verts) - 2))
    a = verts[segment].co
    b = verts[segment + 1].co
    span = tknots[segment + 1] - tknots[segment]
    t = 0.0 if span <= 1e-12 else (m - tknots[segment]) / span

    positions = []
    if segment - 1 >= 0:
        center = circle_through_points(verts[segment - 1].co, a, b)
        if center is not None:
            positions.append(arc_point(a, b, center, t))
    if segment + 2 < len(verts):
        center = circle_through_points(a, b, verts[segment + 2].co)
        if center is not None:
            positions.append(arc_point(a, b, center, t))

    if not positions:
        # collinear neighbourhood or a two-vertex loop, the chord is the curve
        return a.lerp(b, t)
    if len(positions) == 1:
        return positions[0]
    # blend towards the arc whose extra vertex is nearer to the sample
    return positions[0].lerp(positions[1], t)


def ratio_spaced_tpoints(tknots, len_total):
    """Target arc length positions where each segment keeps one constant ratio to
    the previous one - a geometric progression of segment lengths.

    The ratio is fitted to the current segments (least squares on the log lengths),
    so the loop keeps its own tendency and only regularizes it into an exact
    progression. Returns None when the segments don't define a usable ratio, the
    caller then falls back to even spacing.
    """
    lengths = [tknots[i + 1] - tknots[i] for i in range(len(tknots) - 1)]
    count = len(lengths)
    if count < 2 or any(l < 1e-9 for l in lengths):
        return None

    # fit log(length_i) = a + slope * i, so slope is the log of the mean ratio
    logs = [log(l) for l in lengths]
    mean_index = (count - 1) / 2.0
    mean_log = sum(logs) / count
    denominator = sum((i - mean_index) ** 2 for i in range(count))
    slope = sum((i - mean_index) * (lg - mean_log) for i, lg in enumerate(logs)) / denominator
    # keep a degenerate loop from collapsing all its verts into one end
    slope = max(-log(4.0), min(log(4.0), slope))
    ratio = exp(slope)
    if abs(ratio - 1.0) < 1e-9:
        # the even fallback is exactly this
        return None

    first = len_total * (1.0 - ratio) / (1.0 - ratio ** count)
    tpoints = [0.0]
    segment = first
    for i in range(count):
        tpoints.append(tpoints[-1] + segment)
        segment *= ratio
    # kill the accumulated float error at the anchored far end
    tpoints[-1] = len_total
    return tpoints


def add_loop_offset(target_offsets, counts, index, offset):
    """Accumulate an offset for a vertex instead of overwriting it.

    On grid-like assignments a vertex belongs to two crossing loops - a row
    and a column. Both loops' corrections must count; a plain dict write
    would keep only whichever loop happened to be processed last, so one
    whole direction of the grid would be silently ignored.
    """
    if index in target_offsets:
        target_offsets[index] = target_offsets[index] + offset
        counts[index] = counts.get(index, 1) + 1
    else:
        target_offsets[index] = offset


def finish_loop_offsets(target_offsets, counts):
    """Average the accumulated offsets of vertices shared by several loops."""
    for index, count in counts.items():
        if count > 1:
            target_offsets[index] /= count
    return target_offsets


def space_calculate(loops_data, interpolation="arc", method="EVEN", step_weight=1.0):
    """Calculate positions to space vertices along loops

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        interpolation: 'arc', 'cubic' or 'linear'
        method: 'EVEN' for equal distances, 'RATIO' for one constant ratio between
            neighbouring segments - open loops only, closed loops can't keep a
            ratio other than one all the way around and fall back to even.
        step_weight: the weight move_verts_to_targets will scale the offsets by.
            The offsets are prepared so that the weighted step lands exactly ON
            the curve, a bit further along it - a plain weighted step toward the
            full target cuts a chord inside the curve every iteration, and that
            slowly shrinks the shape.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}
    offset_counts = {}
    step_weight = max(step_weight, 1e-3)

    for loop_data in loops_data:
        verts = loop_data[0]
        is_circular = loop_data[1]

        if len(verts) < 2:
            continue

        # Calculate t values (distances along the loop)
        tknots = [0.0]
        for i in range(1, len(verts)):
            tknots.append(tknots[-1] + (verts[i].co - verts[i - 1].co).length)

        len_total = tknots[-1]
        if is_circular:
            # the segment closing the loop lies between no two knots
            len_total += (verts[0].co - verts[-1].co).length
        if len_total < 1e-9:
            continue

        if is_circular:
            # A closed loop gets sampled all the way around, so past the last vertex
            # and over the closing segment - but a spline only spans the vertices it
            # was given. Wrapping vertices around the seam extends it over the closing
            # segment, so those samples are interpolated. Extrapolating the cubic out
            # there instead is what made closed loops drift away over the iterations.
            spline_verts, spline_tknots = wrap_loop_for_spline(verts, tknots, len_total)
            segments = len(verts)
        else:
            spline_verts, spline_tknots = verts, tknots
            segments = len(verts) - 1

        # Calculate target points along the arc
        tpoints = None
        if method == "RATIO" and not is_circular:
            tpoints = ratio_spaced_tpoints(tknots, len_total)
        if tpoints is None:
            t_per_segment = len_total / segments
            tpoints = [i * t_per_segment for i in range(len(verts))]

        # Calculate splines - arc interpolation works straight off the vertices
        splines = None
        if interpolation == 'cubic':
            splines = calculate_cubic_splines_simple(spline_verts, spline_tknots)
            if not splines:
                continue
        elif interpolation == 'linear':
            splines = calculate_linear_splines_simple(spline_verts, spline_tknots)
            if not splines:
                continue

        # Calculate new positions for each vertex
        for i, v in enumerate(verts):
            # advance the parameter only by the step weight, then compensate the
            # offset for the weighting applied later - the vertex slides along
            # the curve instead of stepping onto the chord toward the full target
            m = tknots[i] + (tpoints[i] - tknots[i]) * step_weight
            if interpolation == 'arc':
                new_pos = evaluate_arc(spline_verts, spline_tknots, m)
            else:
                new_pos = evaluate_spline(splines, spline_tknots, m, interpolation)
            add_loop_offset(
                target_offsets, offset_counts, v.index, (new_pos - v.co) / step_weight
            )

    return finish_loop_offsets(target_offsets, offset_counts)


def wrap_loop_for_spline(verts, tknots, len_total, pad=4):
    """Repeat vertices from the far side of the seam, so a closed loop can be splined
    through the point where it closes.

    The loop's own vertices stay in the middle with their knots untouched, the copies
    around them carry knots one turn earlier and one turn later, keeping the knots
    increasing all the way through.
    """
    count = len(verts)
    pad = min(pad, count - 1)

    wrapped_verts = []
    wrapped_tknots = []

    # tail of the loop, one turn earlier
    for i in range(count - pad, count):
        wrapped_verts.append(verts[i])
        wrapped_tknots.append(tknots[i] - len_total)

    wrapped_verts.extend(verts)
    wrapped_tknots.extend(tknots)

    # head of the loop, one turn later
    for i in range(pad + 1):
        wrapped_verts.append(verts[i])
        wrapped_tknots.append(tknots[i] + len_total)

    return wrapped_verts, wrapped_tknots


def evaluate_spline(splines, tknots, m, interpolation):
    """Position at distance m along a spline built over tknots."""
    # Find which spline segment this point is on
    if m in tknots:
        n = tknots.index(m)
    else:
        t = tknots[:]
        t.append(m)
        t.sort()
        n = t.index(m) - 1

    if n > len(splines) - 1:
        n = len(splines) - 1
    elif n < 0:
        n = 0

    if interpolation == 'cubic':
        ax, bx, cx, dx, tx = splines[n][0]
        x = ax + bx * (m - tx) + cx * (m - tx) ** 2 + dx * (m - tx) ** 3
        ay, by, cy, dy, ty = splines[n][1]
        y = ay + by * (m - ty) + cy * (m - ty) ** 2 + dy * (m - ty) ** 3
        az, bz, cz, dz, tz = splines[n][2]
        z = az + bz * (m - tz) + cz * (m - tz) ** 2 + dz * (m - tz) ** 3
        return Vector([x, y, z])

    # linear
    a, d, t, u = splines[n]
    if u != 0:
        return ((m - t) / u) * d + a
    return a


def calculate_cubic_splines_simple(verts, tknots):
    """Calculate cubic splines for vertex positions.

    The spline is open, it spans only the vertices given. Closed loops are handled by
    wrap_loop_for_spline, which passes in vertices wrapped around the seam.
    """
    n = len(verts)
    if n < 2:
        return False
    
    x = tknots[:n]
    locs = [v.co[:] for v in verts]
    
    result = []
    for j in range(3):
        a = [loc[j] for loc in locs]
        h = []
        for i in range(n - 1):
            if x[i + 1] - x[i] == 0:
                h.append(1e-8)
            else:
                h.append(x[i + 1] - x[i])
        
        q = [False]
        for i in range(1, n - 1):
            q.append(3 / h[i] * (a[i + 1] - a[i]) - 3 / h[i - 1] * (a[i] - a[i - 1]))
        
        l = [1.0]
        u = [0.0]
        z = [0.0]
        for i in range(1, n - 1):
            l.append(2 * (x[i + 1] - x[i - 1]) - h[i - 1] * u[i - 1])
            if l[i] == 0:
                l[i] = 1e-8
            u.append(h[i] / l[i])
            z.append((q[i] - h[i - 1] * z[i - 1]) / l[i])
        
        l.append(1.0)
        z.append(0.0)
        b = [False for i in range(n - 1)]
        c = [False for i in range(n)]
        d = [False for i in range(n - 1)]
        c[n - 1] = 0.0
        
        for i in range(n - 2, -1, -1):
            c[i] = z[i] - u[i] * c[i + 1]
            b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3
            d[i] = (c[i + 1] - c[i]) / (3 * h[i])
        
        for i in range(n - 1):
            result.append([a[i], b[i], c[i], d[i], x[i]])
    
    splines = []
    for i in range(n - 1):
        splines.append([result[i], result[i + n - 1], result[i + (n - 1) * 2]])
    
    return splines


def calculate_linear_splines_simple(verts, tknots):
    """Calculate linear splines for vertex positions"""
    splines = []
    for i in range(len(verts) - 1):
        a = verts[i].co
        b = verts[i + 1].co
        d = b - a
        t = tknots[i]
        u = tknots[i + 1] - t
        splines.append([a, d, t, u])
    return splines


def slide_optimize_calculate(loops_data):
    """Calculate slide offsets for vertices to balance angles where quads meet
    
    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
    """
    target_offsets = {}
    
    # Process each loop separately
    for loop_data in loops_data:
        selected_verts = loop_data[0]
        if len(selected_verts) == 0:
            continue
        
        selected_verts_set = set(selected_verts)
        selected_edges = []
        for v in selected_verts:
            for edge in v.link_edges:
                if edge.other_vert(v) in selected_verts_set:
                    selected_edges.append(edge)
        
        if len(selected_edges) == 0:
            continue
        
        total_length = 0
        for e in selected_edges:
            total_length += e.calc_length()
        avg_length = total_length / len(selected_edges)

        # this is additional to the global weight settings, 
        # because slide optimize can become very unstable soon.
        
        multiplier = 0.1 

        for v1 in selected_verts:
            slide_directions = []
            slide_candidates_edges = []
            sel_edges_count = 0
            
            for e in v1.link_edges:
                slide_directions.append(0)
                if e.other_vert(v1) not in selected_verts_set:
                    continue
                sel_edges_count += 1
                slide_candidates_edges.append(e)
            
            if sel_edges_count != 2:
                continue
            if len(v1.link_edges) < 4:
                continue
            
            tot_normal = v1.normal
            a = edge_angle(
                slide_candidates_edges[0], slide_candidates_edges[1], tot_normal
            )
            
            for fi, f in enumerate(v1.link_faces):
                for e1fi, e1 in enumerate(f.edges):
                    if v1 not in e1.verts:
                        continue
                    
                    e2 = f.edges[e1fi - 1]
                    if v1 not in e2.verts:
                        continue
                    a = edge_angle(e2, e1, tot_normal)
                    
                    for e3i, e3 in enumerate(v1.link_edges):
                        if e3 == e2 and e2.other_vert(v1) not in selected_verts_set:
                            slide_directions[e3i] += a
                        if e3 == e1 and e1.other_vert(v1) not in selected_verts_set:
                            slide_directions[e3i] += a
            
            slide_candidates = []
            for i, a in enumerate(slide_directions):
                if a > 0:
                    slide_candidates.append([i, a])
            
            if len(slide_candidates) != 2:
                continue
            
            mina = 10000
            minindex = -1
            for c in slide_candidates:
                if c[1] < mina:
                    mina = c[1]
                    minindex = c[0]
            
            for c in slide_candidates:
                if c[0] != minindex:
                    other_candidate_angle = c[1]
                    other_edge = v1.link_edges[c[0]]
            
            slide_edge = v1.link_edges[minindex]
            angle_difference = abs(slide_candidates[0][1] - slide_candidates[1][1])
            
            slide_edge_vector = slide_edge.other_vert(v1).co - v1.co
            slide_direction = slide_edge_vector
            counter_slide_edge_vector = other_edge.other_vert(v1).co - v1.co
            contraslide_direction = counter_slide_edge_vector
            
            slide_offset = (slide_direction) * multiplier * (angle_difference)
            counter_slide_offset = contraslide_direction * slide_offset.length
            slide_offset = slide_offset - counter_slide_offset
            
            target_offsets[v1.index] = slide_offset
    
    return target_offsets


def get_length(data):
    return data["slide_offset"].length


class SlideOptimizeOperator(bpy.types.Operator):
    """Slide all verts along the selected loop so that things look better"""

    bl_idname = "mesh.edge_slide_optimizer"
    bl_label = "Edge Slide Optimize"
    bl_options = {"REGISTER", "UNDO"}

    first_mouse_x: IntProperty()
    first_value: FloatProperty()
    strength: IntProperty(name="intensity")
    width: FloatProperty(
        name="Width",
        description="Box Width",
        min=0.01,
        max=100.0,
        default=1.0,
    )

    def opti_slide_get_slide_verts(self, iterations=10):
        all_slide_verts = []
        # limit iterations, then go with higher steps
        #        if iterations>20:
        #            multiplier = iterations/20
        #            iterations = 20
        #        print(iterations)
        for iter in range(0, iterations):
            ob = bpy.context.active_object
            me = ob.data
            bm = bmesh.from_edit_mesh(me)

            selected_verts = [v for v in bm.verts if v.select]
            selected_edges = [e for e in bm.edges if e.select]
            multiplier = 1.0

            move_verts = []

            total_length = 0
            for e in selected_edges:
                total_length += e.calc_length()
            avg_length = total_length / len(selected_edges)
            for v1 in selected_verts:
                # edges that can be slided on
                slide_directions = []
                slide_candidates_edges = []

                sel_edges_count = 0

                for e in v1.link_edges:
                    slide_directions.append(0)
                    if not e.select:
                        continue

                    sel_edges_count += 1
                    slide_candidates_edges.append(e)

                if sel_edges_count != 2:
                    continue
                if len(v1.link_edges) < 4:
                    continue
                move_verts.append(v1)

                # tot_normal = Vector((0, 0, 0))
                # for f in v1.link_faces:
                #     tot_normal += f.normal  # * f.calc_area()
                # tot_normal.normalize()
                tot_normal = v1.normal
                a = edge_angle(
                    slide_candidates_edges[0], slide_candidates_edges[1], tot_normal
                )

                for fi, f in enumerate(v1.link_faces):
                    for e1fi, e1 in enumerate(f.edges):
                        if v1 not in e1.verts:
                            continue
                        # and then selected edges

                        e2 = f.edges[e1fi - 1]
                        if v1 not in e2.verts:
                            continue
                        a = edge_angle(e2, e1, tot_normal)

                        # just to get the index in vert
                        for e3i, e3 in enumerate(v1.link_edges):
                            # stored by indices in the link_edges field
                            if e3 == e2 and e2.select is False:
                                slide_directions[e3i] += a
                            if e3 == e1 and e1.select is False:
                                slide_directions[e3i] += a

                slide_candidates = []
                for i, a in enumerate(slide_directions):
                    if a > 0:
                        slide_candidates.append([i, a])

                if len(slide_candidates) != 2:
                    continue
                else:
                    mina = 10000
                    minindex = -1
                    for c in slide_candidates:
                        if c[1] < mina:
                            mina = c[1]
                            minindex = c[0]

                    for c in slide_candidates:
                        if c[0] != minindex:
                            other_candidate_angle = c[1]
                            other_edge = v1.link_edges[c[0]]

                    slide_edge = v1.link_edges[minindex]
                    angle_difference = abs(
                        slide_candidates[0][1] - slide_candidates[1][1]
                    )

                    slide_edge_vector = slide_edge.other_vert(v1).co - v1.co
                    orig_length = slide_edge_vector.length
                    slide_direction = slide_edge_vector  # .normalized() * avg_length
                    counter_slide_edge_vector = other_edge.other_vert(v1).co - v1.co
                    contraslide_direction = (
                        counter_slide_edge_vector  # .normalized() * avg_length
                    )

                    # not used by now
                    slide_offset = (
                        (slide_direction) * 0.01 * multiplier * (angle_difference)
                    )
                    #                    if slide_offset.length>orig_length:
                    #                        slide_offset.length = orig_length
                    # keep same length
                    counter_slide_offset = contraslide_direction * slide_offset.length
                    slide_offset = slide_offset - counter_slide_offset

                    slide_position = slide_offset + v1.co
                all_slide_verts.append(
                    {
                        "position": slide_position,
                        "slide_offset": slide_offset,
                        "original_position": v1.co.copy(),
                        "angle_difference": angle_difference,
                        "index": v1.index,
                        "transform_multiplier": avg_length,
                        "slide_edge_index": slide_edge.index,
                        "counter_slide_edge_index": other_edge.index,
                    },
                )

            all_slide_verts.sort(key=get_length)
            all_slide_verts.reverse()

            for v1data in all_slide_verts:
                # if v1data['slide_offset'].length>0.0001:
                bm.verts[v1data["index"]].co = v1data["position"]
            bmesh.update_edit_mesh(me)
        return all_slide_verts

    def slide_vertices(self, iterations=10):
        #        ob = bpy.context.active_object
        #        me= ob.data
        #        bm = bmesh.from_edit_mesh(me)
        self.restore_init_positions()
        self.opti_slide_get_slide_verts(iterations=iterations)

    #        bmesh.update_edit_mesh(me)

    def store_init_positions(self):
        ob = bpy.context.active_object
        me = ob.data
        bm = bmesh.from_edit_mesh(me)
        saved_positions = []
        selected_verts = [v for v in bm.verts if v.select]
        for v in selected_verts:
            saved_positions.append(v.co.to_tuple())
        self.saved_positions = saved_positions

    def restore_init_positions(self):
        ob = bpy.context.active_object
        me = ob.data
        bm = bmesh.from_edit_mesh(me)
        selected_verts = [v for v in bm.verts if v.select]
        for i, v in enumerate(selected_verts):
            v.co = self.saved_positions[i]
        bmesh.update_edit_mesh(me)

    def modal(self, context, event):
        if event.type == "MOUSEMOVE":
            delta = event.mouse_x - self.first_mouse_x

            # self.slide_vertices(iterations=int(delta * .2))
            self.slide_vertices(iterations=200)  # int(delta *.2))
            # return {'RUNNING_MODAL'}
            return {"FINISHED"}
        elif event.type == "LEFTMOUSE" and event.value == "RELEASE":
            return {"FINISHED"}

        elif event.type in {"RIGHTMOUSE", "ESC"}:
            self.restore_init_positions()
            #            context.object.location.x = self.first_value
            return {"CANCELLED"}
        return {"RUNNING_MODAL"}

    def invoke(self, context, event):
        if context.active_object:
            self.first_mouse_x = event.mouse_x
            self.first_value = context.active_object.location.x
            self.store_init_positions()
            self.slide_verts = self.opti_slide_get_slide_verts()

            context.window_manager.modal_handler_add(self)
            return {"RUNNING_MODAL"}
        else:
            self.report({"WARNING"}, "No active object, could not finish")
            return {"CANCELLED"}


# class FunTopologyOperator(bpy.types.Operator):
#     bl_idname = "mesh.fun_topology"
#     bl_label = "Fun Topology Operator"
#     bl_options = {'REGISTER', 'UNDO'}
#
#     max_iterations: bpy.props.IntProperty(name="Max Iterations", default=10)
#     max_distance: bpy.props.FloatProperty(name="Max Distance", default=0.1)
#     min_distance: bpy.props.FloatProperty(name="Min Distance", default=0.01)
#
#     def modal(self, context, event):
#         if event.type in {'RIGHTMOUSE', 'ESC'}:
#             self.cancel(context)
#             return {'CANCELLED'}
#
#         if event.type == 'TIMER':
#             self.iteration += 1
#             if self.iteration > self.max_iterations:
#                 self.cancel(context)
#                 return {'FINISHED'}
#
#             global draw_lines
#             global draw_faces
#             user_preferences = bpy.context.preferences.addons['final_topology'].preferences
#
#             draw_lines.clear()
#             draw_faces.clear()
#             draw_faces_list.clear()
#             tool_settings = context.tool_settings
#             bpy.context.view_layer.update()
#
#             obj = bpy.context.edit_object
#             me = obj.data
#             bm = bmesh.from_edit_mesh(me)
#
#             selected_verts = [v for v in bm.verts if v.select]
#
#             # we need to evaluate result subdivided mesh every iteration,
#             # so it's affected by previous iterations
#             depsgraph = bpy.context.evaluated_depsgraph_get()
#
#             bm_eval = get_evaluated_bm(obj, depsgraph)
#
#             active_obj = bpy.context.active_object
#             if active_obj.final_topology.use_object_or_collection == "COLLECTION":
#                 target_objects = active_obj.final_topology.target_collection.objects
#             else:
#                 target_objects = [active_obj.final_topology.target_object]
#             level_subs_neighbours = 1 * 2 ** (get_subdivision_modifier_level(obj) - 1)
#             for v in bm.verts:
#                 offset = calculate_subdivide_offset(obj, target_objects, bm_eval, v, depsgraph, level_subs_neighbours)
#                 v.co += offset
#
#             bmesh.update_edit_mesh(me)
#
#             return {'PASS_THROUGH'}
#
#         return {'PASS_THROUGH'}
#
#     def execute(self, context):
#         self.iteration = 0
#         self.timer = context.window_manager.event_timer_add(0.1, window=context.window)
#         context.window_manager.modal_handler_add(self)
#         return {'RUNNING_MODAL'}
#
#     def cancel(self, context):
#         context.window_manager.event_timer_remove(self.timer)


def find_longest_shared_edges(bm, only_triangles=False):
    longest_shared_edges = []

    for face in bm.faces:
        # Skip if only_triangles is True and the face is not a triangle
        if only_triangles and len(face.verts) != 3:
            continue

        # Find the longest edge in the current face, considering only edges where the other linked_face is also a triangle
        if only_triangles:
            candidate_edges = [
                e for e in face.edges if all(len(f.verts) == 3 for f in e.link_faces)
            ]
        else:
            candidate_edges = face.edges

        if not candidate_edges:
            continue

        longest_edge = max(candidate_edges, key=lambda e: e.calc_length())

        # Check if the longest edge is also the longest for its neighboring faces
        for linked_face in longest_edge.link_faces:
            if linked_face != face:
                # Skip if only_triangles is True and the linked_face is not a triangle
                if only_triangles and len(linked_face.verts) != 3:
                    continue

                if only_triangles:
                    other_candidate_edges = [
                        e
                        for e in linked_face.edges
                        if all(len(f.verts) == 3 for f in e.link_faces)
                    ]
                else:
                    other_candidate_edges = linked_face.edges

                if not other_candidate_edges:
                    continue

                other_longest_edge = max(
                    other_candidate_edges, key=lambda e: e.calc_length()
                )

                if longest_edge == other_longest_edge:
                    longest_shared_edges.append(longest_edge)

    # Remove duplicates
    longest_shared_edges = list(set(longest_shared_edges))

    return longest_shared_edges


def get_selected_vertices(object, bm):
    return [v for v in bm.verts if v.select]


def set_selected_vertices(object, bm, verts):
    for v in bm.verts:
        v.select = False
    for v in verts:
        v.select = True


constraints_cache = []

# def compare_constraints(object):


def build_kd_curve_cache(source_curve, endpoints_only=False, flatten=False):
    # Ensure source_curve is a valid curve object
    if source_curve is None or source_curve.type != "CURVE":
        raise ValueError("source_curve must be a valid Blender Curve object")

    depsgraph = bpy.context.evaluated_depsgraph_get()
    bmesh_curve = utils.get_evaluated_bm(source_curve, depsgraph)
    curve_world_matrix = source_curve.matrix_world
    local_z = Vector((0, 0, 1))
    curve_plane_normal = source_curve.rotation_euler.to_matrix() @ local_z

    # Create a KDTree for efficient nearest point search
    size = len(bmesh_curve.verts)
    kd = kdtree.KDTree(size)

    just_verts = []
    if not endpoints_only:
        for i, v in enumerate(bmesh_curve.verts):
            if flatten:
                co = project_point_to_plane(
                    curve_world_matrix @ v.co,
                    source_curve.location,
                    curve_plane_normal,
                )
                kd.insert(
                    co,
                    i,
                )
                if DEBUG_DRAW:
                    draw.add_point(
                        co,
                        (1, 0, 0, 1),
                    )
                just_verts.append(co)
            else:
                kd.insert(curve_world_matrix @ v.co, i)
                if DEBUG_DRAW:
                    draw.add_point(curve_world_matrix @ v.co, (1, 0, 0, 1))
                just_verts.append(curve_world_matrix @ v.co)
    else:
        if flatten:
            co_start = project_point_to_plane(
                curve_world_matrix @ bmesh_curve.verts[0].co,
                source_curve.location,
                curve_plane_normal,
            )
            co_end = project_point_to_plane(
                curve_world_matrix @ bmesh_curve.verts[-1].co,
                source_curve.location,
                curve_plane_normal,
            )
            kd.insert(
                co_start,
                0,
            )
            kd.insert(
                co_end,
                size - 1,
            )
            if DEBUG_DRAW:
                draw.add_point(
                    co_start,
                    (1, 0, 0, 1),
                )
                draw.add_point(
                    co_end,
                    (1, 0, 0, 1),
                )
        else:
            kd.insert(curve_world_matrix @ bmesh_curve.verts[0].co, 0)
            kd.insert(curve_world_matrix @ bmesh_curve.verts[-1].co, size - 1)
            if DEBUG_DRAW:
                draw.add_point(
                    curve_world_matrix @ bmesh_curve.verts[0].co, (1, 0, 0, 1)
                )
                draw.add_point(
                    curve_world_matrix @ bmesh_curve.verts[-1].co, (1, 0, 0, 1)
                )

    kd.balance()
    return kd

def calculate_joined_normal(c, constraint_verts_loops):
    # Calculate average normal from all loops (each loop keeps its own center)
    loop_normals = []
    joined_center = Vector((0, 0, 0))
    total_verts = 0
    for loop_data in constraint_verts_loops:
        if len(loop_data[0]) > 2:
            _, loop_normal = utils.estimate_best_fit_plane(loop_data[0], "best_fit")
            loop_normals.append(loop_normal)
            for v in loop_data[0]:
                joined_center += v.co
                total_verts += 1
    joined_center /= total_verts
    if len(loop_normals) > 0:
        # Use stored normal as reference for consistency, or first loop if not available
        stored_normal = Vector(c.normal)
        if stored_normal.length > 0.01:
            reference_normal = stored_normal
        else:
            reference_normal = loop_normals[0]
        
        # Align all normals to reference direction using dot product
        joined_normal = Vector((0, 0, 0))
        for loop_normal in loop_normals:
            # Flip if pointing opposite direction (dot product < 0)
            if loop_normal.dot(reference_normal) < 0:
                loop_normal = -loop_normal
            joined_normal += loop_normal
        
        joined_normal.normalize()
        # Store for next frame consistency - but never overwrite a fixed
        # normal, the user owns that value
        if not c.fix_normal:
            c.normal = joined_normal
    return joined_normal, joined_center

def check_constraints_cache(object):
    global constraints_cache
    constraints_cache = []
    # same_cache = compare_constraints(object)
    if len(constraints_cache) != len(object.data.ft_custom_constraints):
        for c in object.data.ft_custom_constraints:
            cc_dict = {}
            if c.constraint_type == "CURVE" and c.target_curve is not None:
                endpoints_only = not c.target_curve.data.splines[0].use_cyclic_u
                kd = build_kd_curve_cache(
                    c.target_curve,
                    endpoints_only=endpoints_only,
                    flatten=c.projection == "PROJECTED",
                )
                cc_dict["kd"] = kd
            constraints_cache.append(cc_dict)


def _get_inclination_axis_vector(axis_name):
    axis_map = {
        "+X": Vector((1.0, 0.0, 0.0)),
        "-X": Vector((-1.0, 0.0, 0.0)),
        "+Y": Vector((0.0, 1.0, 0.0)),
        "-Y": Vector((0.0, -1.0, 0.0)),
        "+Z": Vector((0.0, 0.0, 1.0)),
        "-Z": Vector((0.0, 0.0, -1.0)),
    }
    return axis_map.get(axis_name, Vector((0.0, 0.0, -1.0)))


def inclination_limit_calculate(
    bmesh_edit,
    constrained_faces,
    axis_name="-Z",
    max_angle=50.0,
    threshold_faces_subdiv=None,
):
    """Limit face inclination by sliding lower-half vertices toward upper half with axis lock."""
    axis = _get_inclination_axis_vector(axis_name)
    max_angle = max(0.0, min(max_angle, 89.9))
    threshold_angle = radians(90.0 - max_angle)
    epsilon = 1e-6

    target_offsets = {}
    target_counts = {}

    def get_face_angle(face):
        face_normal = face.normal.copy()
        if face_normal.length_squared == 0.0:
            return None
        face_normal.normalize()
        return face_normal.angle(axis)

    subdiv_face_centers = []
    if threshold_faces_subdiv:
        subdiv_face_centers = [(f, f.calc_center_median()) for f in threshold_faces_subdiv]

    for face in constrained_faces:
        angle = get_face_angle(face)
        if angle is None:
            continue
        if subdiv_face_centers:
            face_center = face.calc_center_median()
            nearest_subdiv_face = min(
                subdiv_face_centers,
                key=lambda fc: (fc[1] - face_center).length_squared,
            )[0]
            subdiv_angle = get_face_angle(nearest_subdiv_face)
            if subdiv_angle is not None:
                angle = subdiv_angle
        if angle >= threshold_angle:
            continue

        verts = list(face.verts)

        # Split the face vertices into "lower" and "upper" halves along measured axis.
        sorted_verts = sorted(verts, key=lambda v: v.co.dot(axis), reverse=True)
        moving_count = max(1, min(len(sorted_verts) // 2, len(sorted_verts) - 1))
        moving_verts = sorted_verts[:moving_count]
        stationary_verts = sorted_verts[moving_count:]
        if len(moving_verts) == 0 or len(stationary_verts) == 0:
            continue
        moving_set = set(moving_verts)
        stationary_set = set(stationary_verts)

        # "Responsible" edges are the boundary edges that connect moving and stationary halves.
        responsible_edges = []
        for edge in face.edges:
            v1_moving = edge.verts[0] in moving_set
            v2_moving = edge.verts[1] in moving_set
            if v1_moving != v2_moving:
                responsible_edges.append(edge)

        # Scale correction by threshold violation; larger overshoot receives stronger correction.
        correction_scale = min(1.0, (threshold_angle - angle) / max(threshold_angle, 1e-6))
        for v in moving_verts:
            slide_direction = Vector((0.0, 0.0, 0.0))
            direction_count = 0

            # Primary direction: slide along responsible boundary edges.
            for edge in responsible_edges:
                if edge.verts[0] != v and edge.verts[1] != v:
                    continue
                other = edge.other_vert(v)
                if other not in stationary_set:
                    continue
                edge_vec = other.co - v.co
                edge_vec -= axis * edge_vec.dot(axis)
                if edge_vec.length_squared <= epsilon:
                    continue
                slide_direction += edge_vec
                direction_count += 1

            # Fallback: direct pull to closest stationary vertex, still axis-locked.
            if direction_count == 0:
                closest = min(stationary_verts, key=lambda sv: (sv.co - v.co).length_squared)
                edge_vec = closest.co - v.co
                edge_vec -= axis * edge_vec.dot(axis)
                if edge_vec.length_squared > epsilon:
                    slide_direction += edge_vec
                    direction_count = 1

            if direction_count == 0:
                continue
            slide_direction /= direction_count
            offset = slide_direction * correction_scale

            if v.index not in target_offsets:
                target_offsets[v.index] = Vector((0.0, 0.0, 0.0))
                target_counts[v.index] = 0
            target_offsets[v.index] += offset
            target_counts[v.index] += 1

    for v_index, count in target_counts.items():
        if count > 1:
            target_offsets[v_index] /= count

    return target_offsets


def curvature_loop_samples(verts, is_circular, measure="LENGTH"):
    """Measure curvature at every vertex of a loop that has one on both sides.

    measure 'LENGTH' uses the arc length parametrization, so both the parameter
    and the second derivative are built from the real edge lengths - the
    curvature is the one of the whole segment, not of an idealized evenly
    spaced polyline. measure 'ANGLE' takes only the turn angle at the vertex,
    so denser vertices allow the shape to turn more tightly.

    Returns a list of dicts with the loop index, the arc length parameter t,
    the curvature k along the vertex normal (positive when the loop bulges in the
    normal direction), the two neighbouring edge lengths and the vertex normal.
    """
    count = len(verts)

    # arc length parameter along the loop
    tknots = [0.0] * count
    length_total = 0.0
    for i in range(1, count):
        length_total += (verts[i].co - verts[i - 1].co).length
        tknots[i] = length_total

    if is_circular:
        indices = range(count)
    else:
        # endpoints have no curvature defined, they anchor the loop
        indices = range(1, count - 1)

    # vertex normals mean nothing on wire vertices without any faces - Blender
    # fills them from the normalized vertex position, which points anywhere.
    # The loop's own plane crossed with the local tangent gives the in-plane
    # curve normal instead, so a flat wire loop is handled within its plane.
    wire_plane_normal = None
    if count >= 3 and any(not getattr(v, "link_faces", True) for v in verts):
        _, wire_plane_normal = utils.estimate_best_fit_plane(verts, "best_fit")
        wire_plane_normal = Vector(wire_plane_normal)
        if wire_plane_normal.length_squared < 0.5:
            wire_plane_normal = None

    samples = []
    for i in indices:
        # negative index wraps to the last vertex on circular loops
        prev_co = verts[i - 1].co
        next_co = verts[(i + 1) % count].co
        co = verts[i].co

        h1 = (co - prev_co).length
        h2 = (next_co - co).length
        if h1 < 1e-9 or h2 < 1e-9:
            continue

        if wire_plane_normal is not None and not getattr(verts[i], "link_faces", True):
            normal = wire_plane_normal.cross(next_co - prev_co)
        else:
            normal = verts[i].normal.copy()
        if normal.length_squared < 1e-12:
            continue
        normal.normalize()

        if measure == "ANGLE":
            # turn between the incoming and outgoing direction along the normal,
            # segment lengths deliberately don't enter - vertex density then
            # decides how tightly the shape may turn
            d1 = (co - prev_co) / h1
            d2 = (next_co - co) / h2
            k = -(d2 - d1).dot(normal)
        else:
            # second derivative on an unevenly spaced grid
            second_derivative = (
                prev_co * h2 - co * (h1 + h2) + next_co * h1
            ) * (2.0 / (h1 * h2 * (h1 + h2)))
            k = -second_derivative.dot(normal)

        samples.append(
            {
                "index": i,
                "t": tknots[i],
                "k": k,
                "h1": h1,
                "h2": h2,
                "normal": normal,
            }
        )
    return samples


def fit_linear_curvature(samples, weights):
    """Weighted least squares fit of k(t) = a + b*t, returns the fitted curvatures."""
    sum_w = sum(weights)
    sum_t = sum(w * s["t"] for w, s in zip(weights, samples))
    sum_tt = sum(w * s["t"] * s["t"] for w, s in zip(weights, samples))
    sum_k = sum(w * s["k"] for w, s in zip(weights, samples))
    sum_tk = sum(w * s["t"] * s["k"] for w, s in zip(weights, samples))

    denominator = sum_w * sum_tt - sum_t * sum_t
    if abs(denominator) < 1e-12:
        return [sum_k / sum_w] * len(samples)

    b = (sum_w * sum_tk - sum_t * sum_k) / denominator
    a = (sum_k - b * sum_t) / sum_w
    return [a + b * s["t"] for s in samples]


def curvature_calculate(
    loops_data,
    mode="CONSTANT",
    max_offset_ratio=0.5,
    measure="LENGTH",
    movable_ranges=None,
):
    """Push vertices along their normals so the loop keeps an even curvature.

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        mode: 'CONSTANT' for one curvature along the loop (an arc),
              'LINEAR' for a curvature that changes at a constant rate.
        measure: 'LENGTH' evens out the true curvature, 'ANGLE' evens out the
            turn angle per vertex regardless of segment lengths.
        movable_ranges: optional list of (start, end) index ranges per loop.
            Every sample still feeds the estimate, but only vertices within
            the range receive offsets - the way surrounding context vertices
            shape the target without being moved themselves.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}
    offset_counts = {}

    for loop_index, loop_data in enumerate(loops_data):
        verts = loop_data[0]
        is_circular = loop_data[1]
        if len(verts) < 3:
            continue

        samples = curvature_loop_samples(verts, is_circular, measure)
        if len(samples) < 2:
            continue

        if measure == "ANGLE":
            # the angle is a per-vertex quantity, every vertex counts the same
            weights = [1.0] * len(samples)
        else:
            # each sample stands for the half edge on both of its sides
            weights = [(s["h1"] + s["h2"]) * 0.5 for s in samples]
        sum_w = sum(weights)
        if sum_w < 1e-9:
            continue

        # a curvature changing at a constant rate can't close on itself,
        # so circular loops always aim for a single curvature
        if mode == "LINEAR" and not is_circular and len(samples) >= 3:
            targets = fit_linear_curvature(samples, weights)
        else:
            average = sum(w * s["k"] for w, s in zip(weights, samples)) / sum_w
            targets = [average] * len(samples)

        movable = None
        if movable_ranges is not None:
            movable = movable_ranges[loop_index]

        for s, target in zip(samples, targets):
            if movable is not None and not (movable[0] <= s["index"] < movable[1]):
                continue
            if measure == "ANGLE":
                # moving a vertex by d along its normal changes its angle
                # by d * (h1+h2) / (h1*h2), invert that to hit the target
                offset = (target - s["k"]) * s["h1"] * s["h2"] / (s["h1"] + s["h2"])
            else:
                # moving a vertex by d along its normal changes its curvature
                # by d * 2 / (h1*h2), so invert that to hit the target curvature
                offset = (target - s["k"]) * s["h1"] * s["h2"] * 0.5
            # keep a single step from overshooting into a neighbour
            limit = max_offset_ratio * min(s["h1"], s["h2"])
            offset = max(-limit, min(limit, offset))
            add_loop_offset(
                target_offsets,
                offset_counts,
                verts[s["index"]].index,
                s["normal"] * offset,
            )

    return finish_loop_offsets(target_offsets, offset_counts)


def walk_loop_continuation(prev_vert, end_vert, max_steps, forbidden):
    """Continue an edge loop beyond its end, at most max_steps edges further.

    Follows the usual edge loop rule - the next edge at a vertex is the one
    sharing no face with the incoming one. Stops at poles, boundaries and
    vertices already claimed. Returns the vertices beyond end_vert, nearest
    first.
    """
    result = []
    if max_steps < 1:
        return result
    incoming = None
    for edge in end_vert.link_edges:
        if edge.other_vert(end_vert) == prev_vert:
            incoming = edge
            break
    if incoming is None:
        return result

    visited = set(forbidden)
    current = end_vert
    while len(result) < max_steps:
        incoming_faces = set(incoming.link_faces)
        candidates = [
            e
            for e in current.link_edges
            if e != incoming and not (set(e.link_faces) & incoming_faces)
        ]
        if len(candidates) != 1:
            break
        edge = candidates[0]
        next_vert = edge.other_vert(current)
        if next_vert is None or next_vert in visited:
            break
        result.append(next_vert)
        visited.add(next_vert)
        incoming = edge
        current = next_vert
    return result


def mean_vertex_normal(verts):
    """Normalized mean of the vertex normals, None when they cancel out."""
    normal = Vector((0.0, 0.0, 0.0))
    for v in verts:
        normal += v.normal
    if normal.length < 1e-9:
        return None
    return normal.normalized()


def line_verts_calculate(
    loops_data,
    distribution="ORIGINAL",
    fixed_lines=None,
    projected=False,
    draw_matrix=None,
    draw_color=None,
):
    """Pull the vertices of open loops onto a straight line.

    The line runs from the first to the last vertex of each loop, so the ends stay
    where they are and only the vertices between them move. With fixed_lines given
    (a list of (start, end) pairs, one per loop) each loop instead gets pulled onto
    the nearest stored line.

    With projected on, the constraint is only solved as seen along the loop's mean
    vertex normal - the offsets lose their component along it, so a loop lying on
    a curved surface straightens in the projected view while keeping the surface's
    relief.

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        distribution: 'ORIGINAL' moves each vertex straight onto the line and keeps
            its own position along it, 'EVEN' spreads the vertices between the ends.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}
    offset_counts = {}

    for loop_data in loops_data:
        verts = loop_data[0]
        is_circular = loop_data[1]
        # a closed loop has no two ends to span a line between
        if is_circular or len(verts) < 2:
            continue

        projection_axis = mean_vertex_normal(verts) if projected else None

        if fixed_lines:
            # each loop follows its own stored line - the nearest one, so the
            # match survives loops being re-discovered in a different order
            loop_mid = (verts[0].co + verts[-1].co) * 0.5
            line_start, line_end = min(
                fixed_lines,
                key=lambda se: ((se[0] + se[1]) * 0.5 - loop_mid).length_squared,
            )
        else:
            line_start = verts[0].co.copy()
            line_end = verts[-1].co.copy()

        direction = line_end - line_start
        line_length = direction.length
        if line_length < 1e-9:
            continue
        direction /= line_length

        if draw_matrix is not None and draw_color is not None:
            # the target line itself
            draw.add_line(draw_matrix @ line_start, draw_matrix @ line_end, draw_color)

        for i, v in enumerate(verts):
            if distribution == "EVEN":
                target = line_start + direction * (line_length * i / (len(verts) - 1))
            else:
                # closest point on the line, so the vertex only moves sideways onto it
                target = line_start + direction * (v.co - line_start).dot(direction)
            offset = target - v.co
            if projection_axis is not None:
                offset -= projection_axis * offset.dot(projection_axis)
            add_loop_offset(target_offsets, offset_counts, v.index, offset)

    return finish_loop_offsets(target_offsets, offset_counts)


def fit_circle_to_loop(verts):
    """Best-fit circle through the loop vertices.

    Fits the best plane first, then a least squares circle in that plane, so it
    recovers the true circle also from a partial arc - where the centroid with a
    mean radius would land far off.

    Returns (center, normal, radius) or None when the vertices don't define one.
    """
    if len(verts) < 3:
        return None

    plane_center, normal = utils.estimate_best_fit_plane(verts, "best_fit")
    normal = Vector(normal)
    if normal.length_squared < 1e-12:
        return None
    normal.normalize()
    u = normal.orthogonal().normalized()
    v = normal.cross(u)

    import numpy as np

    # Kasa fit: x^2 + y^2 + D*x + E*y + F = 0 is linear in D, E, F
    pts = [((vert.co - plane_center).dot(u), (vert.co - plane_center).dot(v)) for vert in verts]
    A = np.array([[x, y, 1.0] for x, y in pts])
    b = np.array([-(x * x + y * y) for x, y in pts])
    try:
        (D, E, F), _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    if rank < 3:
        # collinear points, no circle
        return None

    cx = -D / 2.0
    cy = -E / 2.0
    r_squared = cx * cx + cy * cy - F
    if r_squared <= 1e-18:
        return None

    center = plane_center + u * cx + v * cy
    return center, normal, sqrt(r_squared)


class _FitPoint:
    """Bare coordinate wrapper, lets computed points join a fit over vertices."""

    __slots__ = ("co",)

    def __init__(self, co):
        self.co = co


def seam_plane_for_loop(verts, mirror_planes):
    """The mirror plane both open loop ends sit on, None when there isn't one."""
    for plane_co, plane_no, threshold in mirror_planes:
        on_plane = True
        for v in (verts[0], verts[-1]):
            limit = max(threshold, 1e-6)
            if v.link_edges:
                # same generosity as the seam healing, a drifted seam end
                # still counts
                limit = max(limit, min(e.calc_length() for e in v.link_edges) * 0.3)
            if abs((v.co - plane_co).dot(plane_no)) > limit:
                on_plane = False
                break
        if on_plane:
            return plane_co, plane_no, threshold
    return None


def mirrored_fit_verts(verts, plane_co, plane_no, threshold):
    """The loop vertices plus their mirror images across the plane.

    A least squares fit over a mirror-symmetric point set comes out symmetric
    itself: the circle center lands on the plane and its normal in it, so the
    mirrored halves join into one circle instead of kinking at the seam.
    Vertices already sitting on the plane reflect onto themselves and are not
    duplicated.
    """
    points = list(verts)
    limit = max(threshold, 1e-6)
    for v in verts:
        distance = (v.co - plane_co).dot(plane_no)
        if abs(distance) > limit:
            points.append(_FitPoint(v.co - plane_no * (2.0 * distance)))
    return points


def circle_verts_calculate(
    loops_data,
    distribution="ORIGINAL",
    fixed_circles=None,
    projected=False,
    mirror_planes=None,
    join_center=False,
    join_normal=False,
    draw_matrix=None,
    draw_color=None,
):
    """Pull the vertices of loops onto a circle.

    Without fixed circles each loop gets its own best fit. With fixed_circles given
    (a list of (center, normal, radius) tuples, one per loop) each loop instead gets
    pulled onto the nearest stored circle. Open loops work too, they land on an arc
    of the circle.

    With projected on, the constraint is only solved as seen along the circle
    normal - the offsets lose their component along it, so the loop becomes a
    circle in the projected view while keeping its relief along the normal, e.g.
    a circle projected on a curved surface.

    With join_normal the loops share one averaged normal, with join_center they
    share one axis: each loop keeps its own position along its normal, so
    coplanar loops become concentric and stacked loops coaxial, without being
    flattened onto one plane. Each loop keeps its own radius, refitted around
    the shared values.

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        distribution: 'ORIGINAL' keeps each vertex at its own angle around the
            circle, 'EVEN' spreads the vertices at equal angles.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}
    offset_counts = {}

    # first fit every loop its own circle
    fitted_loops = []
    for loop_data in loops_data:
        verts = loop_data[0]
        is_circular = loop_data[1]
        if len(verts) < 3:
            continue

        if fixed_circles:
            # each loop follows its own stored circle - the nearest one, so the
            # match survives loops being re-discovered in a different order
            loop_center = Vector((0, 0, 0))
            for vert in verts:
                loop_center += vert.co
            loop_center /= len(verts)
            c, n, r = min(
                fixed_circles,
                key=lambda cnr: (cnr[0] - loop_center).length_squared,
            )
            if n.length_squared < 1e-12 or r <= 0.0:
                continue
            n = n.normalized()
        else:
            fit_verts = verts
            if mirror_planes and not is_circular:
                # an arc ending on a mirror plane must fit a circle that its
                # mirror image completes: fit it together with that image
                seam_plane = seam_plane_for_loop(verts, mirror_planes)
                if seam_plane is not None:
                    fit_verts = mirrored_fit_verts(verts, *seam_plane)
            fit = fit_circle_to_loop(fit_verts)
            if fit is None:
                continue
            c, n, r = fit

        fitted_loops.append([verts, is_circular, c, n, r])

    # stored circles already share whatever the user fixed - joining applies
    # to the live fits
    if (join_center or join_normal) and not fixed_circles and len(fitted_loops) > 1:
        if join_normal:
            reference = fitted_loops[0][3]
            joined_normal = Vector((0.0, 0.0, 0.0))
            for _, _, _, n, _ in fitted_loops:
                joined_normal += -n if n.dot(reference) < 0 else n
            if joined_normal.length_squared > 1e-12:
                joined_normal.normalize()
                for fitted in fitted_loops:
                    fitted[3] = joined_normal.copy()
        if join_center:
            joined_center = Vector((0.0, 0.0, 0.0))
            for _, _, c, _, _ in fitted_loops:
                joined_center += c
            joined_center /= len(fitted_loops)
            for fitted in fitted_loops:
                c, n = fitted[2], fitted[3]
                # share the axis but keep the loop's own height along it, so
                # stacked loops turn coaxial instead of collapsing together
                fitted[2] = joined_center + n * (c - joined_center).dot(n)
        # the shared center or normal moved the circle's plane - refit each
        # loop's radius around it
        for fitted in fitted_loops:
            verts, _, c, n, _ = fitted
            radius = 0.0
            for vert in verts:
                d = vert.co - c
                radius += (d - n * d.dot(n)).length
            fitted[4] = radius / len(verts)

    for verts, is_circular, c, n, r in fitted_loops:
        if r <= 1e-12:
            continue
        u = n.orthogonal().normalized()
        w = n.cross(u)

        if draw_matrix is not None and draw_color is not None:
            # the target circle itself
            previous = None
            for i in range(49):
                a = 2.0 * pi * i / 48
                point = draw_matrix @ (c + (u * cos(a) + w * sin(a)) * r)
                if previous is not None:
                    draw.add_line(previous, point, draw_color)
                previous = point

        # angle of each vertex around the circle, unwrapped along the loop so the
        # sequence carries the winding direction instead of jumping at +-pi
        angles = []
        for vert in verts:
            d = vert.co - c
            x = d.dot(u)
            y = d.dot(w)
            if x * x + y * y < 1e-18:
                # a vertex on the axis has no angle, reuse the previous one
                angles.append(angles[-1] if angles else 0.0)
            else:
                angles.append(atan2(y, x))
        unwrapped = [angles[0]]
        for i in range(1, len(angles)):
            delta = angles[i] - angles[i - 1]
            while delta > pi:
                delta -= 2.0 * pi
            while delta <= -pi:
                delta += 2.0 * pi
            unwrapped.append(unwrapped[-1] + delta)

        if distribution == "EVEN":
            if is_circular:
                # a closed loop spans the full turn, in its own winding direction
                sweep = 2.0 * pi if unwrapped[-1] >= unwrapped[0] else -2.0 * pi
                step = sweep / len(verts)
                base = [step * i for i in range(len(verts))]
                # rotate the even fan to where it needs the least total turning,
                # instead of pinning it to whichever vertex starts the loop - that
                # vertex would never feel a tangential pull, and combined with
                # other constraints the spacing could never balance out
                phase = sum(a - b for a, b in zip(unwrapped, base)) / len(verts)
                target_angles = [phase + b for b in base]
            else:
                # an open loop keeps its ends, the arc between them gets divided
                step = (unwrapped[-1] - unwrapped[0]) / (len(verts) - 1)
                target_angles = [unwrapped[0] + step * i for i in range(len(verts))]
        else:
            target_angles = unwrapped

        for vert, angle in zip(verts, target_angles):
            target = c + (u * cos(angle) + w * sin(angle)) * r
            offset = target - vert.co
            if projected:
                # solve only across the normal, the vertex keeps its own height
                offset -= n * offset.dot(n)
            add_loop_offset(target_offsets, offset_counts, vert.index, offset)

    return finish_loop_offsets(target_offsets, offset_counts)


def thickness_calculate(
    bm,
    verts,
    use_min=True,
    min_thickness=0.005,
    use_max=False,
    max_thickness=0.02,
    ray_length=0.05,
    draw_matrix=None,
    draw_alpha=0.4,
):
    """Keep the wall thickness under each vertex within the given bounds.

    Thickness is measured by a ray cast from the vertex inwards along its
    normal against the mesh itself. The ray starts a hair inside the surface
    and steps past faces that contain the very vertex it started from, so the
    corner geometry around the start can't read as a zero-thickness hit.
    Walls thicker than ray_length don't get measured and don't react.

    Vertices on too thin walls get pushed outwards along their normal, on too
    thick walls pulled inwards - by half the deficit, since the far side of
    the wall is usually constrained too and contributes the other half.

    With draw_matrix given, the surrounding faces are added to the overlay,
    colored by the deviation: red grading in for too thin, green within the
    bounds, blue grading in for too thick, transparent where nothing was in
    reach of the ray.

    Returns a dictionary mapping vertex indices to offset vectors.
    """
    from mathutils.bvhtree import BVHTree

    bvh = BVHTree.FromBMesh(bm)
    bm.faces.ensure_lookup_table()
    epsilon = 1e-5

    def measuring_normals(v, sharp_cos=0.7071):
        """Testing normals for a vertex, built from its surrounding faces.

        Faces of roughly the same orientation group into one larger surface
        and share its mean normal. Where faces meet at a sharp angle - box
        edges and corners - each side keeps its own normal, so a corner of a
        flat plate measures the plate's real thickness straight through it,
        instead of a slanted reading along the averaged diagonal normal.
        """
        clusters = []
        for f in v.link_faces:
            face_normal = f.normal
            if face_normal.length_squared < 1e-12:
                continue
            for cluster in clusters:
                if face_normal.dot(cluster.normalized()) > sharp_cos:
                    cluster += face_normal
                    break
            else:
                if len(clusters) < 4:
                    clusters.append(face_normal.copy())
        if clusters:
            return [c.normalized() for c in clusters]
        # wire vertex without faces, the vertex normal is all there is
        if v.normal.length_squared > 1e-12:
            return [v.normal.normalized()]
        return []

    def cast_through(v, direction):
        """Distance to the mesh along direction, skipping the own corner."""
        origin = v.co + direction * epsilon
        traveled = epsilon
        for _ in range(4):
            remaining = ray_length - traveled
            if remaining <= 0:
                return None
            location, _, face_index, distance = bvh.ray_cast(origin, direction, remaining)
            if location is None:
                return None
            if v in bm.faces[face_index].verts:
                # grazed a face of its own corner, step just past it
                advance = distance + epsilon
                origin = origin + direction * advance
                traveled += advance
                continue
            return traveled + distance
        return None

    thickness = {}
    wall_directions = {}
    for v in verts:
        # the wall is the nearest material any of the surface groups finds.
        # Inward rays are authoritative; outward ones only cover flipped
        # normals, so a narrow air cavity can't get mistaken for the wall.
        measured = None
        wall_direction = None
        directions = measuring_normals(v)
        for normal in directions:
            hit = cast_through(v, -normal)
            if hit is not None and (measured is None or hit < measured):
                measured = hit
                wall_direction = -normal
        if measured is None:
            for normal in directions:
                hit = cast_through(v, normal)
                if hit is not None and (measured is None or hit < measured):
                    measured = hit
                    wall_direction = normal
        thickness[v.index] = measured
        if wall_direction is not None:
            wall_directions[v.index] = wall_direction

    target_offsets = {}
    for v in verts:
        measured = thickness.get(v.index)
        if measured is None or v.index not in wall_directions:
            continue
        # push away from the opposing surface to thicken, toward it to thin
        away = -wall_directions[v.index]
        if use_min and measured < min_thickness:
            target_offsets[v.index] = away * ((min_thickness - measured) * 0.5)
        elif use_max and measured > max_thickness:
            target_offsets[v.index] = away * (-(measured - max_thickness) * 0.5)

    if draw_matrix is not None:
        def vert_color(vert):
            measured = thickness.get(vert.index)
            if measured is None:
                return (0.0, 0.0, 0.0, 0.0)
            if use_min and measured < min_thickness:
                ratio = max(measured / min_thickness, 0.0)
                return (1.0, ratio * 0.8, 0.0, draw_alpha)
            if use_max and measured > max_thickness:
                over = min((measured - max_thickness) / max_thickness, 1.0)
                return (0.0, 1.0 - over * 0.8, 0.4 + over * 0.6, draw_alpha)
            return (0.0, 1.0, 0.0, draw_alpha)

        faces = {f for v in verts if v.index in thickness for f in v.link_faces}
        for f in faces:
            face_verts = f.verts
            colors = [vert_color(fv) for fv in face_verts]
            if all(c[3] == 0.0 for c in colors):
                continue
            base = draw_matrix @ face_verts[0].co
            for i in range(1, len(face_verts) - 1):
                draw.add_colored_tri(
                    (base, draw_matrix @ face_verts[i].co, draw_matrix @ face_verts[i + 1].co),
                    (colors[0], colors[i], colors[i + 1]),
                )

        # the testing rays themselves: the ray that found the wall in the
        # vertex's deviation color, the other tested directions as grey stubs
        stub = min(ray_length, 1.5 * max(max_thickness if use_max else min_thickness, 1e-4))
        for v in verts:
            start = draw_matrix @ v.co
            wall_direction = wall_directions.get(v.index)
            measured = thickness.get(v.index)
            if wall_direction is not None and measured is not None:
                col = vert_color(v)
                draw.add_line(
                    start,
                    draw_matrix @ (v.co + wall_direction * measured),
                    (col[0], col[1], col[2], min(1.0, draw_alpha * 2.0)),
                )
            for normal in measuring_normals(v):
                inward = -normal
                if wall_direction is not None and (
                    (wall_direction - inward).length < 1e-6
                    or (wall_direction - normal).length < 1e-6
                ):
                    continue
                draw.add_line(
                    start,
                    draw_matrix @ (v.co + inward * stub),
                    (0.55, 0.55, 0.55, draw_alpha * 0.6),
                )

    return target_offsets


def surface_curvature_calculate(verts, factor=1.0, max_offset_ratio=0.3):
    """Fair the patch into a curvature-continuous blend with its surroundings.

    Works per quad direction: every vertex lies on two crossing edge loops,
    and along each of them the signed profile curvature (arc-length second
    difference against the vertex normal, the same math as the loop
    curvature constraint) is relaxed toward the mean of its two neighbours
    on that loop - harmonic curvature along the loop, anchored by the
    unselected surroundings. A crease spike diffuses into a fillet whose
    total turn is preserved, so a plane-to-wall corner rounds outward.

    Doing this per direction is the point: a cylinder ring has constant
    curvature along itself, so rings never move and necking is impossible.
    Earlier attempts used 2D mean curvature or umbrella biharmonic fairing,
    and both mix the circumferential direction into the profile - a change
    of radius then reads as curvature variation and pulls walls inward.

    Vertices where a direction can't be walked two steps (borders, poles,
    non-manifold fans) skip that direction; with no usable direction they
    stay anchored. Returns vertex index -> offset vector.
    """

    def continue_straight(prev_vert, through_edge):
        """The edge continuing through_edge across its far vertex, or None."""
        u = through_edge.other_vert(prev_vert)
        candidates = [
            e
            for e in u.link_edges
            if e is not through_edge and not set(e.link_faces) & set(through_edge.link_faces)
        ]
        if len(candidates) != 1:
            return None
        return candidates[0]

    def walk(v, edge, steps):
        """Vertices reached by walking straight from v through edge."""
        chain = []
        current, through = v, edge
        for _ in range(steps):
            nxt = through.other_vert(current)
            chain.append(nxt)
            through = continue_straight(current, through)
            if through is None:
                break
            current = nxt
        return chain

    def signed_k(prev_v, v, next_v):
        """Arc-length curvature at v along prev->v->next, on v's normal."""
        h1 = (v.co - prev_v.co).length
        h2 = (next_v.co - v.co).length
        if h1 < 1e-12 or h2 < 1e-12:
            return None
        second = 2.0 * (h2 * prev_v.co - (h1 + h2) * v.co + h1 * next_v.co) / (
            h1 * h2 * (h1 + h2)
        )
        return -second.dot(v.normal), h1, h2

    target_offsets = {}
    offset_counts = {}
    for v in verts:
        if not v.link_faces:
            continue
        # pair the edges into the two crossing loop directions
        paired = set()
        for edge in v.link_edges:
            if edge in paired:
                continue
            partner = [
                e
                for e in v.link_edges
                if e is not edge and not set(e.link_faces) & set(edge.link_faces)
            ]
            if len(partner) != 1:
                continue
            partner = partner[0]
            paired.add(edge)
            paired.add(partner)

            back = walk(v, edge, 2)
            fore = walk(v, partner, 2)
            if len(back) < 2 or len(fore) < 2:
                continue
            own = signed_k(back[0], v, fore[0])
            k_back = signed_k(back[1], back[0], v)
            k_fore = signed_k(v, fore[0], fore[1])
            if own is None or k_back is None or k_fore is None:
                continue
            k, h1, h2 = own
            target = 0.5 * (k_back[0] + k_fore[0])
            step = (target - k) * h1 * h2 * 0.5 * factor
            limit = max_offset_ratio * min(h1, h2)
            step = max(-limit, min(limit, step))
            add_loop_offset(target_offsets, offset_counts, v.index, v.normal * step)
    return finish_loop_offsets(target_offsets, offset_counts)


def smooth_verts_calculate(verts, factor=0.5, mode="BLEND"):
    """Smooth each vertex toward its neighbours.

    BLEND pulls each vertex toward its neighbours' average, like Blender's
    own vertex smoothing. With the region's boundary held by unselected
    surroundings this converges to a soap-film-like transition between the
    shapes - the mode for blending. Selecting a whole closed mesh leaves no
    anchor, there it deflates like any Laplacian smooth would.

    SHAPE is Taubin lambda/mu smoothing: a positive pass followed by a
    slightly stronger negative pass, plus removal of the mean normal drift.
    It filters out high frequency jaggedness while preserving the overall
    shape and volume - the mode for polishing without losing form.

    In both modes, vertices on boundary, wire or non-manifold edges only
    average across those feature edges - smoothing the border as a curve
    instead of letting the one-sided face fan drag them inwards. Feature
    corners and border endpoints, where the count of such edges isn't two,
    stay anchored.

    Returns a dictionary mapping vertex indices to offset vectors.
    """
    lam = factor
    mu = -1.06 * factor  # Taubin's classic 0.5/-0.53 ratio

    neighbor_map = {}
    for v in verts:
        if not v.link_edges:
            continue
        feature_edges = [e for e in v.link_edges if len(e.link_faces) != 2]
        if feature_edges:
            if len(feature_edges) != 2:
                continue
            # sharp feature corners stay put too - smoothing would cut them
            a = feature_edges[0].other_vert(v)
            b = feature_edges[1].other_vert(v)
            d1 = (v.co - a.co).normalized()
            d2 = (b.co - v.co).normalized()
            if d1.dot(d2) < 0.7071:  # bends more than 45 degrees
                continue
            edges = feature_edges
        else:
            edges = v.link_edges
        neighbor_map[v] = [edge.other_vert(v) for edge in edges]

    def averaged(positions, apply_factor):
        result = {}
        for v, neighbors in neighbor_map.items():
            average = Vector((0.0, 0.0, 0.0))
            for n in neighbors:
                average += positions.get(n.index, n.co)
            average /= len(neighbors)
            current = positions.get(v.index, v.co)
            result[v.index] = current + (average - current) * apply_factor
        return result

    first_pass = averaged({}, lam)
    if mode == "BLEND":
        return {v.index: first_pass[v.index] - v.co for v in neighbor_map}

    second_pass = averaged(first_pass, mu)
    offsets = {v.index: second_pass[v.index] - v.co for v in neighbor_map}

    # the lambda/mu pair still eats a little genuine curvature every pass,
    # and the modal applies it endlessly - so a curved region would slowly
    # deflate. The high frequency wiggle the smoothing is for has offsets
    # alternating along the normals, mean near zero; the deflation is their
    # shared inward component. Removing the mean normal drift keeps the
    # wiggle removal and cancels the shrink.
    if offsets:
        drift = sum(offsets[v.index].dot(v.normal) for v in neighbor_map)
        drift /= len(neighbor_map)
        for v in neighbor_map:
            offsets[v.index] = offsets[v.index] - v.normal * drift
    return offsets


def draw_loop_deviation(loops_data, target_offsets, matrix, alpha):
    """Gradient along the loops showing how far each vertex still is from
    satisfying the constraint - green settled, through yellow to red.

    The pending correction offsets are the deviation: they shrink to zero as
    the constraint converges, so the loop visibly cools down while it solves.
    """
    for loop_data in loops_data:
        verts = loop_data[0]
        is_circular = loop_data[1]
        if len(verts) < 2:
            continue
        total = 0.0
        for i in range(1, len(verts)):
            total += (verts[i].co - verts[i - 1].co).length
        # a correction of a quarter edge length reads as fully red
        scale = max(total / max(len(verts) - 1, 1) * 0.25, 1e-9)
        colors = []
        for v in verts:
            offset = target_offsets.get(v.index)
            s = min((offset.length if offset is not None else 0.0) / scale, 1.0)
            colors.append((min(2.0 * s, 1.0), min(2.0 * (1.0 - s), 1.0), 0.0, alpha))
        count = len(verts)
        last = count if is_circular else count - 1
        for i in range(last):
            j = (i + 1) % count
            draw.add_colored_line(
                (matrix @ verts[i].co, matrix @ verts[j].co),
                (colors[i], colors[j]),
            )


def evaluate_constraints(object, bmesh_edit=None, bmesh_eval=None, inverse_subdivide_prep=None):
    global constraints_cache
    # separate caching for performance
    check_constraints_cache(object)
    cs = object.data.ft_custom_constraints
    target_offsets_all = []
    user_preferences = bpy.context.preferences.addons[__package__].preferences

    # vertices held by pin constraints: they move only under the user's own
    # transforms, no constraint nor inverse subdivision may touch them
    pinned_verts = set()
    for c in cs:
        if c.constraint_type == "PIN" and c.enabled:
            layer = bmesh_edit.verts.layers.float.get(c.attribute_name)
            if layer is None:
                continue
            for v in bmesh_edit.verts:
                if v[layer] == 1.0:
                    pinned_verts.add(v.index)
                    if user_preferences.enable_draw_constraints:
                        draw.add_pin(object.matrix_world @ v.co)

    # remember which vertices sit on a mirror seam before anything moves
    mirror_data = []
    mirror_seam_sets = []
    if user_preferences.use_mirror:
        mirror_data = utils.get_mirror_data(object)
        if mirror_data:
            mirror_seam_sets = utils.collect_mirror_seam_verts(bmesh_edit, mirror_data)
            # pins outrank the seam snapping too
            mirror_seam_sets = [seam - pinned_verts for seam in mirror_seam_sets]

    for i, c in enumerate(cs):
        # skip disabled constraints
        if not c.enabled:
            continue
        # pins don't compute anything, they only exempt their vertices
        if c.constraint_type == "PIN":
            continue

        target_offsets = {}

        domain = get_constraint_domain_type(c.constraint_type)
        if c.constraint_type == "INCLINATION_LIMIT":
            constraint_elements_edit = utils.get_attribute_elements(
                object, bmesh_edit, c, domain=domain, as_domain="FACE"
            )
            threshold_faces_subdiv = None
            if c.works_on_subdivision and bmesh_eval is not None:
                face_layer_keys = bmesh_eval.faces.layers.float.keys()
                if c.attribute_name in face_layer_keys:
                    threshold_faces_subdiv = utils.get_attribute_elements(
                        object, bmesh_eval, c, domain=domain, as_domain="FACE"
                    )
        else:
            # Get the vertices affected by the constraint
            constraint_elements_edit = utils.get_attribute_elements(
                object, bmesh_edit, c, domain=domain, as_domain="POINT"
            )

        if len(constraint_elements_edit) == 0:
            continue

        if c.constraint_type in ("INCLINATION_LIMIT", "INVERSE_SUBDIVIDE", "THICKNESS", "SMOOTH"):
            # face and point domain constraints don't use the loop format -
            # wrapping their flat element lists as loops would crash the
            # subdivision mapping below
            constraint_verts_loops = []
        else:
            # All edge-based constraints receive multi-loop format: [[verts1, is_circular1], [verts2, is_circular2], ...]
            # Ensure it's always in multi-loop format
            if isinstance(constraint_elements_edit[0], list) and len(constraint_elements_edit[0]) == 2:
                # Already multi-loop format
                constraint_verts_loops = constraint_elements_edit
            else:
                # Convert single loop to multi-loop format
                constraint_verts_loops = [constraint_elements_edit]

        # Handle subdivision
        constraint_verts_loops_edit_for_endpoints = None
        if c.works_on_subdivision:
            constraint_verts_loops_edit_for_endpoints = constraint_verts_loops
            constraint_verts_loops_subdiv = []
            for loop_data in constraint_verts_loops:
                new_verts = []
                for v in loop_data[0]:
                    new_verts.append(bmesh_eval.verts[v.index])
                constraint_verts_loops_subdiv.append([new_verts, loop_data[1]])
            constraint_verts_loops = constraint_verts_loops_subdiv
        # evaluate plane constraint
        if c.constraint_type == "PLANE":
            # PLANE constraint handles loops internally based on fix_center and fix_normal
            # Calculate common center/normal if join options are enabled
            common_center = None
            joined_normal = None
            
            if (c.join_center and not c.fix_center) or (c.join_normal and not c.fix_normal):
                # Calculate common center and normal from all loops
                joined_normal, joined_center = calculate_joined_normal(c,constraint_verts_loops)
            if not c.join_center:
                joined_center = None
            if not c.join_normal:
                joined_normal = None
            
            for loop_data in constraint_verts_loops:
                if len(loop_data[0]) > 2:
                    # Determine center and normal for this loop
                    center = Vector(c.center) if c.fix_center else joined_center
                    normal = Vector(c.normal) if c.fix_normal else joined_normal
                    
                    loop_offsets = utils.flatten_verts_calculate(
                        loop_data[0],
                        slide=False,
                        center=center,
                        normal=normal,
                        fix_center=c.fix_center or (common_center is not None),
                        fix_normal=c.fix_normal or (joined_normal is not None),
                    )
                    target_offsets.update(loop_offsets)
                

        # evaluate curve constraint
        elif c.constraint_type == "CURVE":
            # CURVE handles single loop - extract first
            constraint_verts_loop = constraint_verts_loops[0]
            if c.target_curve is not None and c.target_curve.type == "CURVE":
                for idx, loop_data in enumerate(constraint_verts_loops):
                    edit_loop_data = None
                    if constraint_verts_loops_edit_for_endpoints is not None:
                        edit_loop_data = constraint_verts_loops_edit_for_endpoints[idx]
                    loop_offsets = to_curve_verts_calculate(
                        loop_data,
                        curve_snapping="PROJECT_PLANE" if c.projection == "PROJECTED" else "3D",
                        curve_distribution="EVEN" if c.even_distribution else "ORIGINAL",
                        kd=constraints_cache[i]["kd"],
                        source_curve=c.target_curve,
                        edit_loop_for_endpoints=edit_loop_data,
                    )
                    target_offsets.update(loop_offsets)

        # evaluate inverse subdivide constraint
        elif c.constraint_type == "INVERSE_SUBDIVIDE":
            from . import inverse_subdivide

            # Get constraint-specific target objects and normal offset.
            constraint_target_objects = []
            if c.use_object_or_collection == "COLLECTION":
                if c.invsubdiv_target_collection:
                    for ob in c.invsubdiv_target_collection.objects:
                        if ob.type == "MESH" and ob.visible_get():
                            constraint_target_objects.append(ob)
            elif c.use_object_or_collection == "OBJECT":
                if c.invsubdiv_target_object and c.invsubdiv_target_object.type == "MESH":
                    constraint_target_objects.append(c.invsubdiv_target_object)
            else:  # SCENE
                constraint_target_objects = [
                    obj for obj in bpy.context.scene.objects
                    if (obj.visible_get() and obj.type == "MESH" and obj != object)
                ]

            if len(constraint_target_objects) > 0:
                # Create constraint-specific prep data
                constraint_prep = inverse_subdivide_prep.copy()
                constraint_prep["target_objects"] = constraint_target_objects
                constraint_prep["normal_offset"] = c.invsubdiv_normal_offset

                target_offsets = inverse_subdivide.evaluate_inverse_subdivide(
                    object,
                    bmesh_edit,
                    bmesh_eval,
                    constraint_prep,
                    attribute_name=c.attribute_name,
                    normal_offset=c.invsubdiv_normal_offset,
                )

        # evaluate thickness constraint
        elif c.constraint_type == "THICKNESS":
            measure_bm = bmesh_edit
            measure_verts = constraint_elements_edit
            # ray directions come from the vertex normals, keep them in step
            # with the moves of the previous iterations
            bmesh_edit.normal_update()
            if c.works_on_subdivision and bmesh_eval is not None:
                bmesh_eval.verts.ensure_lookup_table()
                measure_bm = bmesh_eval
                measure_verts = [bmesh_eval.verts[v.index] for v in constraint_elements_edit]
            draw_matrix = None
            if (
                user_preferences.enable_draw_constraints
                and cs[object.data.ft_custom_constraints_index] == c
            ):
                draw_matrix = object.matrix_world
            target_offsets = thickness_calculate(
                measure_bm,
                measure_verts,
                use_min=c.thickness_use_min,
                min_thickness=c.thickness_min,
                use_max=c.thickness_use_max,
                max_thickness=c.thickness_max,
                ray_length=c.thickness_ray_length,
                draw_matrix=draw_matrix,
                draw_alpha=0.4 * user_preferences.overlays_alpha,
            )

        # evaluate smooth constraint
        elif c.constraint_type == "SMOOTH":
            measure_verts = constraint_elements_edit
            if c.works_on_subdivision and bmesh_eval is not None:
                bmesh_eval.verts.ensure_lookup_table()
                measure_verts = [bmesh_eval.verts[v.index] for v in constraint_elements_edit]
            if c.smooth_mode == "CURVATURE":
                target_offsets = surface_curvature_calculate(
                    measure_verts, factor=c.smooth_factor
                )
            else:
                target_offsets = smooth_verts_calculate(
                    measure_verts, factor=c.smooth_factor, mode=c.smooth_mode
                )

        # evaluate inclination limit constraint
        elif c.constraint_type == "INCLINATION_LIMIT":
            target_offsets = inclination_limit_calculate(
                bmesh_edit,
                constrained_faces=constraint_elements_edit,
                axis_name=c.inclination_axis,
                max_angle=c.inclination_max_angle,
                threshold_faces_subdiv=threshold_faces_subdiv,
            )
        
        # evaluate slide optimize constraint
        elif c.constraint_type == "SLIDE_OPTIMIZE":
            # SLIDE_OPTIMIZE handles multi-loop format
            target_offsets = slide_optimize_calculate(constraint_verts_loops)
        
        # evaluate line constraint
        elif c.constraint_type == "LINE":
            # LINE handles multi-loop format
            fixed_lines = None
            if c.fix_line and len(c.fixed_lines) > 0:
                fixed_lines = [(Vector(item.start), Vector(item.end)) for item in c.fixed_lines]
            draw_matrix = None
            draw_color = None
            if (
                user_preferences.enable_draw_constraints
                and cs[object.data.ft_custom_constraints_index] == c
            ):
                draw_matrix = object.matrix_world
                draw_color = (c.color[0], c.color[1], c.color[2], 0.9)
            target_offsets = line_verts_calculate(
                constraint_verts_loops,
                distribution="EVEN" if c.even_distribution else "ORIGINAL",
                fixed_lines=fixed_lines,
                projected=c.projection == "PROJECTED",
                draw_matrix=draw_matrix,
                draw_color=draw_color,
            )

        # evaluate circle constraint
        elif c.constraint_type == "CIRCLE":
            # CIRCLE handles multi-loop format
            fixed_circles = None
            if c.fix_circle and len(c.fixed_circles) > 0:
                fixed_circles = [
                    (Vector(item.center), Vector(item.normal), item.radius)
                    for item in c.fixed_circles
                ]
            draw_matrix = None
            draw_color = None
            if (
                user_preferences.enable_draw_constraints
                and cs[object.data.ft_custom_constraints_index] == c
            ):
                draw_matrix = object.matrix_world
                draw_color = (c.color[0], c.color[1], c.color[2], 0.9)
            target_offsets = circle_verts_calculate(
                constraint_verts_loops,
                distribution="EVEN" if c.even_distribution else "ORIGINAL",
                fixed_circles=fixed_circles,
                projected=c.projection == "PROJECTED",
                mirror_planes=mirror_data,
                join_center=c.join_center,
                join_normal=c.join_normal,
                draw_matrix=draw_matrix,
                draw_color=draw_color,
            )

        # evaluate curvature constraint
        elif c.constraint_type == "CURVATURE":
            # CURVATURE handles multi-loop format
            loops_for_eval = constraint_verts_loops
            movable_ranges = None
            if c.curvature_context_steps > 0:
                # extend open loops with surrounding vertices along the edge
                # loops: they feed the curvature estimate, but never move -
                # the assigned loop blends into its surroundings this way
                edit_loops = constraint_verts_loops_edit_for_endpoints
                if edit_loops is None:
                    edit_loops = constraint_verts_loops
                loops_for_eval = []
                movable_ranges = []
                for loop_index, loop_data in enumerate(constraint_verts_loops):
                    verts_list = loop_data[0]
                    is_circ = loop_data[1]
                    prefix = []
                    suffix = []
                    edit_verts = edit_loops[loop_index][0]
                    if not is_circ and len(edit_verts) >= 2:
                        # walk in the edit cage, its topology carries the loops
                        forbidden = set(edit_verts)
                        walked_start = walk_loop_continuation(
                            edit_verts[1], edit_verts[0], c.curvature_context_steps, forbidden
                        )
                        walked_end = walk_loop_continuation(
                            edit_verts[-2],
                            edit_verts[-1],
                            c.curvature_context_steps,
                            forbidden | set(walked_start),
                        )
                        prefix = list(reversed(walked_start))
                        suffix = walked_end
                        if c.works_on_subdivision and bmesh_eval is not None:
                            prefix = [bmesh_eval.verts[v.index] for v in prefix]
                            suffix = [bmesh_eval.verts[v.index] for v in suffix]
                    loops_for_eval.append([prefix + list(verts_list) + suffix, is_circ])
                    if is_circ:
                        movable_ranges.append((0, len(verts_list)))
                    else:
                        # the loop's own end vertices stay anchored, even though
                        # the context samples beyond them would let them move
                        movable_ranges.append(
                            (len(prefix) + 1, len(prefix) + len(verts_list) - 1)
                        )
            target_offsets = curvature_calculate(
                loops_for_eval,
                mode=c.curvature_mode,
                measure=c.curvature_measure,
                movable_ranges=movable_ranges,
            )

        # evaluate space constraint
        elif c.constraint_type == "SPACE":
            # SPACE handles multi-loop format
            target_offsets = space_calculate(
                constraint_verts_loops,
                interpolation=c.space_interpolation,
                method=c.space_method,
                step_weight=user_preferences.step_weight,
            )

        # pinned vertices receive no offsets from anything
        if pinned_verts:
            for index in pinned_verts:
                target_offsets.pop(index, None)

        # gradient along the active constraint's loops: how far each vertex
        # still is from satisfying it
        if (
            user_preferences.enable_draw_constraints
            and constraint_verts_loops
            and cs[object.data.ft_custom_constraints_index] == c
        ):
            draw_loop_deviation(
                constraint_verts_loops,
                target_offsets,
                object.matrix_world,
                0.9 * user_preferences.overlays_alpha,
            )

        # move vertices to new locations
        # this has already weighting which might be a good idea with many constraints working together.
        if DEBUG_DRAW:
            for v_index in target_offsets.keys():
                draw.add_arrow(
                    bmesh_edit.verts[v_index].co,
                    bmesh_edit.verts[v_index].co + target_offsets[v_index],
                    (1, 0, 0, 1),
                    scale=1,
                )
        utils.move_verts_to_targets(bmesh_edit, target_offsets, weight=user_preferences.step_weight)

    # constraints must not pull the mirror seam apart - vertices that started
    # on a mirror plane get put back onto it, they may only slide along it
    if mirror_data:
        utils.snap_mirror_seam_verts(bmesh_edit, mirror_data, mirror_seam_sets)

    # if there's inclination limit constraint, we need to recalculate normals of faces
    has_inclination_limit_constraint = any(c.constraint_type == "INCLINATION_LIMIT" for c in object.data.ft_custom_constraints)
    if has_inclination_limit_constraint:
        bmesh_edit.normal_update()
    return bmesh_edit


class FunTopologyDecimateOperator(bpy.types.Operator):
    bl_idname = "mesh.fun_topology_decimate"
    bl_label = "Fun Topology Decimate Operator"
    bl_options = {"REGISTER", "UNDO"}

    max_iterations: bpy.props.IntProperty(name="Max Iterations", default=1)
    max_distance: bpy.props.FloatProperty(name="Max Distance", default=0.1)
    min_distance: bpy.props.FloatProperty(name="Min Distance", default=0.01)

    def modal(self, context, event):
        if event.type in {"RIGHTMOUSE", "ESC"}:
            self.cancel(context)
            return {"CANCELLED"}

        if event.type == "TIMER":
            self.iteration += 1
            if self.iteration > self.max_iterations:
                self.cancel(context)
                return {"FINISHED"}

            global draw_lines
            global draw_faces
            user_preferences = bpy.context.preferences.addons[
                "final_topology"
            ].preferences

            draw.clear_draw_list()
            tool_settings = context.tool_settings
            bpy.context.view_layer.update()

            obj = bpy.context.edit_object
            me = obj.data
            bm = bmesh.from_edit_mesh(me)

            dis_edges = find_longest_shared_edges(bm, only_triangles=True)
            bmesh.ops.dissolve_edges(
                bm, edges=dis_edges, use_verts=True, use_face_split=True
            )
            

            bmesh.update_edit_mesh(me)

            return {"PASS_THROUGH"}

        return {"PASS_THROUGH"}

    def execute(self, context):
        self.iteration = 0
        self.timer = context.window_manager.event_timer_add(0.1, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def cancel(self, context):
        context.window_manager.event_timer_remove(self.timer)


def update_constraint_index(self, context):
    if _restoring_undo:
        # don't touch the selection while undo puts the list back
        return
    # select constraint vertices
    constraints = context.object.data.ft_custom_constraints
    index = context.object.data.ft_custom_constraints_index
    if not (0 <= index < len(constraints)):
        # e.g. -1 after the last constraint was deleted
        return
    constraint = constraints[index]
    bm = bmesh.from_edit_mesh(context.object.data)
    draw.clear_draw_list()
    domain = get_constraint_domain_type(constraint.constraint_type)
    
    try:
        elements = utils.get_attribute_elements(
            context.object, bm, constraint, domain=domain, as_domain=domain
        )

        if len(elements) > 0:
            bpy.ops.mesh.select_all(action="DESELECT")
            for e in elements:
                e.select = True
    except:
        pass

def clear_constraints_cache():
    global constraints_cache
    constraints_cache = []


_suppress_undo_push = False

# Undo for the constraint list. The list lives in ID properties, which edit
# mode undo steps do not record - they only store the bmesh. What the bmesh
# DOES carry through undo is its attribute layers. So every constraint action
# stamps a version number into a hidden per-vertex attribute and files the
# matching constraint state in this session dict. When the user undoes or
# redoes, Blender rolls the attribute back or forward along with the mesh,
# and the undo_post/redo_post handler reads the stamped version and restores
# the filed constraint state for it. One Ctrl+Z, no mode hopping.
_undo_states = {}  # (mesh name, version) -> serialized constraint state
_restoring_undo = False
UNDO_VERSION_LAYER = "ft_undo_version"


def _serialize_props(property_group):
    data = {}
    for prop in property_group.bl_rna.properties:
        identifier = prop.identifier
        if identifier == "rna_type":
            continue
        if prop.type == "COLLECTION":
            data[identifier] = [
                _serialize_props(item) for item in getattr(property_group, identifier)
            ]
        elif prop.type == "POINTER":
            value = getattr(property_group, identifier)
            data[identifier] = None if value is None else value.name
        elif prop.is_readonly:
            continue
        elif getattr(prop, "is_array", False):
            data[identifier] = list(getattr(property_group, identifier))
        else:
            data[identifier] = getattr(property_group, identifier)
    return data


def _apply_props(property_group, data):
    # the type enum first, the other properties describe that type
    identifiers = sorted(data.keys(), key=lambda key: key != "constraint_type")
    for identifier in identifiers:
        value = data[identifier]
        prop = property_group.bl_rna.properties.get(identifier)
        if prop is None:
            continue
        if prop.type == "COLLECTION":
            collection = getattr(property_group, identifier)
            collection.clear()
            for item_data in value:
                _apply_props(collection.add(), item_data)
        elif prop.type == "POINTER":
            if value is None:
                setattr(property_group, identifier, None)
            else:
                if prop.fixed_type.identifier == "Collection":
                    container = bpy.data.collections
                else:
                    container = bpy.data.objects
                setattr(property_group, identifier, container.get(value))
        else:
            try:
                setattr(property_group, identifier, value)
            except Exception:
                pass


def _serialize_constraint_state(mesh):
    return {
        "constraints": [_serialize_props(c) for c in mesh.ft_custom_constraints],
        "index": mesh.ft_custom_constraints_index,
    }


def _apply_constraint_state(mesh, state):
    mesh.ft_custom_constraints.clear()
    for constraint_data in state["constraints"]:
        _apply_props(mesh.ft_custom_constraints.add(), constraint_data)
    mesh.ft_custom_constraints_index = min(
        state["index"], len(mesh.ft_custom_constraints) - 1
    )


def _read_undo_version(mesh):
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
        layer = bm.verts.layers.float.get(UNDO_VERSION_LAYER)
        if layer is None or len(bm.verts) == 0:
            return 0
        return int(max(v[layer] for v in bm.verts))
    attribute = mesh.attributes.get(UNDO_VERSION_LAYER)
    if attribute is None or len(attribute.data) == 0:
        return 0
    values = [0.0] * len(attribute.data)
    attribute.data.foreach_get("value", values)
    return int(max(values))


def _stamp_undo_version(mesh, version):
    # written to every vertex - newly created vertices default to 0, so the
    # maximum is what counts
    if mesh.is_editmode:
        bm = bmesh.from_edit_mesh(mesh)
        layer = bm.verts.layers.float.get(UNDO_VERSION_LAYER)
        if layer is None:
            layer = bm.verts.layers.float.new(UNDO_VERSION_LAYER)
        for v in bm.verts:
            v[layer] = version
        bmesh.update_edit_mesh(mesh, loop_triangles=False, destructive=False)
    else:
        attribute = mesh.attributes.get(UNDO_VERSION_LAYER)
        if attribute is None:
            attribute = mesh.attributes.new(
                name=UNDO_VERSION_LAYER, type="FLOAT", domain="POINT"
            )
        attribute.data.foreach_set("value", [float(version)] * len(attribute.data))


def record_constraint_undo_state(mesh):
    """File the constraint state for the mesh's current stamped version, if
    this session hasn't seen it yet - the pre-action baseline."""
    key = (mesh.name, _read_undo_version(mesh))
    if key not in _undo_states:
        _undo_states[key] = _serialize_constraint_state(mesh)


def push_constraint_undo(message):
    """Stamp a new version and file the constraint state for it.

    No undo step is pushed here - the UI pushes its own step right after the
    operator or property edit that got us called, and that step carries the
    freshly stamped attribute. Headless tests push their own steps.
    """
    if _suppress_undo_push or _restoring_undo:
        return
    obj = bpy.context.active_object
    if obj is None or obj.type != "MESH":
        return
    mesh = obj.data
    version = _read_undo_version(mesh) + 1
    _stamp_undo_version(mesh, version)
    _undo_states[(mesh.name, version)] = _serialize_constraint_state(mesh)


def checkpoint_memfile_undo(message):
    """Full object mode checkpoint, taken BEFORE an action that creates or
    deletes datablocks - those don't ride along with edit mode undo steps,
    only a memfile snapshot from before the action can bring them back.
    Reverting then takes two undo steps and hops through object mode."""
    if _suppress_undo_push:
        return
    try:
        if bpy.context.mode == "EDIT_MESH":
            bpy.ops.object.mode_set(mode="OBJECT")
            bpy.ops.ed.undo_push(message=message)
            bpy.ops.object.mode_set(mode="EDIT")
            bpy.ops.ed.undo_push(message=message)
        else:
            bpy.ops.ed.undo_push(message=message)
    except Exception:
        pass


def _sync_constraints_after_undo(scene=None):
    global _restoring_undo
    if _restoring_undo:
        return
    context = bpy.context
    objects = set(getattr(context, "objects_in_mode", None) or [])
    if context.active_object is not None:
        objects.add(context.active_object)
    for obj in objects:
        if obj.type != "MESH":
            continue
        mesh = obj.data
        state = _undo_states.get((mesh.name, _read_undo_version(mesh)))
        if state is None:
            continue
        if state == _serialize_constraint_state(mesh):
            continue
        _restoring_undo = True
        try:
            _apply_constraint_state(mesh, state)
        finally:
            _restoring_undo = False
        clear_constraints_cache()


@bpy.app.handlers.persistent
def _constraint_undo_post(scene, _depsgraph=None):
    _sync_constraints_after_undo(scene)


@bpy.app.handlers.persistent
def _constraint_redo_post(scene, _depsgraph=None):
    _sync_constraints_after_undo(scene)


@bpy.app.handlers.persistent
def _constraint_load_post(scene, _depsgraph=None):
    # baseline every mesh with constraints, so the first action in this
    # session has a pre-state to undo back to
    _undo_states.clear()
    for mesh in bpy.data.meshes:
        if len(mesh.ft_custom_constraints):
            record_constraint_undo_state(mesh)


def update_constraint_data(self, context):
    clear_constraints_cache()
    # constraint property edits happen in edit mode, where Blender's own
    # property undo push doesn't reliably record them
    push_constraint_undo("Change Constraint")


def update_constraint_type(self, context):
    """Type changes can move the constraint to another attribute domain -
    convert the stored attribute along, so the assignment survives the switch
    instead of crashing the evaluation with a missing layer."""
    clear_constraints_cache()
    if _restoring_undo:
        # the undo-restored attribute already matches the restored type
        return
    convert_constraint_attribute_domain(self, context)
    push_constraint_undo("Change Constraint Type")


def convert_constraint_attribute_domain(self, context):
    ob = context.object
    if ob is None or ob.type != "MESH" or ob.mode != "EDIT" or not self.attribute_name:
        return
    mesh = ob.data
    attribute = mesh.attributes.get(self.attribute_name)
    domain = get_constraint_domain_type(self.constraint_type)
    if attribute is None or domain is None or attribute.domain == domain:
        return

    # collect the marked vertices under the old domain
    bm = bmesh.from_edit_mesh(mesh)
    marked_verts = set()
    if attribute.domain == "POINT":
        layer = bm.verts.layers.float.get(self.attribute_name)
        if layer is not None:
            marked_verts = {v for v in bm.verts if v[layer] == 1.0}
    elif attribute.domain == "EDGE":
        layer = bm.edges.layers.float.get(self.attribute_name)
        if layer is not None:
            for e in bm.edges:
                if e[layer] == 1.0:
                    marked_verts.update(e.verts)
    elif attribute.domain == "FACE":
        layer = bm.faces.layers.float.get(self.attribute_name)
        if layer is not None:
            for f in bm.faces:
                if f[layer] == 1.0:
                    marked_verts.update(f.verts)

    # express them in the new domain, before the bmesh gets invalidated
    if domain == "POINT":
        values = [v in marked_verts for v in bm.verts]
    elif domain == "EDGE":
        values = [
            e.verts[0] in marked_verts and e.verts[1] in marked_verts
            for e in bm.edges
        ]
    else:  # FACE
        values = [all(v in marked_verts for v in f.verts) for f in bm.faces]

    # recreating the attribute needs object mode, like the other attribute helpers
    bpy.ops.object.mode_set(mode="OBJECT")
    mesh.attributes.remove(mesh.attributes.get(self.attribute_name))
    attribute = mesh.attributes.new(name=self.attribute_name, type="FLOAT", domain=domain)
    self.attribute_name = attribute.name
    attribute.data.foreach_set("value", values)
    bpy.ops.object.mode_set(mode="EDIT")


def update_plane_fix_center(self, context):
    """Capture the current loop center when the center gets fixed"""
    if _restoring_undo:
        return
    _capture_plane_fix(self, context, capture_center=True, capture_normal=False)
    push_constraint_undo("Change Constraint")


def update_plane_fix_normal(self, context):
    """Capture the current loop normal when the normal gets fixed"""
    if _restoring_undo:
        return
    _capture_plane_fix(self, context, capture_center=False, capture_normal=True)
    push_constraint_undo("Change Constraint")


def _capture_plane_fix(self, context, capture_center, capture_normal):
    """Refresh only the value whose fix flag was just toggled - the other one
    may hold a manual edit that must survive."""
    if self.constraint_type != "PLANE":
        return
    capture_center = capture_center and self.fix_center
    capture_normal = capture_normal and self.fix_normal
    if not (capture_center or capture_normal):
        return

    # Get the constraint's vertices
    mesh = context.active_object.data
    bm = bmesh.from_edit_mesh(mesh)

    # Get vertices from attribute
    if not self.attribute_name or self.attribute_name not in bm.edges.layers.float:
        print(f"Constraint {self.name} Attribute {self.attribute_name} not found")
        return

    loops = utils.get_attribute_elements(context.active_object, bm, self, domain="EDGE", as_domain="POINT")

    # Calculate common center and normal from all loops
    joined_normal, joined_center = calculate_joined_normal(self, loops)

    if capture_center:
        self.center = joined_center
    if capture_normal:
        self.normal = joined_normal


def update_line_fix_flags(self, context):
    """Capture the current ends of every open loop when the line gets fixed"""
    if _restoring_undo:
        return
    _capture_line_fix(self, context)
    push_constraint_undo("Change Constraint")


def _capture_line_fix(self, context):
    if self.constraint_type != "LINE" or not self.fix_line:
        return

    mesh = context.active_object.data
    bm = bmesh.from_edit_mesh(mesh)

    if not self.attribute_name or self.attribute_name not in bm.edges.layers.float:
        print(f"Constraint {self.name} Attribute {self.attribute_name} not found")
        return

    loops = utils.get_attribute_elements(
        context.active_object, bm, self, domain="EDGE", as_domain="POINT"
    )
    # one stored line per loop - a single shared line would suck all loops
    # onto the first one
    self.fixed_lines.clear()
    for loop_data in loops:
        verts = loop_data[0]
        is_circular = loop_data[1]
        if is_circular or len(verts) < 2:
            continue
        item = self.fixed_lines.add()
        item.start = verts[0].co.copy()
        item.end = verts[-1].co.copy()


def update_circle_fix_flags(self, context):
    """Capture the current best-fit circle of every loop when the circle gets fixed"""
    if _restoring_undo:
        return
    _capture_circle_fix(self, context)
    push_constraint_undo("Change Constraint")


def _capture_circle_fix(self, context):
    if self.constraint_type != "CIRCLE" or not self.fix_circle:
        return

    mesh = context.active_object.data
    bm = bmesh.from_edit_mesh(mesh)

    if not self.attribute_name or self.attribute_name not in bm.edges.layers.float:
        print(f"Constraint {self.name} Attribute {self.attribute_name} not found")
        return

    loops = utils.get_attribute_elements(
        context.active_object, bm, self, domain="EDGE", as_domain="POINT"
    )
    # one stored circle per loop - a single shared circle would suck all loops
    # onto the first one
    mirror_planes = []
    user_preferences = bpy.context.preferences.addons[__package__].preferences
    if user_preferences.use_mirror:
        mirror_planes = utils.get_mirror_data(context.active_object)
    self.fixed_circles.clear()
    for loop_data in loops:
        fit_verts = loop_data[0]
        if mirror_planes and not loop_data[1] and len(fit_verts) >= 2:
            seam_plane = seam_plane_for_loop(fit_verts, mirror_planes)
            if seam_plane is not None:
                fit_verts = mirrored_fit_verts(fit_verts, *seam_plane)
        fit = fit_circle_to_loop(fit_verts)
        if fit is not None:
            item = self.fixed_circles.add()
            item.center, item.normal, item.radius = fit


def filter_curves(self, object):
    return object.type == "CURVE"


class FixedLineItem(bpy.types.PropertyGroup):
    """One stored line of a fixed line constraint, one per loop"""

    start: bpy.props.FloatVectorProperty(name="Start", size=3, subtype="XYZ")
    end: bpy.props.FloatVectorProperty(name="End", size=3, subtype="XYZ")


class FixedCircleItem(bpy.types.PropertyGroup):
    """One stored circle of a fixed circle constraint, one per loop"""

    center: bpy.props.FloatVectorProperty(name="Center", size=3, subtype="XYZ")
    normal: bpy.props.FloatVectorProperty(name="Normal", size=3, subtype="XYZ")
    radius: bpy.props.FloatProperty(name="Radius", default=0.0, unit="LENGTH")


class CustomConstraint(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name="Name")
    constraint_type: bpy.props.EnumProperty(
        name="Type",
        default="PLANE",
        items=[
            ("PLANE", "Plane", "Planar constraint", "MESH_PLANE", 0),
            ("CURVE", "Curve", "Curve constraint", "CURVE_DATA", 2),
            (
                "INVERSE_SUBDIVIDE",
                "Inverse subdivide",
                "If added as a constraint, this works only on the assigned vertices, you can snap to more objects this way.",
                "MOD_SUBSURF",
                3,
            ),
            (
                "INCLINATION_LIMIT",
                "Inclination limit",
                "Limit maximal face inclination relative to selected axis, useful for 3D print overhang control",
                "ORIENTATION_GLOBAL",
                4,
            ),
            (
                "SLIDE_OPTIMIZE",
                "Slide Optimize",
                "Balance angles where quads meet by sliding vertices along edges",
                "DRIVER_ROTATIONAL_DIFFERENCE",
                5,
            ),
            (
                "SPACE",
                "Space",
                "Distribute vertices at equal distances along the loop",
                "TRACKING_FORWARDS_SINGLE",
                6,
            ),
            (
                "CURVATURE",
                "Curvature",
                "Even out the curvature along the loop by pushing vertices along their normals",
                "SPHERECURVE",
                7,
            ),
            (
                "LINE",
                "Line",
                "Straighten an open loop onto the line between its two ends."
                "\nClosed loops are skipped, they have no two ends",
                "IPO_LINEAR",
                8,
            ),
            (
                "CIRCLE",
                "Circle",
                "Pull the loop onto its best-fit circle."
                "\nOpen loops land on an arc of it",
                "MESH_CIRCLE",
                9,
            ),
            (
                "THICKNESS",
                "Thickness",
                "Keep the wall thickness under the vertices within bounds,"
                "\nmeasured by rays cast inwards against the mesh itself",
                "MOD_SOLIDIFY",
                10,
            ),
            (
                "PIN",
                "Pin",
                "Freeze the vertices against all constraints and inverse"
                "\nsubdivision - only your own transforms move them",
                "PINNED",
                11,
            ),
            (
                "SMOOTH",
                "Smooth",
                "Pull each vertex toward the average of its neighbours,"
                "\nlike the smooth vertices operator, a bit every iteration",
                "MOD_SMOOTH",
                12,
            ),
            # (
            #     "ANGLE",
            #     "Angle",
            #     "Limit angle for manufacturing purposes",
            #     "LINCURVE",
            #     7,
            # ),
        ],
        update=update_constraint_type,
    )
    fix_center: bpy.props.BoolProperty(
        name="Fix Center",
        default=False,
        description="Fix the plane center position",
        update=update_plane_fix_center,
    )
    fix_normal: bpy.props.BoolProperty(
        name="Fix Normal",
        default=False,
        description="Fix the plane normal/rotation",
        update=update_plane_fix_normal,
    )
    join_center: bpy.props.BoolProperty(
        name="Join Center",
        default=False,
        description="Calculate common center for all loops (only when center is not fixed)."
        "\nOn circles the loops share one axis - coplanar loops become concentric,"
        "\nstacked loops coaxial",
        update=update_constraint_data,
    )
    join_normal: bpy.props.BoolProperty(
        name="Join Normal",
        default=False,
        description="Calculate common normal for all loops (only when normal is not fixed)",
        update=update_constraint_data,
    )
    enabled: bpy.props.BoolProperty(name="Enabled", default=True)
    works_on_subdivision: bpy.props.BoolProperty(
        name="Works on Subdivision",
        default=False,
        description="\n OFF: works on control mesh, if \n ON: works on subdivided mesh.",
    )
    center: bpy.props.FloatVectorProperty(name="Center", size=3)
    normal: bpy.props.FloatVectorProperty(name="Normal", size=3)
    attribute_name: bpy.props.StringProperty(name="Attribute Name")
    color: bpy.props.FloatVectorProperty(
        name="Color", size=3, default=(1.0, 0.0, 0.0), subtype="COLOR"
    )
    
    # Inverse subdivide constraint properties
    use_object_or_collection: bpy.props.EnumProperty(
        name="Use Object or Collection",
        items=[
            ("SCENE", "Scene", "All evaluated objects in scene"),
            ("OBJECT", "Object", "Single object"),
            ("COLLECTION", "Collection", "Collection"),
        ],
        default="SCENE",
        description="Snap to",
    )
    invsubdiv_target_object: bpy.props.PointerProperty(
        type=bpy.types.Object, name="Target Object"
    )
    invsubdiv_target_collection: bpy.props.PointerProperty(
        type=bpy.types.Collection, name="Target Collection"
    )
    invsubdiv_normal_offset: bpy.props.FloatProperty(
        name="Normal Offset",
        default=0.0,
        soft_min=-0.2,
        soft_max=0.2,
        description="Normal offset",
        unit="LENGTH",
    )

    # Inclination limit constraint properties
    inclination_axis: bpy.props.EnumProperty(
        name="Axis",
        items=[
            ("+X", "+X", "Measure inclination against +X axis"),
            ("-X", "-X", "Measure inclination against -X axis"),
            ("+Y", "+Y", "Measure inclination against +Y axis"),
            ("-Y", "-Y", "Measure inclination against -Y axis"),
            ("+Z", "+Z", "Measure inclination against +Z axis"),
            ("-Z", "-Z", "Measure inclination against -Z axis"),
        ],
        default="-Z",
        description="Axis used for inclination measurement",
    )
    inclination_max_angle: bpy.props.FloatProperty(
        name="Max Inclination",
        default=50.0,
        min=0.0,
        max=89.9,
        description="Maximum allowed inclination angle in degrees (compared using 90-angle against face normal/axis angle)",
    )
    
    # Curve constraint properties
    target_curve: bpy.props.PointerProperty(
        type=bpy.types.Object,
        name="Target Curve",
        update=update_constraint_data,
        poll=filter_curves,
    )
    # shared by the curve, line and circle constraints
    projection: bpy.props.EnumProperty(
        name="Projection",
        default="3D",
        items=[
            ("3D", "3D", "Solve the constraint fully in 3D"),
            (
                "PROJECTED",
                "Projected",
                "Solve the constraint only as seen along its projection axis -"
                "\nthe curve plane, the circle normal, or the line loop's mean"
                "\nvertex normal. Vertices keep their offset along the axis, so"
                "\nthe shape can follow a curved surface",
            ),
        ],
        update=update_constraint_data,
    )
    # shared by the curve, line and circle constraints
    even_distribution: bpy.props.BoolProperty(
        name="Even",
        default=False,
        description="Spread the vertices evenly along the target,"
        "\ninstead of each keeping its own position on it",
        update=update_constraint_data,
    )


    # Space constraint properties
    space_method: bpy.props.EnumProperty(
        name="Spacing",
        default="EVEN",
        items=[
            ("EVEN", "Even", "Distribute vertices at equal distances along the loop"),
            (
                "RATIO",
                "Same Ratio",
                "Keep one constant ratio between neighbouring segment lengths,"
                "\nfitted to the loop's current tendency."
                "\nOpen loops only, closed loops fall back to even spacing",
            ),
        ],
        description="How the vertices get distributed along the loop",
        update=update_constraint_data,
    )
    space_interpolation: bpy.props.EnumProperty(
        name="Interpolation",
        default="arc",
        items=[
            (
                "arc",
                "Arc",
                "Slide along circular arcs through the neighbouring vertices."
                "\nFollows the curve the loop describes, so spacing doesn't"
                "\nflatten the shape - exact on circles",
            ),
            ("cubic", "Cubic", "Natural cubic spline, smooth results"),
            ("linear", "Linear", "Simple and fast, slides along the edges - cuts corners"),
        ],
        description="Path the vertices slide along when being spaced",
        update=update_constraint_data,
    )

    # Curvature constraint properties
    curvature_mode: bpy.props.EnumProperty(
        name="Curvature",
        default="CONSTANT",
        items=[
            (
                "CONSTANT",
                "Constant",
                "Aim for one curvature along the whole loop, an arc",
            ),
            (
                "LINEAR",
                "Constant Change",
                "Let the curvature change at a constant rate along the loop."
                "\nClosed loops fall back to constant curvature",
            ),
        ],
        description="Curvature profile the loop is pushed towards",
        update=update_constraint_data,
    )
    curvature_measure: bpy.props.EnumProperty(
        name="Measure",
        default="LENGTH",
        items=[
            (
                "LENGTH",
                "Arc Length",
                "True curvature, from angles and segment lengths."
                "\nEven curvature regardless of how densely the loop is divided",
            ),
            (
                "ANGLE",
                "Angle",
                "Turn angle per vertex only, segment lengths don't matter."
                "\nDenser vertices then let the shape turn more tightly",
            ),
        ],
        description="What gets evened out along the loop",
        update=update_constraint_data,
    )
    curvature_context_steps: bpy.props.IntProperty(
        name="Surroundings",
        default=0,
        min=0,
        soft_max=10,
        description="Also sample the curvature this many edges beyond the loop"
        "\nends, following the edge loops. Only the assigned vertices move,"
        "\nso the loop blends into its surroundings instead of averaging"
        "\nonly itself. Zero samples the assigned loop alone",
        update=update_constraint_data,
    )

    # Line constraint properties
    fix_line: bpy.props.BoolProperty(
        name="Fix Line",
        default=False,
        description="Keep the line where it is now, instead of letting the loop ends define it",
        update=update_line_fix_flags,
    )
    fixed_lines: bpy.props.CollectionProperty(type=FixedLineItem)

    # Smooth constraint properties
    smooth_mode: bpy.props.EnumProperty(
        name="Mode",
        default="BLEND",
        items=[
            (
                "BLEND",
                "Blend",
                "Relax each vertex toward its neighbours - the region melts"
                "\ninto its surroundings, held in place by the unselected"
                "\nboundary. The mode for smooth transitions between shapes",
            ),
            (
                "CURVATURE",
                "Round",
                "Fair the region into a curvature-continuous blend with its"
                "\nsurroundings - creases round outward into fillets, walls"
                "\nand planes keep their identity up to the selection border",
            ),
            (
                "SHAPE",
                "Keep Shape",
                "Remove jaggedness while preserving the overall shape and"
                "\nvolume. The mode for polishing a surface without"
                "\ndeflating it",
            ),
        ],
        update=update_constraint_data,
    )
    smooth_factor: bpy.props.FloatProperty(
        name="Factor",
        default=0.5,
        min=0.0,
        soft_max=1.0,
        description="How strongly each vertex is pulled toward the average"
        "\nof its neighbours per iteration",
        update=update_constraint_data,
    )

    # Thickness constraint properties
    thickness_use_min: bpy.props.BoolProperty(
        name="Minimum",
        default=True,
        description="Push vertices apart where the wall is thinner than the minimum",
        update=update_constraint_data,
    )
    thickness_min: bpy.props.FloatProperty(
        name="Min Thickness",
        default=0.005,
        min=0.0,
        soft_max=0.1,
        unit="LENGTH",
        description="Walls thinner than this get thickened",
        update=update_constraint_data,
    )
    thickness_use_max: bpy.props.BoolProperty(
        name="Maximum",
        default=False,
        description="Pull vertices together where the wall is thicker than the maximum",
        update=update_constraint_data,
    )
    thickness_max: bpy.props.FloatProperty(
        name="Max Thickness",
        default=0.02,
        min=0.0,
        soft_max=0.5,
        unit="LENGTH",
        description="Walls thicker than this get thinned",
        update=update_constraint_data,
    )
    thickness_ray_length: bpy.props.FloatProperty(
        name="Threshold",
        default=0.05,
        min=0.0,
        soft_max=1.0,
        unit="LENGTH",
        description="Length of the measuring ray - anything thicker than this"
        "\ndoesn't count as a wall and the constraint leaves it alone",
        update=update_constraint_data,
    )

    # Circle constraint properties
    fix_circle: bpy.props.BoolProperty(
        name="Fix Circle",
        default=False,
        description="Keep the circle where it is now, instead of refitting it to the loop",
        update=update_circle_fix_flags,
    )
    fixed_circles: bpy.props.CollectionProperty(type=FixedCircleItem)


def labeled_enum_row(layout, data, prop_name):
    """A label naming the property, with its options as a button row below."""
    col = layout.column(align=True)
    col.label(text=data.bl_rna.properties[prop_name].name + ":")
    col.row(align=True).prop(data, prop_name, expand=True)


class VIEW3D_PT_final_topology_constraints(Panel):
    bl_label = "Constraints"
    bl_idname = "VIEW3D_PT_final_topology_constraints"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_description = "Constraints are used to define areas of the mesh that should be preserved during optimization"
    bl_category = "Edit"
    bl_parent_id = "VIEW3D_PT_final_topology_editmode"

    @classmethod
    def poll(self, context):
        return (
            context.active_object is not None
            and context.active_object.type == "MESH"
            and context.mode == "EDIT_MESH"
        )

    def draw(self, context):
        layout = self.layout
        mesh = context.active_object.data

        row = layout.row(align=True)
        row.label(text="Add Constraint:")
        row.operator_context = "EXEC_DEFAULT"
        # we need to run operators without invoke
        op = row.operator("object.final_topology_add_constraint", text="", icon="MESH_PLANE")
        op.constraint_type = "PLANE"
        op.name = "Plane"

        op = row.operator("object.final_topology_add_constraint", text="", icon="MESH_PLANE")
        op.constraint_type = "PLANE_FIXED"
        op.name = "Plane Fixed"
        
        op = row.operator("object.final_topology_add_constraint", text="", icon="CURVE_DATA")
        op.constraint_type = "CURVE"
        op.name = "Curve"
        
        op = row.operator("object.final_topology_add_constraint", text="", icon="MOD_SUBSURF")
        op.constraint_type = "INVERSE_SUBDIVIDE"
        op.name = "Inverse Subdivide"

        op = row.operator("object.final_topology_add_constraint", text="", icon="ORIENTATION_GLOBAL")
        op.constraint_type = "INCLINATION_LIMIT"
        op.name = "Inclination Limit"
        
        op = row.operator("object.final_topology_add_constraint", text="", icon="DRIVER_ROTATIONAL_DIFFERENCE")
        op.constraint_type = "SLIDE_OPTIMIZE"
        op.name = "Slide Optimize"
        
        op = row.operator("object.final_topology_add_constraint", text="", icon="TRACKING_FORWARDS_SINGLE")
        op.constraint_type = "SPACE"
        op.name = "Space"

        op = row.operator("object.final_topology_add_constraint", text="", icon="SPHERECURVE")
        op.constraint_type = "CURVATURE"
        op.name = "Curvature"

        op = row.operator("object.final_topology_add_constraint", text="", icon="IPO_LINEAR")
        op.constraint_type = "LINE"
        op.name = "Line"

        op = row.operator("object.final_topology_add_constraint", text="", icon="IPO_LINEAR")
        op.constraint_type = "LINE_FIXED"
        op.name = "Line Fixed"

        op = row.operator("object.final_topology_add_constraint", text="", icon="MESH_CIRCLE")
        op.constraint_type = "CIRCLE"
        op.name = "Circle"

        op = row.operator("object.final_topology_add_constraint", text="", icon="MESH_CIRCLE")
        op.constraint_type = "CIRCLE_FIXED"
        op.name = "Circle Fixed"

        op = row.operator("object.final_topology_add_constraint", text="", icon="MOD_SOLIDIFY")
        op.constraint_type = "THICKNESS"
        op.name = "Thickness"

        op = row.operator("object.final_topology_add_constraint", text="", icon="PINNED")
        op.constraint_type = "PIN"
        op.name = "Pin"

        op = row.operator("object.final_topology_add_constraint", text="", icon="MOD_SMOOTH")
        op.constraint_type = "SMOOTH"
        op.name = "Smooth"
        layout.operator_context = "INVOKE_DEFAULT"
        row = layout.row()
        row.template_list(
            "CUSTOM_UL_constraint_list",
            "",
            mesh,
            "ft_custom_constraints",
            mesh,
            "ft_custom_constraints_index",
        )

        col = row.column(align=True)
        col.operator("object.final_topology_add_constraint", icon="ADD", text="")
        col.operator("object.final_topology_delete_constraint", icon="REMOVE", text="")
        if len(mesh.ft_custom_constraints) > 0:
            row = layout.row(align=True)
            op = row.operator(
                "object.final_topology_add_selection_to_constraint",
                text="Add Selection",
            )
            op.remove = False
            op = row.operator(
                "object.final_topology_add_selection_to_constraint",
                text="Remove Selection",
            )
            op.remove = True
            row.operator(
                "object.final_topology_remove_selection_from_all",
                text="From All",
            )

        if len(mesh.ft_custom_constraints) > 0:
            ac = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]
            layout.prop(ac, "name")
            layout.prop(ac, "constraint_type")
            if ac.constraint_type not in ("INVERSE_SUBDIVIDE", "PIN"):
                layout.prop(ac, "works_on_subdivision")

            if ac.constraint_type == "PLANE":
                row = layout.row()
                row.prop(ac, "fix_center")
                if not ac.fix_center:
                    row.prop(ac, "join_center")
                else:
                    row.prop(ac, "center")
                
                row = layout.row()
                row.prop(ac, "fix_normal")
                if not ac.fix_normal:
                    row.prop(ac, "join_normal")
                else:
                    row.prop(ac, "normal")

            if ac.constraint_type == "CURVE":
                layout.prop(ac, "target_curve")
                layout.operator(
                    CurveFromSelectionOperator.bl_idname,
                    icon="CURVE_BEZCURVE",
                )
                labeled_enum_row(layout, ac, "projection")
                layout.prop(ac, "even_distribution")
            
            if ac.constraint_type == "SPACE":
                labeled_enum_row(layout, ac, "space_method")
                labeled_enum_row(layout, ac, "space_interpolation")

            if ac.constraint_type == "CURVATURE":
                labeled_enum_row(layout, ac, "curvature_mode")
                labeled_enum_row(layout, ac, "curvature_measure")
                layout.prop(ac, "curvature_context_steps")

            if ac.constraint_type == "LINE":
                labeled_enum_row(layout, ac, "projection")
                layout.prop(ac, "even_distribution")
                layout.prop(ac, "fix_line")
                if ac.fix_line:
                    for item in ac.fixed_lines:
                        col = layout.column(align=True)
                        col.prop(item, "start")
                        col.prop(item, "end")

            if ac.constraint_type == "PIN":
                col = layout.column(align=True)
                col.scale_y = 0.8
                col.label(text="Pinned vertices only move under", icon="PINNED")
                col.label(text="your own transforms.", icon="BLANK1")

            if ac.constraint_type == "SMOOTH":
                labeled_enum_row(layout, ac, "smooth_mode")
                layout.prop(ac, "smooth_factor")

            if ac.constraint_type == "THICKNESS":
                row = layout.row(align=True)
                row.prop(ac, "thickness_use_min", text="")
                sub = row.row(align=True)
                sub.enabled = ac.thickness_use_min
                sub.prop(ac, "thickness_min")
                row = layout.row(align=True)
                row.prop(ac, "thickness_use_max", text="")
                sub = row.row(align=True)
                sub.enabled = ac.thickness_use_max
                sub.prop(ac, "thickness_max")
                layout.prop(ac, "thickness_ray_length")

            if ac.constraint_type == "CIRCLE":
                labeled_enum_row(layout, ac, "projection")
                layout.prop(ac, "even_distribution")
                if not ac.fix_circle:
                    # e.g. concentric circles: shared axis and/or orientation
                    row = layout.row()
                    row.prop(ac, "join_center")
                    row.prop(ac, "join_normal")
                layout.prop(ac, "fix_circle")
                if ac.fix_circle:
                    for item in ac.fixed_circles:
                        col = layout.column(align=True)
                        col.prop(item, "center")
                        col.prop(item, "normal")
                        col.prop(item, "radius")
            
            if ac.constraint_type == "INVERSE_SUBDIVIDE":
                layout.label(text="Snap to")
                active_obj = context.active_object
                freeze_name = f"FROZEN_MESH_STATE_{active_obj.name}_{ac.name}"
                if bpy.data.objects.get(freeze_name) is not None:
                    op = layout.operator(
                        "mesh.freeze_shape", text="Unfreeze shape", depress=True, icon="FREEZE"
                    )
                    op.constraint_index = mesh.ft_custom_constraints_index
                else:
                    op = layout.operator(
                        "mesh.freeze_shape", text="Freeze Shape", depress=False, icon="FREEZE"
                    )
                    op.constraint_index = mesh.ft_custom_constraints_index
                    labeled_enum_row(layout, ac, "use_object_or_collection")
                    if ac.use_object_or_collection == "OBJECT":
                        row.prop(ac, "invsubdiv_target_object", text="")
                    elif ac.use_object_or_collection == "COLLECTION":
                        row.prop(ac, "invsubdiv_target_collection", text="")
                
                layout.prop(ac, "invsubdiv_normal_offset", text="Normal Offset")

            if ac.constraint_type == "INCLINATION_LIMIT":
                layout.prop(ac, "inclination_axis")
                layout.prop(ac, "inclination_max_angle")


class VIEW3D_PT_final_topology_extra_operators(Panel):
    bl_category = "Edit"
    bl_idname = "VIEW3D_PT_final_topology_extra_operators"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Extra loop tools"
    bl_parent_id = "VIEW3D_PT_final_topology_editmode"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(self, context):
        return (
            context.active_object is not None
            and context.active_object.type == "MESH"
            and context.mode == "EDIT_MESH"
        )

    def draw(self, context):
        layout = self.layout
        
        # Hide flatten selection, seems broken by now.
        # layout.operator(FlattenSelectionOperator.bl_idname, text="Flatten Selection")
        layout.operator(NormalLoopAlign.bl_idname, text="Loop Align to Normal Plane")
        layout.operator(SlideOptimizeOperator.bl_idname, text="Loop Slide Optimize")


def add_selection_to_attribute(
    attribute_name, mesh, type="FLOAT", domain="POINT", remove=False
):
    """Add selection to attribute, if remove is True, remove selection from attribute"""

    bm = bmesh.from_edit_mesh(mesh)

    if domain == "POINT":
        iter_elements = bm.verts
        attribute_layer = bm.verts.layers.float[attribute_name]

    elif domain == "EDGE":
        iter_elements = bm.edges
        attribute_layer = bm.edges.layers.float[attribute_name]
    elif domain == "FACE":
        iter_elements = bm.faces
        attribute_layer = bm.faces.layers.float[attribute_name]
    # Transform vertex coordinates to world space
    if not remove:
        values = [(v.select or v[attribute_layer] > 0.5) for v in iter_elements]
    else:
        values = [(not v.select and v[attribute_layer] > 0.5) for v in iter_elements]

    bpy.ops.object.mode_set(mode="OBJECT")
    # do this in object mode now

    attribute = mesh.attributes.get(attribute_name)
    if attribute is None:
        attribute = mesh.attributes.new(
            name=attribute_name, type="FLOAT", domain=domain
        )
    attribute.data.foreach_set("value", values)

    bpy.ops.object.mode_set(mode="EDIT")
    return attribute.name


def fill_attribute_with_selection(
    attribute_name, mesh, type="FLOAT", domain="POINT", new=False
):
    """Fill a new attribute with selection"""
    bm = bmesh.from_edit_mesh(mesh)
    if domain == "POINT":
        values = [v.select for v in bm.verts]
    elif domain == "EDGE":
        values = [e.select for e in bm.edges]
    elif domain == "FACE":
        values = [f.select for f in bm.faces]

    bpy.ops.object.mode_set(mode="OBJECT")
    # do this in object mode now

    attribute = mesh.attributes.get(attribute_name)
    if attribute is None or new:
        attribute = mesh.attributes.new(
            name=attribute_name, type="FLOAT", domain=domain
        )
    attribute.data.foreach_set("value", values)

    bpy.ops.object.mode_set(mode="EDIT")
    return attribute.name


class RemoveSelectionFromAllConstraintsOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_remove_selection_from_all"
    bl_label = "Remove Selection from All Constraints"
    bl_description = (
        "\n\nRemove the selected elements from every constraint of this mesh"
    )
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        global _suppress_undo_push
        mesh = context.active_object.data
        record_constraint_undo_state(mesh)
        _suppress_undo_push = True
        try:
            for c in mesh.ft_custom_constraints:
                if not c.attribute_name:
                    continue
                domain = get_constraint_domain_type(c.constraint_type)
                bm = bmesh.from_edit_mesh(mesh)
                layers = {
                    "POINT": bm.verts.layers.float,
                    "EDGE": bm.edges.layers.float,
                    "FACE": bm.faces.layers.float,
                }.get(domain)
                if layers is None or layers.get(c.attribute_name) is None:
                    continue
                add_selection_to_attribute(
                    c.attribute_name, mesh, remove=True, domain=domain
                )
        finally:
            _suppress_undo_push = False
        clear_constraints_cache()
        push_constraint_undo("Remove Selection from All Constraints")
        return {"FINISHED"}


class PinSelectionOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_pin_selection"
    bl_label = "Pin Selection"
    bl_description = (
        "\n\nPin the selected vertices - adds them to the active or first pin"
        "\nconstraint, creating one when none exists. When the whole selection"
        "\nis already pinned, unpins it from all pin constraints instead"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob is not None and ob.type == "MESH" and ob.mode == "EDIT"

    def execute(self, context):
        global _suppress_undo_push
        mesh = context.active_object.data
        bm = bmesh.from_edit_mesh(mesh)
        selected = {v.index for v in bm.verts if v.select}
        if not selected:
            self.report({"ERROR"}, "No vertices selected")
            return {"CANCELLED"}

        pins = [c for c in mesh.ft_custom_constraints if c.constraint_type == "PIN"]
        pinned = set()
        for c in pins:
            layer = (
                bm.verts.layers.float.get(c.attribute_name)
                if c.attribute_name
                else None
            )
            if layer is not None:
                pinned.update(v.index for v in bm.verts if v[layer] == 1.0)

        record_constraint_undo_state(mesh)

        if pins and selected <= pinned:
            # the whole selection is pinned already - toggle it free again
            _suppress_undo_push = True
            try:
                for c in pins:
                    bm = bmesh.from_edit_mesh(mesh)
                    if not c.attribute_name or bm.verts.layers.float.get(
                        c.attribute_name
                    ) is None:
                        continue
                    add_selection_to_attribute(
                        c.attribute_name, mesh, remove=True, domain="POINT"
                    )
            finally:
                _suppress_undo_push = False
            clear_constraints_cache()
            push_constraint_undo("Unpin Selection")
            return {"FINISHED"}

        # pin: into the active pin constraint, else the first one - and when
        # there is none yet, a fresh one takes the selection with it
        target = None
        index = mesh.ft_custom_constraints_index
        if 0 <= index < len(mesh.ft_custom_constraints):
            active = mesh.ft_custom_constraints[index]
            if active.constraint_type == "PIN":
                target = active
        if target is None and pins:
            target = pins[0]
        if target is None:
            return bpy.ops.object.final_topology_add_constraint(
                "EXEC_DEFAULT", constraint_type="PIN", name="Pin"
            )

        _suppress_undo_push = True
        try:
            if target.attribute_name and bm.verts.layers.float.get(
                target.attribute_name
            ) is not None:
                add_selection_to_attribute(
                    target.attribute_name, mesh, remove=False, domain="POINT"
                )
            else:
                target.attribute_name = fill_attribute_with_selection(
                    "ft_constraint", mesh, domain="POINT", new=True
                )
        finally:
            _suppress_undo_push = False
        clear_constraints_cache()
        push_constraint_undo("Pin Selection")
        return {"FINISHED"}


class AddSelectionToConstraintOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_add_selection_to_constraint"
    bl_label = "Add Selection to Constraint"
    bl_description = (
        "\n\nSelect vertices and run this operator to add them to the constraint."
    )
    bl_options = {"REGISTER", "UNDO"}

    remove: bpy.props.BoolProperty(name="Remove", default=False)

    def execute(self, context):
        global _suppress_undo_push
        record_constraint_undo_state(context.active_object.data)
        _suppress_undo_push = True
        try:
            # Access the mesh data block
            mesh = context.active_object.data
            bm = bmesh.from_edit_mesh(mesh)
            constraint = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]
            attribute_name = constraint.attribute_name
            domain = get_constraint_domain_type(constraint.constraint_type)
            attribute_name = add_selection_to_attribute(
                attribute_name, mesh, remove=self.remove, domain=domain
            )
            constraint.attribute_name = attribute_name
        finally:
            _suppress_undo_push = False
        push_constraint_undo(
            "Remove Selection from Constraint" if self.remove else "Add Selection to Constraint"
        )
        return {"FINISHED"}

def get_constraint_domain_type(constraint_type):
    if constraint_type in ["PLANE", "CURVE", "SLIDE_OPTIMIZE", "SPACE", "CURVATURE", "LINE", "CIRCLE"]:
        return "EDGE"
    elif constraint_type in ["INVERSE_SUBDIVIDE", "THICKNESS", "PIN", "SMOOTH"]:
        return "POINT"
    elif constraint_type == "INCLINATION_LIMIT":
        return "FACE"


class AddConstraintOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_add_constraint"
    bl_label = "Add Constraint"
    bl_description = (
        "\n\nSelect vertices and run this operator to add a constraint."
        "\nConstraints get evaluated each step if enabled."
        "\n To use also inverse subdivision snapping, \n"
        "add it as one of the constraints."
    )
    bl_options = {"REGISTER", "UNDO"}

    name: bpy.props.StringProperty(name="Name", default="Constraint")
    constraint_type: bpy.props.EnumProperty(
        name="Type",
        default="PLANE",
        items=[
            ("PLANE", "Plane", "Planar constraint", "MESH_PLANE", 0),
            ("PLANE_FIXED", "Plane Fixed", "Planar constraint fixed", "MESH_PLANE", 1),
            ("CURVE", "Curve", "Curve constraint", "CURVE_DATA", 2),
            (
                "INVERSE_SUBDIVIDE",
                "Inverse subdivide",
                "Inverse subdivide",
                "MOD_SUBSURF",
                3,
            ),
            (
                "INCLINATION_LIMIT",
                "Inclination limit",
                "Limit maximal face inclination relative to selected axis, useful for 3D print overhang control",
                "ORIENTATION_GLOBAL",
                4,
            ),
            (
                "SLIDE_OPTIMIZE",
                "Slide Optimize",
                "Balance angles where quads meet",
                "DRIVER_ROTATIONAL_DIFFERENCE",
                5,
            ),
            (
                "SPACE",
                "Space",
                "Distribute vertices at equal distances along the loop",
                "TRACKING_FORWARDS_SINGLE",
                6,
            ),
            (
                "CURVATURE",
                "Curvature",
                "Even out the curvature along the loop",
                "SPHERECURVE",
                7,
            ),
            (
                "LINE",
                "Line",
                "Straighten an open loop onto the line between its two ends",
                "IPO_LINEAR",
                8,
            ),
            (
                "LINE_FIXED",
                "Line Fixed",
                "Straighten an open loop onto a line that stays where it is now",
                "IPO_LINEAR",
                9,
            ),
            (
                "CIRCLE",
                "Circle",
                "Pull the loop onto its best-fit circle",
                "MESH_CIRCLE",
                10,
            ),
            (
                "CIRCLE_FIXED",
                "Circle Fixed",
                "Pull the loop onto a circle that stays where it is now",
                "MESH_CIRCLE",
                11,
            ),
            (
                "THICKNESS",
                "Thickness",
                "Keep the wall thickness under the vertices within bounds",
                "MOD_SOLIDIFY",
                12,
            ),
            (
                "PIN",
                "Pin",
                "Freeze the vertices against all constraints and inverse subdivision",
                "PINNED",
                13,
            ),
            (
                "SMOOTH",
                "Smooth",
                "Pull each vertex toward the average of its neighbours",
                "MOD_SMOOTH",
                14,
            ),
            # (
            #     "ANGLE",
            #     "Angle",
            #     "Limit angle for manufacturing purposes",
            #     "LINCURVE",
            #     7,
            # ),
        ],
    )
    works_on_subdivision: bpy.props.BoolProperty(
        name="Works on Subdivision",
        default=False,
        description="\n OFF: works on control mesh, if \n ON: works on subdivided mesh.",
    )
    center: bpy.props.FloatVectorProperty(
        name="Center", size=3, default=(0.0, 0.0, 0.0)
    )
    normal: bpy.props.FloatVectorProperty(
        name="Normal", size=3, default=(0.0, 0.0, 0.0)
    )
    # no target_curve pointer here: operators cannot register data-block
    # pointer properties - the panel assigns the curve after adding
    projection: bpy.props.EnumProperty(
        name="Projection",
        default="3D",
        items=[
            ("3D", "3D", "Solve the constraint fully in 3D"),
            (
                "PROJECTED",
                "Projected",
                "Solve the constraint only as seen along its projection axis",
            ),
        ],
    )
    # attribute_name: bpy.props.StringProperty(name="Attribute Name", default="Attribute Name")
    tooltip: bpy.props.StringProperty(  # type: ignore[valid-type]
        default="Select vertices and run this operator to add a constraint."
        "\nConstraints get evaluated each step if enabled."
        "\n To use also inverse subdivision snapping, \n"
        "add it as one of the constraints."
    )

    @classmethod
    def description(cls, context, properties):
        t = f"Select vertices and run this operator to add a {properties.constraint_type.capitalize()} constraint."
        "\nConstraints get evaluated each step if enabled."
        "\n To use also inverse subdivision snapping, \n"
        "add it as one of the constraints."
        return t

    def execute(self, context):
        # one undo version for the whole add, not one per property it sets up
        global _suppress_undo_push
        record_constraint_undo_state(context.active_object.data)
        _suppress_undo_push = True
        try:
            result = self.add_constraint(context)
        finally:
            _suppress_undo_push = False
        if result == {"FINISHED"}:
            push_constraint_undo("Add Constraint")
        return result

    def add_constraint(self, context):
        # Access the mesh data block
        mesh = context.active_object.data

        bm = bmesh.from_edit_mesh(mesh)

        # Get the selected vertices
        selected_verts = [v for v in bm.verts if v.select]
        if len(selected_verts) == 0:
            self.report({"ERROR"}, "No vertices selected")
            return {"CANCELLED"}

        # Create a new constraint
        new_constraint = mesh.ft_custom_constraints.add()
        new_constraint.name = self.name
        
        # Convert the FIXED variants to their base type with fix flags
        actual_type = self.constraint_type
        if actual_type == "PLANE_FIXED":
            actual_type = "PLANE"
        elif actual_type == "LINE_FIXED":
            actual_type = "LINE"
        elif actual_type == "CIRCLE_FIXED":
            actual_type = "CIRCLE"
        new_constraint.constraint_type = actual_type
        new_constraint.works_on_subdivision = self.works_on_subdivision
        if actual_type in ("CURVE", "LINE", "CIRCLE"):
            new_constraint.projection = self.projection

        attribute_name = f"ft_constraint"

        # Set the newly added constraint as the active one
        mesh.ft_custom_constraints_index = len(mesh.ft_custom_constraints) - 1
        
        if self.constraint_type == "PLANE_FIXED" or self.constraint_type == "PLANE":
            # Calculate plane from selection
            center, normal = utils.estimate_best_fit_plane(selected_verts, "best_fit")
            new_constraint.center = center
            new_constraint.normal = normal
            
            # Set fix flags for PLANE_FIXED
            if self.constraint_type == "PLANE_FIXED":
                new_constraint.fix_center = True
                new_constraint.fix_normal = True
        
        if self.constraint_type == "CURVE":
            # Calculate plane from selection for curve
            center, normal = utils.estimate_best_fit_plane(selected_verts, "best_fit")
            new_constraint.center = center
            new_constraint.normal = normal
            # curve snapping distributed evenly by default, as before
            new_constraint.even_distribution = True
        
        if self.constraint_type == "INVERSE_SUBDIVIDE":
            # Copy settings from object-level inverse subdivide settings
            obj_props = context.active_object.final_topology
            new_constraint.use_object_or_collection = obj_props.use_object_or_collection
            new_constraint.invsubdiv_target_object = obj_props.target_object
            new_constraint.invsubdiv_target_collection = obj_props.target_collection
            new_constraint.invsubdiv_normal_offset = obj_props.normal_offset

        domain = get_constraint_domain_type(actual_type)
        attribute_name = fill_attribute_with_selection(
            attribute_name, mesh, type="FLOAT", domain=domain, new=True
        )
        # we name the attribute after its creation, since we couldn't be sure about it's .00x ending
        new_constraint.attribute_name = attribute_name

        # only now can the update find the loop through the attribute and store its ends
        if self.constraint_type == "LINE_FIXED":
            new_constraint.fix_line = True
        elif self.constraint_type == "CIRCLE_FIXED":
            new_constraint.fix_circle = True

        new_constraint.color = (random(), random(), random())

        # Set name to the type of constraint, if the name is default
        if self.name == "Constraint":
            new_constraint.name = self.constraint_type.capitalize()
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "name")
        layout.prop(self, "constraint_type")
        layout.prop(self, "works_on_subdivision")
        if self.constraint_type == "CURVE":
            labeled_enum_row(layout, self, "projection")


def resample_polyline(points, count, cyclic):
    """Evenly spaced positions along a polyline, count of them."""
    tknots = [0.0]
    for i in range(1, len(points)):
        tknots.append(tknots[-1] + (points[i] - points[i - 1]).length)
    total = tknots[-1]
    if cyclic:
        total += (points[0] - points[-1]).length
    if total < 1e-9 or count < 2:
        return list(points[:count])

    segments = count if cyclic else count - 1
    result = []
    for i in range(count):
        m = total * i / segments
        segment = bisect_right(tknots, m) - 1
        segment = max(0, min(segment, len(points) - 2 if not cyclic else len(points) - 1))
        a = points[segment]
        b = points[(segment + 1) % len(points)]
        span_start = tknots[segment]
        span = (b - a).length
        t = 0.0 if span < 1e-12 else (m - span_start) / span
        result.append(a.lerp(b, min(t, 1.0)))
    return result


class CurveFromSelectionOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_curve_from_selection"
    bl_label = "Create Curve from Loop"
    bl_description = (
        "Create a bezier curve object from the constraint's loops and link it"
        "\nas the target curve. With Works on Subdivision the curve follows the"
        "\nsubdivided shape, otherwise the control loop"
    )
    bl_options = {"REGISTER", "UNDO"}

    curve_type: bpy.props.EnumProperty(
        name="Type",
        default="NURBS",
        items=[
            (
                "NURBS",
                "Nurbs",
                "A nurbs curve clamped to its end points - controllable"
                "\nwithout handles",
            ),
            ("BEZIER", "Bezier", "A bezier curve with automatic handles"),
        ],
    )
    point_mode: bpy.props.EnumProperty(
        name="Points",
        default="ORIGINAL",
        items=[
            (
                "ORIGINAL",
                "Original",
                "One control point per loop vertex",
            ),
            (
                "COUNT",
                "Best Fit",
                "A smooth curve with a chosen number of control points,"
                "\nspread evenly along the loop",
            ),
        ],
    )
    point_count: bpy.props.IntProperty(
        name="Point Count",
        default=6,
        min=2,
        soft_max=64,
        description="Number of control points in Best Fit mode",
    )

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return (
            ob is not None
            and ob.type == "MESH"
            and ob.mode == "EDIT"
            and hasattr(ob.data, "ft_custom_constraints")
            and len(ob.data.ft_custom_constraints) > 0
        )

    def execute(self, context):
        global _suppress_undo_push
        record_constraint_undo_state(context.active_object.data)
        # checkpoint first - undoing must also remove the created curve
        # object, which only a memfile snapshot from before it can do
        checkpoint_memfile_undo("Create Constraint Curve")
        _suppress_undo_push = True
        try:
            result = self.create_curve(context)
        finally:
            _suppress_undo_push = False
        if result == {"FINISHED"}:
            push_constraint_undo("Create Constraint Curve")
        return result

    def create_curve(self, context):
        ob = context.active_object
        mesh = ob.data
        constraint = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]

        bm = bmesh.from_edit_mesh(mesh)
        bm.verts.ensure_lookup_table()
        domain = get_constraint_domain_type(constraint.constraint_type)
        loops = utils.get_attribute_elements(
            ob, bm, constraint, domain=domain, as_domain="POINT"
        )
        if not loops or not isinstance(loops[0], list) or len(loops[0]) != 2:
            self.report({"ERROR"}, "The constraint has no loops to build a curve from")
            return {"CANCELLED"}

        # positions come from the control loop, or from the subdivided mesh
        # when the constraint works on subdivision
        eval_bm = None
        if constraint.works_on_subdivision:
            depsgraph = context.evaluated_depsgraph_get()
            eval_bm = utils.get_evaluated_bm(ob, depsgraph)
            eval_bm.verts.ensure_lookup_table()

        curve_data = bpy.data.curves.new(f"{constraint.name}_curve", "CURVE")
        curve_data.dimensions = "3D"
        spline_count = 0
        for loop_data in loops:
            verts = loop_data[0]
            is_circular = loop_data[1]
            if len(verts) < 2:
                continue
            if eval_bm is not None:
                points = [eval_bm.verts[v.index].co.copy() for v in verts]
            else:
                points = [v.co.copy() for v in verts]
            if self.point_mode == "COUNT":
                points = resample_polyline(points, self.point_count, is_circular)
            if len(points) < 2:
                continue

            if self.curve_type == "BEZIER":
                spline = curve_data.splines.new("BEZIER")
                spline.bezier_points.add(len(points) - 1)
                for bez, co in zip(spline.bezier_points, points):
                    bez.co = co
                    bez.handle_left_type = "AUTO"
                    bez.handle_right_type = "AUTO"
            else:
                spline = curve_data.splines.new("NURBS")
                spline.points.add(len(points) - 1)
                for nurbs_point, co in zip(spline.points, points):
                    nurbs_point.co = (co.x, co.y, co.z, 1.0)
                # clamped to the end points, so the curve starts and ends
                # exactly at the loop ends and is controllable without handles
                spline.use_endpoint_u = True
                spline.order_u = min(4, len(points))
                # the constraint evaluates non-bezier curves through their
                # tessellation, give it a finer one than the display default
                spline.resolution_u = 24
            spline.use_cyclic_u = is_circular
            spline_count += 1

        if spline_count == 0:
            bpy.data.curves.remove(curve_data)
            self.report({"ERROR"}, "The constraint's loops are too short for a curve")
            return {"CANCELLED"}

        curve_ob = bpy.data.objects.new(curve_data.name, curve_data)
        # same transform as the mesh, so the local point coordinates overlay
        # the loop exactly
        curve_ob.matrix_world = ob.matrix_world
        context.collection.objects.link(curve_ob)

        constraint.target_curve = curve_ob
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "curve_type")
        layout.prop(self, "point_mode")
        row = layout.row()
        row.enabled = self.point_mode == "COUNT"
        row.prop(self, "point_count")


class DeleteConstraintOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_delete_constraint"
    bl_label = "Delete Constraint"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        mesh = context.active_object.data
        active_obj = context.active_object
        record_constraint_undo_state(mesh)

        if (
            mesh.ft_custom_constraints_index >= 0
            and mesh.ft_custom_constraints_index < len(mesh.ft_custom_constraints)
        ):
            # Remove the active constraint
            constraint = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]

            if constraint.constraint_type == "INVERSE_SUBDIVIDE":
                freeze_name = f"FROZEN_MESH_STATE_{active_obj.name}_{constraint.name}"
                freeze_object = bpy.data.objects.get(freeze_name)
                if freeze_object is not None:
                    bpy.data.objects.remove(freeze_object)

            attribute = mesh.attributes.get(constraint.attribute_name)
            if attribute is not None:
                mesh.attributes.remove(attribute)

            mesh.ft_custom_constraints.remove(mesh.ft_custom_constraints_index)

            # Ensure the active index is within bounds
            if mesh.ft_custom_constraints_index >= len(mesh.ft_custom_constraints):
                mesh.ft_custom_constraints_index = len(mesh.ft_custom_constraints) - 1

            push_constraint_undo("Delete Constraint")

        return {"FINISHED"}


class CUSTOM_UL_constraint_list(bpy.types.UIList):
    def draw_item(
        self, context, layout, data, item, icon, active_data, active_propname, index
    ):
        custom_constraints = data.ft_custom_constraints
        constraint = custom_constraints[index]

        # Use layout to display the constraint properties
        # layout.label(text=constraint.name)

        if constraint.constraint_type == "PLANE":
            icon = "MESH_PLANE"
        elif constraint.constraint_type == "CURVE":
            icon = "CURVE_DATA"
        elif constraint.constraint_type == "CIRCLE":
            icon = "MESH_CIRCLE"
        elif constraint.constraint_type == "INVERSE_SUBDIVIDE":
            icon = "MOD_SUBSURF"
        elif constraint.constraint_type == "INCLINATION_LIMIT":
            icon = "ORIENTATION_GLOBAL"
        elif constraint.constraint_type == "SLIDE_OPTIMIZE":
            icon = "DRIVER_ROTATIONAL_DIFFERENCE"
        elif constraint.constraint_type == "SPACE":
            icon = "TRACKING_FORWARDS_SINGLE"
        elif constraint.constraint_type == "CURVATURE":
            icon = "SPHERECURVE"
        elif constraint.constraint_type == "LINE":
            icon = "IPO_LINEAR"
        elif constraint.constraint_type == "THICKNESS":
            icon = "MOD_SOLIDIFY"
        elif constraint.constraint_type == "PIN":
            icon = "PINNED"
        elif constraint.constraint_type == "SMOOTH":
            icon = "MOD_SMOOTH"

        layout.prop(constraint, "name", text="", emboss=False, icon=icon)
        
        if constraint.enabled:
            layout.prop(
                constraint, "enabled", text="", emboss=False, icon="RESTRICT_VIEW_OFF"
            )
        else:
            layout.prop(
                constraint, "enabled", text="", emboss=False, icon="RESTRICT_VIEW_ON"
            )
        # layout.prop(constraint, "center")
        # layout.prop(constraint, "normal")
        # layout.prop(constraint, "attribute_name")


class FixedCircleGizmoTarget:
    """Gizmo adapter transforming one stored circle of a fixed circle constraint.

    The gizmo frame is aligned to the circle: its Z axis is the circle normal,
    X and Y span the circle plane. Translating moves the stored center, rotating
    tilts the stored normal around the circle's own center. All gizmo input
    arrives in world space and gets converted into the object's local space here.
    """

    def __init__(self, object, constraint, index):
        self.object = object
        self.constraint = constraint
        self.index = index

    def _item(self):
        return self.constraint.fixed_circles[self.index]

    def _world_normal(self):
        normal = (
            self.object.matrix_world.inverted().transposed().to_3x3()
            @ Vector(self._item().normal)
        )
        if normal.length < 1e-12:
            normal = Vector((0.0, 0.0, 1.0))
        return normal.normalized()

    def location(self):
        return self.object.matrix_world @ Vector(self._item().center)

    def orientation(self):
        # follow Blender's transform orientation: global axes, the object's
        # axes, or the circle's own frame for normal orientation
        mode = gizmos.orientation_mode(bpy.context)
        if mode == "GLOBAL":
            return Matrix.Identity(3)
        if mode == "LOCAL":
            return self.object.matrix_world.to_quaternion().to_matrix()
        z = self._world_normal()
        x = z.orthogonal().normalized()
        y = z.cross(x)
        return Matrix((x, y, z)).transposed()

    def rotation_axes(self):
        # in the circle's own frame, spinning around the normal changes nothing;
        # in global or local frames every axis tilts it
        if gizmos.orientation_mode(bpy.context) == "NORMAL":
            return (True, True, False)
        return (True, True, True)

    def snapshot(self):
        item = self._item()
        # the drag keeps working in the frame it started in, even while the
        # tilt it applies is changing that frame
        return (Vector(item.center), Vector(item.normal), self.orientation())

    def translate(self, snapshot, axis_index, distance):
        world_offset = snapshot[2].col[axis_index] * distance
        local_offset = self.object.matrix_world.inverted().to_3x3() @ world_offset
        self._item().center = snapshot[0] + local_offset

    def rotate(self, snapshot, axis_index, angle):
        world_axis = snapshot[2].col[axis_index]
        local_axis = self.object.matrix_world.inverted().to_3x3() @ world_axis
        if local_axis.length < 1e-12:
            return
        rotation = Matrix.Rotation(angle, 3, local_axis.normalized())
        self._item().normal = rotation @ snapshot[1]

    def radius(self):
        # approximate world radius, exact for uniform object scale
        scale = self.object.matrix_world.to_3x3().median_scale
        return self._item().radius * scale

    def outline_matrix(self):
        matrix = self._world_normal().to_track_quat("Z", "Y").to_matrix().to_4x4()
        matrix.translation = self.location()
        return matrix


class TargetCurveGizmoTarget:
    """Gizmo adapter moving and rotating a curve constraint's target curve object.

    Rotation pivots around the curve object's own origin. World gizmo input maps
    straight onto the object's location and rotation.
    """

    def __init__(self, curve_object):
        self.curve_object = curve_object

    def location(self):
        return self.curve_object.matrix_world.translation.copy()

    def orientation(self):
        mode = gizmos.orientation_mode(bpy.context)
        if mode == "GLOBAL":
            return Matrix.Identity(3)
        # for an object, its own axes are both the local and the normal frame
        return self.curve_object.matrix_world.to_quaternion().to_matrix()

    def snapshot(self):
        return (
            self.curve_object.location.copy(),
            self.curve_object.rotation_euler.copy(),
            self.orientation(),
        )

    def translate(self, snapshot, axis_index, distance):
        offset = snapshot[2].col[axis_index] * distance
        self.curve_object.location = snapshot[0] + offset

    def rotate(self, snapshot, axis_index, angle):
        axis = Vector(snapshot[2].col[axis_index])
        if axis.length < 1e-12:
            return
        rotation = Matrix.Rotation(angle, 3, axis.normalized())
        base = snapshot[1].to_matrix()
        self.curve_object.rotation_euler = (rotation @ base).to_euler(snapshot[1].order)


def get_active_curve_constraint(context):
    """The active constraint, if it's an enabled curve one with a target curve."""
    ob = context.object
    if not (ob and ob.type == "MESH" and ob.mode == "EDIT"):
        return None, None
    if not hasattr(ob.data, "ft_custom_constraints"):
        return None, None
    cs = ob.data.ft_custom_constraints
    index = ob.data.ft_custom_constraints_index
    if not (0 <= index < len(cs)):
        return None, None
    c = cs[index]
    if (
        c.constraint_type == "CURVE"
        and c.enabled
        and c.target_curve is not None
        and c.target_curve.type == "CURVE"
    ):
        return ob, c
    return None, None


def get_curve_constraint_curves(context):
    """All target curves of the mesh's enabled curve constraints, deduplicated,
    for when the active constraint being a curve one shows all their handles."""
    ob, active = get_active_curve_constraint(context)
    if active is None:
        return None, []
    curves = []
    for c in ob.data.ft_custom_constraints:
        if (
            c.constraint_type == "CURVE"
            and c.enabled
            and c.target_curve is not None
            and c.target_curve.type == "CURVE"
            and c.target_curve not in curves
        ):
            curves.append(c.target_curve)
    return ob, curves


class TargetCurveControlPoints:
    """Point handles adapter for the control points of a target curve.

    Covers bezier points - their two handles ride along rigidly, like
    Blender's own grab of a bezier point - as well as nurbs and poly spline
    points, which are bare positions.
    """

    def __init__(self, curve_object):
        self.curve_object = curve_object
        self._index_map = []
        for spline_index, spline in enumerate(curve_object.data.splines):
            if spline.type == "BEZIER":
                for point_index in range(len(spline.bezier_points)):
                    self._index_map.append((spline_index, point_index, True))
            else:
                for point_index in range(len(spline.points)):
                    self._index_map.append((spline_index, point_index, False))

    def _point(self, index):
        spline_index, point_index, is_bezier = self._index_map[index]
        spline = self.curve_object.data.splines[spline_index]
        if is_bezier:
            return spline.bezier_points[point_index], True
        return spline.points[point_index], False

    def count(self):
        return len(self._index_map)

    def location(self, index):
        point, is_bezier = self._point(index)
        co = point.co if is_bezier else point.co.to_3d()
        return self.curve_object.matrix_world @ co

    def snapshot(self, index):
        point, is_bezier = self._point(index)
        if is_bezier:
            return (point.co.copy(), point.handle_left.copy(), point.handle_right.copy())
        return (point.co.copy(),)

    def translate(self, index, snapshot, world_offset):
        local_offset = self.curve_object.matrix_world.inverted().to_3x3() @ world_offset
        point, is_bezier = self._point(index)
        if is_bezier:
            point.co = snapshot[0] + local_offset
            point.handle_left = snapshot[1] + local_offset
            point.handle_right = snapshot[2] + local_offset
        else:
            # nurbs and poly points are 4d, keep the weight
            point.co = (
                snapshot[0].x + local_offset.x,
                snapshot[0].y + local_offset.y,
                snapshot[0].z + local_offset.z,
                snapshot[0].w,
            )


class TargetCurveControlPointsMulti:
    """Point handles over the control points of several curves at once.

    Control points sitting (nearly) on the same world position share a single
    handle and move together - also across different curves. That keeps the
    touching ends of two curves joined while dragging, instead of tearing
    them apart.
    """

    def __init__(self, curve_objects):
        self.adapters = [TargetCurveControlPoints(ob) for ob in curve_objects]
        points = []
        for adapter_index, adapter in enumerate(self.adapters):
            for point_index in range(adapter.count()):
                points.append(
                    (adapter_index, point_index, adapter.location(point_index))
                )
        # near-coincident points cluster - tolerance follows the overall spread
        tolerance = 1e-6
        if points:
            for axis in range(3):
                values = [co[axis] for _, _, co in points]
                tolerance = max(tolerance, (max(values) - min(values)) * 1e-3)
        self.clusters = []
        for adapter_index, point_index, co in points:
            for cluster in self.clusters:
                if (cluster[0] - co).length <= tolerance:
                    cluster[1].append((adapter_index, point_index))
                    break
            else:
                self.clusters.append((co.copy(), [(adapter_index, point_index)]))

    def count(self):
        return len(self.clusters)

    def location(self, index):
        return self.clusters[index][0].copy()

    def snapshot(self, index):
        # the members ride in the snapshot - the drag stays glued to them
        # even if moving them reshuffles the clustering
        return [
            (adapter_index, point_index, self.adapters[adapter_index].snapshot(point_index))
            for adapter_index, point_index in self.clusters[index][1]
        ]

    def translate(self, index, snapshot, world_offset):
        for adapter_index, point_index, point_snapshot in snapshot:
            if adapter_index < len(self.adapters):
                adapter = self.adapters[adapter_index]
                if point_index < adapter.count():
                    adapter.translate(point_index, point_snapshot, world_offset)


# members of the cluster whose handle was clicked, handed from the gizmo
# group to the tweak operator: (curve name, spline index, point index, bezier)
_pending_curve_tweak = []
# where the tweak came from: the mesh to hop back to, and whether the
# always-on modal was running there and must be restarted on return
_curve_tweak_state = {"mesh_name": None, "restart_modal": False}


def enter_curve_tweak(context, members):
    """Hop from mesh edit mode into curve edit mode with exactly the given
    control points selected. Returns the mesh object to come back to, or
    None when the hop failed."""
    mesh_object = context.active_object
    curve_objects = []
    for name in {m[0] for m in members}:
        ob = bpy.data.objects.get(name)
        if ob is not None and ob.type == "CURVE":
            curve_objects.append(ob)
    if not curve_objects or mesh_object is None:
        return None

    # the mode switch will make the always-on modal cancel itself - note it
    # down, the way back restarts it
    from . import final_topology as ft_main

    _curve_tweak_state["mesh_name"] = mesh_object.name
    _curve_tweak_state["restart_modal"] = ft_main.running_operator is not None

    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.select_all(action="DESELECT")
    for ob in curve_objects:
        ob.select_set(True)
    context.view_layer.objects.active = curve_objects[0]
    # multi-object edit: all selected curves enter together
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.curve.select_all(action="DESELECT")
    for name, spline_index, point_index, is_bezier in members:
        ob = bpy.data.objects.get(name)
        if ob is None or spline_index >= len(ob.data.splines):
            continue
        spline = ob.data.splines[spline_index]
        if is_bezier:
            if point_index < len(spline.bezier_points):
                point = spline.bezier_points[point_index]
                point.select_control_point = True
                point.select_left_handle = True
                point.select_right_handle = True
        else:
            if point_index < len(spline.points):
                spline.points[point_index].select = True
    return mesh_object


def finish_curve_tweak(context):
    """Back from the curve tweak into the mesh's edit mode, restarting the
    always-on modal when it was running before the hop."""
    mesh_object = bpy.data.objects.get(_curve_tweak_state["mesh_name"] or "")
    if mesh_object is None:
        return False
    try:
        bpy.ops.object.mode_set(mode="OBJECT")
        bpy.ops.object.select_all(action="DESELECT")
        mesh_object.select_set(True)
        context.view_layer.objects.active = mesh_object
        bpy.ops.object.mode_set(mode="EDIT")
    except Exception:
        return False
    if _curve_tweak_state["restart_modal"]:
        from . import final_topology as ft_main

        try:
            # only when the old instance is really gone - invoking while one
            # runs would toggle it off instead
            if ft_main.running_operator is None:
                bpy.ops.mesh.final_topology_modal("INVOKE_DEFAULT")
        except Exception:
            pass
    _curve_tweak_state["mesh_name"] = None
    _curve_tweak_state["restart_modal"] = False
    return True


class CurvePointTweakOperator(bpy.types.Operator):
    """Move the grabbed control points with Blender's own translate.

    Clicking a point handle hops into the curves' edit mode, selects the
    grabbed points and starts a tweak-style translate - so the full native
    transform applies: snapping to vertices and edges, axis locking,
    numeric input. When that one drag ends, the mode hops back to the mesh;
    if the user chains further transforms instead, they stay in curve edit
    mode and come back with the panel button.
    """

    bl_idname = "object.final_topology_curve_point_tweak"
    bl_label = "Tweak Curve Control Point"
    bl_options = {"REGISTER", "INTERNAL"}

    def invoke(self, context, event):
        members = list(_pending_curve_tweak)
        if not members:
            return {"CANCELLED"}
        if enter_curve_tweak(context, members) is None:
            return {"CANCELLED"}
        bpy.ops.transform.translate("INVOKE_DEFAULT", release_confirm=True)
        self._saw_transform = False
        self._had_gap = False
        self._idle_ticks = 0
        self._timer = context.window_manager.event_timer_add(
            0.05, window=context.window
        )
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def _transform_running(self, context):
        window = context.window
        if window is None or not hasattr(window, "modal_operators"):
            return False
        return any(
            op.bl_idname.startswith("TRANSFORM_OT_")
            for op in window.modal_operators
        )

    def _stop(self, context):
        context.window_manager.event_timer_remove(self._timer)
        return {"FINISHED"}

    def modal(self, context, event):
        # the transform modal runs below this watcher
        if not bpy.data.objects.get(_curve_tweak_state["mesh_name"] or ""):
            return self._stop(context)
        if self._transform_running(context):
            if self._had_gap:
                # a second transform after the tweak drag: the user is
                # editing on - stay in curve edit, the panel button leads back
                return self._stop(context)
            self._saw_transform = True
            self._idle_ticks = 0
            return {"PASS_THROUGH"}
        if event.type == "TIMER":
            self._idle_ticks += 1
        if self._saw_transform:
            self._had_gap = True
            # a short grace window, in case the user chains the next
            # transform right after releasing
            if self._idle_ticks >= 4:
                finish_curve_tweak(context)
                return self._stop(context)
        elif self._idle_ticks >= 40:
            # the translate never showed up - don't watch forever
            return self._stop(context)
        return {"PASS_THROUGH"}


class FinishCurveTweakOperator(bpy.types.Operator):
    """Return to the mesh this curve tweak came from - restarts the
    always-on modal if it was running when the tweak started"""

    bl_idname = "object.final_topology_finish_curve_tweak"
    bl_label = "Back to Mesh"

    @classmethod
    def poll(cls, context):
        return bpy.data.objects.get(_curve_tweak_state["mesh_name"] or "") is not None

    def execute(self, context):
        if not finish_curve_tweak(context):
            return {"CANCELLED"}
        return {"FINISHED"}


class VIEW3D_PT_final_topology_curve_tweak(Panel):
    """The way back while editing a constraint's target curve."""

    bl_label = "Final Topology"
    bl_idname = "VIEW3D_PT_final_topology_curve_tweak"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Edit"

    @classmethod
    def poll(cls, context):
        return (
            context.mode == "EDIT_CURVE"
            and bpy.data.objects.get(_curve_tweak_state["mesh_name"] or "") is not None
        )

    def draw(self, context):
        layout = self.layout
        mesh_name = _curve_tweak_state["mesh_name"]
        layout.operator(
            FinishCurveTweakOperator.bl_idname,
            text=f"Back to {mesh_name}",
            icon="LOOP_BACK",
        )


class CurvePointsGizmoGroup(gizmos.PointHandlesGizmoGroupBase, GizmoGroup):
    bl_idname = "OBJECT_GGT_ft_curve_points"
    bl_label = "Curve Constraint Control Points"
    bl_space_type = "VIEW_3D"
    bl_region_type = "WINDOW"
    bl_options = {"3D", "PERSISTENT"}

    @classmethod
    def poll(cls, context):
        ob, c = get_active_curve_constraint(context)
        return c is not None

    def get_point_target(self, context):
        ob, curves = get_curve_constraint_curves(context)
        if not curves:
            return None
        return TargetCurveControlPointsMulti(curves)

    def on_point_tweak(self, index):
        """Hand the clicked cluster to the native-transform tweak."""
        target = self.get_point_target(bpy.context)
        if target is None or index >= target.count():
            return False
        members = []
        for adapter_index, point_index in target.clusters[index][1]:
            adapter = target.adapters[adapter_index]
            spline_index, spline_point, is_bezier = adapter._index_map[point_index]
            members.append(
                (adapter.curve_object.name, spline_index, spline_point, is_bezier)
            )
        global _pending_curve_tweak
        _pending_curve_tweak = members
        bpy.ops.object.final_topology_curve_point_tweak("INVOKE_DEFAULT")
        return True


class CurveConstraintGizmoGroup(gizmos.TransformGizmoGroupBase, GizmoGroup):
    bl_idname = "OBJECT_GGT_ft_curve_constraint"
    bl_label = "Curve Constraint Transform"
    bl_space_type = "VIEW_3D"
    bl_region_type = "WINDOW"
    bl_options = {"3D", "PERSISTENT"}

    @classmethod
    def poll(cls, context):
        ob, c = get_active_curve_constraint(context)
        return c is not None

    def get_targets(self, context):
        ob, c = get_active_curve_constraint(context)
        if c is None:
            return []
        return [TargetCurveGizmoTarget(c.target_curve)]


class FixedPlaneGizmoTarget:
    """Gizmo adapter transforming a fixed plane constraint.

    Arrows move the stored center and only show while the center is fixed,
    dials tilt the stored normal and only show while the normal is fixed.
    World gizmo input gets converted into the object's local space here.
    """

    def __init__(self, object, constraint):
        self.object = object
        self.constraint = constraint

    def _world_normal(self):
        normal = (
            self.object.matrix_world.inverted().transposed().to_3x3()
            @ Vector(self.constraint.normal)
        )
        if normal.length < 1e-12:
            normal = Vector((0.0, 0.0, 1.0))
        return normal.normalized()

    def location(self):
        return self.object.matrix_world @ Vector(self.constraint.center)

    def orientation(self):
        mode = gizmos.orientation_mode(bpy.context)
        if mode == "GLOBAL":
            return Matrix.Identity(3)
        if mode == "LOCAL":
            return self.object.matrix_world.to_quaternion().to_matrix()
        z = self._world_normal()
        x = z.orthogonal().normalized()
        y = z.cross(x)
        return Matrix((x, y, z)).transposed()

    def translation_axes(self):
        fixed = self.constraint.fix_center
        return (fixed, fixed, fixed)

    def rotation_axes(self):
        if not self.constraint.fix_normal:
            return (False, False, False)
        # in the plane's own frame, spinning around the normal changes nothing
        if gizmos.orientation_mode(bpy.context) == "NORMAL":
            return (True, True, False)
        return (True, True, True)

    def snapshot(self):
        return (
            Vector(self.constraint.center),
            Vector(self.constraint.normal),
            self.orientation(),
        )

    def translate(self, snapshot, axis_index, distance):
        world_offset = snapshot[2].col[axis_index] * distance
        local_offset = self.object.matrix_world.inverted().to_3x3() @ world_offset
        self.constraint.center = snapshot[0] + local_offset

    def rotate(self, snapshot, axis_index, angle):
        world_axis = snapshot[2].col[axis_index]
        local_axis = self.object.matrix_world.inverted().to_3x3() @ world_axis
        if local_axis.length < 1e-12:
            return
        rotation = Matrix.Rotation(angle, 3, local_axis.normalized())
        self.constraint.normal = rotation @ snapshot[1]


def get_active_fixed_plane_constraint(context):
    """The active constraint, if it's an enabled plane one with a fixed part."""
    ob = context.object
    if not (ob and ob.type == "MESH" and ob.mode == "EDIT"):
        return None, None
    if not hasattr(ob.data, "ft_custom_constraints"):
        return None, None
    cs = ob.data.ft_custom_constraints
    index = ob.data.ft_custom_constraints_index
    if not (0 <= index < len(cs)):
        return None, None
    c = cs[index]
    if (
        c.constraint_type == "PLANE"
        and c.enabled
        and (c.fix_center or c.fix_normal)
    ):
        return ob, c
    return None, None


class PlaneConstraintGizmoGroup(gizmos.TransformGizmoGroupBase, GizmoGroup):
    bl_idname = "OBJECT_GGT_ft_plane_constraint"
    bl_label = "Plane Constraint Transform"
    bl_space_type = "VIEW_3D"
    bl_region_type = "WINDOW"
    bl_options = {"3D", "PERSISTENT"}

    @classmethod
    def poll(cls, context):
        ob, c = get_active_fixed_plane_constraint(context)
        return c is not None

    def get_targets(self, context):
        ob, c = get_active_fixed_plane_constraint(context)
        if c is None:
            return []
        return [FixedPlaneGizmoTarget(ob, c)]


def get_active_fixed_circle_constraint(context):
    """The active constraint, if it's an enabled fixed circle one."""
    ob = context.object
    if not (ob and ob.type == "MESH" and ob.mode == "EDIT"):
        return None, None
    if not hasattr(ob.data, "ft_custom_constraints"):
        return None, None
    cs = ob.data.ft_custom_constraints
    index = ob.data.ft_custom_constraints_index
    if not (0 <= index < len(cs)):
        return None, None
    c = cs[index]
    if (
        c.constraint_type == "CIRCLE"
        and c.fix_circle
        and c.enabled
        and len(c.fixed_circles) > 0
    ):
        return ob, c
    return None, None


class CircleConstraintGizmoGroup(gizmos.TransformGizmoGroupBase, GizmoGroup):
    bl_idname = "OBJECT_GGT_ft_circle_constraint"
    bl_label = "Circle Constraint Transform"
    bl_space_type = "VIEW_3D"
    bl_region_type = "WINDOW"
    bl_options = {"3D", "PERSISTENT"}

    @classmethod
    def poll(cls, context):
        ob, c = get_active_fixed_circle_constraint(context)
        return c is not None

    def get_targets(self, context):
        ob, c = get_active_fixed_circle_constraint(context)
        if c is None:
            return []
        return [FixedCircleGizmoTarget(ob, c, i) for i in range(len(c.fixed_circles))]


class TransformConstraintGizmo(GizmoGroup):
    bl_idname = "OBJECT_GGT_final_topology_constraint_gizmo"
    bl_label = "Final Topology Constraint Gizmo"
    bl_space_type = "VIEW_3D"
    bl_region_type = "WINDOW"
    bl_options = {"3D", "PERSISTENT", "SCALE", "EXCLUDE_MODAL"}

    @classmethod
    def poll(cls, context):
        ob = context.object
        is_edit_mode = ob and ob.mode == "EDIT" and ob.type == "MESH"
        if not is_edit_mode:
            return False
        has_constraint = len(ob.data.ft_custom_constraints) > 0
        return has_constraint

    def get_active_constraint(self):
        ob = bpy.context.object
        return ob.data.ft_custom_constraints[ob.data.ft_custom_constraints_index]

    def set_gizmo_matrix(self, axis=0):
        ob = bpy.context.object
        gz = self.gizmos[axis]
        active_constraint = self.get_active_constraint()

        # Common position for all gizmos
        if active_constraint.constraint_type in ["PLANE", "PLANE_FIXED", "CIRCLE"]:
            gizmo_position = Vector(active_constraint.center)
        elif active_constraint.constraint_type == "CURVE":
            gizmo_position = Vector(active_constraint.target_curve.location)

        # Set the orientation of the gizmo based on its axis
        matrix_basis = (
            self.gizmo_axes[axis].to_track_quat("Z", "Y").to_matrix().to_4x4()
        )

        # offset_position = self.gizmo_axes[axis] - gz.offset
        # Set the translation for all gizmos to the common position
        matrix_basis.translation = gizmo_position
        # active_constraint_matrix = Matrix(myLocRotScale) @ matrix_basis
        gz.matrix_basis = matrix_basis

    def setup(self, context):
        def move_get_x():
            return 0

        def move_set_x(value):
            active_constraint = self.get_active_constraint()
            if active_constraint.constraint_type in ["PLANE", "PLANE_FIXED", "CIRCLE"]:
                active_constraint.center[0] = self.original_location.x + value
            elif active_constraint.constraint_type == "CURVE":
                active_constraint.target_curve.location[0] = (
                    self.original_location.x + value / 2
                )

        def move_get_y():
            return 0

        def move_set_y(value):
            active_constraint = self.get_active_constraint()
            if active_constraint.constraint_type in ["PLANE", "PLANE_FIXED", "CIRCLE"]:
                active_constraint.center[1] = self.original_location.y + value
            elif active_constraint.constraint_type == "CURVE":
                active_constraint.target_curve.location[1] = (
                    self.original_location.y + value
                )

        def move_get_z():
            return 0

        def move_set_z(value):
            active_constraint = self.get_active_constraint()
            if active_constraint.constraint_type in ["PLANE", "PLANE_FIXED", "CIRCLE"]:
                active_constraint.center[2] = self.original_location.z + value
            elif active_constraint.constraint_type == "CURVE":
                active_constraint.target_curve.location[2] = (
                    self.original_location.z + value
                )

        def rotate_get_x():
            return 0

        def rotate_set_x(value):
            active_constraint = self.get_active_constraint()
            if active_constraint.constraint_type in ["PLANE", "PLANE_FIXED", "CIRCLE"]:
                active_constraint.normal = Vector(active_constraint.normal).rotate(
                    Euler((value, 0, 0))
                )
            if active_constraint.constraint_type == "CURVE":
                active_constraint.target_curve.rotation_euler[0] = value

        def rotate_get_y():
            return 0

        ob = context.object
        active_constraint = self.get_active_constraint()
        self.original_location = Vector(active_constraint.center)
        if active_constraint.constraint_type == "CURVE":
            self.original_location = active_constraint.target_curve.location.copy()

        gizmos = self.gizmos
        gizmo_functions = [
            (move_get_x, move_set_x),
            (move_get_y, move_set_y),
            (move_get_z, move_set_z),
            (rotate_get_x, rotate_set_x),
        ]
        self.gizmo_axes = [
            ob.matrix_world @ Vector((1, 0, 0)),
            ob.matrix_world @ Vector((0, 1, 0)),
            ob.matrix_world @ Vector((0, 0, 1)),
            ob.matrix_world @ Vector((1, 0, 0)),
        ]
        self.gizmo_colors = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 0.0, 0.0),
        ]
        gizmo_named_list = ["X", "Y", "Z", "RX"]
        gizmo_dict = {}
        for i in range(4):
            if i <= 2:
                gz = gizmos.new("GIZMO_GT_arrow_3d")
            else:
                gz = gizmos.new("GIZMO_GT_rotate_3d")

            self.set_gizmo_matrix(axis=i)

            gz.color = self.gizmo_colors[i]
            gz.alpha = 0.5

            gz.color_highlight = (
                self.gizmo_colors[i][0] * 1.2,
                self.gizmo_colors[i][1] * 1.2,
                self.gizmo_colors[i][1] * 1.2,
            )
            gz.alpha_highlight = 0.5

            gz.target_set_handler(
                "offset", get=gizmo_functions[i][0], set=gizmo_functions[i][1]
            )
            gizmo_dict[gizmo_named_list[i]] = gz

    def refresh(self, context):
        ob = context.object
        active_constraint = self.get_active_constraint()
        gizmo_position = active_constraint.center
        for i, gz in enumerate(self.gizmos):
            # gz.matrix_basis.translation = gizmo_position
            gz.hide = not active_constraint.enabled
            gz.hide_select = not active_constraint.enabled
            self.set_gizmo_matrix(axis=i)


#
# class TransformConstraint(Operator):
#     """Select all vertices on one side of a plane defined by a location and a direction"""
#
#     bl_idname = "mesh.final_topology_transform_constraint"
#     bl_label = "Transform Constraint"
#     bl_options = {"REGISTER", "UNDO"}
#
#     plane_co: FloatVectorProperty(
#         size=3,
#         default=(0, 0, 0),
#     )
#     plane_no: FloatVectorProperty(
#         size=3,
#         default=(0, 0, 1),
#     )
#
#     @classmethod
#     def poll(cls, context):
#         return context.mode == "EDIT_MESH"
#
#     def invoke(self, context, event):
#         mesh = context.active_object.data
#         active_constraint = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]
#         if not self.properties.is_property_set("plane_co"):
#             self.plane_co = active_constraint.center
#
#         if not self.properties.is_property_set("plane_no"):
#             if context.space_data.type == "VIEW_3D":
#                 self.plane_no = active_constraint.normal
#
#         self.execute(context)
#
#         if context.space_data.type == "VIEW_3D":
#             wm = context.window_manager
#             wm.gizmo_group_type_ensure(TransformConstraintGizmoGroup.bl_idname)
#
#         return {"FINISHED"}
#
#     def execute(self, context):
#         mesh = context.active_object.data
#         active_constraint = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]
#         if active_constraint.constraint_type in ["PLANE", "PLANE_FIXED"]:
#             active_constraint.center = self.plane_co
#             active_constraint.normal = self.plane_no
#         elif active_constraint.constraint_type == "CURVE":
#             active_constraint.target_curve.location = self.plane_co
#             plane_no = Vector(self.plane_no)
#             active_constraint.target_curve.rotation_euler = plane_no.to_track_quat(
#                 "Z", "Y"
#             ).to_euler()
#             active_constraint.center = self.plane_co
#             active_constraint.normal = self.plane_no
#
#         return {"FINISHED"}


classes = [
    SlideOptimizeOperator,
    # FunTopologyOperator,
    FlattenSelectionOperator,
    NormalLoopAlign,
    FunTopologyDecimateOperator,
    FixedLineItem,
    FixedCircleItem,
    CustomConstraint,
    AddConstraintOperator,
    AddSelectionToConstraintOperator,
    CurveFromSelectionOperator,
    DeleteConstraintOperator,
    CUSTOM_UL_constraint_list,
    VIEW3D_PT_final_topology_constraints,
    VIEW3D_PT_final_topology_extra_operators,
    gizmos.FTPointHandleGizmo,
    CircleConstraintGizmoGroup,
    CurveConstraintGizmoGroup,
    RemoveSelectionFromAllConstraintsOperator,
    PinSelectionOperator,
    CurvePointTweakOperator,
    FinishCurveTweakOperator,
    VIEW3D_PT_final_topology_curve_tweak,
    CurvePointsGizmoGroup,
    PlaneConstraintGizmoGroup,
    # TransformConstraintGizmo,
]


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Mesh.ft_custom_constraints = bpy.props.CollectionProperty(
        type=CustomConstraint
    )
    bpy.types.Mesh.ft_custom_constraints_index = bpy.props.IntProperty(
        "Actve FT Constraint", default=0, update=update_constraint_index
    )
    bpy.app.handlers.undo_post.append(_constraint_undo_post)
    bpy.app.handlers.redo_post.append(_constraint_redo_post)
    bpy.app.handlers.load_post.append(_constraint_load_post)
    # baseline for the file already open when the addon comes up - bpy.data
    # is restricted while Blender enables addons at startup, and there the
    # load_post handler does this job anyway
    try:
        meshes = bpy.data.meshes
    except AttributeError:
        meshes = []
    for mesh in meshes:
        if len(mesh.ft_custom_constraints):
            record_constraint_undo_state(mesh)


def unregister():
    for handler_list, handler in (
        (bpy.app.handlers.undo_post, _constraint_undo_post),
        (bpy.app.handlers.redo_post, _constraint_redo_post),
        (bpy.app.handlers.load_post, _constraint_load_post),
    ):
        if handler in handler_list:
            handler_list.remove(handler)
    for cls in classes:
        bpy.utils.unregister_class(cls)
    del bpy.types.Mesh.ft_custom_constraints
