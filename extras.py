#


import bmesh

# this file has separate tools for experts in cad and automotive design
import bpy
from mathutils import Vector, kdtree, Matrix, Euler, geometry


# project into XY plane,
up = Vector((0, 0, 1))
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

from . import draw, utils

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

def to_curve_verts_calculate(
    loop,
    curve_snapping="PROJECT_PLANE",
    curve_distribution="EVEN",
    source_curve=None,
    kd=None,
    normal=None,
    edit_loop_for_endpoints=None,
):
    """Calculate new positions for vertices to snap to the closest point on a curve, Return a dictionary with the new positions"""
    target_offsets = {}
    # print(loop)
    loop_closed = loop[1]

    curve_world_matrix = source_curve.matrix_world

    object_world_matrix = bpy.context.active_object.matrix_world
    # Local Z-axis vector
    local_z = Vector((0, 0, 1))

    # Transform the local Z-axis vector by the object's rotation matrix
    curve_plane_normal = source_curve.rotation_euler.to_matrix() @ local_z
    curve_plane_normal.normalize()
    # print("curve_plane_normal", curve_plane_normal)
    # KD was not passed, build it
    if 1:  # kd is None:
        kd = build_kd_curve_cache(
            source_curve,
            endpoints_only=not loop_closed,
            flatten=curve_snapping == "PROJECT_PLANE",
        )

    # TODO MOVE THIS TO CACHE
    depsgraph = bpy.context.evaluated_depsgraph_get()
    bmesh_curve = utils.get_evaluated_bm(source_curve, depsgraph)
    
    #curve_length = calculate_curve_length(source_curve, curve_snapping)
    # use blender's original function to calculate curve length
    curve_length =  source_curve.data.splines[0].calc_length() * source_curve.scale.x

    
    # calculate loop length
    loop_length = 0
    for i, vert in enumerate(loop[0]):
        if i == 0:
            continue
        loop_length += (vert.co - loop[0][i - 1].co).length
    # cyclic loops have one more edge
    if loop_closed:
        loop_length += (loop[0][0].co - loop[0][-1].co).length

    # Find the closest point on the curve for each input vertex
    direction = 1
    distance_traveled = 0
    target_distance = 0
    for i, vert in enumerate(loop[0]):
        # For first and last vertices in open loops with subdivision,
        # use edit mesh vertex positions as reference
        use_edit_vert_for_endpoint = (
            edit_loop_for_endpoints is not None
            and not loop_closed
            and (i == 0 or i == len(loop[0]) - 1)
        )

        if use_edit_vert_for_endpoint:
            edit_vert = edit_loop_for_endpoints[0][i]
            reference_co = eval_point(
                object_world_matrix @ edit_vert.co, curve_snapping, source_curve
            )
            vert_index_for_offset = edit_vert.index
        else:
            reference_co = eval_point(
                object_world_matrix @ vert.co, curve_snapping, source_curve
            )
            vert_index_for_offset = vert.index

        if i == 0:
            next_point_index = i + 1
            if next_point_index >= len(loop[0]):
                next_point_index = 0

            reference_co1 = eval_point(
                object_world_matrix @ loop[0][next_point_index].co,
                curve_snapping,
                source_curve,
            )
            reference_co2 = eval_point(
                object_world_matrix @ loop[0][i - 1].co, curve_snapping, source_curve
            )

        if i == 0:
            # find first point on the curve for closed loops
            if loop_closed:
                co, index, dist = kd.find(reference_co)
                target_offsets[vert_index_for_offset] = co - reference_co
                last_index = index
                next_point_index = index + 1
                if next_point_index >= len(bmesh_curve.verts):
                    next_point_index = 0
                next_point = eval_point(
                    curve_world_matrix @ bmesh_curve.verts[next_point_index].co,
                    curve_snapping,
                    source_curve,
                )

                # estimate which direction to go by angle - this seems to be wrong by now
                
                angle1 = (reference_co1 - reference_co).angle(next_point - co)
                angle2 = (reference_co2 - reference_co).angle(next_point - co)
                # print(angle1, angle2)
                if angle1 > angle2:
                    direction = -1
            else:
                # find closest end point of the curve for open loops
                curve_start = eval_point(
                    curve_world_matrix @ bmesh_curve.verts[0].co,
                    curve_snapping,
                    source_curve,
                )
                curve_end = eval_point(
                    curve_world_matrix @ bmesh_curve.verts[-1].co,
                    curve_snapping,
                    source_curve,
                )

                dist_start = (reference_co - curve_start).length
                dist_end = (reference_co - curve_end).length
                if dist_start < dist_end:
                    co = curve_start
                    index = 0
                else:
                    co = curve_end
                    index = len(bmesh_curve.verts) - 1
                    direction = -1

                last_index = index

            start_point = co
            target_offsets[vert_index_for_offset] = co - reference_co

        else:
            # iterate through rest of points of spline depending on distribution type
            if curve_distribution == "EVEN":
                target_distance += curve_length / (len(loop[0]) - 1 + loop_closed)
            if curve_distribution == "ORIGINAL":
                ratio = curve_length / loop_length
                target_distance += ratio * (vert.co - loop[0][i - 1].co).length

            for i_curve_offset in range(0, len(bmesh_curve.verts)):
                # Get the end point of current segment to check
                check_index = last_index + direction
                
                if check_index >= len(bmesh_curve.verts):
                    if not loop_closed:  # Finish for not closed loops
                        end_point = eval_point(
                            curve_world_matrix @ bmesh_curve.verts[-1].co,
                            curve_snapping,
                            source_curve,
                        )
                        target_offsets[vert_index_for_offset] = end_point - reference_co
                        break
                    check_index = 0  # Wrap around for cyclic curves
                    
                if check_index < 0:
                    if not loop_closed and i > 1:  # Finish for not closed loops
                        end_point = eval_point(
                            curve_world_matrix @ bmesh_curve.verts[0].co,
                            curve_snapping,
                            source_curve,
                        )
                        target_offsets[vert_index_for_offset] = end_point - reference_co
                        break
                    check_index = len(bmesh_curve.verts) - 1  # Wrap around for cyclic curves

                end_point = eval_point(
                    curve_world_matrix @ bmesh_curve.verts[check_index].co,
                    curve_snapping,
                    source_curve,
                )

                segment_length = (end_point - start_point).length
                distance_would_be_traveled = distance_traveled + segment_length
                
                if distance_would_be_traveled >= target_distance:
                    # found the segment, let's interpolate
                    ratio = (target_distance - distance_traveled) / segment_length
                    co = start_point.lerp(end_point, ratio)
                    target_offsets[vert_index_for_offset] = co - reference_co
                    distance_traveled = target_distance
                    start_point = co
                    # Don't update last_index - we're still in this segment
                    break
                else:
                    # Move to next segment
                    distance_traveled += segment_length
                    start_point = end_point
                    last_index = check_index

        # Store the offset for each vertex

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


def space_calculate(loops_data, interpolation="cubic", method="EVEN"):
    """Calculate positions to space vertices along loops

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        interpolation: 'cubic' or 'linear'
        method: 'EVEN' for equal distances, 'RATIO' for one constant ratio between
            neighbouring segments - open loops only, closed loops can't keep a
            ratio other than one all the way around and fall back to even.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}

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

        # Calculate splines
        if interpolation == 'cubic':
            splines = calculate_cubic_splines_simple(spline_verts, spline_tknots)
        else:  # linear
            splines = calculate_linear_splines_simple(spline_verts, spline_tknots)

        if not splines:
            continue

        # Calculate new positions for each vertex
        for i, v in enumerate(verts):
            new_pos = evaluate_spline(splines, spline_tknots, tpoints[i], interpolation)
            target_offsets[v.index] = new_pos - v.co

    return target_offsets


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
        # Store for next frame consistency
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
                    flatten=c.curve_snapping == "PROJECT_PLANE",
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


def curvature_loop_samples(verts, is_circular):
    """Measure curvature at every vertex of a loop that has one on both sides.

    Uses the arc length parametrization, so both the parameter and the second
    derivative are built from the real edge lengths - the curvature is the one of
    the whole segment, not of an idealized evenly spaced polyline.

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

        normal = verts[i].normal.copy()
        if normal.length_squared < 1e-12:
            continue
        normal.normalize()

        # second derivative on an unevenly spaced grid
        second_derivative = (
            prev_co * h2 - co * (h1 + h2) + next_co * h1
        ) * (2.0 / (h1 * h2 * (h1 + h2)))

        samples.append(
            {
                "index": i,
                "t": tknots[i],
                "k": -second_derivative.dot(normal),
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


def curvature_calculate(loops_data, mode="CONSTANT", max_offset_ratio=0.5):
    """Push vertices along their normals so the loop keeps an even curvature.

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        mode: 'CONSTANT' for one curvature along the loop (an arc),
              'LINEAR' for a curvature that changes at a constant rate.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}

    for loop_data in loops_data:
        verts = loop_data[0]
        is_circular = loop_data[1]
        if len(verts) < 3:
            continue

        samples = curvature_loop_samples(verts, is_circular)
        if len(samples) < 2:
            continue

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

        for s, target in zip(samples, targets):
            # moving a vertex by d along its normal changes its curvature
            # by d * 2 / (h1*h2), so invert that to hit the target curvature
            offset = (target - s["k"]) * s["h1"] * s["h2"] * 0.5
            # keep a single step from overshooting into a neighbour
            limit = max_offset_ratio * min(s["h1"], s["h2"])
            offset = max(-limit, min(limit, offset))
            target_offsets[verts[s["index"]].index] = s["normal"] * offset

    return target_offsets


def line_verts_calculate(loops_data, distribution="ORIGINAL", fixed_lines=None):
    """Pull the vertices of open loops onto a straight line.

    The line runs from the first to the last vertex of each loop, so the ends stay
    where they are and only the vertices between them move. With fixed_lines given
    (a list of (start, end) pairs, one per loop) each loop instead gets pulled onto
    the nearest stored line.

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        distribution: 'ORIGINAL' moves each vertex straight onto the line and keeps
            its own position along it, 'EVEN' spreads the vertices between the ends.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}

    for loop_data in loops_data:
        verts = loop_data[0]
        is_circular = loop_data[1]
        # a closed loop has no two ends to span a line between
        if is_circular or len(verts) < 2:
            continue

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

        for i, v in enumerate(verts):
            if distribution == "EVEN":
                target = line_start + direction * (line_length * i / (len(verts) - 1))
            else:
                # closest point on the line, so the vertex only moves sideways onto it
                target = line_start + direction * (v.co - line_start).dot(direction)
            target_offsets[v.index] = target - v.co

    return target_offsets


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


def circle_verts_calculate(loops_data, distribution="ORIGINAL", fixed_circles=None):
    """Pull the vertices of loops onto a circle.

    Without fixed circles each loop gets its own best fit. With fixed_circles given
    (a list of (center, normal, radius) tuples, one per loop) each loop instead gets
    pulled onto the nearest stored circle. Open loops work too, they land on an arc
    of the circle.

    Args:
        loops_data: List of loops, each as [verts_list, is_circular]
        distribution: 'ORIGINAL' keeps each vertex at its own angle around the
            circle, 'EVEN' spreads the vertices at equal angles.

    Returns:
        Dictionary mapping vertex indices to offset vectors
    """
    target_offsets = {}

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
            fit = fit_circle_to_loop(verts)
            if fit is None:
                continue
            c, n, r = fit

        u = n.orthogonal().normalized()
        w = n.cross(u)

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
            target_offsets[vert.index] = target - vert.co

    return target_offsets


def evaluate_constraints(object, bmesh_edit=None, bmesh_eval=None, inverse_subdivide_prep=None):
    global constraints_cache
    # separate caching for performance
    check_constraints_cache(object)
    cs = object.data.ft_custom_constraints
    target_offsets_all = []
    user_preferences = bpy.context.preferences.addons[__package__].preferences

    for i, c in enumerate(cs):
        # skip disabled constraints
        if not c.enabled:
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

        if c.constraint_type == "INCLINATION_LIMIT":
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
                        curve_snapping=c.curve_snapping,
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
            target_offsets = line_verts_calculate(
                constraint_verts_loops,
                distribution="EVEN" if c.even_distribution else "ORIGINAL",
                fixed_lines=fixed_lines,
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
            target_offsets = circle_verts_calculate(
                constraint_verts_loops,
                distribution="EVEN" if c.even_distribution else "ORIGINAL",
                fixed_circles=fixed_circles,
            )

        # evaluate curvature constraint
        elif c.constraint_type == "CURVATURE":
            # CURVATURE handles multi-loop format
            target_offsets = curvature_calculate(
                constraint_verts_loops, mode=c.curvature_mode
            )

        # evaluate space constraint
        elif c.constraint_type == "SPACE":
            # SPACE handles multi-loop format
            target_offsets = space_calculate(
                constraint_verts_loops,
                interpolation=c.space_interpolation,
                method=c.space_method,
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
    # select constraint vertices
    constraint = context.object.data.ft_custom_constraints[
        context.object.data.ft_custom_constraints_index
    ]
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

def update_constraint_data(self, context):
    global constraints_cache
    constraints_cache = []


def update_plane_fix_flags(self, context):
    """Recalculate center/normal when fix flags are toggled on"""
    if self.constraint_type != "PLANE":
        return
    
    # If neither fix flag is on, nothing to recalculate
    if not (self.fix_center or self.fix_normal):
        return
    
    # Get the constraint's vertices
    mesh = context.active_object.data
    bm = bmesh.from_edit_mesh(mesh)
    
    # Get vertices from attribute
    if not self.attribute_name or self.attribute_name not in bm.edges.layers.float:
        print(f"Constraint {self.name} Attribute {self.attribute_name} not found")
        return
        
    loops = utils.get_attribute_elements(context.active_object, bm, self, domain="EDGE", as_domain="POINT")
    
    joined_normal = None
    
    # Calculate common center and normal from all loops
    joined_normal, joined_center = calculate_joined_normal(self, loops)

    # Only update the values that are now fixed (since this callback is triggered on change)
    if self.fix_center:
        self.center = joined_center
    if self.fix_normal:
        self.normal = joined_normal


def update_line_fix_flags(self, context):
    """Capture the current ends of every open loop when the line gets fixed"""
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
    self.fixed_circles.clear()
    for loop_data in loops:
        fit = fit_circle_to_loop(loop_data[0])
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
            # (
            #     "ANGLE",
            #     "Angle",
            #     "Limit angle for manufacturing purposes",
            #     "LINCURVE",
            #     7,
            # ),
        ],
        update=update_constraint_data,
    )
    fix_center: bpy.props.BoolProperty(
        name="Fix Center",
        default=False,
        description="Fix the plane center position",
        update=update_plane_fix_flags,
    )
    fix_normal: bpy.props.BoolProperty(
        name="Fix Normal",
        default=False,
        description="Fix the plane normal/rotation",
        update=update_plane_fix_flags,
    )
    join_center: bpy.props.BoolProperty(
        name="Join Center",
        default=False,
        description="Calculate common center for all loops (only when center is not fixed)",
    )
    join_normal: bpy.props.BoolProperty(
        name="Join Normal",
        default=False,
        description="Calculate common normal for all loops (only when normal is not fixed)",
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
    curve_snapping: bpy.props.EnumProperty(
        name="Curve Snapping",
        default="PROJECT_PLANE",
        items=[
            ("3D", "3D", "Project to curve in 3D"),
            (
                "PROJECT_PLANE",
                "Project on Curve plane",
                "Project curve on the mesh and snap closest",
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
        default="cubic",
        items=[
            ("cubic", "Cubic", "Natural cubic spline, smooth results"),
            ("linear", "Linear", "Simple and fast linear algorithm"),
        ],
        description="Algorithm used for spacing interpolation",
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

    # Line constraint properties
    fix_line: bpy.props.BoolProperty(
        name="Fix Line",
        default=False,
        description="Keep the line where it is now, instead of letting the loop ends define it",
        update=update_line_fix_flags,
    )
    fixed_lines: bpy.props.CollectionProperty(type=FixedLineItem)

    # Circle constraint properties
    fix_circle: bpy.props.BoolProperty(
        name="Fix Circle",
        default=False,
        description="Keep the circle where it is now, instead of refitting it to the loop",
        update=update_circle_fix_flags,
    )
    fixed_circles: bpy.props.CollectionProperty(type=FixedCircleItem)


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

        if len(mesh.ft_custom_constraints) > 0:
            ac = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]
            layout.prop(ac, "name")
            layout.prop(ac, "constraint_type")
            if ac.constraint_type != "INVERSE_SUBDIVIDE":
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
                layout.prop(ac, "curve_snapping")
                layout.prop(ac, "even_distribution")
            
            if ac.constraint_type == "SPACE":
                layout.prop(ac, "space_method")
                layout.prop(ac, "space_interpolation")

            if ac.constraint_type == "CURVATURE":
                layout.prop(ac, "curvature_mode")

            if ac.constraint_type == "LINE":
                layout.prop(ac, "even_distribution")
                layout.prop(ac, "fix_line")
                if ac.fix_line:
                    for item in ac.fixed_lines:
                        col = layout.column(align=True)
                        col.prop(item, "start")
                        col.prop(item, "end")

            if ac.constraint_type == "CIRCLE":
                layout.prop(ac, "even_distribution")
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
                    row = layout.row()
                    row.prop(ac, "use_object_or_collection", text="")
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


class AddSelectionToConstraintOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_add_selection_to_constraint"
    bl_label = "Add Selection to Constraint"
    bl_description = (
        "\n\nSelect vertices and run this operator to add them to the constraint."
    )
    bl_options = {"REGISTER", "UNDO"}

    remove: bpy.props.BoolProperty(name="Remove", default=False)

    def execute(self, context):
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
        return {"FINISHED"}

def get_constraint_domain_type(constraint_type):
    if constraint_type in ["PLANE", "CURVE", "SLIDE_OPTIMIZE", "SPACE", "CURVATURE", "LINE", "CIRCLE"]:
        return "EDGE"
    elif constraint_type == "INVERSE_SUBDIVIDE":
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
    target_curve: bpy.props.PointerProperty(
        type=bpy.types.Object, name="Target Curve", poll=filter_curves
    )
    curve_snapping: bpy.props.EnumProperty(
        name="Curve Snapping",
        default="PROJECT_PLANE",
        items=[
            ("3D", "3D", "Project to curve in 3D"),
            (
                "PROJECT_PLANE",
                "Project on Curve plane",
                "Project curve on the mesh and snap closest",
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
        # push undo step
        bpy.ops.ed.undo_push()
        return {"FINISHED"}

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "name")
        layout.prop(self, "constraint_type")
        layout.prop(self, "works_on_subdivision")
        if self.constraint_type == "CURVE":
            layout.prop(self, "target_curve")
            layout.prop(self, "curve_snapping")


class DeleteConstraintOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_delete_constraint"
    bl_label = "Delete Constraint"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        mesh = context.active_object.data
        active_obj = context.active_object

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
    DeleteConstraintOperator,
    CUSTOM_UL_constraint_list,
    VIEW3D_PT_final_topology_constraints,
    VIEW3D_PT_final_topology_extra_operators,
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


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)
    del bpy.types.Mesh.ft_custom_constraints
