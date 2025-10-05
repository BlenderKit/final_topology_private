#


import bmesh

# this file has separate tools for experts in cad and automotive design
import bpy
from mathutils import Vector, kdtree, Matrix, Euler, geometry


# project into XY plane,
up = Vector((0, 0, 1))
from math import atan2, pi
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
        reference_co = eval_point(
            object_world_matrix @ vert.co, curve_snapping, source_curve
        )
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
                target_offsets[vert.index] = co - reference_co
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
            target_offsets[vert.index] = co - reference_co

        else:
            # iterate through rest of points of spline depending on distribution type
            if curve_distribution == "EVEN":
                target_distance += curve_length / (len(loop[0]) - 1 + loop_closed)
            if curve_distribution == "ORIGINAL":
                ratio = curve_length / loop_length
                target_distance += ratio * (vert.co - loop[0][i - 1].co).length

            for i_curve_offset in range(0, len(bmesh_curve.verts)):
                last_index += direction

                if last_index >= len(bmesh_curve.verts):
                    if not loop_closed:  # Finish for not closed loops
                        end_point = eval_point(
                            curve_world_matrix @ bmesh_curve.verts[-1].co,
                            curve_snapping,
                            source_curve,
                        )

                        target_offsets[vert.index] = end_point - reference_co
                        last_index -= 1  # get one step back for possible more points
                        break
                    last_index = 0  # Wrap around for cyclic curves
                if last_index < 0:
                    if not loop_closed and i > 1:  # Finish for not closed loops
                        end_point = eval_point(
                            curve_world_matrix @ bmesh_curve.verts[0].co,
                            curve_snapping,
                            source_curve,
                        )

                        target_offsets[vert.index] = end_point - reference_co
                        last_index += 1  # get one step back for possible more points
                        break
                    last_index = (
                        len(bmesh_curve.verts) - 1
                    )  # Wrap around for cyclic curves

                end_point = eval_point(
                    curve_world_matrix @ bmesh_curve.verts[last_index].co,
                    curve_snapping,
                    source_curve,
                )

                segment_length = (end_point - start_point).length
                distance_would_be_traveled = distance_traveled + segment_length
                
                if distance_would_be_traveled >= target_distance:
                    # found the segment, let's interpolate
                    ratio = (target_distance - distance_traveled) / segment_length
                    co = start_point.lerp(end_point, ratio)
                    target_offsets[vert.index] = co - reference_co
                    distance_traveled = target_distance
                    start_point = co
                    break
                else:
                    distance_traveled += segment_length
                    start_point = end_point

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
        # Get the vertices affected by the constraint
        constraint_verts_loops_edit = utils.get_attribute_elements(
            object, bmesh_edit, c, domain=domain, as_domain="POINT"
        )
        if len(constraint_verts_loops_edit) == 0:
            continue

        # All edge-based constraints receive multi-loop format: [[verts1, is_circular1], [verts2, is_circular2], ...]
        # Ensure it's always in multi-loop format
        if isinstance(constraint_verts_loops_edit[0], list) and len(constraint_verts_loops_edit[0]) == 2:
            # Already multi-loop format
            constraint_verts_loops = constraint_verts_loops_edit
        else:
            # Convert single loop to multi-loop format
            constraint_verts_loops = [constraint_verts_loops_edit]

        # Handle subdivision
        if c.works_on_subdivision:
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
            # Shared center, independent orientations
            for loop_data in constraint_verts_loops:
                if len(loop_data[0]) > 2:
                    loop_offsets = utils.flatten_verts_calculate(
                        loop_data[0],
                        slide=False,
                        center=Vector(c.center),
                        normal=Vector(c.normal),
                        fix_center=c.fix_center,
                        fix_normal=c.fix_normal,
                    )
                    target_offsets.update(loop_offsets)
                

        # evaluate curve constraint
        elif c.constraint_type == "CURVE":
            # CURVE handles single loop - extract first
            constraint_verts_loop = constraint_verts_loops[0]
            if c.target_curve is not None and c.target_curve.type == "CURVE":
                target_offsets = to_curve_verts_calculate(
                    constraint_verts_loop,
                    curve_snapping=c.curve_snapping,
                    curve_distribution=c.curve_distribution,
                    kd=constraints_cache[i]["kd"],
                    source_curve=c.target_curve,
                )
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
                )
        
        # evaluate slide optimize constraint
        elif c.constraint_type == "SLIDE_OPTIMIZE":
            # SLIDE_OPTIMIZE handles multi-loop format
            target_offsets = slide_optimize_calculate(constraint_verts_loops)

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
            # selected_verts = [v for v in bm.verts if v.select]
            #
            # # we need to evaluate result subdivided mesh every iteration,
            # # so it's affected by previous iterations
            # depsgraph = bpy.context.evaluated_depsgraph_get()
            #
            # bm_eval = get_evaluated_bm(obj, depsgraph)
            #
            # if user_preferences.use_object_or_collection == "COLLECTION":
            #     target_objects = bpy.context.scene.inverse_subdivide_target_collection.objects
            # else:
            #     target_objects = [bpy.context.scene.inverse_subdivide_target_object]
            # level_subs_neighbours = 1 * 2 ** (get_subdivision_modifier_level(obj) - 1)
            # for v in bm.verts:
            #     offset = calculate_subdivide_offset(obj, target_objects, bm_eval, v, depsgraph, level_subs_neighbours)
            #     v.co += offset

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
    
    elements = utils.get_attribute_elements(
        context.object, bm, constraint, domain=domain, as_domain=domain
    )

    if len(elements) > 0:
        bpy.ops.mesh.select_all(action="DESELECT")
        for e in elements:
            e.select = True


def update_constraint_data(self, context):
    global constraints_cache
    constraints_cache = []


def update_plane_fix_flags(self, context):
    """Recalculate center/normal when fix flags are toggled"""
    if self.constraint_type != "PLANE":
        return
    
    # Only recalculate if we're enabling a fix flag
    if not (self.fix_center or self.fix_normal):
        return
    
    # Get the constraint's vertices
    mesh = context.active_object.data
    bm = bmesh.from_edit_mesh(mesh)
    
    # Get vertices from attribute
    if not self.attribute_name or self.attribute_name not in bm.verts.layers.float:
        return
    
    attribute_layer = bm.verts.layers.float[self.attribute_name]
    verts = [v for v in bm.verts if v[attribute_layer] == 1.0]
    
    if len(verts) > 2:
        center, normal = utils.estimate_best_fit_plane(verts, "best_fit")
        self.center = center
        self.normal = normal


def filter_curves(self, object):
    return object.type == "CURVE"


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
                "SLIDE_OPTIMIZE",
                "Slide Optimize",
                "Balance angles where quads meet by sliding vertices along edges",
                "DRIVER_ROTATIONAL_DIFFERENCE",
                4,
            ),
            # ("CIRCLE", "Circle", "Circle constraint", "MESH_CIRCLE", 5),
            # (
            #     "ANGLE",
            #     "Angle",
            #     "Limit angle for manufacturing purposes",
            #     "LINCURVE",
            #     6,
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
    curve_distribution: bpy.props.EnumProperty(
        name="Curve Distribution",
        default="EVEN",
        items=[
            ("EVEN", "Even", "Even distribution"),
            ("ORIGINAL", "Original", "Original distribution"),
        ],
        update=update_constraint_data,
    )


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
                layout.prop(ac, "fix_center")
                layout.prop(ac, "fix_normal")
                if ac.fix_center or ac.fix_normal:
                    if ac.fix_center:
                        layout.prop(ac, "center")
                    if ac.fix_normal:
                        layout.prop(ac, "normal")

            if ac.constraint_type == "CURVE":
                layout.prop(ac, "target_curve")
                layout.prop(ac, "curve_snapping")
                layout.prop(ac, "curve_distribution")
            
            if ac.constraint_type == "INVERSE_SUBDIVIDE":
                layout.label(text="Snap to")
                if bpy.data.objects.get("FROZEN_MESH_STATE") is not None:
                    layout.operator(
                        "mesh.freeze_shape", text="Unfreeze shape", depress=True, icon="FREEZE"
                    )
                else:
                    layout.operator(
                        "mesh.freeze_shape", text="Freeze Shape", depress=False, icon="FREEZE"
                    )
                    row = layout.row()
                    row.prop(ac, "use_object_or_collection", text="")
                    if ac.use_object_or_collection == "OBJECT":
                        row.prop(ac, "invsubdiv_target_object", text="")
                    elif ac.use_object_or_collection == "COLLECTION":
                        row.prop(ac, "invsubdiv_target_collection", text="")
                
                layout.prop(ac, "invsubdiv_normal_offset", text="Normal Offset")


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
    if constraint_type in ["PLANE", "CURVE", "SLIDE_OPTIMIZE"]:
        return "EDGE"
    elif constraint_type == "INVERSE_SUBDIVIDE":
        return "POINT"


class AddConstraintOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_add_constraint"
    bl_label = "Add Constraint"
    bl_description = (
        "\n\nSelect vertices and run this operator to add a constraint."
        "\nCurrently only planar constraints are supported."
        "\nConstraints get evaluated only during Inverse subdivision steps/modal operator."
        "\n These work together with inverse subdivision snapping, \n"
        "so you can optimize more parameters of the mesh."
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
                "SLIDE_OPTIMIZE",
                "Slide Optimize",
                "Balance angles where quads meet",
                "DRIVER_ROTATIONAL_DIFFERENCE",
                4,
            ),
            # ("CIRCLE", "Circle", "Circle constraint", "MESH_CIRCLE", 5),
            # (
            #     "ANGLE",
            #     "Angle",
            #     "Limit angle for manufacturing purposes",
            #     "LINCURVE",
            #     6,
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
        
        # Convert PLANE_FIXED to PLANE with fix flags
        actual_type = "PLANE" if self.constraint_type == "PLANE_FIXED" else self.constraint_type
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
            layout.prop(self, "curve_distribution")


class DeleteConstraintOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_delete_constraint"
    bl_label = "Delete Constraint"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        # Access the mesh data block
        mesh = context.active_object.data

        # Ensure there are constraints to delete
        if (
            mesh.ft_custom_constraints_index >= 0
            and mesh.ft_custom_constraints_index < len(mesh.ft_custom_constraints)
        ):
            # Remove the active constraint
            constraint = mesh.ft_custom_constraints[mesh.ft_custom_constraints_index]

            attribute = mesh.attributes.get(constraint.attribute_name)
            try:
                mesh.attributes.remove(attribute)
            except:
                print("Attribute for deleting not found")

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
        elif constraint.constraint_type == "SLIDE_OPTIMIZE":
            icon = "DRIVER_ROTATIONAL_DIFFERENCE"

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
