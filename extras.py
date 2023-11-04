import bpy
from mathutils import Vector
import bmesh

# project into XY plane,
up = Vector((0, 0, 1))
from bpy.types import Operator
from bpy.props import IntProperty, FloatProperty
from math import pi, atan2

def estimate_best_fit_plane(verts, method="best_fit"):
    """
    Estimate the best fit plane for a given set of vertices.

    Parameters:
    - verts: A list of bmesh vertices.
    - method: A string that determines the method to compute the plane's orientation.
              "best_fit" (default) computes the best fit plane.
              "mean_normal" computes the plane's orientation based on the mean normal of the vertices.

    Returns:
    - A tuple containing the center of the plane and the plane's normal.
    """

    # Calculate the center of the vertices
    center = Vector((0, 0, 0))
    for vert in verts:
        center += vert.co
    center /= len(verts)

    if method == "best_fit":
        # Calculate the covariance matrix
        cov_matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for vert in verts:
            p = vert.co - center
            for i in range(3):
                for j in range(3):
                    cov_matrix[i][j] += p[i] * p[j]

        # Compute the normal of the plane using the eigenvector corresponding to the smallest eigenvalue
        from numpy import linalg
        _, eigenvectors = linalg.eigh(cov_matrix)
        normal = Vector(eigenvectors[:, 0])

    elif method == "mean_normal":
        # Calculate the mean normal of the vertices
        # normal = Vector((0, 0, 0))
        # for vert in verts:
        #     normal += vert.normal
        # normal.normalize()

        cov_matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        for vert in verts:
            # p = vert.co - center
            # for i in range(3):
            #     for j in range(3):
            #         cov_matrix[i][j] += p[i] * p[j]
            p = vert.co + vert.normal - center
            for i in range(3):
                for j in range(3):
                    cov_matrix[i][j] += p[i] * p[j]

        # Compute the normal of the plane using the eigenvector corresponding to the smallest eigenvalue
        from numpy import linalg
        _, eigenvectors = linalg.eigh(cov_matrix)
        normal = Vector(eigenvectors[:, 0])

    return center, normal


class NormalLoopAlign(Operator):
    bl_idname = "mesh.flatten_loop_normal"
    bl_label = "Loop to normal-plane"

    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == 'MESH'

    def execute(self, context):
        obj = context.active_object
        # this simple solution actually works pretty nice, but not for closed loops :)
        bpy.ops.transform.resize(value=(0, 1, 1), orient_type='NORMAL',
                                 orient_matrix_type='NORMAL', mirror=True,
                                 use_proportional_edit=False)
        return {'FINISHED'}


class FlattenSelectionOperator(Operator):
    bl_idname = "mesh.flatten_selection"
    bl_label = "Flatten Selection"
    bl_options = {'REGISTER', 'UNDO'}

    method: bpy.props.EnumProperty(
        items=[
            ("best_fit", "Least Squares", "Use the least squares method to estimate the plane"),
            ("mean_normal", "Mean Normal", "Use the mean normal of the vertices to estimate the plane")
        ],
        name="Method",
        default="best_fit"
    )

    slide: bpy.props.BoolProperty(
        name="Slide Along Edges",
        description="Slide vertices along edges towards the target plane instead of direct projection",
        default=True
    )

    @classmethod
    def poll(cls, context):
        return context.active_object is not None and context.active_object.type == 'MESH'

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

        # Estimate the best fit plane
        center, normal = estimate_best_fit_plane(selected_verts, self.method)

        # Define a function to get the intersection point of a line with the plane
        def line_plane_intersection(line_start, line_end, plane_point, plane_normal):
            line_dir = line_end - line_start
            d = (plane_point - line_start).dot(plane_normal) / line_dir.dot(plane_normal)
            return line_start + d * line_dir

        if self.slide:
            for vert in selected_verts:
                # For each vertex, find the closest edge intersection with the plane
                closest_intersection = None
                min_distance = float('inf')
                mean_intersection = Vector((0, 0, 0))
                end_vertex = False
                edges_selected = 0
                for edge in vert.link_edges:
                    if edge.select:
                        edges_selected += 1
                if edges_selected == 1:
                    end_vertex = True

                edges_included = 0

                for edge in vert.link_edges:
                    if edge.select:
                        continue

                    # find out if edge has shared face with a selected edge
                    if end_vertex:
                        shared_face = False
                        for face in edge.link_faces:
                            for edge2 in face.edges:
                                if edge2.select:
                                    shared_face = True
                        if shared_face is False:
                            continue

                    other_vert = edge.other_vert(vert)
                    intersection = line_plane_intersection(vert.co, other_vert.co, center, normal)
                    distance = (vert.co - intersection).length
                    if distance < min_distance:
                        min_distance = distance
                        closest_intersection = intersection
                    mean_intersection += intersection
                    edges_included += 1

                mean_intersection /= edges_included
                if mean_intersection.length > 0:
                    vert.co = mean_intersection
                # if closest_intersection:
                #     vert.co = closest_intersection
        else:
            # Project the vertices onto the plane
            for vert in selected_verts:
                to_center = center - vert.co
                distance_to_plane = to_center.dot(normal)
                vert.co += distance_to_plane * normal

        bmesh.update_edit_mesh(obj.data)
        return {'FINISHED'}


def edge_angle(e1, e2, face_normal):
    b = set(e1.verts).intersection(e2.verts).pop()
    a = e1.other_vert(b).co - b.co
    c = e2.other_vert(b).co - b.co
    a.negate()
    axis = a.cross(c).normalized()
    if axis.length < 1e-5:
        return pi  # inline vert

    if axis.dot(face_normal) < 0:
        axis.negate()
    M = axis.rotation_difference(up).to_matrix().to_4x4()

    a = (M @ a).xy.normalized()
    c = (M @ c).xy.normalized()

    return pi - atan2(a.cross(c), a.dot(c))


def get_length(data):
    return data["slide_offset"].length


class SlideOptimizeOperator(bpy.types.Operator):
    """Slide all verts along the selected loop so that things look better"""
    bl_idname = "mesh.edge_slide_optimizer"
    bl_label = "Edge Slide Optimize"
    bl_options = {'REGISTER', 'UNDO'}

    first_mouse_x: IntProperty()
    first_value: FloatProperty()
    strength: IntProperty(name='intensity')
    width: FloatProperty(
        name="Width",
        description="Box Width",
        min=0.01, max=100.0,
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
                a = edge_angle(slide_candidates_edges[0], slide_candidates_edges[1], tot_normal)

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
                    angle_difference = abs(slide_candidates[0][1] - slide_candidates[1][1])

                    slide_edge_vector = (slide_edge.other_vert(v1).co - v1.co)
                    orig_length = slide_edge_vector.length
                    slide_direction = slide_edge_vector  # .normalized() * avg_length
                    counter_slide_edge_vector = other_edge.other_vert(v1).co - v1.co
                    contraslide_direction = counter_slide_edge_vector  # .normalized() * avg_length

                    # not used by now
                    slide_offset = (slide_direction) * .01 * multiplier * (angle_difference)
                    #                    if slide_offset.length>orig_length:
                    #                        slide_offset.length = orig_length
                    # keep same length
                    counter_slide_offset = (contraslide_direction * slide_offset.length)
                    slide_offset = slide_offset - counter_slide_offset

                    slide_position = slide_offset + v1.co
                all_slide_verts.append({"position": slide_position,
                                        "slide_offset": slide_offset,
                                        "original_position": v1.co.copy(),
                                        "angle_difference": angle_difference,
                                        "index": v1.index,
                                        "transform_multiplier": avg_length,
                                        "slide_edge_index": slide_edge.index,
                                        "counter_slide_edge_index": other_edge.index}, )

            all_slide_verts.sort(key=get_length)
            all_slide_verts.reverse()

            for v1data in all_slide_verts:
                # if v1data['slide_offset'].length>0.0001:
                bm.verts[v1data['index']].co = v1data['position']
            bmesh.update_edit_mesh(me)
        return all_slide_verts

    def slide_vertices(self, iterations=10):
        #        ob = bpy.context.active_object
        #        me= ob.data
        #        bm = bmesh.from_edit_mesh(me)
        self.restore_init_positions()
        print(iterations)
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
        if event.type == 'MOUSEMOVE':
            delta = event.mouse_x - self.first_mouse_x

            self.slide_vertices(iterations=int(delta * .2))
            # self.slide_vertices(iterations=200)  # int(delta *.2))
            return {'RUNNING_MODAL'}
        elif event.type == 'LEFTMOUSE' and event.value == 'RELEASE':
            return {'FINISHED'}

        elif event.type in {'RIGHTMOUSE', 'ESC'}:
            self.restore_init_positions()
            #            context.object.location.x = self.first_value
            return {'CANCELLED'}
        return {'RUNNING_MODAL'}

    def invoke(self, context, event):
        if context.object:
            self.first_mouse_x = event.mouse_x
            self.first_value = context.object.location.x
            self.store_init_positions()
            self.slide_verts = self.opti_slide_get_slide_verts()

            context.window_manager.modal_handler_add(self)
            return {'RUNNING_MODAL'}
        else:
            self.report({'WARNING'}, "No active object, could not finish")
            return {'CANCELLED'}




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
#             if user_preferences.use_object_or_collection == "COLLECTION":
#                 target_objects = bpy.context.scene.inverse_subdivide_target_collection.objects
#             else:
#                 target_objects = [bpy.context.scene.inverse_subdivide_target_object]
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
            candidate_edges = [e for e in face.edges if all(len(f.verts) == 3 for f in e.link_faces)]
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
                    other_candidate_edges = [e for e in linked_face.edges if
                                             all(len(f.verts) == 3 for f in e.link_faces)]
                else:
                    other_candidate_edges = linked_face.edges

                if not other_candidate_edges:
                    continue

                other_longest_edge = max(other_candidate_edges, key=lambda e: e.calc_length())

                if longest_edge == other_longest_edge:
                    longest_shared_edges.append(longest_edge)

    # Remove duplicates
    longest_shared_edges = list(set(longest_shared_edges))

    return longest_shared_edges


class FunTopologyDecimateOperator(bpy.types.Operator):
    bl_idname = "mesh.fun_topology_decimate"
    bl_label = "Fun Topology Decimate Operator"
    bl_options = {'REGISTER', 'UNDO'}

    max_iterations: bpy.props.IntProperty(name="Max Iterations", default=1)
    max_distance: bpy.props.FloatProperty(name="Max Distance", default=0.1)
    min_distance: bpy.props.FloatProperty(name="Min Distance", default=0.01)

    def modal(self, context, event):
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            self.iteration += 1
            if self.iteration > self.max_iterations:
                self.cancel(context)
                return {'FINISHED'}

            global draw_lines
            global draw_faces
            user_preferences = bpy.context.preferences.addons['final_topology'].preferences

            draw_lines.clear()
            draw_faces.clear()
            draw_faces_list.clear()
            tool_settings = context.tool_settings
            bpy.context.view_layer.update()

            obj = bpy.context.edit_object
            me = obj.data
            bm = bmesh.from_edit_mesh(me)

            dis_edges = find_longest_shared_edges(bm, only_triangles=True)
            bmesh.ops.dissolve_edges(bm, edges=dis_edges,
                                     use_verts=True, use_face_split=True)
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

            return {'PASS_THROUGH'}

        return {'PASS_THROUGH'}

    def execute(self, context):
        self.iteration = 0
        self.timer = context.window_manager.event_timer_add(0.1, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        context.window_manager.event_timer_remove(self.timer)
