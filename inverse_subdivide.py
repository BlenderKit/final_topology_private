from math import radians

# from .draw import *
# from .utils import *
import bmesh
import bpy
from bpy.props import BoolProperty, IntProperty
from bpy.types import Operator
from mathutils import Vector

from . import draw, utils

has_extras = True
try:
    from . import extras
except:
    has_extras = False

running_operator = None


def get_closest_ray_hit(
    objects=[], source_position=Vector(), cast_direction=Vector(), depsgraph=None
):
    """
    Cast a ray and return the closest hit object and hit point in world space.
    Returns None if no hit.
    """
    hit_data = []

    for ob in objects:
        # Compute the transformation from world coordinates to object coordinates
        world_to_object = ob.matrix_world.inverted()

        # Transform the ray's origin and direction to object space
        local_origin = world_to_object @ source_position
        local_direction = world_to_object.to_3x3() @ cast_direction

        for direction in [local_direction, -local_direction]:
            # Cast the ray in object space
            hit, hit_position, hit_normal, hit_index = ob.ray_cast(
                local_origin, direction, depsgraph=depsgraph
            )

            if hit:
                # Transform the hit position back to world space
                world_hit_position = ob.matrix_world @ hit_position

                # Store hit position and distance
                hit_data.append(
                    (world_hit_position, (world_hit_position - source_position).length)
                )

    # Return None if no hit
    if len(hit_data) == 0:
        return None

    # Sort by distance
    hit_data.sort(key=lambda x: x[1])
    # Return the closest hit
    return hit_data[0][0]


def get_neighbors_subdivide_levels(input_verts, levels, only_last=True):
    """
    Get neighboring vertices in a quad mesh, traversing in a "straight" direction
    through junction vertices.

    Parameters:
        input_verts (list): List of root vertices to start from.
        levels (int): Number of levels to traverse.
        only_last (bool): Whether to include only the last level vertices.

    Returns:
        list: List of unique neighboring vertices.
    """

    # Initialize list to store neighboring vertices within levels
    neighbors_within_levels = input_verts[:]

    def find_neighbors(vertex, incoming_edge, level):
        """Recursive function to find neighbor vertices."""
        if level == 0:
            return

        # Traverse in all directions for the first level of neighbors
        if incoming_edge is None:
            for edge in vertex.link_edges:
                opposite_vertex = edge.other_vert(vertex)
                if not only_last or level == 1:
                    # add_line(vertex.co, opposite_vertex.co, GREEN)

                    neighbors_within_levels.append(opposite_vertex)
                find_neighbors(opposite_vertex, edge, level - 1)
            return

        # Identify neighboring faces of the incoming edge
        neighbor_faces = incoming_edge.link_faces
        incoming_face_edges = set(e for f in neighbor_faces for e in f.edges)

        # Find the edge that is opposite to the incoming edge
        for edge in vertex.link_edges:
            if edge not in incoming_face_edges:
                opposite_vertex = edge.other_vert(vertex)
                if not only_last or level == 1:
                    # add_line(vertex.co, opposite_vertex.co, GREEN)

                    neighbors_within_levels.append(opposite_vertex)
                find_neighbors(opposite_vertex, edge, level - 1)

    # Begin traversal from each input vertex
    for vert in input_verts:
        find_neighbors(vert, None, level=levels)

    # Convert to a list of unique vertices
    unique_neighbors_within_levels = list(set(neighbors_within_levels))

    return unique_neighbors_within_levels


def get_neighbors_within_levels(input_verts, levels):
    # List to store neighboring vertices within levels
    neighbors_within_levels = input_verts[:]

    def find_neighbors(vertex, level):
        """
        Recursive function to find neighbor vertices.
        """
        if level == 0:
            return
        for face in vertex.link_faces:
            for neighbor_vertex in face.verts:
                if (
                    neighbor_vertex != vertex
                    and neighbor_vertex not in neighbors_within_levels
                ):
                    neighbors_within_levels.append(neighbor_vertex)
                    find_neighbors(neighbor_vertex, level - 1)

    # Loop through selected vertices and find neighbors for specified levels
    for vert in input_verts:
        find_neighbors(vert, level=levels)

    # Convert neighbors to a unique list
    unique_neighbors_within_levels = list(set(neighbors_within_levels))
    # for v in unique_neighbors_within_levels:
    #     add_arrow(v.co, v.co+ v.normal*.02, GREEN)
    # Free the BMesh

    return unique_neighbors_within_levels


def set_modifiers_start(obj):
    modifiers_state_start = []
    for modifier in obj.modifiers:
        mod_settings = {}
        mod_settings["show_viewport"] = modifier.show_viewport
        mod_settings["show_in_editmode"] = modifier.show_in_editmode
        mod_settings["virtual"] = False
        if modifier.type not in ["ARMATURE", "MIRROR", "SUBSURF"]:
            modifier.show_viewport = False
            modifier.show_in_editmode = False

        if modifier.type == "SUBSURF":
            modifier.show_viewport = True
            modifier.show_in_editmode = True
            mod_settings["levels"] = modifier.levels
            # Only support levels 1 and 2
            modifier.levels = min(2, modifier.levels)
            modifier.levels = max(1, modifier.levels)

        modifiers_state_start.append(mod_settings)

    if len(obj.modifiers) == 0:
        bpy.ops.object.modifier_add(type="SUBSURF")
        modifier = obj.modifiers[0]
        modifier.show_viewport = True
        modifier.show_in_editmode = True
        modifier.levels = 2
        mod_settings = {}
        mod_settings["show_viewport"] = modifier.show_viewport
        mod_settings["show_in_editmode"] = modifier.show_in_editmode
        mod_settings["levels"] = modifier.levels
        mod_settings["virtual"] = True

        modifiers_state_start.append(mod_settings)
    return modifiers_state_start


def set_modifiers_end(obj, modifiers_state_start):
    for i, modifier in enumerate(obj.modifiers):
        mod_settings = modifiers_state_start[i]
        if modifier.type == "SUBSURF":
            modifier.levels = mod_settings["levels"]
        modifier.show_viewport = mod_settings["show_viewport"]
        modifier.show_in_editmode = mod_settings["show_in_editmode"]
        # careful if this wouldn't be the last one could cause problems with for loop
        if mod_settings["virtual"]:
            obj.modifiers.remove(modifier)


def get_subdivision_modifier_level(obj):
    """
    Get the subdivision modifier level of an object if present.

    Args:
    obj (bpy.types.Object): The Blender object to check for the subdivision modifier.

    Returns:
    int: The subdivision modifier level (number of subdivisions), or None if no subdivision modifier is present.
    """
    if obj is not None and obj.type == "MESH":
        for modifier in obj.modifiers:
            if modifier.type == "SUBSURF":
                if modifier.show_in_editmode and modifier.show_viewport:
                    return modifier.levels
    return None


def process_vertex_raycast(
    i, bm_eval, offset_verts_hit_positions, user_preferences, target_objects, obj
):
    """
    Process a single vertex in the mesh to calculate its offset based on raycasting.

    Parameters:
    - i: The index of the vertex in the BMesh
    - bm_eval: The evaluated BMesh
    - offset_verts_hit_positions: Dictionary to store hit positions and differences
    - user_preferences: User-defined settings
    - target_objects: List of target objects for raycasting
    - ob_matrix_world: Object's world transformation matrix
    """

    # Raycast logic
    res_v = bm_eval.verts[i]

    ob_matrix_world = obj.matrix_world
    # Transform vertex coordinates to world space
    world_source_position = ob_matrix_world @ res_v.co

    # Transform vertex normal to world space
    world_cast_direction = (
        ob_matrix_world.to_3x3().transposed().inverted() @ -res_v.normal
    )
    world_cast_direction.normalize()

    # Get the closest hit on the target mesh
    hit_position = get_closest_ray_hit(
        objects=target_objects,
        source_position=world_source_position,
        cast_direction=world_cast_direction,
    )

    # add_arrow(world_source_position, world_source_position + world_cast_direction * .01, RED)

    if hit_position is not None:
        # Transform hit_position to the active object's local space
        ob_matrix_world_inv = ob_matrix_world.inverted()
        local_hit_position = ob_matrix_world_inv @ hit_position

        # Calculate the difference vector
        difference = local_hit_position - res_v.co

        # Add draw data
        l = difference.length / user_preferences.gradient_sensitivity_distance
        if difference.length < user_preferences.max_distance:
            # let's not draw radical overshoots that won't be counted anyway.
            color = (min(1, l), max(0, 1 - l), 0.0, 0.1)
            draw.add_arrow(
                world_source_position,
                hit_position,
                color,
                scale=user_preferences.arrow_scale,
            )

            for f in res_v.link_faces:
                draw.add_face(f, obj, color)
    else:
        difference = Vector((0, 0, 0))

    # Store the hit position and difference
    offset_verts_hit_positions[i] = (hit_position, difference)


def calculate_offset(
    bm_eval,
    offset_verts_indices,
    offset_verts_hit_positions,
    v_index,
    user_preferences,
    target_objects,
    depsgraph,
):
    """Calculate offset of vertices. takes the whole groups of vertices that are taken into account (by now middle of connecting edges).
    it calculates the offset with weights, where longer edges get higher weight than shorter
    """
    user_preferences = bpy.context.preferences.addons["final_topology"].preferences

    total_difference = Vector((0, 0, 0))
    results_counted = 0
    total_weight = 0

    main_vert = bm_eval.verts[v_index]
    distances = []
    # iterate twice, we first need to get the max distance between verts
    total_distance = 0
    for range_i, i in enumerate(offset_verts_indices):
        res_v = bm_eval.verts[i]
        dist = (main_vert.co - res_v.co).length
        distances.append(dist)
        if i != v_index:
            total_distance += dist

    max_distance = max(distances)

    for range_i, i in enumerate(offset_verts_indices):
        res_v = bm_eval.verts[i]
        hit_position, difference = offset_verts_hit_positions[i]
        if hit_position is not None:
            if difference.length < user_preferences.max_distance:
                results_counted += 1
                if i == v_index:
                    total_weight += 1
                    total_difference += difference
                elif len(offset_verts_indices) > 1:
                    dist = distances[range_i]
                    if user_preferences.weight_algorithm == "FIRSTONLY":
                        others_weight = 0
                    elif user_preferences.weight_algorithm == "ALL1":
                        others_weight = 1
                    elif user_preferences.weight_algorithm == "FIRST":
                        others_weight = 1 / (len(offset_verts_indices) - 1)
                    elif user_preferences.weight_algorithm == "FIRSTDIST":
                        if total_distance == 0:
                            others_weight = 1
                        else:
                            others_weight = dist / total_distance

                    total_weight += others_weight
                    total_difference += difference * others_weight

    offset = Vector((0, 0, 0))
    if results_counted > 0 and total_weight > 0:
        offset = total_difference / total_weight
    return offset


def final_topology_optimization_step(self, context, iterations=1, neighbours=1):
    """Runs all optimization steps:
    - inverse subdivide
    - constraints
    (these should be the same after addon rewrite)
    """
    # bpy.context.view_layer.update()
    user_preferences = bpy.context.preferences.addons["final_topology"].preferences
    s_levels = get_subdivision_modifier_level(self.object)
    # Keep running, but do nothing
    if s_levels is None:
        self.report(
            {"WARNING"},
            "Only objects with subdivision modifier are supported by now.",
        )
        return False

    #  this needs proper iteration of real neighbours, should actually try to find the 4 center vertices around if more levels are there.
    self.level_subs_neighbours = 1 * 2 ** (s_levels - 1)
    # get inverse subdivision target objects
    target_objects = get_target_objects(self)
    if len(target_objects) == 0:
        if not self.warning_posted:
            self.warning_posted = True
            bpy.ops.wm.final_topo_popup_dialog(
                "INVOKE_DEFAULT",
                message="Inverse-subdivide target objects should be visible mesh objects.\n "
                "Please check your settup in the snap settings.",
                width=600,
            )
        return False

    me = self.object.data
    bm_edit = bmesh.from_edit_mesh(me)
    selected_verts = [v for v in bm_edit.verts if v.select]
    # neighbors are all vertices (selected and neighbors) of the edit mesh that can be tweaked.
    neighbors = get_neighbors_within_levels(selected_verts, neighbours)

    depsgraph = bpy.context.evaluated_depsgraph_get()
    bm_eval = utils.get_evaluated_bm(self.object, depsgraph)

    # we need to ray-cast every iteration.
    # let's get the neighbours for each vert separately on the subdivided mesh
    offset_verts_indices = {}
    for v in neighbors:
        offset_verts = [bm_eval.verts[v.index]]
        # get neighbours on eval mesh
        if user_preferences.weight_algorithm != "FIRSTONLY":
            offset_verts = get_neighbors_subdivide_levels(
                offset_verts, self.level_subs_neighbours
            )
        # store the indices of the offset verts(basically cage vert with it's neighbours) per vertex
        offset_verts_indices[v.index] = [v.index for v in offset_verts]

    # now we only want to calculate raycast for each vertex once, so we need to get unique indices
    # and then calculate the offset for each vertex
    unique_indices = set()
    for v in neighbors:
        unique_indices.update(offset_verts_indices[v.index])

    for a in range(0, iterations):
        draw.clear_draw_list()
        if a > 0:
            # we need to evaluate result subdivided mesh every iteration,
            # so need a fresh bm_eval, except for first iteration
            depsgraph = bpy.context.evaluated_depsgraph_get()
            bm_eval = utils.get_evaluated_bm(self.object, depsgraph)

        # apply mirror constraints
        if user_preferences.use_mirror:
            mirror_data = None
            # check if there's a mirror modifier
            for mod in self.object.modifiers:
                if mod.type == "MIRROR":
                    mirror_data = utils.get_mirror_data(self.object)
                    break
        if has_extras:
            bm_edit.verts.ensure_lookup_table()

            bm_edit = extras.evaluate_constraints(
                self.object, bmesh_edit=bm_edit, bmesh_eval=bm_eval
            )

        bm_edit = bmesh.from_edit_mesh(me)

        # we need to ray-cast every iteration.
        offset_verts_hit_positions = {}
        for i in unique_indices:
            # raycast logic
            process_vertex_raycast(
                i,
                bm_eval,
                offset_verts_hit_positions,
                user_preferences,
                target_objects,
                self.object,
            )

        for v in neighbors:
            if offset_verts_indices.get(v.index) is None:
                # TODO find out why sometimes the key isn't in the dict, otherwise this condition wouldn't be here.
                continue
            offset = calculate_offset(
                bm_eval,
                offset_verts_indices[v.index],
                offset_verts_hit_positions,
                v.index,
                user_preferences,
                target_objects,
                depsgraph,
            )

            # align offset with vert normal.
            if offset.length > 0:
                # weight the offset down, to prevent instabilities.
                offset *= user_preferences.offset_weight

                if v.normal.angle(offset) > radians(90):
                    # if the offset is in the opposite direction of the normal,
                    # we need to invert it to go along with the original offset
                    v_normal_offset = -v.normal * offset.length
                else:
                    v_normal_offset = v.normal * offset.length

                # check if vert is close to some of the mirror planes, and if so, snap the offset to the mirror plane
                if mirror_data is not None:
                    for mirror_center, mirror_normal, merge_distance in mirror_data:
                        if (v.co - mirror_center).dot(mirror_normal) < merge_distance:
                            # vertex is close to mirror plane
                            v.co += v_normal_offset
                            to_center = mirror_center - v.co
                            distance_to_plane = to_center.dot(mirror_normal)
                            v.co += distance_to_plane * mirror_normal
                        else:
                            # vertex is not close to mirror plane
                            v.co += v_normal_offset
                else:
                    v.co += v_normal_offset

        bmesh.update_edit_mesh(me)
    return True


def get_target_objects(self):
    # Define which objects to raycast against
    user_preferences = bpy.context.preferences.addons["final_topology"].preferences

    target_objects = []
    if user_preferences.use_object_or_collection == "COLLECTION":
        for ob in bpy.context.scene.inverse_subdivide_target_collection.objects:
            if ob.type == "MESH" and ob.visible_get():
                target_objects.append(ob)
    elif user_preferences.use_object_or_collection == "OBJECT":
        tob = bpy.context.scene.inverse_subdivide_target_object
        if tob is not None and tob.type == "MESH" and tob.hide_viewport is False:
            target_objects.append(tob)

    else:
        # get all visible objects for scene option
        target_objects = [
            obj
            for obj in bpy.context.scene.objects
            if (obj.visible_get() and obj.type == "MESH" and obj != self.object)
        ]
    return target_objects


class InverseSubdivideStep(Operator):
    bl_idname = "mesh.final_topology_optimization_step"
    bl_label = "Inverse Subdivide Snapping Step"
    bl_description = (
        "Run the subdivision compensation one step. "
        "\n\nIf you need stable behaviour and \n"
        "want only use the tool locally, run this operator whenever needed"
    )
    bl_options = {"REGISTER", "UNDO"}

    iterations: bpy.props.IntProperty(
        name="Iterations",
        default=20,
        min=1,
        max=500,
        description="Number of iterations for vertex position compensation. \n Quite slow when over 50",
    )

    neighbours: bpy.props.IntProperty(
        name="Neighbours",
        default=1,
        min=1,
        max=500,
        description="Number of neighbours to take into account for each vertex. \n Higher number will result in more accurate results, but in complex areas can screw up.",
    )

    warning_posted = BoolProperty(default=False)

    def execute(self, context):
        user_preferences = bpy.context.preferences.addons["final_topology"].preferences

        # check if there's subdivision modifier
        self.object = bpy.context.active_object
        modifiers_state_start = set_modifiers_start(self.object)

        done = final_topology_optimization_step(
            self, context, self.iterations, self.neighbours
        )
        if not done:
            return {"CANCELLED"}

        set_modifiers_end(self.object, modifiers_state_start)
        return {"FINISHED"}

    def invoke(self, context, event):
        return self.execute(context)


class InverseSubdivideModal(Operator):
    bl_idname = "mesh.inverse_subdivide_modal"
    bl_label = "Inverse Subdivide Snapping Modal"
    bl_description = (
        "Start compensating for inverse subdivision."
        "\n\nUse CTRL during transorms to initiate."
        "\nDo not combine with face projection if you snap larger parts of mesh"
    )
    bl_options = {"REGISTER", "UNDO"}

    warning_posted = BoolProperty(default=False)

    # TODOS:
    # - Work on vertices which have no hit, but their neigbours do have the hit (borders of meshes)
    # - Add weights based on length of edges. Which one should be more important, the longer ones or the shorter ones?
    # - works now after move, but not after selection?
    # respect mirror modifier

    def handle_event(self, context, event, user_preferences):
        global running_operator

        if (
            user_preferences.enable_operator == False
            or running_operator is None
            or bpy.context.mode != "EDIT_MESH"
        ):
            wm = context.window_manager
            try:
                wm.event_timer_remove(self._timer)
            except:
                pass
            bpy.types.SpaceView3D.draw_handler_remove(self._handle, "WINDOW")
            bpy.types.SpaceView3D.draw_handler_remove(self._2d_handle, "WINDOW")
            running_operator = None
            draw.clear_draw_list()
            user_preferences.enable_operator = False
            set_modifiers_end(self.object, self.modifiers_state_start)
            return {"CANCELLED"}

        return {"PASS_THROUGH"}

    def modal(self, context, event):
        user_preferences = bpy.context.preferences.addons["final_topology"].preferences

        # check if there's subdivision modifier

        is_timer = (
            event.type == "TIMER"
            and user_preferences.use_timer
            and user_preferences.always_on
        )
        if (event.type == "LEFTMOUSE" or is_timer) and context.mode == "EDIT_MESH":
            self.object = bpy.context.active_object

            draw.clear_draw_list()
            tool_settings = context.tool_settings

            # on click release and the options
            if (event.value == "RELEASE" or is_timer) and (
                event.ctrl
                or (tool_settings.use_snap and not event.ctrl)
                or user_preferences.always_on
            ):
                done = final_topology_optimization_step(
                    self,
                    context,
                    user_preferences.iterations,
                    user_preferences.neighbours,
                )
                if not done:
                    return {"RUNNING_MODAL"}
        return self.handle_event(context, event, user_preferences)

    def invoke(self, context, event):
        global running_operator

        user_preferences = bpy.context.preferences.addons["final_topology"].preferences

        # return if we are already running
        if running_operator is not None:
            user_preferences.enable_operator = False
            running_operator = None
            return {"CANCELLED"}

        draw.clear_draw_list()

        # Add the region OpenGL drawing callback
        # draw in view space with 'POST_VIEW' and 'PRE_VIEW'
        args = (self, context)
        self._handle = bpy.types.SpaceView3D.draw_handler_add(
            draw.draw_callback_px_3d, args, "WINDOW", "POST_VIEW"
        )
        self._2d_handle = bpy.types.SpaceView3D.draw_handler_add(
            draw.draw_callback_px_2d, args, "WINDOW", "POST_PIXEL"
        )

        # if user_preferences.always_on and user_preferences.use_timer:
        # We start timer always, but use it only when the setting is enabled
        wm = context.window_manager
        self._timer = wm.event_timer_add(
            0.3, window=context.window
        )  # 1 second interval

        context.window_manager.modal_handler_add(self)
        # set running operator to be aware of it already running
        running_operator = self
        # enable this if user did run the operator e.g. from search menu
        user_preferences.enable_operator = True
        self.warning_posted = False
        self.object = bpy.context.active_object
        self.modifiers_state_start = set_modifiers_start(self.object)

        return {"RUNNING_MODAL"}


def create_freeze_mesh_object():
    orig_ob = bpy.context.active_object
    orig_mode = bpy.context.mode
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.duplicate()
    freeze_mesh_object = bpy.context.active_object
    freeze_mesh_object.name = "FROZEN_MESH_STATE"
    freeze_mesh_object.name = (
        "FROZEN_MESH_STATE"  # double setting name removes the .001s
    )

    for m in freeze_mesh_object.modifiers:
        if m.type != "MIRROR":
            m.show_viewport = False
            m.show_render = False
    # freeze_mesh_object.modifiers.clear()
    # hide the object
    # freeze_mesh_object.hide_viewport = True
    freeze_mesh_object.hide_set(True)

    freeze_mesh_object.hide_render = True

    m = freeze_mesh_object.modifiers.new("Subdivision", "SUBSURF")
    m.levels = 5
    utils.activate_object(orig_ob)
    bpy.ops.object.mode_set(mode="EDIT")
    prefs = bpy.context.preferences.addons["final_topology"].preferences
    prefs.use_object_or_collection = "OBJECT"
    bpy.context.scene.inverse_subdivide_target_object = freeze_mesh_object
    return freeze_mesh_object


def delete_frozen_mesh():
    prefs = bpy.context.preferences.addons["final_topology"].preferences
    # bpy.ops.object.mode_set(mode='OBJECT')

    # bpy.ops.object.select_all(action='DESELECT')
    object = bpy.data.objects.get("FROZEN_MESH_STATE")
    if object is not None:
        bpy.data.objects.remove(object)
    prefs.use_object_or_collection = "SCENE"


class FreezeShape(bpy.types.Operator):
    bl_idname = "mesh.freeze_shape"
    bl_label = "Freeze Subdivision Shape"
    bl_description = (
        "Switch on the Modal operator or perform Inverse Subdivide steps."
        "\n\nCreates a copy of self and switches on snapping to it."
        "\nUse to reorganize your topology, but try to preserve shape."
    )
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        object = bpy.data.objects.get("FROZEN_MESH_STATE")
        if object is not None:
            delete_frozen_mesh()
        else:
            create_freeze_mesh_object()
        return {"FINISHED"}


def custom_unsubdivide(bm, mesh, levels):
    # Step 1: Identify starting vertices (corners or verts with edge count != 4)
    start_verts = [v for v in bm.verts if len(v.link_edges) == 2]

    # Step 2: Find all original vertices by traversing the mesh
    checked_edges = set()
    checked_verts = set()
    step = 2**levels  # Calculate the step size based on the levels of unsubdivide

    def traverse_from_start(start_vert, step):
        """Recursive function to traverse and find original vertices."""
        if start_vert in checked_verts:
            return
        checked_verts.add(start_vert)
        for incoming_edge in start_vert.link_edges:
            next_vert = incoming_edge.other_vert(start_vert)
            if incoming_edge in checked_edges:
                continue
            checked_edges.add(incoming_edge)
            incoming_edge.select = True
            if next_vert is None:
                continue
            dead_end = False
            for i in range(step - 1):
                if dead_end:
                    break
                neighbor_faces = incoming_edge.link_faces
                incoming_face_edges = set(e for f in neighbor_faces for e in f.edges)

                # Find the edge that is opposite to the incoming edge
                for edge1 in next_vert.link_edges:
                    if edge1 not in incoming_face_edges:
                        incoming_edge = edge1
                        checked_edges.add(edge1)
                        edge1.select = True
                        checked_verts.add(next_vert)
                        next_vert = edge1.other_vert(next_vert)
                        if next_vert is None or next_vert in checked_verts:
                            dead_end = True
                            break

            if next_vert is not None:
                traverse_from_start(next_vert, step)

    for v in start_verts:
        traverse_from_start(v, step)

    # Step 3: Dissolve edges that are not part of the original mesh
    bmesh.ops.dissolve_edges(
        bm,
        edges=[e for e in bm.edges if e not in checked_edges],
        use_verts=True,
        use_face_split=True,
    )
    bmesh.update_edit_mesh(mesh)


def dissolve_tris_to_ngons(bm):
    dissolve_ngons_verts = (
        []
    )  # list of verts that are in the middle of a potential n-gon.

    for v in bm.verts:
        all_tris = True
        for f in v.link_faces:
            if len(f.verts) != 3:
                all_tris = False
                break
        if all_tris:
            dissolve_ngons_verts.append(v)

    bmesh.ops.dissolve_verts(bm, verts=dissolve_ngons_verts)


class FinalUnsubdivide(bpy.types.Operator):
    bl_idname = "object.final_unsubdivide"
    bl_label = "Final Unsubdivide"
    bl_description = (
        "Unsubdivide mesh. "
        "\n\nWorks on meshes that had subdivision surface applied."
        "\n won't work on triangulated meshes."
        "\nFor high precision, increase number of iterations (up to 500) and wait! ;)"
    )
    bl_options = {"REGISTER", "UNDO"}

    unsubdivide_levels: bpy.props.IntProperty(
        name="Unsubdivide Levels",
        default=2,
        min=1,
        max=10,
        description="Number of levels to unsubdivide",
    )

    iterations: bpy.props.IntProperty(
        name="Iterations",
        default=5,
        min=1,
        max=500,
        description="Number of iterations for vertex compensation. \n Quite slow when over 50",
    )

    prefer_ngons: bpy.props.BoolProperty(
        name="Prefer Ngons",
        default=True,
        description="Favour Ngons over Tris wherever possible",
    )

    def execute(self, context):
        # Dictionary to store original vertex positions
        original_positions = {}

        # Step 1: Duplicate the active object
        original_object = bpy.context.active_object
        bpy.ops.object.duplicate()
        duplicate_object = bpy.context.active_object
        duplicate_object.name = original_object.name + "_unsubdivided"
        bpy.ops.object.mode_set(mode="EDIT")

        bm = bmesh.from_edit_mesh(duplicate_object.data)

        bmesh.ops.unsubdivide(
            bm, verts=bm.verts, iterations=self.unsubdivide_levels * 2
        )
        # custom_unsubdivide(bm, duplicate_object.data, self.unsubdivide_levels)

        # Step 3: Dissolve tris to ngons, if enabled. gets better results in most of cases where the original
        # authors are assumed to have used ngons too.
        if self.prefer_ngons:
            dissolve_tris_to_ngons(bm)

        # Step 4: Add Subdivision modifier
        bpy.ops.object.modifier_add(type="SUBSURF")
        subdivide_modifier = duplicate_object.modifiers["Subdivision"]
        subdivide_modifier.levels = self.unsubdivide_levels

        # Step 5: Switch to Edit mode
        bpy.ops.object.mode_set(mode="EDIT")

        # Step 6: Store original vertex positions
        bm = bmesh.from_edit_mesh(duplicate_object.data)
        for vert in bm.verts:
            original_positions[vert.index] = vert.co.copy()

        # Update & Free BMesh
        bmesh.update_edit_mesh(duplicate_object.data)

        # Step 7: Compensate for the subdivision surface
        for _ in range(self.iterations):
            # Get the evaluated mesh
            depsgraph = context.evaluated_depsgraph_get()
            eval_obj = duplicate_object.evaluated_get(depsgraph)
            eval_mesh = eval_obj.to_mesh()

            # Calculate the offset for each vertex
            offset_dict = {}
            for i, vert in enumerate(eval_mesh.vertices):
                if i in original_positions:
                    offset = original_positions[i] - vert.co
                    offset_dict[i] = offset

            # Apply the offset to the original mesh vertices
            bpy.ops.object.mode_set(mode="EDIT")
            bm = bmesh.from_edit_mesh(duplicate_object.data)
            for vert in bm.verts:
                if vert.index in offset_dict:
                    vert.co += offset_dict[vert.index]

            # Update & Free BMesh
            bmesh.update_edit_mesh(duplicate_object.data)
            bpy.ops.object.mode_set(mode="OBJECT")

        # subdivide_modifier.show_viewport=False
        return {"FINISHED"}
