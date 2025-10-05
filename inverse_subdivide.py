import bmesh
import bpy

from math import radians
from mathutils import Vector

from . import draw, utils

def get_target_objects(self):
    active_obj = bpy.context.active_object
    if not active_obj:
        return []

    target_objects = []
    if active_obj.final_topology.use_object_or_collection == "COLLECTION":
        target_collection = active_obj.final_topology.target_collection
        if target_collection:
            for ob in target_collection.objects:
                if ob.type == "MESH" and ob.visible_get():
                    target_objects.append(ob)
    elif active_obj.final_topology.use_object_or_collection == "OBJECT":
        tob = active_obj.final_topology.target_object
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

def get_neighbours_within_levels(input_verts, levels):
    # List to store neighboring vertices within levels
    neighbours_within_levels = input_verts[:]

    def find_neighbours(vertex, level):
        """
        Recursive function to find neighbor vertices.
        """
        if level == 0:
            return
        for face in vertex.link_faces:
            for neighbor_vertex in face.verts:
                if (
                    neighbor_vertex != vertex
                    and neighbor_vertex not in neighbours_within_levels
                ):
                    neighbours_within_levels.append(neighbor_vertex)
                    find_neighbours(neighbor_vertex, level - 1)

    # Loop through selected vertices and find neighbours for specified levels
    for vert in input_verts:
        find_neighbours(vert, level=levels)

    # Convert neighbours to a unique list
    unique_neighbours_within_levels = list(set(neighbours_within_levels))
    # for v in unique_neighbours_within_levels:
    #     add_arrow(v.co, v.co+ v.normal*.02, GREEN)
    # Free the BMesh

    return unique_neighbours_within_levels
def get_neighbours_subdivide_levels(input_verts, levels, only_last=True):
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
    neighbours_within_levels = input_verts[:]

    def find_neighbours(vertex, incoming_edge, level):
        """Recursive function to find neighbor vertices."""
        if level == 0:
            return

        # Traverse in all directions for the first level of neighbours
        if incoming_edge is None:
            for edge in vertex.link_edges:
                opposite_vertex = edge.other_vert(vertex)
                if not only_last or level == 1:
                    # add_line(vertex.co, opposite_vertex.co, GREEN)

                    neighbours_within_levels.append(opposite_vertex)
                find_neighbours(opposite_vertex, edge, level - 1)
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

                    neighbours_within_levels.append(opposite_vertex)
                find_neighbours(opposite_vertex, edge, level - 1)

    # Begin traversal from each input vertex
    for vert in input_verts:
        find_neighbours(vert, None, level=levels)

    # Convert to a list of unique vertices
    unique_neighbours_within_levels = list(set(neighbours_within_levels))

    return unique_neighbours_within_levels


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


def process_vertex_raycast(
    i, bm_eval, offset_verts_hit_positions, user_preferences, target_objects, obj, normal_offset = 0.0
):
    """
    Process a single vertex in the mesh to calculate its offset based on raycasting.

    Parameters:
    - i: The index of the vertex in the BMesh
    - bm_eval: The evaluated BMesh
    - offset_verts_hit_positions: Dictionary to store hit positions and offset_vectors
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

        # Calculate the offset_vector vector
        offset_vector = local_hit_position - res_v.co
        # add the normal offset to the offset_vector
        if normal_offset != 0.0:
            offset_vector += normal_offset * res_v.normal
        # need this to visualize correct offset
        global_offset_hit_position = (res_v.co + offset_vector) 
        global_offset_hit_position = ob_matrix_world @ global_offset_hit_position
        
        # Add draw data
        l = offset_vector.length / user_preferences.gradient_sensitivity_distance
        if offset_vector.length < user_preferences.max_distance:
            # let's not draw radical overshoots that won't be counted anyway.
            color = (min(1, l), max(0, 1 - l), 0.0, 0.1)
            draw.add_arrow(
                world_source_position,
                global_offset_hit_position,
                color,
                scale=user_preferences.arrow_scale,
            )

            for f in res_v.link_faces:
                draw.add_face(f, obj, color)
    else:
        offset_vector = Vector((0, 0, 0))

    # Store the hit position and offset_vector
    offset_verts_hit_positions[i] = (hit_position, offset_vector)


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
    user_preferences = bpy.context.preferences.addons[__package__].preferences

    total_offset_vector = Vector((0, 0, 0))
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
        hit_position, offset_vector = offset_verts_hit_positions[i]
        if hit_position is not None:
            if offset_vector.length < user_preferences.max_distance:
                results_counted += 1
                if i == v_index:
                    total_weight += 1
                    total_offset_vector += offset_vector
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
                    total_offset_vector += offset_vector * others_weight

    offset = Vector((0, 0, 0))
    if results_counted > 0 and total_weight > 0:
        offset = total_offset_vector / total_weight
    return offset


def prepare_inverse_subdivide(obj, bmesh_edit, bm_eval):
    """
    Prepare data needed for inverse subdivide evaluation.
    This is called once before the iteration loop.

    Returns a dictionary with preparation data.
    """
    user_preferences = bpy.context.preferences.addons[__package__].preferences
    neighbours = []
    prep_data = {}

    s_levels = get_subdivision_modifier_level(obj)
    if s_levels is None:
        return None

    level_subs_neighbours = 1 * 2 ** (s_levels - 1)
    prep_data["level_subs_neighbours"] = level_subs_neighbours

    target_objects = get_target_objects(obj)
    if len(target_objects) == 0:
        return False

    mirror_data = None
    if user_preferences.use_mirror:
        for mod in obj.modifiers:
            if mod.type == "MIRROR":
                mirror_data = utils.get_mirror_data(obj)
                break

    selected_verts = [v for v in bmesh_edit.verts if v.select]

    neighbours = get_neighbours_within_levels(selected_verts, level_subs_neighbours)

    offset_verts_indices = {}
    for v in neighbours:
        offset_verts = [bm_eval.verts[v.index]]
        if user_preferences.weight_algorithm != "FIRSTONLY":
            offset_verts = get_neighbours_subdivide_levels(
                offset_verts, level_subs_neighbours
            )
        offset_verts_indices[v.index] = [v.index for v in offset_verts]

    unique_indices = set()
    for v in neighbours:
        unique_indices.update(offset_verts_indices[v.index])

    mirror_data = None
    if user_preferences.use_mirror:
        for mod in obj.modifiers:
            if mod.type == "MIRROR":
                mirror_data = utils.get_mirror_data(obj)
                break

    prep_data["offset_verts_indices"] = offset_verts_indices
    prep_data["unique_indices"] = unique_indices
    prep_data["neighbours"] = neighbours
    prep_data["mirror_data"] = mirror_data
    prep_data["target_objects"] = target_objects
    
    return prep_data


def evaluate_inverse_subdivide(
    obj,
    bm_edit,
    bm_eval,
    prep_data,
    attribute_name=None,
):
    """
    Evaluate inverse subdivide for one iteration.
    Returns a dictionary of target_offsets {vertex_index: offset_vector}.

    If attribute_name is provided, only vertices with that attribute set to 1.0 are affected.
    """
    user_preferences = bpy.context.preferences.addons[__package__].preferences

    if prep_data is None:
        return {}

    offset_verts_indices = prep_data["offset_verts_indices"]
    neighbours = prep_data["neighbours"]
    unique_indices = prep_data["unique_indices"]
    mirror_data = prep_data["mirror_data"]
    target_objects = prep_data["target_objects"]

    if attribute_name is not None and len(neighbours) > 0:
        if attribute_name in bm_edit.verts.layers.float.keys():
            attribute_layer = bm_edit.verts.layers.float[attribute_name]
            filtered_neighbours = []
            affected_indices = set()
            for v in neighbours:
                bm_v = bm_edit.verts[v.index]
                if bm_v[attribute_layer] > 0.001:
                    filtered_neighbours.append(v)
                    affected_indices.add(v.index)
            neighbours = filtered_neighbours
            
            # Filter offset_verts_indices to only include vertices affected by the constraint
            # This prevents boundary vertices from being influenced by non-affected neighbors
            filtered_offset_verts_indices = {}
            for v_index in offset_verts_indices:
                if v_index in affected_indices:
                    filtered_indices = [idx for idx in offset_verts_indices[v_index] if idx in affected_indices]
                    if filtered_indices:
                        filtered_offset_verts_indices[v_index] = filtered_indices
            offset_verts_indices = filtered_offset_verts_indices
            
            # Update unique_indices to only include affected vertices
            unique_indices = set()
            for v_index in filtered_offset_verts_indices:
                unique_indices.update(filtered_offset_verts_indices[v_index])

    if len(neighbours) == 0:
        return {}

    depsgraph = bpy.context.evaluated_depsgraph_get()

    offset_verts_hit_positions = {}
    for i in unique_indices:
        process_vertex_raycast(
            i,
            bm_eval,
            offset_verts_hit_positions,
            user_preferences,
            target_objects,
            obj,
            obj.final_topology.normal_offset,
        )

    target_offsets = {}
    for v in neighbours:
        if offset_verts_indices.get(v.index) is None:
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

        if offset.length > 0:
            offset *= user_preferences.step_weight

            if v.normal.angle(offset) > radians(90):
                v_normal_offset = -v.normal * offset.length
            else:
                v_normal_offset = v.normal * offset.length

            if mirror_data is not None:
                for mirror_center, mirror_normal, merge_distance in mirror_data:
                    if (v.co - mirror_center).dot(mirror_normal) < merge_distance:
                        temp_co = v.co + v_normal_offset
                        to_center = mirror_center - temp_co
                        distance_to_plane = to_center.dot(mirror_normal)
                        v_normal_offset += distance_to_plane * mirror_normal
                        break

            target_offsets[v.index] = v_normal_offset

    return target_offsets

