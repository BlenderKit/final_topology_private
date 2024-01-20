import bmesh
import bpy
from . import draw

from mathutils import Vector, Matrix
from math import cos, sin, pi


def get_evaluated_bm(obj, depsgraph):
    if obj.type == "MESH":
        mesh = obj.evaluated_get(depsgraph).data
    elif obj.type == "CURVE":
        mesh = bpy.data.meshes.new_from_object(obj, depsgraph=depsgraph)

    bm_eval = bmesh.new()
    bm_eval.from_mesh(mesh)
    bm_eval.verts.ensure_lookup_table()

    return bm_eval


def activate_object(ob):
    bpy.ops.object.select_all(action="DESELECT")
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob


# return the edgekey ([v1.index, v2.index]) of a bmesh edge
def edgekey(edge):
    return tuple(sorted([edge.verts[0].index, edge.verts[1].index]))


# calculate input loops


# input: list of edge-keys, output: dictionary with vertex-vertex connections
def dict_vert_verts(edge_keys):
    # create connection data
    vert_verts = {}
    for ek in edge_keys:
        for i in range(2):
            if ek[i] in vert_verts:
                vert_verts[ek[i]].append(ek[1 - i])
            else:
                vert_verts[ek[i]] = [ek[1 - i]]

    return vert_verts


# sorts all edge-keys into a list of loops
def get_connected_selections(edge_keys):
    # create connection data
    vert_verts = dict_vert_verts(edge_keys)

    # find loops consisting of connected selected edges
    loops = []
    while len(vert_verts) > 0:
        loop = [iter(vert_verts.keys()).__next__()]
        growing = True
        flipped = False

        # extend loop
        while growing:
            # no more connection data for current vertex
            if loop[-1] not in vert_verts:
                if not flipped:
                    loop.reverse()
                    flipped = True
                else:
                    growing = False
            else:
                extended = False
                for i, next_vert in enumerate(vert_verts[loop[-1]]):
                    if next_vert not in loop:
                        vert_verts[loop[-1]].pop(i)
                        if len(vert_verts[loop[-1]]) == 0:
                            del vert_verts[loop[-1]]
                        # remove connection both ways
                        if next_vert in vert_verts:
                            if len(vert_verts[next_vert]) == 1:
                                del vert_verts[next_vert]
                            else:
                                vert_verts[next_vert].remove(loop[-1])
                        loop.append(next_vert)
                        extended = True
                        break
                if not extended:
                    # found one end of the loop, continue with next
                    if not flipped:
                        loop.reverse()
                        flipped = True
                    # found both ends of the loop, stop growing
                    else:
                        growing = False

        # check if loop is circular
        if loop[0] in vert_verts:
            if loop[-1] in vert_verts[loop[0]]:
                # is circular
                if len(vert_verts[loop[0]]) == 1:
                    del vert_verts[loop[0]]
                else:
                    vert_verts[loop[0]].remove(loop[-1])
                if len(vert_verts[loop[-1]]) == 1:
                    del vert_verts[loop[-1]]
                else:
                    vert_verts[loop[-1]].remove(loop[0])
                loop = [loop, True]
            else:
                # not circular
                loop = [loop, False]
        else:
            # not circular
            loop = [loop, False]

        loops.append(loop)

    return loops


def get_attribute_elements(
    object, bm, constraint, domain="POINT", as_domain="POINT", sorted=False
):
    # get all elements with attribute value 1.0, also add them to draw list
    user_preferences = bpy.context.preferences.addons["final_topology"].preferences

    # Return elements might not be the same as draw elements if as_domain is different than domain.
    return_elements = []
    draw_elements = []

    if domain == "POINT":
        attribute_layer = bm.verts.layers.float[constraint.attribute_name]
        for vert in bm.verts:
            val = vert[attribute_layer]
            if val == 1.0:
                draw_elements.append(vert)
    elif domain == "EDGE":
        attribute_layer = bm.edges.layers.float[constraint.attribute_name]
        for edge in bm.edges:
            val = edge[attribute_layer]
            if val == 1.0:
                draw_elements.append(edge)
        edge_keys = [edgekey(edge) for edge in draw_elements]
        # use looptools to sort the edges into loops
        loops = get_connected_selections(edge_keys)

    elif domain == "FACE":
        attribute_layer = bm.faces.layers.float[constraint.attribute_name]
        for face in bm.faces:
            val = face[attribute_layer]
            if val == 1.0:
                draw_elements.append(face)

    if domain == as_domain:
        return_elements = draw_elements

    # Convert to as_domain
    # not all are implemented now, let's add them as needed
    if as_domain != domain:
        if as_domain == "POINT" and domain == "EDGE":
            # return now edges as sorted
            return_elements = []
            # By now I see no reason why to use multiple loops withing one constraint,
            # but it might be useful in the future
            if len(loops) == 0:
                return return_elements

            for i in loops[0][0]:
                return_elements.append(bm.verts[i])
            # add back info if loop is circular
            return_elements = [return_elements, loops[0][1]]

        elif as_domain == "POINT" and domain == "FACE":
            unique_vertices = set()
            for face in draw_elements:
                for vert in face.verts:
                    unique_vertices.add(vert)
            return_elements = [list(unique_vertices), False]

        elif as_domain == "EDGE" and domain == "POINT":
            unique_edges = set()
            for vert in draw_elements:
                for edge in vert.link_edges:
                    if edge.other_vert(vert) in draw_elements:
                        unique_edges.add(edge)
            return_elements = [list(unique_edges), False]

    # Return if drawing is disabled
    if not user_preferences.enable_draw_constraints:
        return return_elements

    # Draw the elements
    alpha = 0.1
    color = (constraint.color[0], constraint.color[1], constraint.color[2], alpha)
    if (
        object.data.ft_custom_constraints[object.data.ft_custom_constraints_index]
        == constraint
    ):
        alpha = 0.4
        color = (
            max(constraint.color[0] * 2, 0.6),
            max(constraint.color[1] * 2, 0.6),
            max(constraint.color[2] * 2, 0.6),
            alpha,
        )

    ob_matrix_world = object.matrix_world
    if domain == "POINT":
        for e in bm.edges:
            if e.verts[0] in draw_elements and e.verts[1] in draw_elements:
                world_vert_position = ob_matrix_world @ e.verts[0].co
                world_vert_position2 = ob_matrix_world @ e.verts[1].co
                draw.add_line(world_vert_position, world_vert_position2, color)
    elif domain == "EDGE":
        for e in draw_elements:
            world_vert_position = ob_matrix_world @ e.verts[0].co
            world_vert_position2 = ob_matrix_world @ e.verts[1].co
            draw.add_line(world_vert_position, world_vert_position2, color)

    return return_elements


def create_circle(center, normal, radius, num_verts):
    """
    Create a circle in 3D space with given center, normal, radius, and number of vertices.
    Returns a list of vertices as Blender mathutils.Vector objects in world coordinates.
    """
    # Create a rotation matrix aligning the Z-axis with the given normal
    z_axis = Vector((0, 0, 1))
    align_matrix = z_axis.rotation_difference(normal).to_matrix().to_4x4()

    # Create a translation matrix for the center
    translate_matrix = Matrix.Translation(center)

    # Combine the matrices to transform the circle points to the correct position and orientation
    transform_matrix = translate_matrix @ align_matrix

    # Generate circle vertices
    angle_step = 2 * pi / num_verts
    circle_verts = []
    for i in range(num_verts):
        angle = i * angle_step
        x = cos(angle) * radius
        y = sin(angle) * radius
        vert_local = Vector((x, y, 0))
        vert_world = transform_matrix @ vert_local
        circle_verts.append(vert_world)

    return circle_verts
