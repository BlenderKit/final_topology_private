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


def is_pole(v):
    """A vertex where edge loops end: not the regular four edges inside the
    mesh, not the regular three on a border, not two on a wire."""
    valence = len(v.link_edges)
    if not v.link_faces:
        return valence != 2
    if v.is_boundary:
        return valence != 3
    return valence != 4


def sort_edges_into_loops(edges, stop_at_poles=True, stop_at_turns=True, crease_layer=None):
    """Sort marked edges into loops, following the mesh topology.

    At each vertex the walk continues through the OPPOSITE edge - the marked
    edge sharing no face with the one it came in through, which is Blender's
    own edge loop continuation on quad meshes. A fully selected grid thereby
    decomposes into its two families of parallel loops, instead of the
    arbitrary zigzag paths a plain adjacency walk produces. Where no opposite
    exists, a sole remaining connection is followed (a loop bending around a
    corner), several ambiguous ones end the loop (a perpendicular junction).
    Wire edges have no faces, there the straightest continuation wins.

    With stop_at_poles the walk ends at every pole (see is_pole), the way
    Blender's own loop select does - a whole assigned area then splits into
    the clean loops between its poles and corners instead of paths that
    wander through them, and a plane's border becomes four straight open
    loops rather than one ring that even spacing would round off.

    With stop_at_turns a loop only ever passes a regular four-edge crossing
    straight through, via the opposite edge; where the marked edges bend
    around the crossing instead, the loop ends there and the bend starts a
    new one - an L-shaped selection is two loops, not one with a corner.

    With crease_layer (the bmesh edge crease layer) a loop ends where a
    creased edge touches it: an edge at the vertex that is neither the edge
    the walk came in through nor the one it would leave by. The loop's own
    edges may be creased, that changes nothing - a crease running along the
    loop is fine, one crossing it is a break.

    Returns loops as [[vertex indices], is_circular], like
    get_connected_selections.
    """
    marked = list(edges)
    vert_edges = {}
    for edge in marked:
        for v in edge.verts:
            vert_edges.setdefault(v, []).append(edge)

    used = set()

    def continuation(v, incoming):
        if stop_at_poles and is_pole(v):
            return None
        candidates = [
            c for c in vert_edges.get(v, []) if c is not incoming and c not in used
        ]
        if not candidates:
            return None
        incoming_faces = set(incoming.link_faces)
        opposite = [
            c for c in candidates if not incoming_faces & set(c.link_faces)
        ]
        if opposite:
            pool = opposite
        elif len(candidates) == 1:
            if stop_at_turns and v.link_faces and len(v.link_edges) == 4:
                # a bend at a regular crossing: the loop doesn't turn corners
                return None
            pool = candidates
        else:
            return None
        if len(pool) == 1:
            chosen = pool[0]
        else:
            # several equally valid continuations - take the straightest
            direction = (v.co - incoming.other_vert(v).co).normalized()
            chosen = max(
                pool,
                key=lambda c: direction.dot((c.other_vert(v).co - v.co).normalized()),
            )
        if crease_layer is not None and any(
            e[crease_layer] > 0.0
            for e in v.link_edges
            if e is not incoming and e is not chosen
        ):
            # a crease touching the path breaks it here
            return None
        return chosen

    loops = []
    for start in marked:
        if start in used:
            continue
        used.add(start)
        chain = [start.verts[0], start.verts[1]]
        circular = False

        # forward from the chain end
        edge = start
        while True:
            next_edge = continuation(chain[-1], edge)
            if next_edge is None:
                break
            used.add(next_edge)
            next_vert = next_edge.other_vert(chain[-1])
            if next_vert is chain[0]:
                circular = True
                break
            chain.append(next_vert)
            edge = next_edge

        if not circular:
            # backward from the chain start
            edge = start
            while True:
                next_edge = continuation(chain[0], edge)
                if next_edge is None:
                    break
                used.add(next_edge)
                chain.insert(0, next_edge.other_vert(chain[0]))
                edge = next_edge

        loops.append([[v.index for v in chain], circular])

    return loops


def get_attribute_elements(
    object, bm, constraint, domain="POINT", as_domain="POINT", sorted=False
):
    # get all elements with attribute value 1.0, also add them to draw list
    user_preferences = bpy.context.preferences.addons[__package__].preferences

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
        # topology-aware sorting: follows edge loops through junctions, so a
        # fully assigned grid splits into its parallel loops instead of
        # zigzag paths
        # circles and arcs take their loops exactly as assigned - the pole
        # and turn stops aren't offered for them
        unsplit = getattr(constraint, "constraint_type", "") in ("CIRCLE", "ARC")
        crease_layer = None
        if not unsplit and getattr(constraint, "stop_at_crease", True):
            crease_layer = bm.edges.layers.float.get("crease_edge")
        loops = sort_edges_into_loops(
            draw_elements,
            stop_at_poles=not unsplit and getattr(constraint, "stop_at_poles", True),
            stop_at_turns=not unsplit and getattr(constraint, "stop_at_turns", True),
            crease_layer=crease_layer,
        )

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
            # return now edges as sorted into loops
            if len(loops) == 0:
                return_elements = []
            else:
                # loops carry vertex indices, so the lookup table has to be valid here
                bm.verts.ensure_lookup_table()
                # Convert vertex indices to actual vertex objects for all loops
                return_elements = []
                for loop in loops:
                    vert_list = []
                    for i in loop[0]:
                        vert_list.append(bm.verts[i])
                    return_elements.append([vert_list, loop[1]])  # [vertices, is_circular]

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
    print("creating circle")
    for i in range(num_verts):
        angle = i * angle_step
        x = cos(angle) * radius
        y = sin(angle) * radius
        vert_local = Vector((x, y, 0))
        vert_world = transform_matrix @ vert_local
        circle_verts.append(vert_world)
        if len(circle_verts) > 1:
            print("adding line")
            draw.add_line(circle_verts[-1], circle_verts[-2], (0, 1, 0, 1))

    return circle_verts



def move_verts_to_targets(bmesh_edit, target_offsets, weight=1.0):
    """Move vertices to target positions, using a dictionary of target positions"""
    for v_index in target_offsets.keys():
        v = bmesh_edit.verts[v_index]
        # v.co = v.colerp(target_offsets[v_index], weight)
        v.co += target_offsets[v_index] * weight


def get_verts_near_plane(bm, center, normal, distance):
    """Return vertices near a plane defined by a center point and a normal vector"""
    verts_near_plane = []
    for vert in bm.verts:
        if (vert.co - center).dot(normal) < distance:
            verts_near_plane.append(vert)
    return verts_near_plane



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


def flatten_verts_calculate(
    verts, slide=False, center=None, normal=None, fix_center=False, fix_normal=False
):
    """Calculate new positions for vertices to be flattened, Return a dictionary with the new positions
    
    Args:
        fix_center: If True, use provided center; otherwise calculate from verts
        fix_normal: If True, use provided normal; otherwise calculate from verts
    """
    target_offsets = {}
    
    # Calculate best-fit plane
    estimated_center, estimated_normal = estimate_best_fit_plane(verts, "best_fit")

    if not fix_normal:
        normal = estimated_normal
    if not fix_center:
        center = estimated_center

    # Define a function to get the intersection point of a line with the plane
    def line_plane_intersection(line_start, line_end, plane_point, plane_normal):
        line_dir = line_end - line_start
        d = (plane_point - line_start).dot(plane_normal) / line_dir.dot(plane_normal)
        return line_start + d * line_dir

    
    if slide:
        for vert in verts:
            # For each vertex, find the closest edge intersection with the plane
            closest_intersection = None
            min_distance = float("inf")
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
                intersection = line_plane_intersection(
                    vert.co, other_vert.co, center, normal
                )
                distance = (vert.co - intersection).length
                if distance < min_distance:
                    min_distance = distance
                    closest_intersection = intersection
                mean_intersection += intersection
                edges_included += 1

            mean_intersection /= edges_included
            if mean_intersection.length > 0:
                add_target_offset(
                    target_offsets, vert.index, mean_intersection - vert.co
                )
            # if closest_intersection:
            #     vert.co = closest_intersection
    else:
        # Project the vertices onto the plane
        for vert in verts:
            # print(vert)
            to_center = center - vert.co
            distance_to_plane = to_center.dot(normal)
            add_target_offset(target_offsets, vert.index, distance_to_plane * normal)
    return target_offsets


def flatten_verts(verts, method="best_fit", slide=False, center=None, normal=None):
    # Estimate the best fit plane
    fix_center = center is not None
    fix_normal = normal is not None
    target_offsets = flatten_verts_calculate(verts, slide, center, normal, fix_center, fix_normal)
    # Move the vertices to the new positions
    for vert in verts:
        vert.co = target_offsets[vert.index]

def get_mirror_data(object):
    mirror_modifiers = [mod for mod in object.modifiers if mod.type == "MIRROR"]
    mirror_data = []
    for m in mirror_modifiers:
        # clipping and merging both mean the seam belongs on the plane
        if m.use_clip or m.use_mirror_merge:
            if m.mirror_object is not None:
                # get the plane from the modifier
                plane_co = m.mirror_object.matrix_world.translation
                plane_no = m.mirror_object.matrix_world.to_3x3() @ m.use_axis
                # convert to local space
                plane_co = object.matrix_world.inverted() @ plane_co
                plane_no = object.matrix_world.inverted().to_3x3() @ plane_no
            else:
                plane_co = Vector((0, 0, 0))

            # get the mirrored planes from the modifier
            mirror_planes = []
            if m.use_axis[0]:
                mirror_data.append((plane_co, Vector((1, 0, 0)), m.merge_threshold))
            if m.use_axis[1]:
                mirror_data.append((plane_co, Vector((0, 1, 0)), m.merge_threshold))
            if m.use_axis[2]:
                mirror_data.append((plane_co, Vector((0, 0, 1)), m.merge_threshold))
    return mirror_data

def collect_mirror_seam_verts(bm, mirror_data):
    """Vertex indices belonging to each mirror seam, one set per plane.

    Collected before the constraints move anything: seam membership must come
    from where a vertex started, a single strong constraint step can carry it
    far beyond the merge threshold within one iteration.

    Besides vertices within the merge threshold, boundary vertices close to the
    plane count as seam too - a seam vertex that has already drifted past the
    threshold (its merging is broken, which is what we're fixing) gets captured
    through the mesh boundary it sits on, and healed back onto the plane.
    """
    seam_sets = []
    for plane_co, plane_no, threshold in mirror_data:
        limit = max(threshold, 1e-6)
        seam = set()
        for v in bm.verts:
            distance = abs((v.co - plane_co).dot(plane_no))
            if distance <= limit:
                seam.add(v.index)
            elif v.is_boundary and v.link_edges:
                # generous capture for drifted seam verts, but never further
                # than a fraction of the local edge length, so open borders
                # elsewhere in the mesh are left alone
                edge_scale = min(e.calc_length() for e in v.link_edges)
                if distance <= max(limit, edge_scale * 0.3):
                    seam.add(v.index)
        seam_sets.append(seam)
    return seam_sets


def snap_mirror_seam_verts(bm, mirror_data, seam_sets):
    """Put the collected seam vertices back onto their mirror planes.

    Run after constraints moved vertices, so they can't pull the mirror seam
    apart and break the merging - they may still slide seam vertices along the
    plane, and a pull toward a target off the plane turns into a slide toward
    the target's projection onto it.
    """
    for (plane_co, plane_no, threshold), indices in zip(mirror_data, seam_sets):
        for index in indices:
            v = bm.verts[index]
            v.co -= plane_no * (v.co - plane_co).dot(plane_no)


#TODO: Test mirror and if it works as is, remove this one.
# it seems to not work that well, since it's not directly part of the inverse subdivide algo.
def evaluate_mirror_constraints(object, bmesh_edit=None, bmesh_eval=None):
    # Mirror modifier support
    # This should work now same as fixed plane constraint, except that it gets the planes from the modifier.
    mirror_modifiers = [mod for mod in object.modifiers if mod.type == "MIRROR"]
    for m in mirror_modifiers:
        if m.use_clip:
            if m.mirror_object is not None:
                # get the plane from the modifier
                plane_co = m.mirror_object.matrix_world.translation
                plane_no = m.mirror_object.matrix_world.to_3x3() @ m.use_axis
                # convert to local space
                plane_co = object.matrix_world.inverted() @ plane_co
                plane_no = object.matrix_world.inverted().to_3x3() @ plane_no
            else:
                plane_co = Vector((0, 0, 0))

            # get the mirrored planes from the modifier
            mirror_planes = []
            if m.use_axis[0]:
                mirror_planes.append(Vector((1, 0, 0)))
            if m.use_axis[1]:
                mirror_planes.append(Vector((0, 1, 0)))
            if m.use_axis[2]:
                mirror_planes.append(Vector((0, 0, 1)))
            for mirror_plane in mirror_planes:
                verts = get_verts_near_plane(bmesh_edit, plane_co, mirror_plane,m.merge_threshold)
                target_offsets = flatten_verts_calculate(
                    verts,
                    slide=False,
                    center=plane_co,
                    normal=mirror_plane,
                    fix_center=True,
                    fix_normal=True,
                )
                move_verts_to_targets(bmesh_edit, target_offsets, weight=0.5)

def add_target_offset(target_offsets, index, co):
    """Add a new target position for a vertex to the dictionary"""
    # tp = target_offsets.get(index, [])
    # tp.append(co)
    target_offsets[index] = co