import bpy
from math import acos, radians, sqrt


def _axis_tuple(axis_name):
    axis_map = {
        "+X": (1.0, 0.0, 0.0),
        "-X": (-1.0, 0.0, 0.0),
        "+Y": (0.0, 1.0, 0.0),
        "-Y": (0.0, -1.0, 0.0),
        "+Z": (0.0, 0.0, 1.0),
        "-Z": (0.0, 0.0, -1.0),
    }
    return axis_map.get(axis_name, (0.0, 0.0, -1.0))


def _dot3(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub3(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add3(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _mul3(v, s):
    return (v[0] * s, v[1] * s, v[2] * s)


def _len2_3(v):
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2]


def _cross3(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize3(v):
    l2 = _len2_3(v)
    if l2 <= 1e-20:
        return None
    l = sqrt(l2)
    return (v[0] / l, v[1] / l, v[2] / l)


def _face_normal_from_indices(face_indices, coords):
    if len(face_indices) < 3:
        return None
    i0 = face_indices[0]
    p0 = coords[i0]
    for i in range(1, len(face_indices) - 1):
        p1 = coords[face_indices[i]]
        p2 = coords[face_indices[i + 1]]
        e1 = _sub3(p1, p0)
        e2 = _sub3(p2, p0)
        n = _cross3(e1, e2)
        nn = _normalize3(n)
        if nn is not None:
            return nn
    return None


def _project_perpendicular_to_axis(v, axis):
    proj = _dot3(v, axis)
    return (v[0] - axis[0] * proj, v[1] - axis[1] * proj, v[2] - axis[2] * proj)


def _optimize_print_safety(mesh, axis_name="-Z", max_angle_deg=50.0, iterations=10, strength=0.3):
    axis = _axis_tuple(axis_name)
    max_angle_deg = max(0.0, min(max_angle_deg, 89.9))
    threshold_angle = radians(90.0 - max_angle_deg)
    epsilon = 1e-20

    vert_count = len(mesh.vertices)
    if vert_count == 0 or len(mesh.polygons) == 0:
        return 0

    face_loops = [tuple(poly.vertices) for poly in mesh.polygons if len(poly.vertices) >= 3]
    if len(face_loops) == 0:
        return 0

    coords = [tuple(v.co) for v in mesh.vertices]
    moved_total = 0

    for _ in range(iterations):
        offsets = [(0.0, 0.0, 0.0)] * vert_count
        counts = [0] * vert_count

        for face_indices in face_loops:
            face_normal = _face_normal_from_indices(face_indices, coords)
            if face_normal is None:
                continue

            cosang = _dot3(face_normal, axis)
            if cosang > 1.0:
                cosang = 1.0
            elif cosang < -1.0:
                cosang = -1.0
            angle = acos(cosang)
            if angle >= threshold_angle:
                continue

            sorted_face = sorted(face_indices, key=lambda vi: _dot3(coords[vi], axis), reverse=True)
            moving_count = max(1, min(len(sorted_face) // 2, len(sorted_face) - 1))
            moving = set(sorted_face[:moving_count])
            stationary = set(sorted_face[moving_count:])
            if len(stationary) == 0:
                continue

            ring = list(face_indices)
            ring_len = len(ring)

            correction_scale = min(1.0, (threshold_angle - angle) / max(threshold_angle, 1e-12))

            for idx, vi in enumerate(ring):
                if vi not in moving:
                    continue

                slide = (0.0, 0.0, 0.0)
                direction_count = 0

                prev_vi = ring[idx - 1]
                next_vi = ring[(idx + 1) % ring_len]
                for nvi in (prev_vi, next_vi):
                    if nvi not in stationary:
                        continue
                    vec = _sub3(coords[nvi], coords[vi])
                    vec = _project_perpendicular_to_axis(vec, axis)
                    if _len2_3(vec) <= epsilon:
                        continue
                    slide = _add3(slide, vec)
                    direction_count += 1

                if direction_count == 0:
                    closest_dist2 = None
                    closest_vec = None
                    for svi in stationary:
                        vec = _sub3(coords[svi], coords[vi])
                        vec = _project_perpendicular_to_axis(vec, axis)
                        d2 = _len2_3(vec)
                        if d2 <= epsilon:
                            continue
                        if closest_dist2 is None or d2 < closest_dist2:
                            closest_dist2 = d2
                            closest_vec = vec
                    if closest_vec is None:
                        continue
                    slide = closest_vec
                    direction_count = 1

                slide = _mul3(slide, 1.0 / direction_count)
                offset = _mul3(slide, correction_scale)
                offsets[vi] = _add3(offsets[vi], offset)
                counts[vi] += 1

        moved_this_iter = 0
        for i in range(vert_count):
            if counts[i] == 0:
                continue
            avg_offset = _mul3(offsets[i], 1.0 / counts[i])
            delta = _mul3(avg_offset, strength)
            if _len2_3(delta) > epsilon:
                moved_this_iter += 1
                coords[i] = _add3(coords[i], delta)

        moved_total += moved_this_iter
        if moved_this_iter == 0:
            break

    flat_coords = [v for co in coords for v in co]
    mesh.vertices.foreach_set("co", flat_coords)
    mesh.update()
    return moved_total


class FinalTopologyPrintSafeOperator(bpy.types.Operator):
    bl_idname = "object.final_topology_print_safe"
    bl_label = "Make Print Safe"
    bl_description = (
        "Make a dense mesh more printable by reducing steep overhang-like slopes."
        "\n\nWhat it does:"
        "\n1) Detects faces that exceed the configured inclination threshold."
        "\n2) Moves only the 'lower' side of violating faces toward safer positions."
        "\n3) Locks movement perpendicular to the chosen axis."
        "\n\nWhy use it:"
        "\n- Fast object-mode solver for high resolution sculpt meshes."
        "\n- Uses panel settings (Axis, Max Inclination, Iterations, Strength)."
        "\n\nTip: Start with low Strength and increase Iterations gradually."
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return (
            context.mode == "OBJECT"
            and context.active_object is not None
            and context.active_object.type == "MESH"
        )

    def execute(self, context):
        obj = context.active_object
        mesh = obj.data
        user_preferences = bpy.context.preferences.addons[__package__].preferences
        moved = _optimize_print_safety(
            mesh,
            axis_name=user_preferences.print_safe_axis,
            max_angle_deg=user_preferences.print_safe_max_inclination,
            iterations=user_preferences.print_safe_iterations,
            strength=user_preferences.print_safe_strength,
        )
        self.report({"INFO"}, f"Print-safe optimization done. Moved vertices: {moved}")
        return {"FINISHED"}
