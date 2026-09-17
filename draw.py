import bpy
import bpy_extras
import gpu
import mathutils
from gpu_extras.batch import batch_for_shader
from gpu_extras.presets import draw_circle_2d
from mathutils import Vector

draw_lines = {}
draw_faces = {}
draw_points = {}
draw_faces_list = []
# triangles with one color per corner, for smooth deviation displays
draw_colored_tris_pos = []
draw_colored_tris_col = []
# lines with one color per end, for smooth gradients along loops
draw_colored_lines_pos = []
draw_colored_lines_col = []
# little red squares marking pinned vertices, like UV editor pins
draw_pins = []

RED = (1.0, 0.0, 0.0, 1.0)
GREEN = (0.0, 1.0, 0.0, 1.0)
BLUE = (0.0, 0.0, 1.0, 1.0)
ORANGE = (1.0, 0.5, 0.0, 1.0)
YELLOW = (1.0, 1.0, 0.0, 1.0)
CYAN = (0.0, 1.0, 1.0, 1.0)


def clear_draw_list():
    global draw_lines
    global draw_faces
    global draw_points
    global draw_faces_list
    draw_lines.clear()
    draw_faces.clear()
    draw_points.clear()
    draw_faces_list.clear()
    draw_colored_tris_pos.clear()
    draw_colored_tris_col.clear()
    draw_colored_lines_pos.clear()
    draw_colored_lines_col.clear()
    draw_pins.clear()


def add_pin(co):
    """Mark a pinned vertex - drawn as a little red square, like UV pins."""
    draw_pins.append(tuple(co))


def add_colored_line(coords, colors):
    """Add one line with a color per end to the draw list.

    coords: two world space positions, colors: two RGBA tuples. Consecutive
    segments with per-vertex colors read as a smooth gradient along a loop.
    """
    global draw_colored_lines_pos, draw_colored_lines_col
    for co in coords:
        draw_colored_lines_pos.append(tuple(co))
    for col in colors:
        draw_colored_lines_col.append(tuple(col))


def add_colored_tri(coords, colors):
    """Add one triangle with a color per corner to the draw list.

    coords: three world space positions, colors: three RGBA tuples. The colors
    interpolate across the triangle, so per-vertex measurements read as a
    smooth field over the faces.
    """
    global draw_colored_tris_pos, draw_colored_tris_col
    for co in coords:
        draw_colored_tris_pos.append(tuple(co))
    for col in colors:
        draw_colored_tris_col.append(tuple(col))


def add_line(v1, v2, col):
    """
    Add a single line to the draw list
    """
    global draw_lines
    col_rounded = (
        round(col[0], 1),
        round(col[1], 1),
        round(col[2], 1),
        round(col[3], 1),
    )
    line_set = draw_lines.get(col_rounded, [])
    line_set.append(v1.to_tuple())
    line_set.append(v2.to_tuple())
    draw_lines[col_rounded] = line_set


def add_point(center, col):
    global draw_points
    col_rounded = (
        round(col[0], 1),
        round(col[1], 1),
        round(col[2], 1),
        round(col[3], 1),
    )
    point_set = draw_points.get(col_rounded, [])
    point_set.append(center)
    draw_points[col_rounded] = point_set


def add_arrow(v1, v2, col, scale=3):
    """
    Add an arrow to the draw list
    """
    global draw_lines
    direction = v2 - v1
    if direction.length < 0.0001:
        return
    col = (round(col[0], 1), round(col[1], 1), round(col[2], 1), round(col[3], 1))

    user_preferences = bpy.context.preferences.addons[__package__].preferences
    if user_preferences.enable_draw_arrows is False:
        return
    arrow_length_fraction = 0.5
    arrow_width_fraction = 0.5
    v2 = v1 + direction * scale

    line_set = draw_lines.get(col, [])

    # Add main line (arrow stem)
    line_set.append(v1.to_tuple())
    line_set.append(v2.to_tuple())

    # Calculate direction and length from v1 to v2
    direction = v2 - v1
    line_length = direction.length
    direction.normalize()

    # Calculate the length and width of the arrowhead based on the main line
    arrow_length = line_length * arrow_length_fraction
    arrow_width = line_length * arrow_width_fraction

    # Calculate the points of the arrowhead
    arrow_tip = v2
    arrow_left = (
        arrow_tip
        - arrow_length * direction
        + arrow_width * mathutils.Vector((-direction.y, direction.x, 0))
    )
    arrow_right = (
        arrow_tip
        - arrow_length * direction
        - arrow_width * mathutils.Vector((-direction.y, direction.x, 0))
    )

    # Add the arrowhead lines to the draw list
    line_set.append(arrow_tip.to_tuple())
    line_set.append(arrow_left.to_tuple())

    line_set.append(arrow_tip.to_tuple())
    line_set.append(arrow_right.to_tuple())

    draw_lines[col] = line_set


def add_face(bm_face, object, col):
    """
    Add a single face to the draw list
    """
    global draw_faces
    global draw_faces_list
    user_preferences = bpy.context.preferences.addons[__package__].preferences
    if user_preferences.enable_draw_faces is False:
        return

    if bm_face.index in draw_faces_list:
        return

    face_set = draw_faces.get(col, [])

    # Object's world transformation matrix
    ob_matrix_world = object.matrix_world

    if len(bm_face.verts) == 4:  # Ensure it's a quad
        # Transform vertex coordinates to world space
        v1, v2, v3, v4 = [(ob_matrix_world @ v.co) for v in bm_face.verts]

        # Add the transformed coordinates to face_set
        face_set.append([v1.to_tuple(), v2.to_tuple(), v3.to_tuple(), v4.to_tuple()])

        draw_faces[col] = face_set
        draw_faces_list.append(bm_face.index)
    else:
        print("The provided bm_face is not a quad.")


def draw_callback_px_2d(self, context):
    """
    Pixel-space overlays in the 3D Viewport: the pinned vertices, as little
    red squares of a fixed screen size like the UV editor's pins. Drawn as
    projected quads rather than GPU points - the point primitive ignores its
    size on Metal and shrinks to an invisible speck. Pins show whenever the
    modal runs, whatever the active constraint and the overlay preference.
    """
    if bpy.context.mode != "EDIT_MESH" or not draw_pins:
        return
    region = context.region
    rv3d = context.region_data
    if region is None or rv3d is None:
        return

    half = 4.0
    coords = []
    for co in draw_pins:
        p = bpy_extras.view3d_utils.location_3d_to_region_2d(region, rv3d, Vector(co))
        if p is None:
            continue
        x, y = p.x, p.y
        coords += [
            (x - half, y - half), (x + half, y - half), (x + half, y + half),
            (x - half, y - half), (x + half, y + half), (x - half, y + half),
        ]
    if not coords:
        return
    if bpy.app.version < (4, 0, 0):
        shader = gpu.shader.from_builtin("2D_UNIFORM_COLOR")
    else:
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")
    gpu.state.blend_set("ALPHA")
    batch = batch_for_shader(shader, "TRIS", {"pos": coords})
    shader.uniform_float("color", (1.0, 0.12, 0.12, 0.95))
    shader.bind()
    batch.draw(shader)
    gpu.state.blend_set("NONE")


def draw_callback_px_3d(self, context):
    """
    Draw lines and faces in the 3D Viewport.
    """
    # this is to avoid spamming console after errors
    global draw_lines, draw_faces
    # 50% alpha, 2 pixel width line
    if bpy.context.mode != "EDIT_MESH":
        return

    if bpy.app.version < (4, 0, 0):
        shader = gpu.shader.from_builtin("3D_UNIFORM_COLOR")
    else:
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")

    gpu.state.blend_set("ALPHA")
    gpu.state.line_width_set(4.0)

    user_preferences = bpy.context.preferences.addons[__package__].preferences
    if user_preferences.enable_draw_arrows or user_preferences.enable_draw_constraints:
        # print('line sets', len(draw_lines))
        for col, lines in draw_lines.items():
            batch = batch_for_shader(shader, "LINES", {"pos": lines})
            col_with_alpha = (
                col[0],
                col[1],
                col[2],
                col[3] * user_preferences.overlays_alpha,
            )

            shader.uniform_float("color", col_with_alpha)

            shader.bind()
            batch.draw(shader)

    if user_preferences.enable_draw_faces:
        for col, faces in draw_faces.items():
            # Break each quad face into two triangles
            tris = []
            for quad in faces:
                tris.extend([quad[0], quad[1], quad[2]])
                tris.extend([quad[2], quad[3], quad[0]])

            # Create the batch
            batch = batch_for_shader(shader, "TRIS", {"pos": tris})
            col_with_alpha = (
                col[0],
                col[1],
                col[2],
                col[3] * user_preferences.overlays_alpha,
            )
            shader.uniform_float("color", col_with_alpha)
            batch.draw(shader)

    if (draw_colored_tris_pos or draw_colored_lines_pos) and user_preferences.enable_draw_constraints:
        if bpy.app.version < (4, 0, 0):
            smooth_shader = gpu.shader.from_builtin("3D_SMOOTH_COLOR")
        else:
            smooth_shader = gpu.shader.from_builtin("SMOOTH_COLOR")
        smooth_shader.bind()
        if draw_colored_tris_pos:
            batch = batch_for_shader(
                smooth_shader,
                "TRIS",
                {"pos": draw_colored_tris_pos, "color": draw_colored_tris_col},
            )
            batch.draw(smooth_shader)
        if draw_colored_lines_pos:
            batch = batch_for_shader(
                smooth_shader,
                "LINES",
                {"pos": draw_colored_lines_pos, "color": draw_colored_lines_col},
            )
            batch.draw(smooth_shader)

    for col, points in draw_points.items():
        for point in points:
            draw_circle_2d(point, col, 0.001, segments=4)

            # batch = batch_for_shader(shader, 'LINES', {"pos": lines})
            # col_with_alpha = (col[0], col[1], col[2], 0.25)
            #
            # shader.uniform_float("color", col_with_alpha)
            #
            # shader.bind()
            # batch.draw(shader)
            # x, y, z = point  # 3D coordinates of the point
            # radius = .2  # Adjust the radius as needed
            #
            # # Set the color
            # col_with_alpha = (col[0], col[1], col[2], 1.0)
            # shader.uniform_float("color", col_with_alpha)
            #
            # # Convert 3D point to 2D screen coordinates
            # coords_2d = bpy_extras.object_utils.world_to_camera_view(bpy.context.scene, bpy.context.scene.camera,
            #                                                          point)
            #
            # if 0 <= coords_2d.x <= 1 and 0 <= coords_2d.y <= 1:
            #     # Draw a 2D circle at the calculated screen coordinates
            #     shader.bind()
            #     draw_circle_2d((coords_2d.x, coords_2d.y),col_with_alpha, radius)

    # restore opengl defaults
    gpu.state.blend_set("NONE")
    # ...
