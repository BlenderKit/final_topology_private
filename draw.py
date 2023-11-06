from gpu_extras.batch import batch_for_shader
from gpu_extras.presets import draw_circle_2d

import bpy_extras
import mathutils
import bpy
import gpu
from mathutils import Vector

draw_lines = {}
draw_faces = {}
draw_points = {}
draw_faces_list = []

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
    print('clearing draw list')
    print(len(draw_lines.items()))
    draw_lines.clear()
    draw_faces.clear()
    draw_points.clear()
    draw_faces_list.clear()
    print(len(draw_lines.items()))


def add_line(v1, v2, col):
    '''
     Add a single line to the draw list
    '''
    global draw_lines
    col_rounded = (round(col[0], 1), round(col[1], 1), round(col[2], 1), round(col[3], 1))
    line_set = draw_lines.get(col_rounded, [])
    line_set.append(v1.to_tuple())
    line_set.append(v2.to_tuple())
    draw_lines[col_rounded] = line_set

def add_point(center, col):
    global draw_points
    col_rounded = (round(col[0], 1), round(col[1], 1), round(col[2], 1), round(col[3], 1))
    point_set = draw_points.get(col_rounded, [])
    point_set.append(center)
    draw_points[col_rounded] = point_set


def add_arrow(v1, v2, col, scale=3):
    '''
        Add an arrow to the draw list
    '''
    global draw_lines
    col = (round(col[0], 1), round(col[1], 1), round(col[2], 1), round(col[3], 1))

    user_preferences = bpy.context.preferences.addons['final_topology'].preferences
    if user_preferences.enable_draw_arrows is False:
        return
    arrow_length_fraction = 0.5
    arrow_width_fraction = 0.5
    direction = v2 - v1
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
    arrow_left = arrow_tip - arrow_length * direction + arrow_width * mathutils.Vector((-direction.y, direction.x, 0))
    arrow_right = arrow_tip - arrow_length * direction - arrow_width * mathutils.Vector((-direction.y, direction.x, 0))

    # Add the arrowhead lines to the draw list
    line_set.append(arrow_tip.to_tuple())
    line_set.append(arrow_left.to_tuple())

    line_set.append(arrow_tip.to_tuple())
    line_set.append(arrow_right.to_tuple())

    draw_lines[col] = line_set


def add_face(bm_face, object, col):
    '''
        Add a single face to the draw list
    '''
    global draw_faces
    global draw_faces_list
    user_preferences = bpy.context.preferences.addons['final_topology'].preferences
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
    '''
    Draw text in the 3D Viewport.
    '''
    # this is to avoid spamming console after errors
    global draw_lines

    font_id = 0  # XXX, need to find out how best to get this.

    # draw some text
    # blf.position(font_id, 15, 30, 0)
    # blf.size(font_id, 20, 72)
    # blf.draw(font_id, "Inverse-subdivide activated ")


def draw_callback_px_3d(self, context):
    '''
    Draw lines and faces in the 3D Viewport.
    '''
    # this is to avoid spamming console after errors
    global draw_lines, draw_faces
    # 50% alpha, 2 pixel width line
    if bpy.context.mode != 'EDIT_MESH':
        return

    if bpy.app.version < (4, 0, 0):
        shader = gpu.shader.from_builtin('3D_UNIFORM_COLOR')
    else:
        shader = gpu.shader.from_builtin('UNIFORM_COLOR')

    gpu.state.blend_set('ALPHA')
    gpu.state.line_width_set(4.0)

    user_preferences = bpy.context.preferences.addons['final_topology'].preferences
    if user_preferences.enable_draw_arrows:
        # print('line sets', len(draw_lines))
        for col, lines in draw_lines.items():
            batch = batch_for_shader(shader, 'LINES', {"pos": lines})
            col_with_alpha = (col[0], col[1], col[2], 0.25)

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
            batch = batch_for_shader(shader, 'TRIS', {"pos": tris})
            col_with_alpha = (col[0], col[1], col[2], 0.1)
            shader.uniform_float("color", col_with_alpha)
            batch.draw(shader)

    for col, points in draw_points.items():
        for point in points:
            draw_circle_2d(point, col, .01, segments=6)

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
    gpu.state.blend_set('NONE')
    # ...


