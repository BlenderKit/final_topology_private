"""Small helper library for constraint transform gizmos.

A gizmo group built on TransformGizmoGroupBase shows a combined translate and
rotate gizmo - three axis arrows and up to three rotation dials - for each
target the group reports. The concrete gizmo group supplies the targets as
small adapter objects, so the same lib serves any constraint that stores
something with a position and an orientation.

The adapter protocol, all coordinates in world space:

    location()                      where the gizmo is drawn
    orientation()                   3x3 world basis the gizmo axes align to
    snapshot()                      opaque copy of the state a drag starts from,
                                    including the basis - the drag stays in the
                                    frame it started in
    translate(snapshot, axis, d)    move distance d along snapshot axis 0..2
    rotate(snapshot, axis, angle)   rotate around snapshot axis 0..2
    translation_axes()              optional, which arrows to show, e.g. a plane
                                    with an unfixed center hides all of them
    rotation_axes()                 optional, which dials to show, e.g. a circle
                                    hides the spin around its own normal
    radius()                        optional, world radius of an outline circle
    outline_matrix()                optional, orientation matrix for the outline

Drag handling: Blender keeps calling the set handler with the total offset
since the drag started, and may call the get handler at any time during the
drag. The snapshot is therefore taken only when none exists and cleared only
when no gizmo is being dragged - taking it fresh on every get call would apply
each update on an already moved base and send the target drifting away. For
the same reason the matrix of the gizmo currently being dragged is never
touched; all the other gizmos of the unit are re-synced from inside the set
handler, so the outline and the remaining handles follow the drag live.
"""

import bpy
from mathutils import Matrix, Vector

AXIS_COLORS = (
    (0.89, 0.2, 0.32),
    (0.42, 0.75, 0.18),
    (0.2, 0.45, 0.9),
)


def orientation_mode(context):
    """Blender's transform orientation, collapsed to the modes gizmos support.

    GLOBAL and LOCAL mean what they mean in Blender, NORMAL means the target's
    own frame. Gimbal, view, cursor and custom orientations get the global axes.
    """
    try:
        mode = context.scene.transform_orientation_slots[0].type
    except (AttributeError, IndexError):
        return "GLOBAL"
    if mode in ("NORMAL", "LOCAL"):
        return mode
    return "GLOBAL"


def _axis_matrix(axis, location):
    matrix = axis.normalized().to_track_quat("Z", "Y").to_matrix().to_4x4()
    matrix.translation = location
    return matrix


class TransformGizmoUnit:
    """Arrows and dials for one transformable target inside a gizmo group."""

    def __init__(self, group, get_target):
        # get_target is a callable resolving to the adapter, or None when the
        # target disappeared - resolved fresh on every use, since constraint
        # data may be reallocated between redraws
        self.get_target = get_target
        self.arrows = []
        self.dials = []
        self._snapshot = None

        for i in range(3):
            gz = group.gizmos.new("GIZMO_GT_arrow_3d")
            gz.color = AXIS_COLORS[i]
            gz.alpha = 0.7
            gz.color_highlight = (1.0, 1.0, 0.6)
            gz.alpha_highlight = 1.0
            gz.line_width = 2
            gz.target_set_handler(
                "offset", get=self._grab_get(), set=self._translate_set(i)
            )
            self.arrows.append(gz)

        for i in range(3):
            gz = group.gizmos.new("GIZMO_GT_dial_3d")
            gz.color = AXIS_COLORS[i]
            gz.alpha = 0.5
            gz.color_highlight = (1.0, 1.0, 0.6)
            gz.alpha_highlight = 1.0
            gz.line_width = 2
            gz.scale_basis = 0.65
            gz.use_draw_value = True
            gz.target_set_handler(
                "offset", get=self._grab_get(), set=self._rotate_set(i)
            )
            self.dials.append(gz)

        # outline of the stored shape itself, drawn in world size, not clickable
        self.outline = group.gizmos.new("GIZMO_GT_dial_3d")
        self.outline.color = (1.0, 1.0, 1.0)
        self.outline.alpha = 0.35
        self.outline.line_width = 1
        self.outline.hide_select = True
        self.outline.use_draw_scale = False

    def _all_gizmos(self):
        return self.arrows + self.dials + [self.outline]

    def _grab_get(self):
        def get():
            # only the first call of a drag takes the snapshot - this may get
            # called again mid-drag, and re-snapshotting the already moved
            # state would compound every update into a runaway
            if self._snapshot is None:
                target = self.get_target()
                if target is not None:
                    self._snapshot = target.snapshot()
            return 0.0

        return get

    def _translate_set(self, axis_index):
        def set_value(value):
            target = self.get_target()
            if target is not None and self._snapshot is not None:
                target.translate(self._snapshot, axis_index, value)
                # draw_prepare isn't reliable during a modal drag, push the
                # live feedback to the other gizmos from here
                self.sync()

        return set_value

    def _rotate_set(self, axis_index):
        def set_value(value):
            target = self.get_target()
            if target is not None and self._snapshot is not None:
                target.rotate(self._snapshot, axis_index, value)
                self.sync()

        return set_value

    def sync(self):
        """Place the gizmos at the target, or hide them when it's gone.

        The gizmo currently being dragged is left alone: moving its matrix
        mid-drag would change what the drag offset means.
        """
        target = self.get_target()
        if target is None:
            for gz in self._all_gizmos():
                gz.hide = True
            self._snapshot = None
            return

        dragging = any(gz.is_modal for gz in self._all_gizmos())
        if not dragging:
            # a finished drag's snapshot must not leak into the next one
            self._snapshot = None

        location = target.location()
        basis = target.orientation()
        translation_axes = (True, True, True)
        if hasattr(target, "translation_axes"):
            translation_axes = target.translation_axes()
        rotation_axes = (True, True, True)
        if hasattr(target, "rotation_axes"):
            rotation_axes = target.rotation_axes()

        for i, gz in enumerate(self.arrows):
            if gz.is_modal:
                continue
            gz.hide = not translation_axes[i]
            if not gz.hide:
                gz.matrix_basis = _axis_matrix(basis.col[i], location)
        for i, gz in enumerate(self.dials):
            if gz.is_modal:
                continue
            gz.hide = not rotation_axes[i]
            if not gz.hide:
                gz.matrix_basis = _axis_matrix(basis.col[i], location)

        radius = getattr(target, "radius", None)
        outline_matrix = getattr(target, "outline_matrix", None)
        if radius is None or outline_matrix is None:
            self.outline.hide = True
            return
        r = radius()
        if r <= 0.0:
            self.outline.hide = True
            return
        self.outline.hide = False
        # bake the world radius into the matrix, the gizmo itself has radius 1
        self.outline.matrix_basis = outline_matrix() @ Matrix.Scale(r, 4)


class TransformGizmoGroupBase:
    """Mixin for a GizmoGroup drawing transform units.

    The subclass provides get_targets(context) returning the adapters, in a
    stable order. Units are pooled and never removed, surplus ones just hide,
    since Blender's gizmo collection has no removal.
    """

    def setup(self, context):
        self._units = []
        self._sync_units(context)

    def _sync_units(self, context):
        targets = self.get_targets(context)
        while len(self._units) < len(targets):
            index = len(self._units)
            self._units.append(TransformGizmoUnit(self, self._make_getter(index)))
        for unit in self._units:
            unit.sync()

    def _make_getter(self, index):
        def getter():
            targets = self.get_targets(bpy.context)
            if index < len(targets):
                return targets[index]
            return None

        return getter

    def refresh(self, context):
        self._sync_units(context)

    def draw_prepare(self, context):
        self._sync_units(context)
