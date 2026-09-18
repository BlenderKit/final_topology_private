# from .draw import *
# from .utils import *
import bmesh
import bpy
from bpy.props import BoolProperty, IntProperty
from bpy.types import Operator
from mathutils import Vector

from . import draw, utils, inverse_subdivide

has_extras = True
try:
    from . import extras
except:
    has_extras = False

running_operator = None




def _set_if_changed(owner, attribute, value):
    """Assign only when the value differs - every RNA write tags the object
    for a depsgraph update, which would re-evaluate the subdivision for
    nothing on each step."""
    if getattr(owner, attribute) == value:
        return 0
    setattr(owner, attribute, value)
    return 1


# only the mesh, its mirror and its subdivision take part in the solve
MODIFIERS_KEPT = ("ARMATURE", "MIRROR", "SUBSURF")


def apply_modifier_rules(obj):
    """Put the modifiers into the state the solver needs: other modifiers
    hidden, the subdivision shown at level 1 or 2. Checks first and writes
    only what differs, so calling it every step is free while nothing
    changed and reverts a manual change when there was one. Returns the
    number of settings written."""
    writes = 0
    for modifier in obj.modifiers:
        if modifier.type == "SUBSURF":
            writes += _set_if_changed(modifier, "show_viewport", True)
            writes += _set_if_changed(modifier, "show_in_editmode", True)
            writes += _set_if_changed(modifier, "levels", max(1, min(2, modifier.levels)))
        elif modifier.type not in MODIFIERS_KEPT:
            writes += _set_if_changed(modifier, "show_viewport", False)
            writes += _set_if_changed(modifier, "show_in_editmode", False)
    return writes


def set_modifiers_start(obj):
    """Remember the modifier settings and apply the solver's rules; the
    remembered state goes back in set_modifiers_end."""
    modifiers_state_start = []
    for modifier in obj.modifiers:
        mod_settings = {
            "show_viewport": modifier.show_viewport,
            "show_in_editmode": modifier.show_in_editmode,
            "virtual": False,
        }
        if modifier.type == "SUBSURF":
            mod_settings["levels"] = modifier.levels
        modifiers_state_start.append(mod_settings)

    if len(obj.modifiers) == 0:
        modifier = obj.modifiers.new("Subdivision", "SUBSURF")
        modifier.levels = 2
        modifiers_state_start.append(
            {
                "show_viewport": True,
                "show_in_editmode": True,
                "levels": 2,
                "virtual": True,
            }
        )
    apply_modifier_rules(obj)
    return modifiers_state_start


def set_modifiers_end(obj, modifiers_state_start):
    """Restore the remembered modifier settings, writing only what differs.
    Returns the number of settings written."""
    writes = 0
    virtual = []
    for i, modifier in enumerate(obj.modifiers):
        if i >= len(modifiers_state_start):
            # added by the user while running - nothing to restore
            continue
        mod_settings = modifiers_state_start[i]
        if mod_settings["virtual"]:
            virtual.append(modifier)
            continue
        if modifier.type == "SUBSURF" and "levels" in mod_settings:
            writes += _set_if_changed(modifier, "levels", mod_settings["levels"])
        writes += _set_if_changed(modifier, "show_viewport", mod_settings["show_viewport"])
        writes += _set_if_changed(modifier, "show_in_editmode", mod_settings["show_in_editmode"])
    for modifier in virtual:
        obj.modifiers.remove(modifier)
        writes += 1
    return writes



def final_topology_optimization_step(self, context, iterations=1, neighbours=1):
    """Runs all optimization steps:
    - if extras are not present or constraint list is empty, only inverse subdivide is run.
    - if there are any constraints, inverse subdivide is skipped, unless it's in the constraints.
    - constraints
    """
    user_preferences = bpy.context.preferences.addons[__package__].preferences

    # the modifiers were set up when the run started; a manual change in
    # the meantime gets reverted here, otherwise this writes nothing
    apply_modifier_rules(self.object)

    s_levels = inverse_subdivide.get_subdivision_modifier_level(self.object)
    if s_levels is None:
        self.report(
            {"WARNING"},
            "Only objects with subdivision modifier are supported by now.",
        )
        return False

    

    depsgraph = bpy.context.evaluated_depsgraph_get()
    bm_eval = utils.get_evaluated_bm(self.object, depsgraph)
    me = self.object.data
    bm_edit = bmesh.from_edit_mesh(me)

    # if there are constraints and no inverse subdivide constraint, skip
    # inverse subdivide - the artists build has no constraint system at all
    constraints = []
    if has_extras and hasattr(self.object.data, "ft_custom_constraints"):
        constraints = self.object.data.ft_custom_constraints
    if len(constraints) > 0 and not any(c.constraint_type == "INVERSE_SUBDIVIDE" for c in constraints):
        inverse_subdivide_prep = None
    else:
        inverse_subdivide_prep = inverse_subdivide.prepare_inverse_subdivide(
            self.object, bm_edit, bm_eval
        )
        # with no constraints there has to be something to snap to, otherwise
        # there is nothing this step could do
        if not inverse_subdivide_prep:
            self.report(
                {"WARNING"},
                "Nothing to snap to. Add a mesh to snap to, use Freeze Shape, or add constraints.",
            )
            return False

    has_constraints = (
        has_extras
        and hasattr(self.object.data, "ft_custom_constraints")
        and len(self.object.data.ft_custom_constraints) > 0
    )
    
    for a in range(0, iterations):
        draw.clear_draw_list()
        if a > 0:
            # we need to evaluate result subdivided mesh every iteration,
            # so need a fresh bm_eval, except for first iteration
            depsgraph = bpy.context.evaluated_depsgraph_get()
            bm_eval = utils.get_evaluated_bm(self.object, depsgraph)

        bm_edit.verts.ensure_lookup_table()

        if has_constraints:
            bm_edit = extras.evaluate_constraints(
                self.object, bmesh_edit=bm_edit, bmesh_eval=bm_eval, inverse_subdivide_prep = inverse_subdivide_prep
            )
        else:
            # default behaviour with no constraints and in artist version - only inverse subdivide
            target_offsets = inverse_subdivide.evaluate_inverse_subdivide(
                self.object,
                bm_edit,
                bm_eval,
                inverse_subdivide_prep,
                normal_offset=self.object.final_topology.normal_offset,
            )
            utils.move_verts_to_targets(bm_edit, target_offsets, weight=user_preferences.step_weight)

        bmesh.update_edit_mesh(me)
    return True




class FinalTopologyStep(Operator):
    bl_idname = "mesh.final_topology_optimization_step"
    bl_label = "Final Topology Step"
    bl_description = (
        "Run the subdivision optimisation just step by step. "
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
        user_preferences = bpy.context.preferences.addons[__package__].preferences

        # check if there's subdivision modifier
        self.object = bpy.context.active_object
        modifiers_state_start = set_modifiers_start(self.object)

        done = final_topology_optimization_step(
            self, context, self.iterations, self.neighbours
        )
        # restore the modifiers also when the step bailed out,
        # otherwise they stay hidden and the subdivision levels stay clamped
        set_modifiers_end(self.object, modifiers_state_start)
        if not done:
            return {"CANCELLED"}

        return {"FINISHED"}

    def invoke(self, context, event):
        return self.execute(context)


class finalTopologyModal(Operator):
    bl_idname = "mesh.final_topology_modal"
    bl_label = "Final Topology Snapping Modal"
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
        active_obj = context.active_object

        if (
            not active_obj
            or not active_obj.final_topology.enable_operator
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
            if active_obj:
                active_obj.final_topology.enable_operator = False
            set_modifiers_end(self.object, self.modifiers_state_start)
            return {"CANCELLED"}

        return {"PASS_THROUGH"}

    def modal(self, context, event):
        user_preferences = bpy.context.preferences.addons[__package__].preferences

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

        active_obj = context.active_object
        if not active_obj:
            return {"CANCELLED"}

        if running_operator is not None:
            active_obj.final_topology.enable_operator = False
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

        wm = context.window_manager
        self._timer = wm.event_timer_add(0.3, window=context.window)

        context.window_manager.modal_handler_add(self)
        # set running operator to be aware of it already running
        running_operator = self
        # enable this if user did run the operator e.g. from search menu
        active_obj.final_topology.enable_operator = True
        self.warning_posted = False
        self.object = active_obj
        self.modifiers_state_start = set_modifiers_start(self.object)

        return {"RUNNING_MODAL"}


def create_freeze_mesh_object(freeze_name="FROZEN_MESH_STATE", constraint=None):
    orig_ob = bpy.context.active_object
    orig_mode = bpy.context.mode
    bpy.ops.object.mode_set(mode="OBJECT")
    bpy.ops.object.duplicate()
    freeze_mesh_object = bpy.context.active_object
    freeze_mesh_object.name = freeze_name
    freeze_mesh_object.name = freeze_name

    for m in freeze_mesh_object.modifiers:
        if m.type != "MIRROR":
            m.show_viewport = False
            m.show_render = False

    freeze_mesh_object.hide_set(True)
    freeze_mesh_object.hide_render = True

    m = freeze_mesh_object.modifiers.new("Subdivision", "SUBSURF")
    m.levels = 5
    utils.activate_object(orig_ob)
    bpy.ops.object.mode_set(mode="EDIT")
    
    if constraint is not None:
        constraint.use_object_or_collection = "OBJECT"
        constraint.invsubdiv_target_object = freeze_mesh_object
    else:
        orig_ob.final_topology.use_object_or_collection = "OBJECT"
        orig_ob.final_topology.target_object = freeze_mesh_object
    return freeze_mesh_object


def delete_frozen_mesh(freeze_name="FROZEN_MESH_STATE", constraint=None):
    active_obj = bpy.context.active_object
    object = bpy.data.objects.get(freeze_name)
    if object is not None:
        bpy.data.objects.remove(object)
    if active_obj:
        if constraint is not None:
            constraint.use_object_or_collection = "SCENE"
        else:
            active_obj.final_topology.use_object_or_collection = "SCENE"


class FreezeShape(bpy.types.Operator):
    bl_idname = "mesh.freeze_shape"
    bl_label = "Freeze Subdivision Shape"
    bl_description = (
        "Switch on the Modal operator or perform Inverse Subdivide steps."
        "\n\nCreates a copy of self and switches on snapping to it."
        "\nUse to reorganize your topology, but try to preserve shape."
    )
    bl_options = {"REGISTER", "UNDO"}

    constraint_index: bpy.props.IntProperty(default=-1)

    def execute(self, context):
        active_obj = context.active_object
        constraint = None
        
        if self.constraint_index >= 0 and hasattr(active_obj.data, 'ft_custom_constraints'):
            if self.constraint_index < len(active_obj.data.ft_custom_constraints):
                constraint = active_obj.data.ft_custom_constraints[self.constraint_index]
                freeze_name = f"FROZEN_MESH_STATE_{active_obj.name}_{constraint.name}"
        else:
            freeze_name = f"FROZEN_MESH_STATE_{active_obj.name}"
        
        object = bpy.data.objects.get(freeze_name)
        if object is not None:
            delete_frozen_mesh(freeze_name, constraint)
        else:
            create_freeze_mesh_object(freeze_name, constraint)
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
