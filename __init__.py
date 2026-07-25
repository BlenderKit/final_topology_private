bl_info = {
    "name": "Final Topology - Inverse subdivide",
    "author": "Vilem Duha, BlenderKit",
    "version": (2, 0, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > Edit > Final Topology",
    "description": "Professional subdivision modelling tools, simply magic.",
    "warning": "",
    "doc_url": "https://github.com/BlenderKit/final_topology",
    "tracker_url": "https://github.com/BlenderKit/final_topology/issues",
    "category": "3D View",
}

from importlib import reload

has_extras = True
if "bpy" in locals():
    try:
        gizmos = reload(gizmos)
        extras = reload(extras)
    except Exception as e:
        has_extras = False
        print(e)

    final_topology = reload(final_topology)
    draw = reload(draw)
    utils = reload(utils)
    print_safe = reload(print_safe)
else:
    from .final_topology import *
    from .print_safe import *

    try:
        from . import gizmos
        from . import extras
        from .extras import *
    except Exception as e:
        has_extras = False
        print(e)
    from . import final_topology
    from . import draw
    from . import utils
    from . import print_safe

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import AddonPreferences, Operator, Panel, PropertyGroup


class FinalTopologyObjectProperties(PropertyGroup):
    enable_operator: BoolProperty(
        name="Inverse-Subsurf",
        default=False,
        description="Toggle to enable Inverse-Subsurf",
    )

    use_object_or_collection: EnumProperty(
        name="Use Object or Collection",
        items=[
            ("SCENE", "Scene", "\nAll evaluated objects in scene"),
            ("OBJECT", "Object", "\nSingle object"),
            ("COLLECTION", "Collection", "\nCollection"),
        ],
        default="SCENE",
        description="Snap to",
    )

    target_object: PointerProperty(type=bpy.types.Object, name="Target Object")
    target_collection: PointerProperty(
        type=bpy.types.Collection, name="Target Collection"
    )
    normal_offset: FloatProperty(
        name="Normal Offset",
        default=0.0,
        soft_min=-0.2,
        soft_max=0.2,
        description="Normal offset",
        unit="LENGTH",
    )


class PopupDialog(bpy.types.Operator):
    """Small popup dialog to inform user."""

    bl_idname = "wm.final_topo_popup_dialog"
    bl_label = "Final Topology message:"
    bl_options = {"REGISTER", "INTERNAL"}

    message: bpy.props.StringProperty(default="")
    width: bpy.props.IntProperty(default=300)

    def draw(self, context):
        layout = self.layout
        lines = self.message.split("\n")
        for l in lines:
            row = layout.row()
            row.label(text=l)
        row.operator("view3d.close_popup_button", text="", icon="CANCEL")
        layout.active_default = True

    def execute(self, context):
        wm = bpy.context.window_manager
        return wm.invoke_popup(self, width=self.width)


class InverseSubdivideAddonPreferences(AddonPreferences):
    bl_idname = __name__

    always_on: BoolProperty(
        name="After each operation",
        default=False,
        description="Run the compensation algorithm on after every editmode operation.",
    )

    use_timer: BoolProperty(
        name="Simulation mode",
        default=False,
        description="The Inverse subdivision will run several times per second.\n\n"
        "This is useful for interactive modelling, "
        "\nbut can be slow or instableon complex meshes.",
    )

    iterations: IntProperty(
        name="Iterations",
        default=1,
        min=1,
        max=100,
        description="Higher numbers are more precise and slower",
    )

    neighbours: IntProperty(
        name="Neighbours levels",
        default=1,
        min=0,
        max=10,
        description="Number of surrounding faces influenced.\n\n"
        "This is useful since subdivided surface \n"
        "always gets influence from the surrounding faces.",
    )
    max_distance: FloatProperty(
        name="Max Distance",
        default=1.0,
        min=0,
        max=5000,
        description="Maximum distance to move vertices. "
        "\n\n If distance is higher, vertices stay in place",
        precision=10,
        unit="LENGTH",
    )

    gradient_sensitivity_distance: FloatProperty(
        name="Gradient Sensitivity",
        default=0.02,
        min=0.0001,
        max=1,
        description="Color gradient sensitivity, \n anything over this distance will be strictly red",
        precision=10,
        unit="LENGTH",
    )

    arrow_scale: FloatProperty(
        name="Arrow scale",
        default=10,
        min=0.1,
        max=100,
        description="Arrow scale - multiplier of the distance by which the vertex was moved in the last iteration.",
        precision=1,
    )

    enable_draw_faces: BoolProperty(
        name="Draw Faces",
        default=False,
        description="Draw faces around influence points.\n\n"
        "Draws faces colored by the distance from target surface.\n"
        "Color gradient influenced by the gradient sensitivity distance.",
    )
    enable_draw_arrows: BoolProperty(
        name="Draw Arrows",
        default=True,
        description="Drawing of arrows at projected points.\n\n"
        "Arrows are scaled by the distance from target surface.\n"
        "Color gradient influenced by the gradient sensitivity distance.",
    )
    enable_draw_constraints: BoolProperty(
        name="Draw Constraints",
        default=True,
        description="Drawing of constraints.\n\n"
        "Selected constraint gets highlighted and selected.\n",
    )
    overlays_alpha: FloatProperty(
        name="Overlays Alpha",
        default=0.5,
        min=0,
        max=1,
        description="Alpha of the overlays",
    )

    step_weight: FloatProperty(
        name="Step Weight",
        default=0.2,
        min=0.1,
        max=1,
        description="Weight of the inverse subdivision.\n\n"
        "Lower values go slower to result, but prevent jiggle.",
    )

    weight_algorithm: EnumProperty(
        name="Weight Algorithm",
        items=[
            ("ALL1", "All same", "Vertex and edge midpoints have same weight"),
            (
                "FIRST",
                "Control vertex more importance",
                "Vertex has weight 1, edge midpoints share weight 1/n",
            ),
            (
                "FIRSTONLY",
                "Only vertices, no edges",
                "Only vertices are used, no edge midpoints",
            ),
            (
                "FIRSTDIST",
                "Distance",
                "Edge midpoints are weighted by distance to main vert",
            ),
        ],
        default="FIRST",
        description="Choose weighting algorithm.\n\n"
        "This influences how the offset is calculated and\n"
        "how much weight do the midpoints get.",
    )

    use_mirror: BoolProperty(
        name="Use Mirror",
        default=False,
        description="Use mirror modifier when evaluating",
    )

    print_safe_axis: EnumProperty(
        name="Axis",
        items=[
            ("+X", "+X", "Measure inclination against +X axis"),
            ("-X", "-X", "Measure inclination against -X axis"),
            ("+Y", "+Y", "Measure inclination against +Y axis"),
            ("-Y", "-Y", "Measure inclination against -Y axis"),
            ("+Z", "+Z", "Measure inclination against +Z axis"),
            ("-Z", "-Z", "Measure inclination against -Z axis"),
        ],
        default="-Z",
        description="Print-safe axis for slope measurement.\n\n"
        "The tool checks face normal angle against this direction.\n"
        "Default -Z is useful for overhang checks in 3D printing.",
    )
    print_safe_max_inclination: FloatProperty(
        name="Max Inclination",
        default=50.0,
        min=0.0,
        max=89.9,
        description="Allowed inclination in degrees for print-safe mode.\n\n"
        "Internally compared as (90 - this value) against\n"
        "angle between face normal and selected axis.\n"
        "Lower values are stricter.",
    )
    print_safe_iterations: IntProperty(
        name="Iterations",
        default=12,
        min=1,
        max=200,
        description="Number of optimization passes for print-safe mode.\n\n"
        "Higher values can improve result on dense meshes,\n"
        "but increase processing time.",
    )
    print_safe_strength: FloatProperty(
        name="Strength",
        default=0.35,
        min=0.01,
        max=1.0,
        description="Per-iteration movement multiplier for print-safe mode.\n\n"
        "Lower values are safer and smoother.\n"
        "Higher values converge faster but can overshoot.",
    )

def final_topology_operators_draw(self, context):
    user_preferences = bpy.context.preferences.addons[__name__].preferences
    layout = self.layout
    active_obj = bpy.context.active_object

    layout.operator(
        FinalTopologyStep.bl_idname,
        text="Step",
        icon="NEXT_KEYFRAME",
    )

    if active_obj.final_topology.enable_operator:
        layout.operator(
            finalTopologyModal.bl_idname,
            text="Pause",
            icon="PAUSE",
            emboss=True,
            depress=True,
        )
    else:
        layout.operator(
            finalTopologyModal.bl_idname,
            text="Run",
            icon="PLAY",
            emboss=True,
            depress=False,
        )

    # constraints replace the default inverse subdivision snapping - spell it
    # out, since an empty viewport result otherwise looks like a broken tool
    constraints = getattr(active_obj.data, "ft_custom_constraints", None)
    if constraints is not None and len(constraints) > 0:
        if not any(c.constraint_type == "INVERSE_SUBDIVIDE" for c in constraints):
            col = layout.column(align=True)
            col.scale_y = 0.8
            col.label(text="With active constraints, you need to add", icon="INFO")
            col.label(text="an inverse subdivision constraint", icon="BLANK1")
            col.label(text="if you want to use it.", icon="BLANK1")

    layout = layout.column()
    # don't hide so it's actually possible to tweak the settings while off

    layout.prop(user_preferences, "always_on", toggle=True, icon="MOD_TIME")
    if user_preferences.always_on:
        layout.prop(user_preferences, "use_timer", toggle=False, icon="TIME")

def final_topology_settings_UI_draw(self, context):
    if not poll_final_topology(self, context):
        return

    user_preferences = bpy.context.preferences.addons[__name__].preferences
    active_obj = bpy.context.active_object
    layout = self.layout

    # Check if any INVERSE_SUBDIVIDE constraints exist
    has_invsubdiv_constraint = False
    if hasattr(active_obj.data, 'ft_custom_constraints'):
        for constraint in active_obj.data.ft_custom_constraints:
            if constraint.constraint_type == "INVERSE_SUBDIVIDE":
                has_invsubdiv_constraint = True
                break
    
    box = layout.box()
    box.label(text="Optimization loop settings:")
    box.prop(user_preferences, "iterations")
    box.prop(user_preferences, "step_weight")

    box = layout.box()
    box.label(text="Inverse Subdivide settings:")
    # Hide freeze shape and target settings if constraint exists
    if has_invsubdiv_constraint:
        box.label(text="Target settings are in constraint.")
        
    if not has_invsubdiv_constraint:
        box.label(text="Snap to")
        object_freeze_name = f"FROZEN_MESH_STATE_{active_obj.name}"
        if bpy.data.objects.get(object_freeze_name) is not None:
            op = box.operator(
                FreezeShape.bl_idname, text="Unfreeze shape", depress=True, icon="FREEZE"
            )
            op.constraint_index = -1
        else:
            op = box.operator(
                FreezeShape.bl_idname, text="Freeze Shape", depress=False, icon="FREEZE"
            )
            op.constraint_index = -1
            row = box.row()
            row.prop(active_obj.final_topology, "use_object_or_collection", text="")
            if active_obj.final_topology.use_object_or_collection == "OBJECT":
                row.prop(active_obj.final_topology, "target_object", text="")
            elif active_obj.final_topology.use_object_or_collection == "COLLECTION":
                row.prop(active_obj.final_topology, "target_collection", text="")

        box.prop(active_obj.final_topology, "normal_offset", text="Normal Offset")
    if has_extras:
        box.prop(user_preferences, "neighbours")
    box.prop(user_preferences, "max_distance")
    if has_extras:
        box.prop(user_preferences, "weight_algorithm")
    box.prop(user_preferences, "use_mirror")


class VIEW3D_PT_final_topology_overlays(Panel):
    bl_category = "Edit"
    bl_idname = "VIEW3D_PT_final_topology_overlays"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Draw Overlays"
    bl_parent_id = "VIEW3D_PT_final_topology_editmode"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        user_preferences = bpy.context.preferences.addons[__name__].preferences

        layout.prop(user_preferences, "overlays_alpha")
        layout.separator()
        row = layout.row()
        row.prop(
            user_preferences,
            "enable_draw_arrows",
            toggle=True,
            text="",
            icon="EMPTY_SINGLE_ARROW",
        )
        row.prop(user_preferences, "arrow_scale", text="Arrow Scale")

        row = layout.row()
        row.prop(
            user_preferences, "enable_draw_faces", toggle=True, text="", icon="FACESEL"
        )
        row.prop(user_preferences, "gradient_sensitivity_distance", text="Sensitivity")
        if has_extras:
            layout.prop(
                user_preferences,
                "enable_draw_constraints",
                toggle=True,
                text="Constraints",
                icon="CONSTRAINT",
            )


class VIEW3D_PT_final_topology_settings(Panel):
    bl_category = "Edit"
    bl_idname = "VIEW3D_PT_final_topology_settings"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Settings"
    bl_parent_id = "VIEW3D_PT_final_topology_editmode"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        final_topology_settings_UI_draw(self, context)


# separate UI panel in the side bar
# in category Final Topology
class VIEW3D_PT_final_topology_objectmode(Panel):
    bl_category = "Edit"
    bl_idname = "VIEW3D_PT_final_topology_objectmode"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Final topology"

    @classmethod
    def poll(self, context):
        return (
            bpy.context.mode == "OBJECT"
            and bpy.context.active_object is not None
            and bpy.context.active_object.type == "MESH"
        )

    def draw(self, context):
        layout = self.layout
        user_preferences = bpy.context.preferences.addons[__name__].preferences

        op = layout.operator("wm.url_open", text="Watch tutorial", icon="SEQUENCE")
        op.url = "https://youtu.be/5JWf-B89msU?si=Nt8t7JwvngNI3bN9"

        layout.operator(FinalUnsubdivide.bl_idname, text="Unsubdivide")
        box = layout.box()
        box.label(text="Print Safe (Object Mode):")
        box.prop(user_preferences, "print_safe_axis")
        box.prop(user_preferences, "print_safe_max_inclination")
        box.prop(user_preferences, "print_safe_iterations")
        box.prop(user_preferences, "print_safe_strength")
        box.operator(
            FinalTopologyPrintSafeOperator.bl_idname,
            text="Make Print Safe",
            icon="MOD_SOLIDIFY",
        )


class VIEW3D_PT_final_topology_editmode(Panel):
    bl_category = "Edit"
    bl_idname = "VIEW3D_PT_final_topology_editmode"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Final topology"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(self, context):
        return poll_final_topology(self, context)

    def draw(self, context):
        layout = self.layout
        op = layout.operator("wm.url_open", text="Watch tutorial", icon="SEQUENCE")
        op.url = "https://youtu.be/5JWf-B89msU?si=Nt8t7JwvngNI3bN9"
        final_topology_operators_draw(self, context)


def slide_menu_func(self, context):
    self.layout.operator(
        SlideOptimizeOperator.bl_idname, text=SlideOptimizeOperator.bl_label
    )


def poll_final_topology(self, context):
    if bpy.context.mode != "EDIT_MESH":
        return False
    ob = bpy.context.active_object
    if ob is None:
        return False
    if ob.type != "MESH":
        return False
    return True


def draw_final_topology_toggle(self, context):
    # This can be added to header, but not sure about it's usefulness by now.
    if not poll_final_topology(self, context):
        return

    active_obj = context.active_object
    if active_obj:
        self.layout.prop(
            active_obj.final_topology,
            "enable_operator",
            toggle=True,
            icon="MOD_SUBSURF",
            text="",
        )


classes = [
    FinalTopologyObjectProperties,
    finalTopologyModal,
    FinalTopologyStep,
    FreezeShape,
    FinalUnsubdivide,
    FinalTopologyPrintSafeOperator,
    InverseSubdivideAddonPreferences,
    PopupDialog,
    VIEW3D_PT_final_topology_editmode,
    VIEW3D_PT_final_topology_settings,
    VIEW3D_PT_final_topology_objectmode,
    VIEW3D_PT_final_topology_overlays,
]

addon_keymapitems = []


def register():
    # Regsiter classes
    for cls in classes:
        bpy.utils.register_class(cls)

    bpy.types.Object.final_topology = PointerProperty(type=FinalTopologyObjectProperties)

    bpy.types.VIEW3D_MT_edit_mesh_edges.append(slide_menu_func)

    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new(name="Window", space_type="EMPTY")

    kmi = km.keymap_items.new(
        "mesh.final_topology_modal",
        type="T",
        value="PRESS",
        ctrl=True,
        shift=False,
        alt=True,
    )
    addon_keymapitems.append(kmi)
    kmi = km.keymap_items.new(
        "mesh.final_topology_optimization_step",
        type="O",
        value="PRESS",
        ctrl=True,
        shift=False,
        alt=True,
    )
    addon_keymapitems.append(kmi)
    if has_extras:
        # toggle pinning of the selected vertices (PRO constraints only)
        kmi = km.keymap_items.new(
            "object.final_topology_pin_selection",
            type="P",
            value="PRESS",
            ctrl=False,
            shift=True,
            alt=False,
        )
        addon_keymapitems.append(kmi)
    print("SHORTCUTS REGISTERED")


    if has_extras:
        extras.register()

def unregister():
    # Remove classes
    for cls in classes:
        bpy.utils.unregister_class(cls)

    del bpy.types.Object.final_topology

    bpy.types.VIEW3D_MT_edit_mesh_edges.remove(slide_menu_func)

    if has_extras:
        extras.unregister()

    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps["Window"]

    try:
        for kmi in addon_keymapitems:
            km.keymap_items.remove(kmi)
            addon_keymapitems.clear()
    except:
        print("Seems you removed your keybindings manually.")
