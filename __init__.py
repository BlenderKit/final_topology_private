bl_info = {
    "name": "Final Topology - Inverse subdivide",
    "author": "Vilem Duha, BlenderKit",
    "version": (1, 0),
    "blender": (3, 6, 0),
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
    extras = reload(extras)
    try:
        extras = reload(extras)
    except Exception as e:
        has_extras = False
        print(e)

    inverse_subdivide = reload(inverse_subdivide)
    draw = reload(draw)
    utils = reload(utils)
else:
    from .inverse_subdivide import *

    from . import extras

    try:
        from . import extras
        from .extras import *
    except Exception as e:
        has_extras = False
        print(e)

    from . import inverse_subdivide
    from . import draw
    from . import utils

import bpy
from bpy.props import (
    BoolProperty,
    EnumProperty,
    FloatProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import AddonPreferences, Operator, Panel


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

    enable_operator: BoolProperty(
        name="Inverse-Subsurf",
        default=False,
        description="Toggle to enable Inverse-Subsurf",
        # update=update_enable_operator
    )

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
    # Enum property
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


def inverse_subdivide_UI_draw(self, context):
    # Draw UI elements
    if not poll_inverse_subdivide(self, context):
        return

    user_preferences = bpy.context.preferences.addons[__name__].preferences
    layout = self.layout

    layout.operator(
        InverseSubdivideStep.bl_idname,
        text="Inverse Subdivide Step",
        icon="TRACKING_FORWARDS_SINGLE",
    )

    if user_preferences.enable_operator:
        layout.operator(
            InverseSubdivideModal.bl_idname,
            text="Inverse Subdsurf Modal",
            icon="MOD_SUBSURF",
            emboss=True,
            depress=True,
        )
    else:
        layout.operator(
            InverseSubdivideModal.bl_idname,
            text="Inverse Subdsurf Modal",
            icon="MOD_SUBSURF",
            emboss=True,
            depress=False,
        )

    layout = layout.column()
    # don't hide so it's actually possible to tweak the settings while off

    layout.prop(user_preferences, "always_on", toggle=True, icon="MOD_TIME")
    if user_preferences.always_on:
        layout.prop(user_preferences, "use_timer", toggle=False, icon="TIME")

    layout.separator()
    layout.label(text="Snap to")
    if bpy.data.objects.get("FROZEN_MESH_STATE") is not None:
        layout.operator(
            FreezeShape.bl_idname, text="Unfreeze shape", depress=True, icon="FREEZE"
        )

    else:
        layout.operator(
            FreezeShape.bl_idname, text="Freeze Shape", depress=False, icon="FREEZE"
        )
        row = layout.row()
        row.prop(user_preferences, "use_object_or_collection", text="")
        if user_preferences.use_object_or_collection == "OBJECT":
            # Layout for target object selection using prop_search
            row.prop(bpy.context.scene, "inverse_subdivide_target_object", text="")
        elif user_preferences.use_object_or_collection == "COLLECTION":
            # Layout for target collection selection using prop_search
            row.prop(bpy.context.scene, "inverse_subdivide_target_collection", text="")
        else:
            # Scene option, no need to display anything
            pass

    if has_extras:
        layout.separator()
        layout.prop(user_preferences, "iterations")
        layout.prop(user_preferences, "neighbours")
    layout.prop(user_preferences, "max_distance")
    if has_extras:
        layout.prop(user_preferences, "weight_algorithm")


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


class VIEW3D_PT_final_topology_inverse_subdivide(Panel):
    bl_category = "Edit"
    bl_idname = "VIEW3D_PT_final_topology_inverse_subdivide"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Inverse Subdivide"
    bl_parent_id = "VIEW3D_PT_final_topology_editmode"
    bl_options = {"DEFAULT_CLOSED"}

    def draw(self, context):
        layout = self.layout
        inverse_subdivide_UI_draw(self, context)


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
        layout.operator()
        op = layout.operator(
            "bpy.ops.wm.url_open", text="Watch tutorial", icon="SEQUENCE"
        )
        op.url = "https://youtu.be/5JWf-B89msU?si=Nt8t7JwvngNI3bN9"

        layout.operator(FinalUnsubdivide.bl_idname, text="Unsubdivide")


class VIEW3D_PT_final_topology_editmode(Panel):
    bl_category = "Edit"
    bl_idname = "VIEW3D_PT_final_topology_editmode"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Final topology"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(self, context):
        return poll_inverse_subdivide(self, context)

    def draw(self, context):
        layout = self.layout
        op = layout.operator(
            "bpy.ops.wm.url_open", text="Watch tutorial", icon="SEQUENCE"
        )
        op.url = "https://youtu.be/5JWf-B89msU?si=Nt8t7JwvngNI3bN9"


def slide_menu_func(self, context):
    self.layout.operator(
        SlideOptimizeOperator.bl_idname, text=SlideOptimizeOperator.bl_label
    )


def poll_inverse_subdivide(self, context):
    if bpy.context.mode != "EDIT_MESH":
        return False
    ob = bpy.context.active_object
    if ob is None:
        return False
    if ob.type != "MESH":
        return False
    return True


def draw_inverse_subdivide_toggle(self, context):
    if not poll_inverse_subdivide(self, context):
        return

    self.layout.prop(
        context.preferences.addons[__name__].preferences,
        "enable_operator",
        toggle=True,
        icon="MOD_SUBSURF",
        text="",
    )


classes = [
    InverseSubdivideModal,
    InverseSubdivideStep,
    FreezeShape,
    FinalUnsubdivide,
    InverseSubdivideAddonPreferences,
    PopupDialog,
    VIEW3D_PT_final_topology_editmode,
    VIEW3D_PT_final_topology_inverse_subdivide,
    VIEW3D_PT_final_topology_objectmode,
    VIEW3D_PT_final_topology_overlays,
]

addon_keymapitems = []


def register():
    # Regsiter classes
    for cls in classes:
        bpy.utils.register_class(cls)
    # Add UI elements
    # bpy.types.VIEW3D_PT_snapping.append(inverse_subdivide_UI_draw)
    bpy.types.Scene.inverse_subdivide_target_object = PointerProperty(
        type=bpy.types.Object, name="Target Object"
    )
    bpy.types.Scene.inverse_subdivide_target_collection = PointerProperty(
        type=bpy.types.Collection, name="Target Collection"
    )
    user_preferences = bpy.context.preferences.addons[__name__].preferences
    user_preferences.enable_operator = False
    bpy.types.VIEW3D_MT_edit_mesh_edges.append(slide_menu_func)
    # bpy.types.VIEW3D_HT_header.append(draw_inverse_subdivide_toggle)

    if has_extras:
        extras.register()
    # Add shortcuts
    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new(name="Window", space_type="VIEW_3D")

    kmi = km.keymap_items.new(
        "mesh.inverse_subdivide_modal",
        type="FIVE",
        value="PRESS",
        ctrl=False,
        shift=False,
        alt=False,
    )
    addon_keymapitems.append(kmi)
    kmi = km.keymap_items.new(
        "mesh.final_topology_optimization_step",
        type="FOUR",
        value="PRESS",
        ctrl=False,
        shift=False,
        alt=False,
    )
    print("SHORTCUTS REGISTERED")
    addon_keymapitems.append(kmi)


def unregister():
    # Remove classes
    for cls in classes:
        bpy.utils.unregister_class(cls)

    # Remove UI elements
    # bpy.types.VIEW3D_PT_snapping.remove(inverse_subdivide_UI_draw)
    bpy.types.VIEW3D_MT_edit_mesh_edges.remove(slide_menu_func)
    # bpy.types.VIEW3D_HT_header.remove(draw_inverse_subdivide_toggle)
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
