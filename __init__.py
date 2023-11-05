bl_info = {
    "name": "Final Topology - Inverse subdivide",
    "author": "Vilem Duha, BlenderKit",
    "version": (1, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Snapping > Inverse-subdivide ",
    "description": "Compensates subdivide during modelling, simply magic.",
    "warning": "",
    "doc_url": "",
    "category": "3D View",
}

from .inverse_subdivide import *
has_extras = True
try:
    from .extras import *
except:
    has_extras = False

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
        name="Sim mode",
        default=False,
        description="Use timer with the always on option",
    )

    iterations: IntProperty(
        name="Iterations",
        default=1,
        min=1, max=100,
        description="Number of iterations.\n Higher numbers are more precise and slower"
    )

    neighbours: IntProperty(
        name="Neighbours levels",
        default=1,
        min=0, max=10,
        description="Number of surrounding faces to which the snapping happens too.\n"
                    "This is useful since subdivided surface \n"
                    "always gets influence from the surrounding faces"
    )
    max_distance: FloatProperty(
        name="Max Distance",
        default=.02,
        min=0, max=1,
        description="Maximum distance to move vertices. \n If distance is higher, vertices stay in place",
        precision=10,
        unit='LENGTH'
    )

    gradient_sensitivity_distance: FloatProperty(
        name="Gradient Sensitivity",
        default=.02,
        min=0, max=1,
        description="Color gradient sensitivity, anything over this distance will be strictly red",
        precision=10,
        unit='LENGTH'
    )

    arrow_scale: FloatProperty(
        name="Arrow scale",
        default=10,
        min=0.1, max=100,
        description="Arrow scale - multiplier of the distance by which the vertex was moved in the last iteration",
        precision=1,
    )

    enable_draw_faces: BoolProperty(
        name="Draw Faces",
        default=False,
        description="Toggle to enable or disable drawing of faces"
    )
    enable_draw_arrows: BoolProperty(
        name="Draw Arrows",
        default=True,
        description="Toggle to enable or disable drawing of arrows"
    )
    enable_draw_arrows: BoolProperty(
        name="Draw Arrows",
        default=True,
        description="Toggle to enable or disable drawing of arrows"
    )
    # Enum property
    use_object_or_collection: EnumProperty(
        name="Use Object or Collection",
        items=[
            ('SCENE', "Scene", "Snap to all objects in scene"),
            ('OBJECT', "Object", "Use the object as the target"),
            ('COLLECTION', "Collection", "Use the collection as the target"),
        ],
        default='SCENE',
        description="Choose whether to use an object or a collection as the target."
    )

    weight_algorithm: EnumProperty(
        name="Weight Algorithm",
        items=[
            ('ALL1', "All 1", "All considered verts have weight 1"),
            ('FIRST', "First 1 ", "Main vert has 1, others split 1"),
            ('FIRSTONLY', "Only first", "Main vert has 1, no midverts counted"),
            ('FIRSTDIST', "First has 1 + distance", "Main vert has 1, others split 1 with distance weight"),
        ],
        default='FIRSTONLY',
        description="Choose whether to use distance or edge length for weighting."
    )
    normal_direction: EnumProperty(
        name="Normal Direction",
        items=[
            ('ORIGINAL', "Normal", "Use vertex normal"),
            ('SUBDIVIDED', "Subdivided", "Use subdivided face normal"),
        ],
        default='SUBDIVIDED',
        description="choose which normal is used for the offset."
    )

def inverse_subdivide_UI_draw(self, context):
    if not poll_inverse_subdivide(self, context):
        return

    user_preferences = bpy.context.preferences.addons[__name__].preferences
    layout = self.layout
    layout.separator()
    layout.separator()

    if user_preferences.enable_operator:
        layout.operator(InverseSubdivideModal.bl_idname, text="Inverse Subdsurf Modal", icon='MOD_SUBSURF', emboss=True,
                        depress=True)
    else:
        layout.operator(InverseSubdivideModal.bl_idname, text="Inverse Subdsurf Modal", icon='MOD_SUBSURF', emboss=True,
                        depress=False)

    layout = layout.column()
    # don't hide so it's actually possible to tweak the settings while off

    layout.prop(user_preferences, "always_on", toggle=True, icon='MOD_TIME')
    if user_preferences.always_on:
        layout.prop(user_preferences, "use_timer", toggle=False, icon='TIME')
    if has_extras:
        layout.separator()
        layout.label(text='Snap to')
        if bpy.data.objects.get("FROZEN_MESH_STATE") is not None:
            layout.operator(FreezeShape.bl_idname, text="Unfreeze shape", depress=True, icon='FREEZE')

        else:
            layout.operator(FreezeShape.bl_idname, text="Freeze Shape", depress=False, icon='FREEZE')
            row = layout.row()
            row.prop(user_preferences, "use_object_or_collection", text="")
            if user_preferences.use_object_or_collection == 'OBJECT':
                # Layout for target object selection using prop_search
                row.prop(bpy.context.scene, "inverse_subdivide_target_object", text="")
            elif user_preferences.use_object_or_collection == 'COLLECTION':
                # Layout for target collection selection using prop_search
                row.prop(bpy.context.scene, "inverse_subdivide_target_collection", text="")
            else:
                # Scene option, no need to display anything
                pass


        layout.separator()
        layout.prop(user_preferences, "iterations")
        layout.prop(user_preferences, "neighbours")
    layout.prop(user_preferences, "max_distance")

    layout.separator()
    layout.label(text="Drawing Overlays:")
    layout.prop(user_preferences, "enable_draw_arrows")
    if user_preferences.enable_draw_arrows:
        layout.prop(user_preferences, "arrow_scale")
    layout.prop(user_preferences, "enable_draw_faces")
    layout.prop(user_preferences, "gradient_sensitivity_distance")
    layout.prop(user_preferences, "weight_algorithm")


# separate UI panel in the side bar
# in category Final Topology
class VIEW3D_PT_final_topology_objectmode(Panel):
    bl_category = "Final topology"
    bl_idname = "VIEW3D_PT_final_topology_objectmode"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_label = "Final topology"
    bl_options = {"DEFAULT_CLOSED"}

    @classmethod
    def poll(self, context):
        return bpy.context.mode == 'OBJECT' and bpy.context.active_object is not None and bpy.context.active_object.type == 'MESH'

    def draw(self, context):
        layout = self.layout
        layout.operator(FinalUnsubdivide.bl_idname, text="Unsubdivide")


class VIEW3D_PT_final_topology_editmode(Panel):
    bl_category = "Final topology"
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
        if has_extras:
            layout.operator(FlattenSelectionOperator.bl_idname, text="Flatten Selection")
            layout.operator(NormalLoopAlign.bl_idname, text="Loop Align to Normal Plane")
            layout.operator(SlideOptimizeOperator.bl_idname, text="Loop Slide Optimize")


        layout.operator(InverseSubdivideStep.bl_idname, text="Inverse Subdivide Step", icon='MOD_SUBSURF')

        inverse_subdivide_UI_draw(self, context)


def slide_menu_func(self, context):
    self.layout.operator(SlideOptimizeOperator.bl_idname, text=SlideOptimizeOperator.bl_label)


def poll_inverse_subdivide(self, context):
    if bpy.context.mode != 'EDIT_MESH':
        return False
    ob = bpy.context.active_object
    if ob is None:
        return False
    if ob.type != 'MESH':
        return False
    return True


def draw_inverse_subdivide_toggle(self, context):
    if not poll_inverse_subdivide(self, context):
        return

    self.layout.prop(context.preferences.addons[__name__].preferences, "enable_operator", toggle=True,
                     icon='MOD_SUBSURF', text='')


classes = [InverseSubdivideModal,
           InverseSubdivideStep,
           FinalUnsubdivide,
           InverseSubdivideAddonPreferences,
           PopupDialog,
           VIEW3D_PT_final_topology_editmode,
           VIEW3D_PT_final_topology_objectmode,

           ]

#extra tools from the advanced version
if has_extras:
    classes.extend(
        [
            SlideOptimizeOperator,
            # FunTopologyOperator,
            FlattenSelectionOperator,
            NormalLoopAlign,
            FunTopologyDecimateOperator,
            FreezeShape
        ]
    )

addon_keymapitems = []


def register():
    # Regsiter classes
    for cls in classes:
        bpy.utils.register_class(cls)
    # Add UI elements
    # bpy.types.VIEW3D_PT_snapping.append(inverse_subdivide_UI_draw)
    bpy.types.Scene.inverse_subdivide_target_object = PointerProperty(type=bpy.types.Object, name="Target Object")
    bpy.types.Scene.inverse_subdivide_target_collection = PointerProperty(type=bpy.types.Collection,
                                                                          name="Target Collection")
    user_preferences = bpy.context.preferences.addons[__name__].preferences
    user_preferences.enable_operator = False
    bpy.types.VIEW3D_MT_edit_mesh_edges.append(slide_menu_func)
    # bpy.types.VIEW3D_HT_header.append(draw_inverse_subdivide_toggle)

    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new(name="Window", space_type="VIEW_3D")
    
    kmi = km.keymap_items.new(
        "mesh.inverse_subdivide_modal", type='FIVE', value='PRESS', ctrl=False, shift=False, alt=False
    )
    addon_keymapitems.append(kmi)
    kmi = km.keymap_items.new(
        "mesh.inverse_subdivide_step", type='FOUR', value='PRESS', ctrl=False, shift=False, alt=False
    )
    addon_keymapitems.append(kmi)


def unregister():
    # Remove classes
    for cls in classes:
        bpy.utils.unregister_class(cls)

    # Remove UI elements
    # bpy.types.VIEW3D_PT_snapping.remove(inverse_subdivide_UI_draw)
    bpy.types.VIEW3D_MT_edit_mesh_edges.remove(slide_menu_func)
    # bpy.types.VIEW3D_HT_header.remove(draw_inverse_subdivide_toggle)

    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps["Window"]
    for kmi in addon_keymapitems:
        km.keymap_items.remove(kmi)
        addon_keymapitems.clear()
