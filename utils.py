import bmesh
import bpy

def get_evaluated_bm(obj, depsgraph):
    self_eval = obj.evaluated_get(depsgraph)
    bm_eval = bmesh.new()
    bm_eval.from_mesh(self_eval.data)
    bm_eval.verts.ensure_lookup_table()
    return bm_eval


# these functions were taken over from looptools (rather for future compatibility ;) )


def activate_object(ob):
    bpy.ops.object.select_all(action='DESELECT')
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
