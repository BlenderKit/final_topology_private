import bmesh


def get_evaluated_bm(obj, depsgraph):
    self_eval = obj.evaluated_get(depsgraph)
    bm_eval = bmesh.new()
    bm_eval.from_mesh(self_eval.data)
    bm_eval.verts.ensure_lookup_table()
    return bm_eval
