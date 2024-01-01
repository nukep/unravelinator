from pxr import Usd, UsdGeom, Gf
import functools

def calculate_skeleton_rest_transforms(prim):
    '''Sets the "restTransforms" attribute of the provided Skeleton primitive.

    Uses the "bindTransforms" attribute of the Skeleton primitive to calculate the "restTransforms" attribute.
    This results in a rest transform that's the same pose at the bind transform.
    '''

    # the 'joints' attribute denotes the joint hierarchy.
    # e.g. ['jnt1', 'jnt1/jnt2', 'jnt1/jnt2/jnt3']
    joints = prim.GetAttribute('joints').Get()
    joints = [x.split('/') for x in joints]
    joint_names = [x[-1] for x in joints]
    parents = [x[-2] if len(x) > 1 else None for x in joints]

    bind_transforms = prim.GetAttribute('bindTransforms').Get()
    joint_to_bind_transform = {j:t for j,t in zip(joint_names, bind_transforms)}

    identity = Gf.Matrix4d()
    inv_parent_tfs = [joint_to_bind_transform[jnt].GetInverse() if jnt else identity
                      for jnt in parents]

    # the rest transforms are joint-relative, so all we need to calculate these
    # are the inverse parent matrix and the joint matrix
    rest_transforms = [
        tf * inv_parent
        for tf,inv_parent in zip(bind_transforms, inv_parent_tfs)
    ]

    prim.GetAttribute('restTransforms').Set(rest_transforms)


def detach_all_faces(mesh):
    '''Detach all faces in the mesh by denormalizing the points.
    This is done by making all faceVertexIndices values unique.'''
    # faceVertexIndices = [0, 1, 2, ..., n]
    # points = [...]

    srcindices = mesh.GetFaceVertexIndicesAttr().Get()
    srcpoints = mesh.GetPointsAttr().Get()

    indices = list(range(len(srcindices)))
    points = [srcpoints[srcindices[i]] for i in range(len(srcindices))]

    # TODO: denormalize primvars with interpolation = "vertex" or "varying"

    mesh.GetFaceVertexIndicesAttr().Set(indices)
    mesh.GetPointsAttr().Set(points)


class MeshCache:
    def __init__(self, mesh: UsdGeom.Mesh):
        self._mesh = mesh
    
    def eager(self):
        '''Calculate everything that directly depends on the mesh now instead of on-demand'''
        self.calc_grouped_vertex_indices()

    @functools.cache
    def calc_grouped_vertex_indices(self):
        f = []
        a = list(self._mesh.GetFaceVertexIndicesAttr().Get())
        for c in self._mesh.GetFaceVertexCountsAttr().Get():
            x = a[:c]
            f.append(x)
            a = a[c:]
        return f
    
    def point(self, point_index):
        return self._mesh.GetPointsAttr().Get()[point_index]

    def face_edges(self, face):
        f = self.calc_grouped_vertex_indices()[face]
        # one point to the next, then it wraps back to the beginning
        return [tuple(x) for x in (list(zip(f, f[1:])) + [[f[-1], f[0]]])]
    
    def face_points(self, face):
        f = self.calc_grouped_vertex_indices()[face]
        return [self.point(i) for i in f]

    def num_faces(self):
        return len(self.calc_grouped_vertex_indices())
    
    @functools.cache
    def calc_edge_faces_lookup(self):
        # Calculate grouped vertex indices

        # Calculate an edge to faces lookup
        # an edge is a 2-tuple (point1, point2). the points are in sorted order.
        lookup = {}

        for i in range(self.num_faces()):
            edges = self.face_edges(i)
            for edge in edges:
                edge = tuple(sorted(edge))
                l = lookup.get(edge, [])
                l.append(i)
                lookup[edge] = l
        
        return lookup

    def connected_faces(self, face):
        edges = self.face_edges(face)
        l = self.calc_edge_faces_lookup()

        result = set()
        for edge in edges:
            edge = tuple(sorted(edge))
            f = l.get(edge, None)
            if f is not None:
                for ff in f:
                    result.add(ff)
        result.remove(face)
        return result