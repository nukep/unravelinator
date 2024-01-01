# The code's a bit messy right now. It's okay, you can heckle me.

from pxr import Usd, UsdGeom, UsdSkel, Sdf, Gf
import numpy as np
from scipy.spatial.transform import Rotation as R

import usdutil
import treealgo


OUTPUT_FILE = './out.usda'
FRAME_TO = 200

stage = Usd.Stage.CreateNew(OUTPUT_FILE)

# Set up some boilerplate
stage.GetRootLayer().ImportFromString('''#usda 1.0
(
    startTimeCode = 1
    endTimeCode = 200
)

def SkelRoot "skelroot" {
    def "mesh" (
        references = @./torus-8x8.usdc@
        prepend apiSchemas = ["SkelBindingAPI"]
    ) {
        int[] primvars:skel:jointIndices (
            elementSize = 1
            interpolation = "vertex"
        )

        float[] primvars:skel:jointWeights (
            elementSize = 1
            interpolation = "vertex"
        )

        uniform token[] skel:joints
        rel skel:skeleton = </skelroot/skel>
    }

    def Skeleton "skel" (
        prepend apiSchemas = ["SkelBindingAPI"]
    ) {
        rel skel:animationSource = </skelroot/skel/anim>
        def SkelAnimation "anim" {

        }
    }
}
''')

faceorder = treealgo.torus_face_order2(8, 8, alternate=False)


skelprim = stage.GetPrimAtPath('/skelroot/skel')
skel = UsdSkel.Skeleton(skelprim)
skelanimprim = skelprim.GetPrimAtPath('anim')
skelanim = UsdSkel.Animation(skelanimprim)

meshprim = stage.GetPrimAtPath('/skelroot/mesh')
mesh = UsdGeom.Mesh(meshprim)
meshcache = usdutil.MeshCache(mesh)
meshcache.eager()


###
### Define some math functions to help us solve the joint transforms ###
###


def calculate_abc(cache, f0, f1):
    '''
    Returns the tuple (a,b,c) - where a,b,c are points that follow the structure:

    _ -- _
    | f0 |
    a -- b
    | f1 |
    c -- _

    f0 and f1 are faces, and need not be quads as depicted above.

    If f0 and f1 share more than one edge, an arbitrary (undefined) edge is chosen.
    If no edges are shared, return None.
    '''
    f0_edges = cache.face_edges(f0)
    f1_edges = cache.face_edges(f1)
    for i0,i1 in f0_edges:
        for j0,j1 in f1_edges:
            if (i0,i1) == (j1,j0):
                # shared edge
                a = cache.point(i0)
                b = cache.point(i1)
                # find the (a,?) edge in f1, assume the ? is our c value
                for k0,k1 in f1_edges:
                    if k0 == j1:
                        c = cache.point(k1)
                        return a,b,c
    return None

def create_ortho_basis_vectors_helper_lossy_b(a, b):
    # The direction of b is lossy, whereas the direction of a is guaranteed

    ap = a / np.linalg.norm(a)    
    c = np.cross(ap, b)
    cp = c / np.linalg.norm(c)
    bp = np.cross(cp, ap)
    return np.array([ap, bp, cp])

def create_ortho_basis_vectors_xy_lossy_y(x, y):
    return create_ortho_basis_vectors_helper_lossy_b(x, y)

def create_ortho_mat_xy_lossy_y(origin, x, y):
    xx = x - origin
    yy = y - origin

    basis = create_ortho_basis_vectors_xy_lossy_y(xx, yy)
    return np.c_[
        np.r_[basis, [origin]],
        np.array([0,0,0,1])
    ]


###
### Solve the world transforms for each joint
###


transforms = [Gf.Matrix4d()] * len(faceorder)

i = 1
for f0, f1 in zip(faceorder, faceorder[1:]):
    a,b,c = calculate_abc(meshcache, f0, f1)
    mat = create_ortho_mat_xy_lossy_y(origin=a, x=b, y=c)
    transforms[i] = Gf.Matrix4d(mat)
    i += 1

# special transform for the first face, because it has no parent face.
# make it relative to an arbitrary point on itself, not the origin.
f0points = meshcache.face_points(faceorder[0])
transforms[0] = Gf.Matrix4d(create_ortho_mat_xy_lossy_y(x=f0points[0], origin=f0points[1], y=f0points[2]))

# prepend the "root" transform
transforms = [Gf.Matrix4d()] + transforms

skel.GetBindTransformsAttr().Set(transforms)

num_faces = len(faceorder)

joints_in_face_order = [None]*num_faces

# note: joints must be in depth-first order (not necessarily alphebetical)
joints = ['root']
for i in faceorder:
    newjnt = f'jnt{i}'
    if len(joints) > 0:
        newjnt = joints[-1] + '/' + newjnt
    joints.append(newjnt)
    joints_in_face_order[i] = newjnt

def set_quat_x_to_0(q):
    r = R.from_quat((*q.imaginary, q.real)).as_euler('xyz')
    r[0] = 0
    qq = R.from_euler('xyz', r).as_quat()
    return Gf.Quatf(qq[3], qq[0], qq[1], qq[2])

skel.GetJointsAttr().Set(joints)
usdutil.calculate_skeleton_rest_transforms(skelprim)


###
### Author the SkelAnimation ###
###


skelanim.GetJointsAttr().Set(joints)

# extract translations and rotations from the rest transforms
rest_tfs = skel.GetRestTransformsAttr().Get()

scales = [(1,1,1)]*(num_faces+1)
translations = [mat.ExtractTranslation() for mat in rest_tfs]

rotations_rest = [Gf.Quatf(mat.ExtractRotationQuat()) for mat in rest_tfs]
rotations_target = [set_quat_x_to_0(q) for q in rotations_rest]


def clamp(v, lo,hi):
    return max(min(v, hi), lo)
def lerp(a,b,p):
    return (b-a)*p + a

def solve_rotation_at(frame, jointid):
    numjoints = len(rotations_rest)

    num = (numjoints-1)-jointid
    # num = jointid

    startframe = num*2 + 1
    # if jointid == 1:
    #     startframe += 20
    endframe = startframe + 10

    p = (frame-startframe) / (endframe-startframe)
    p = clamp(p, 0, 1)
    p = 1 - p

    return Gf.Slerp(p, rotations_rest[jointid], rotations_target[jointid])

for i in range(FRAME_TO):
    frame = i+1
    rot = [solve_rotation_at(frame, j) for j in range(len(rotations_rest))]
    skelanim.GetRotationsAttr().Set(rot, frame)

skelanim.GetScalesAttr().Set(scales)
skelanim.GetTranslationsAttr().Set(translations)


###
### Author the joint weights, detach the faces ###
###


jointindices = []
jointweights = []
for i,count in enumerate(mesh.GetFaceVertexCountsAttr().Get()):
    jointindices += [i]*count
    jointweights += [1]*count

meshprim.GetAttribute('skel:joints').Set(joints_in_face_order)
meshprim.GetAttribute('primvars:skel:jointIndices').Set(jointindices)
meshprim.GetAttribute('primvars:skel:jointWeights').Set(jointweights)

usdutil.detach_all_faces(mesh)


###
### Done ###
###


stage.Save()
