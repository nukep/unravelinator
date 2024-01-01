from collections import namedtuple
Tree = namedtuple('Tree', ['value', 'children'])


def alternate_list(l, n):
    if n % 2 == 0: return l
    else: return l[::-1]

def rotate_list_right(l, n):
    i = (-n) % len(l)
    return l[i:] + l[:i]
def torus_face_order(axisnum, heightnum, alternate=False):
    fn = alternate_list if alternate else rotate_list_right
    return [i*axisnum + j
            for i in range(heightnum)
            for j in fn(list(range(axisnum)), i)]
def torus_face_order2(axisnum, heightnum, alternate=False):
    fn = alternate_list if alternate else rotate_list_right
    return [i + j*axisnum
            for i in range(axisnum)
            for j in fn(list(range(heightnum)), i)]
def spiral_order(w, h, direction=0, clockwise=False, reverse=False, at=None):
    orig_w = w
    if at is None:
        at = lambda x,y: x + y*orig_w

    l = []

    if direction == 0:
        # top-right
        x = w
        y = 0 if clockwise else h-1
    elif direction == 1:
        # top-left
        x = w-1 if clockwise else 0
        y = h
    elif direction == 2:
        # bottom-left
        x = -1
        y = h-1 if clockwise else 0
    elif direction == 3:
        # bottom-right
        x = 0 if clockwise else w-1
        y = -1

    def add(ox, oy):
        nonlocal x,y
        x += ox
        y += oy
        l.append(at(x,y))

    while True:
        if direction == 0:
            # left
            if w <= 0: break
            for _ in range(w): add(-1,0)
            h -= 1
        elif direction == 1:
            # down
            if h <= 0: break
            for _ in range(h): add(0,-1)
            w -= 1
        elif direction == 2:
            # right
            if w <= 0: break
            for _ in range(w): add(1,0)
            h -= 1
        elif direction == 3:
            # up
            if h <= 0: break
            for _ in range(h): add(0,1)
            w -= 1

        if clockwise: direction = (direction-1)%4
        else: direction = (direction+1)%4
    if reverse:
        return l[::-1]
    else:
        return l

def face_sequence_to_tree(faces):
    # Basically making a cons list!
    tree = None
    for face in reversed(faces):
        tree = Tree(face, [tree] if tree else [])
    return tree

def face_adjlist_to_tree(adjlist, startat):
    children = adjlist.get(startat, [])
    return Tree(startat, [face_adjlist_to_tree(adjlist, i) for i in children])
