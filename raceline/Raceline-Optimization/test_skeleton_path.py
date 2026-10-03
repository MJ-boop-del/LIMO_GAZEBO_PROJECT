import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize

MAP_PGM = "/root/maps/limo_map.pgm"

# ------------------------------------------------------------
# Map load
# ------------------------------------------------------------

img = np.array(
    Image.open(MAP_PGM).transpose(Image.FLIP_TOP_BOTTOM)
).astype(float)

free = (img > 210).astype(np.uint8)

# ------------------------------------------------------------
# Skeleton
# ------------------------------------------------------------

dist = distance_transform_edt(free)

centers = dist > 0.17 * dist.max()

skeleton = skeletonize(centers)

ys, xs = np.where(skeleton)

print("Skeleton pixels:", len(xs))

# ------------------------------------------------------------
# Build graph
# ------------------------------------------------------------

points = set(zip(xs, ys))

directions = [
    (-1, -1), (0, -1), (1, -1),
    (-1,  0),          (1,  0),
    (-1,  1), (0,  1), (1,  1)
]

graph = {}

for p in points:

    x, y = p

    neighbors = []

    for dx, dy in directions:

        q = (x + dx, y + dy)

        if q in points:
            neighbors.append(q)

    graph[p] = neighbors

# ------------------------------------------------------------
# Find endpoints
# ------------------------------------------------------------

endpoints = [
    p for p in points
    if len(graph[p]) == 1
]

print("Endpoints:", len(endpoints))

for p in endpoints:
    print(" ", p)

# ------------------------------------------------------------
# Find longest endpoint-to-endpoint path
# ------------------------------------------------------------

def find_path(start, goal):

    stack = [(start, [start])]

    visited = set()

    while stack:

        current, path = stack.pop()

        if current == goal:
            return path

        if current in visited:
            continue

        visited.add(current)

        for nxt in graph[current]:

            if nxt not in visited:

                stack.append(
                    (nxt, path + [nxt])
                )

    return None


best_path = None

for i in range(len(endpoints)):

    for j in range(i + 1, len(endpoints)):

        path = find_path(
            endpoints[i],
            endpoints[j]
        )

        if path is not None:

            if best_path is None or len(path) > len(best_path):

                best_path = path

# ------------------------------------------------------------
# Result
# ------------------------------------------------------------

if best_path is None:

    raise RuntimeError("경로를 찾지 못했습니다.")

print()
print("Longest path:")
print("points:", len(best_path))
print("start :", best_path[0])
print("end   :", best_path[-1])

# ------------------------------------------------------------
# Distance statistics
# ------------------------------------------------------------

xy = np.array(best_path, dtype=float)

d = np.linalg.norm(
    np.diff(xy, axis=0),
    axis=1
)

print()
print("Pixel segment statistics:")
print("min :", d.min())
print("max :", d.max())
print("mean:", d.mean())

print()
print("Physical distance:")
print("total length:", np.sum(d) * 0.05, "m")
print("start-end:", np.linalg.norm(xy[0] - xy[-1]) * 0.05, "m")
