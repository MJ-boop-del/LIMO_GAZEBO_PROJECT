import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize
from collections import deque

# ============================================================
# MAP
# ============================================================

MAP_PGM = "/root/maps/limo_map.pgm"

img = np.array(
    Image.open(MAP_PGM).transpose(Image.FLIP_TOP_BOTTOM)
).astype(float)

free = img > 210

# ============================================================
# SKELETON
# ============================================================

dist = distance_transform_edt(free)

centers = dist > 0.17 * dist.max()

skeleton = skeletonize(centers)

ys, xs = np.where(skeleton)

print("Skeleton pixels:", len(xs))

# ============================================================
# GRAPH 생성
# ============================================================

pixels = set(zip(xs, ys))

directions = [
    (-1,-1), (0,-1), (1,-1),
    (-1, 0),          (1, 0),
    (-1, 1),  (0, 1), (1, 1)
]

graph = {}

for p in pixels:

    x, y = p

    neighbors = []

    for dx, dy in directions:

        q = (x + dx, y + dy)

        if q in pixels:
            neighbors.append(q)

    graph[p] = neighbors

# ============================================================
# ENDPOINT / BRANCH 확인
# ============================================================

endpoints = [
    p for p in pixels
    if len(graph[p]) == 1
]

branches = [
    p for p in pixels
    if len(graph[p]) >= 3
]

print("Endpoints:", len(endpoints))
print("Branches:", len(branches))

print()
print("Endpoints:")

for p in endpoints:
    print(p)

# ============================================================
# 각 endpoint에서 다른 endpoint까지 최단 경로 계산
# ============================================================

def bfs(start):

    queue = deque([start])

    parent = {start: None}
    distance = {start: 0}

    while queue:

        current = queue.popleft()

        for nxt in graph[current]:

            if nxt not in parent:

                parent[nxt] = current
                distance[nxt] = distance[current] + 1

                queue.append(nxt)

    return parent, distance


best = None

for start in endpoints:

    parent, distance = bfs(start)

    for end in endpoints:

        if start == end:
            continue

        if end not in distance:
            continue

        length = distance[end]

        if best is None or length > best[0]:

            best = (
                length,
                start,
                end,
                parent
            )

# ============================================================
# 결과
# ============================================================

if best is None:
    raise RuntimeError("Endpoint path를 찾지 못했습니다.")

length, start, end, parent = best

path = []

current = end

while current is not None:

    path.append(current)
    current = parent[current]

path.reverse()

print()
print("================================")
print("Longest endpoint path")
print("================================")

print("points:", len(path))
print("start :", start)
print("end   :", end)

# ============================================================
# 실제 거리 계산
# ============================================================

xy = np.array(path, dtype=float)

segments = np.linalg.norm(
    np.diff(xy, axis=0),
    axis=1
)

print()
print("Pixel segment:")
print("min :", segments.min())
print("max :", segments.max())
print("mean:", segments.mean())

resolution = 0.05

print()
print("Physical length:")
print("length:", segments.sum() * resolution, "m")

print(
    "start-end:",
    np.linalg.norm(xy[0] - xy[-1]) * resolution,
    "m"
)

# ============================================================
# CSV 저장
# ============================================================

out = np.zeros((len(path), 2))

out[:, 0] = xy[:, 0] * resolution - 11.7
out[:, 1] = xy[:, 1] * resolution - 1.45

np.savetxt(
    "test_longest_path.csv",
    out,
    delimiter=",",
    header="x_m,y_m",
    comments=""
)

print()
print("Saved: test_longest_path.csv")
