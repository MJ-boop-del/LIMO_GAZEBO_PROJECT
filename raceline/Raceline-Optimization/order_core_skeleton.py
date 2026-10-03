import numpy as np
import pandas as pd
from collections import defaultdict, deque

# ============================================================
# 1. CORE SKELETON LOAD
# ============================================================

df = pd.read_csv("core_skeleton.csv")
points = [tuple(p) for p in df[["x", "y"]].to_numpy()]

point_set = set(points)

print("Core skeleton points:", len(points))

# ============================================================
# 2. 8방향 GRAPH 생성
# ============================================================

dirs = [
    (-1, -1), (0, -1), (1, -1),
    (-1,  0),          (1,  0),
    (-1,  1), (0,  1), (1, 1)
]

graph = defaultdict(list)

for x, y in points:

    for dx, dy in dirs:

        neighbor = (x + dx, y + dy)

        if neighbor in point_set:
            graph[(x, y)].append(neighbor)

# ============================================================
# 3. Degree 확인
# ============================================================

degree_count = defaultdict(int)

for p in points:
    degree_count[len(graph[p])] += 1

print("\nDegree distribution:")

for degree in sorted(degree_count):
    print(
        f"degree {degree}: "
        f"{degree_count[degree]}"
    )

# ============================================================
# 4. 가장 가까운 두 점 사이 연결 확인
# ============================================================

print("\n================================")
print("Graph connectivity")
print("================================")

visited = set()
components = []

for start in points:

    if start in visited:
        continue

    queue = deque([start])
    visited.add(start)

    component = []

    while queue:

        p = queue.popleft()
        component.append(p)

        for q in graph[p]:

            if q not in visited:
                visited.add(q)
                queue.append(q)

    components.append(component)

components.sort(key=len, reverse=True)

print("Connected components:", len(components))

for i, comp in enumerate(components[:10]):

    print(
        f"{i+1}: {len(comp)} points"
    )

# ============================================================
# 5. GRAPH EDGE 통계
# ============================================================

edges = set()

for p in points:

    for q in graph[p]:

        edge = tuple(sorted([p, q]))

        edges.add(edge)

print("\nGraph edges:", len(edges))

# ============================================================
# 6. 실제 인접 거리 확인
# ============================================================

distances = []

for p, neighbors in graph.items():

    for q in neighbors:

        if p < q:

            distances.append(
                np.linalg.norm(
                    np.array(p) - np.array(q)
                )
            )

print("\nNeighbor distance:")

print("min :", min(distances))
print("max :", max(distances))
print("mean:", np.mean(distances))

# ============================================================
# 7. 저장
# ============================================================

print("\n================================")
print("Graph analysis completed")
print("================================")
