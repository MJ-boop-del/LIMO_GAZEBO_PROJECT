import pandas as pd
import numpy as np
import networkx as nx

df = pd.read_csv("core_skeleton.csv")
points = [tuple(p) for p in df[["x","y"]].to_numpy()]

G = nx.Graph()
G.add_nodes_from(points)

dirs = [
    (-1,-1),(0,-1),(1,-1),
    (-1,0),(1,0),
    (-1,1),(0,1),(1,1)
]

point_set = set(points)

for x, y in points:
    for dx, dy in dirs:
        q = (x+dx, y+dy)
        if q in point_set:
            G.add_edge((x,y), q, weight=np.hypot(dx,dy))

print("================================")
print("Cycle analysis")
print("================================")

print("Nodes:", G.number_of_nodes())
print("Edges:", G.number_of_edges())

cycles = nx.cycle_basis(G)

print("Number of cycles:", len(cycles))

if len(cycles) == 0:
    print("No cycle found.")
else:
    cycles = sorted(cycles, key=len, reverse=True)

    print("\nLargest cycles:")

    for i, cycle in enumerate(cycles[:10]):
        length = 0.0

        for j in range(len(cycle)):
            p1 = np.array(cycle[j])
            p2 = np.array(cycle[(j+1) % len(cycle)])
            length += np.linalg.norm(p2-p1)

        print(
            f"{i+1}: "
            f"points={len(cycle)}, "
            f"length={length:.2f} px, "
            f"{length*0.05:.2f} m"
        )

    # 가장 긴 cycle 저장
    best = cycles[0]

    out = pd.DataFrame(best, columns=["x","y"])
    out.to_csv("cycle_candidate.csv", index=False)

    print("\nSaved: cycle_candidate.csv")
