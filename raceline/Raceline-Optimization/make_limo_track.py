import numpy as np
import pandas as pd
from PIL import Image
from scipy.ndimage import distance_transform_edt

MAP_PGM = "/root/maps/limo_map.pgm"
MAP_YAML = "/root/maps/limo_map.yaml"

INPUT_CSV = "cycle_candidate.csv"
OUTPUT_CSV = "inputs/tracks/limo_track_closed.csv"

# map 불러오기
img = np.array(
    Image.open(MAP_PGM).transpose(Image.FLIP_TOP_BOTTOM)
).astype(float)

free = img > 210

# distance transform
dist = distance_transform_edt(free)

# cycle centerline
df = pd.read_csv(INPUT_CSV)

xy = df[["x", "y"]].to_numpy()

# map metadata
import yaml

with open(MAP_YAML, "r") as f:
    meta = yaml.safe_load(f)

resolution = meta["resolution"]
origin = meta["origin"]

# pixel -> meter
x_m = xy[:, 0] * resolution + origin[0]
y_m = xy[:, 1] * resolution + origin[1]

# track width
width = dist[xy[:, 1], xy[:, 0]] * resolution

# 최소 폭 확보
width = np.maximum(width, 0.1)

result = np.column_stack([
    x_m,
    y_m,
    width,
    width
])

np.savetxt(
    OUTPUT_CSV,
    result,
    delimiter=",",
    fmt="%.4f",
    header="x_m,y_m,w_tr_right_m,w_tr_left_m",
    comments=""
)

print("================================")
print("LIMO closed track created")
print("================================")
print("Points:", len(result))
print("Output:", OUTPUT_CSV)
print("Track length approx:",
      np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)) * resolution,
      "m")
print("Width min:", width.min(), "m")
print("Width max:", width.max(), "m")
