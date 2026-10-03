import os
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize


# ============================================================
# LIMO MAP 설정
# ============================================================

MAP_PGM = "/root/maps/limo_map.pgm"
MAP_YAML = "/root/maps/limo_map.yaml"

OUTPUT_DIR = "inputs/tracks"
OUTPUT_CSV = os.path.join(OUTPUT_DIR, "limo_track.csv")

# 지도에서 주행 가능 영역으로 판단할 최소 pixel 값
FREE_THRESHOLD = 210

# center 후보 생성 비율
CENTER_THRESHOLD = 0.17


# ============================================================
# 1. MAP LOAD
# ============================================================

print("[1] Loading map...")

img = np.array(
    Image.open(MAP_PGM).transpose(Image.FLIP_TOP_BOTTOM)
)

img = img.astype(np.float64)

height, width = img.shape

print(f"Map size: {width} x {height}")
print(f"Pixel values: {np.unique(img)}")


# ============================================================
# 2. FREE SPACE 추출
# ============================================================

print("\n[2] Extracting free space...")

# 254 = free
# 205 = unknown
#   0 = obstacle

free_space = (img > FREE_THRESHOLD).astype(np.uint8)

print("Free pixels:", np.sum(free_space))


# ============================================================
# 3. DISTANCE TRANSFORM
# ============================================================

print("\n[3] Calculating distance transform...")

dist_transform = distance_transform_edt(free_space)

print("Maximum distance:", dist_transform.max())


# ============================================================
# 4. TRACK CENTER 후보 추출
# ============================================================

print("\n[4] Extracting center candidates...")

threshold = CENTER_THRESHOLD * dist_transform.max()

centers = dist_transform > threshold

print("Center candidate pixels:", np.sum(centers))


# ============================================================
# 5. SKELETONIZE
# ============================================================

print("\n[5] Skeletonizing...")

centerline = skeletonize(centers)

print("Skeleton pixels:", np.sum(centerline))


# ============================================================
# 6. 가장 적절한 시작점 찾기
# ============================================================

print("\n[6] Finding starting point...")

centerline_dist = np.where(
    centerline,
    dist_transform,
    0
)

# 지도 중앙 부근에서 skeleton pixel을 찾음
start_y_min = height // 3
start_y_max = 2 * height // 3

candidates = []

for y in range(start_y_min, start_y_max):
    xs = np.where(centerline_dist[y] > 0)[0]

    if len(xs) > 0:
        for x in xs:
            candidates.append((x, y))

if not candidates:
    raise RuntimeError("Centerline starting point를 찾을 수 없습니다.")

# 왼쪽에 있는 점을 우선 선택
start_x, start_y = min(candidates, key=lambda p: p[0])

print(f"Starting point: x={start_x}, y={start_y}")


# ============================================================
# 7. DFS로 연결된 CENTERLINE 추출
# ============================================================

print("\n[7] Extracting connected centerline...")

visited = set()
centerline_points = []

directions = [
    (0, -1),
    (-1, 0),
    (0, 1),
    (1, 0),
    (-1, -1),
    (-1, 1),
    (1, -1),
    (1, 1)
]


def dfs(x, y):

    if x < 0 or x >= width:
        return

    if y < 0 or y >= height:
        return

    if (x, y) in visited:
        return

    if centerline_dist[y, x] == 0:
        return

    visited.add((x, y))

    centerline_points.append((x, y))

    for dx, dy in directions:
        dfs(x + dx, y + dy)


dfs(start_x, start_y)

print("Connected centerline points:", len(centerline_points))


# ============================================================
# 8. CENTERLINE이 너무 적으면 중단
# ============================================================

if len(centerline_points) < 100:

    raise RuntimeError(
        "추출된 centerline이 너무 짧습니다. "
        "CENTER_THRESHOLD 또는 시작점을 조정해야 합니다."
    )


# ============================================================
# 9. TRACK WIDTH 계산
# ============================================================

print("\n[8] Calculating track widths...")

waypoints = []
track_widths = []

for x, y in centerline_points:

    distance = centerline_dist[y, x]

    waypoints.append([x, y])

    # 좌/우 폭을 동일하게 설정
    track_widths.append([
        distance,
        distance
    ])


waypoints = np.array(waypoints, dtype=np.float64)
track_widths = np.array(track_widths, dtype=np.float64)


# ============================================================
# 10. MAP YAML LOAD
# ============================================================

print("\n[9] Loading map metadata...")

with open(MAP_YAML, "r") as f:
    map_metadata = yaml.safe_load(f)

resolution = map_metadata["resolution"]
origin = map_metadata["origin"]

origin_x = origin[0]
origin_y = origin[1]

print("Resolution:", resolution)
print("Origin:", origin)


# ============================================================
# 11. PIXEL → MAP COORDINATE
# ============================================================

print("\n[10] Converting pixel coordinates to meters...")

data = np.concatenate(
    (waypoints, track_widths),
    axis=1
)

# pixel → meter
data[:, 0] *= resolution
data[:, 1] *= resolution

# map origin 적용
data[:, 0] += origin_x
data[:, 1] += origin_y

# track width도 meter로 변환
data[:, 2] *= resolution
data[:, 3] *= resolution


# ============================================================
# 12. CSV 저장
# ============================================================

print("\n[11] Saving track CSV...")

os.makedirs(OUTPUT_DIR, exist_ok=True)

np.savetxt(
    OUTPUT_CSV,
    data,
    fmt="%.4f",
    delimiter=",",
    header="x_m,y_m,w_tr_right_m,w_tr_left_m",
    comments=""
)

print("\n========================================")
print("Track conversion completed!")
print("========================================")

print("Output:")
print(OUTPUT_CSV)

print("Number of points:", len(data))
print("========================================")
