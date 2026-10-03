import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from skimage.morphology import skeletonize
from scipy.ndimage import convolve

MAP_PGM = "/root/maps/limo_map.pgm"

img = np.array(
    Image.open(MAP_PGM).transpose(Image.FLIP_TOP_BOTTOM)
).astype(float)

free = img > 210

dist = distance_transform_edt(free)

# 기존과 동일
centers = dist > 0.17 * dist.max()

skel = skeletonize(centers)

print("Skeleton pixels:", np.sum(skel))

# ------------------------------------------------------------
# 각 skeleton pixel의 degree 계산
# ------------------------------------------------------------

kernel = np.ones((3, 3), dtype=np.uint8)

neighbors = convolve(
    skel.astype(np.uint8),
    kernel,
    mode="constant"
)

degree = neighbors - skel.astype(np.uint8)

# degree >= 2인 core만 사용
core = skel & (degree >= 2)

print("Core pixels:", np.sum(core))

# ------------------------------------------------------------
# Core의 좌표 출력
# ------------------------------------------------------------

ys, xs = np.where(core)

print()
print("Core bounding box:")
print("x:", xs.min(), "~", xs.max())
print("y:", ys.min(), "~", ys.max())

# ------------------------------------------------------------
# Core를 CSV로 저장
# ------------------------------------------------------------

core_points = np.column_stack((xs, ys))

np.savetxt(
    "core_skeleton.csv",
    core_points,
    fmt="%d",
    delimiter=",",
    header="x,y",
    comments=""
)

print()
print("Saved: core_skeleton.csv")
