import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, convolve
from skimage.morphology import skeletonize

MAP_PGM = "/root/maps/limo_map.pgm"

img = np.array(
    Image.open(MAP_PGM).transpose(Image.FLIP_TOP_BOTTOM)
).astype(float)

free = img > 210

dist = distance_transform_edt(free)
centers = dist > 0.17 * dist.max()
skel = skeletonize(centers)

kernel = np.ones((3,3), dtype=np.uint8)
neighbors = convolve(
    skel.astype(np.uint8),
    kernel,
    mode="constant"
) - skel.astype(np.uint8)

ys, xs = np.where(skel)

print("================================")
print("Skeleton Graph Analysis")
print("================================")
print("Skeleton pixels:", len(xs))

# 각 skeleton pixel의 degree 계산
degrees = neighbors[skel]

for d in range(1, 5):
    print(f"degree {d}: {np.sum(degrees == d)}")

# branch / endpoint 좌표
endpoints = list(zip(xs[degrees == 1], ys[degrees == 1]))
branches = list(zip(xs[degrees >= 3], ys[degrees >= 3]))

print("\nEndpoints:")
for p in endpoints:
    print(p)

print("\nBranches:")
for p in branches:
    print(p)

# 각 branch를 하나의 그룹으로 묶기 위해
# branch 주변의 연결 상태를 출력
print("\n================================")
print("Branch neighborhood")
print("================================")

for x, y in branches:
    local = neighbors[max(0,y-1):y+2, max(0,x-1):x+2]
    print(f"({x},{y}) degree={neighbors[y,x]}")

