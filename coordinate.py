import math

WGS84_a = 6378137
WGS84_e2 = 0.0066943799901413
PI = math.pi


def BLH2XYZ(lat, lng, h):
    e2 = WGS84_e2
    N = WGS84_a / math.sqrt(1 - e2 * math.sin(lat / 180.0 * PI) * math.sin(lat / 180.0 * PI))
    x = (N + h) * math.cos(lat / 180.0 * PI) * math.cos(lng / 180.0 * PI)
    y = (N + h) * math.cos(lat / 180.0 * PI) * math.sin(lng / 180.0 * PI)
    z = (N * (1 - e2) + h) * math.sin(lat / 180.0 * PI)
    return [x, y, z]


def XYZ2ENU(xyzCenter, Sat, lat, lng, h):
    dx = Sat[0] - xyzCenter[0]
    dy = Sat[1] - xyzCenter[1]
    dz = Sat[2] - xyzCenter[2]
    b = lat / 180.0 * PI
    l = lng / 180.0 * PI
    N = -math.sin(b) * math.cos(l) * dx - math.sin(b) * math.sin(l) * dy + math.cos(b) * dz
    E = -math.sin(l) * dx + math.cos(l) * dy
    U = math.cos(b) * math.cos(l) * dx + math.cos(b) * math.sin(l) * dy + math.sin(b) * dz
    return [E, N, U]


def calc_dir(points):
    pt_begin = points[0]
    pt_end = points[len(points)-1]
    xyzCenter = BLH2XYZ(pt_begin[0], pt_begin[1], pt_begin[2])
    xyzRover = BLH2XYZ(pt_end[0], pt_end[1], pt_end[2])
    ENU = XYZ2ENU(xyzCenter, xyzRover, pt_begin[0], pt_begin[1], pt_begin[2])
    return ENU

