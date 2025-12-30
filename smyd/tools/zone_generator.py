#!/usr/bin/env python3
# zone_generator.py
import argparse
import json
from pathlib import Path

import numpy as np

try:
    from scipy.spatial import cKDTree as KDTree
except Exception:
    KDTree = None


def load_points(json_path: str) -> np.ndarray:
    p = Path(json_path)
    data = json.loads(p.read_text(encoding="utf-8"))

    # Case 1) {"X":[...], "Y":[...]}
    if isinstance(data, dict) and "X" in data and "Y" in data:
        x = np.asarray(data["X"], dtype=float)
        y = np.asarray(data["Y"], dtype=float)
        if len(x) != len(y):
            raise ValueError(f"{json_path}: X and Y length mismatch")
        return np.column_stack([x, y])

    # Case 2) {"points":[[x,y], ...]} or {"points":[{"x":..,"y":..}, ...]}
    if isinstance(data, dict) and "points" in data:
        pts = data["points"]
        return _coerce_points(pts, json_path)

    # Case 3) [[x,y], ...] or [{"x":..,"y":..}, ...]
    if isinstance(data, list):
        return _coerce_points(data, json_path)

    raise ValueError(f"{json_path}: unsupported JSON format")


def _coerce_points(obj, src):
    if not obj:
        return np.zeros((0, 2), dtype=float)

    if isinstance(obj[0], (list, tuple)) and len(obj[0]) >= 2:
        arr = np.asarray([[float(p[0]), float(p[1])] for p in obj], dtype=float)
        return arr

    if isinstance(obj[0], dict) and ("x" in obj[0] and "y" in obj[0]):
        arr = np.asarray([[float(p["x"]), float(p["y"])] for p in obj], dtype=float)
        return arr

    raise ValueError(f"{src}: unsupported points array structure")


def mutual_nearest_pairs(p1: np.ndarray, p2: np.ndarray, dist_th: float):
    """
    Return list of (i, j, dist) where i in p1, j in p2 are mutual nearest neighbors within dist_th.
    """
    if len(p1) == 0 or len(p2) == 0:
        return []

    if KDTree is None:
        # Fallback: O(N*M) (use only if paths are small)
        pairs_12 = []
        for i in range(len(p1)):
            dists = np.linalg.norm(p2 - p1[i], axis=1)
            j = int(np.argmin(dists))
            d = float(dists[j])
            if d <= dist_th:
                pairs_12.append((i, j, d))

        # Build reverse nearest
        rev = {}
        for j in range(len(p2)):
            dists = np.linalg.norm(p1 - p2[j], axis=1)
            i = int(np.argmin(dists))
            d = float(dists[i])
            if d <= dist_th:
                rev[j] = (i, d)

        out = []
        for i, j, d in pairs_12:
            if j in rev and rev[j][0] == i:
                out.append((i, j, d))
        return out

    t2 = KDTree(p2)
    d12, j12 = t2.query(p1, k=1)
    mask12 = d12 <= dist_th

    t1 = KDTree(p1)
    d21, i21 = t1.query(p2, k=1)
    mask21 = d21 <= dist_th

    pairs = []
    for i in np.where(mask12)[0]:
        j = int(j12[i])
        if mask21[j] and int(i21[j]) == int(i):
            pairs.append((int(i), j, float(d12[i])))
    return pairs


def cluster_pairs(pairs, max_gap=5):
    """
    pairs: list of (i, j, dist) sorted by i
    clusters: list of list[(i,j,dist)]
    New cluster if i gap or j gap is too large.
    """
    if not pairs:
        return []
    pairs = sorted(pairs, key=lambda x: x[0])
    clusters = []
    cur = [pairs[0]]
    for a, b in zip(pairs, pairs[1:]):
        i0, j0, _ = a
        i1, j1, _ = b
        if (i1 - i0) > max_gap or abs(j1 - j0) > max_gap:
            clusters.append(cur)
            cur = [b]
        else:
            cur.append(b)
    clusters.append(cur)
    return clusters


def make_zone_record(zone_id, cluster, p1, p2, monitor_back, min_segment_len):
    # sort along path1 index
    cluster = sorted(cluster, key=lambda x: x[0])
    i_list = [t[0] for t in cluster]
    j_list = [t[1] for t in cluster]

    c1_s, c1_e = int(min(i_list)), int(max(i_list))
    c2_s, c2_e = int(min(j_list)), int(max(j_list))

    # Determine if it's a "point-like" conflict (intersection point)
    is_point = len(cluster) < min_segment_len or (c1_s == c1_e and c2_s == c2_e)

    m1_s = max(0, c1_s - monitor_back)
    m2_s = max(0, c2_s - monitor_back)

    rec = {
        "zone_id": int(zone_id),
        "kind": "CROSS_POINT" if is_point else "OVERLAP_SEGMENT",
        "path1": {
            "monitor_start_idx": int(m1_s),
            "conflict_start_idx": int(c1_s),
            "conflict_end_idx": None if is_point else int(c1_e),
            "monitor_start_xy": [float(p1[m1_s, 0]), float(p1[m1_s, 1])],
            "conflict_start_xy": [float(p1[c1_s, 0]), float(p1[c1_s, 1])],
            "conflict_end_xy": [float(p1[c1_s, 0]), float(p1[c1_s, 1])] if is_point else [float(p1[c1_e, 0]), float(p1[c1_e, 1])],
        },
        "path2": {
            "monitor_start_idx": int(m2_s),
            "conflict_start_idx": int(c2_s),
            "conflict_end_idx": None if is_point else int(c2_e),
            "monitor_start_xy": [float(p2[m2_s, 0]), float(p2[m2_s, 1])],
            "conflict_start_xy": [float(p2[c2_s, 0]), float(p2[c2_s, 1])],
            "conflict_end_xy": [float(p2[c2_s, 0]), float(p2[c2_s, 1])] if is_point else [float(p2[c2_e, 0]), float(p2[c2_e, 1])],
        },
        # filled later:
        "point_id_start": None,
        "point_id_end": None,
        "pairs_count": int(len(cluster)),
    }
    return rec, cluster


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--path1", required=True, help="waypoints json for path1 (e.g., cav1)")
    ap.add_argument("--path2", required=True, help="waypoints json for path2 (e.g., cav2)")
    ap.add_argument("--out", default="conflict_map.json")
    ap.add_argument("--dist", type=float, default=0.05, help="match distance threshold [m]")
    ap.add_argument("--monitor_back", type=int, default=40, help="monitoring starts this many points before conflict_start")
    ap.add_argument("--min_segment_len", type=int, default=3, help="cluster size < this => treated as point conflict (no in_danger)")
    ap.add_argument("--max_gap", type=int, default=5, help="gap tolerance (index) when clustering matches")
    args = ap.parse_args()

    p1 = load_points(args.path1)
    p2 = load_points(args.path2)

    pairs = mutual_nearest_pairs(p1, p2, args.dist)
    clusters = cluster_pairs(pairs, max_gap=args.max_gap)

    path1_pid = [-1] * len(p1)
    path2_pid = [-1] * len(p2)

    zones = []
    pid = 0

    for zid, cl in enumerate(clusters, start=1):
        rec, cl_sorted = make_zone_record(
            zone_id=zid,
            cluster=cl,
            p1=p1,
            p2=p2,
            monitor_back=args.monitor_back,
            min_segment_len=args.min_segment_len,
        )

        rec["point_id_start"] = int(pid)

        # Assign per-overlap point ids (same id on both paths)
        # Use the sorted list; mutual NN gives 1:1 mapping.
        for (i, j, _) in sorted(cl_sorted, key=lambda x: x[0]):
            path1_pid[i] = pid
            path2_pid[j] = pid
            pid += 1

        rec["point_id_end"] = int(pid - 1)
        zones.append(rec)

    out = {
        "meta": {
            "path1": str(args.path1),
            "path2": str(args.path2),
            "dist_threshold_m": float(args.dist),
            "monitor_back_points": int(args.monitor_back),
            "min_segment_len": int(args.min_segment_len),
            "max_gap": int(args.max_gap),
        },
        "zones": zones,
        "path1_point_id": path1_pid,
        "path2_point_id": path2_pid,
    }

    Path(args.out).write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[OK] zones={len(zones)} total_point_ids={pid} -> {args.out}")


if __name__ == "__main__":
    main()
