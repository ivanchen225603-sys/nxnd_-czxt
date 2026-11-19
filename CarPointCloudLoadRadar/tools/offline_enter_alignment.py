"""只读比较真实进场帧与停稳帧的位移；不用于生产体积计算。"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from app.pointcloud import load_xyz  # noqa: E402


def occupancy(xyz: np.ndarray, x_min: float, x_max: float,
              cell: float) -> np.ndarray:
    """把纵向与高度的实测点压成二值格，减小点数密度的影响。"""
    mask = ((xyz[:, 0] >= x_min) & (xyz[:, 0] < x_max) &
            (xyz[:, 1] >= -8.) & (xyz[:, 1] < 7.) &
            (xyz[:, 2] >= -3.) & (xyz[:, 2] < -0.8))
    rows = round(15. / cell)
    cols = round(2.2 / cell)
    y = np.floor((xyz[mask, 1] + 8.) / cell).astype(np.int32)
    z = np.floor((xyz[mask, 2] + 3.) / cell).astype(np.int32)
    counts = np.bincount(y * cols + z, minlength=rows * cols).reshape(rows, cols)
    return counts >= 2


def fit_shift(moving: np.ndarray, settled: np.ndarray, cell: float) -> dict:
    """只平移 y，以观测占用格的 Dice 分数比较。"""
    candidates = []
    rows = moving.shape[0]
    denom = int(np.count_nonzero(moving) + np.count_nonzero(settled))
    for bins in range(-round(5. / cell), round(5. / cell) + 1):
        if bins >= 0:
            src, dst = moving[:rows - bins], settled[bins:]
        else:
            src, dst = moving[-bins:], settled[:rows + bins]
        both = int(np.count_nonzero(src & dst))
        score = 2. * both / denom if denom else 0.
        candidates.append((score, bins))
    candidates.sort(reverse=True)
    best, bins = candidates[0]
    plateau = [b * cell for s, b in candidates if s >= best - .01]
    return {"dy_m": round(bins * cell, 3), "dice": round(best, 4),
            "near_best_span_m": [round(min(plateau), 3), round(max(plateau), 3)],
            "nearby": [{"dy_m": round(b * cell, 3), "dice": round(s, 4)}
                       for s, b in candidates[:5]]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--enter-dir", type=Path, required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--count", type=int, required=True)
    parser.add_argument("--settled", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--indices", type=int, nargs="+", required=True)
    parser.add_argument("--cell", type=float, choices=(.05, .1, .2), default=.1)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    if manifest.get("source_mode") != "live":
        raise ValueError("归档并非真实数据")
    settled_record = manifest.get("files", {}).get(args.settled.name)
    if settled_record is None or settled_record.get("role") != "volume_surface_source":
        raise ValueError("停稳文件不属于该归档的货面来源")
    digest = hashlib.sha256(args.settled.read_bytes()).hexdigest()
    if digest != settled_record.get("sha256"):
        raise ValueError("停稳文件与归档哈希不一致")
    stable = load_xyz(str(args.settled)).xyz
    bands = {"左侧": (-1.6, -0.8), "右侧": (0.8, 1.6),
             "中心": (-0.8, 0.8)}
    reference = {name: occupancy(stable, *bounds, args.cell) for name, bounds in bands.items()}
    results = []
    for index in args.indices:
        path = args.enter_dir / f"{args.prefix}_{index}of{args.count}.xyz"
        meta_path = Path(str(path) + ".meta.json")
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        if (meta.get("channel") != manifest.get("channel") or
                meta.get("index") != index or meta.get("frames") != args.count):
            raise ValueError("进场帧的通道或序号与归档不匹配")
        if meta.get("points", 0) < 100000 or meta.get("ts", 0) >= manifest["acquired_at"]:
            raise ValueError("进场帧的点数或时序不符")
        moving = load_xyz(str(path)).xyz
        if moving.shape[0] != meta["points"]:
            raise ValueError("进场帧点数与旁注不一致")
        fits = {name: fit_shift(occupancy(moving, *bounds, args.cell), reference[name], args.cell)
                for name, bounds in bands.items()}
        results.append({"index": index, "seq": meta["seq"], "ts": meta["ts"],
                        "points": int(moving.shape[0]), "fits": fits})
    if any(a["seq"] >= b["seq"] or a["ts"] >= b["ts"]
           for a, b in zip(results, results[1:])):
        raise ValueError("进场帧序号或时间未严格递增")
    report = {"method": "纵向平移；各横向带的纵向-高度占用格Dice；探索性诊断",
              "cell_m": args.cell,
              "channel": manifest["channel"],
              "source_mode": manifest["source_mode"],
              "measurement_id": manifest["measurement_id"],
              "passage_id": manifest["passage_id"],
              "settled_sha256": digest,
              "settled": str(args.settled), "results": results}
    value = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(value + "\n", encoding="utf-8")
    print(value)


if __name__ == "__main__":
    main()
