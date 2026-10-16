"""[D] Reproducible benchmark (p.DIRECT, fixed seeds) against engine ground truth.

Metric definitions (copied to the slides):
- Detection recall: GT parts matched / all GT parts. Match = same color, nearest in XY, within 2 cm.
- False positive: detections not matched to any part.
- XY error (mm): ||xy_det - xy_gt||, mean / median / max over matched pairs.
- Z error (mm): z_det - (TABLE_Z + CUBE), reported separately (detection = top face, GT = center).
- Correct-bin rate: parts inside the bin of their color after the run / parts in the scene.
  Parts skipped as suspicious are counted separately.
- Suspicious flag: on seeds 100-104 count hits; on seeds 0-19 count false alarms.
- Cycle time (sim): simulation steps per pick / 240. Depends on team-chosen parameters, NOT real robot speed.
Noise mode: brightness x U[0.7, 1.3] + Gaussian sigma=8 (0-255), fixed noise seed.
"""
import argparse
import csv
import os

import numpy as np
import pybullet as p

from common import BIN_HALF, BINS, CUBE, SEEDS_FAILURE, SEEDS_NORMAL, TABLE_Z
import controller
import perception
from scene import build_scene

MATCH_R = 0.02


def add_noise(rgb, seed):
    rng = np.random.default_rng(1000 + seed)
    img = rgb.astype(np.float32) * rng.uniform(0.7, 1.3) + rng.normal(0, 8, rgb.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def match(dets, gts):
    """Greedy nearest same-color match within MATCH_R. Returns pairs and unmatched detections."""
    pairs, used = [], set()
    for d in dets:
        best, bd = None, MATCH_R
        for i, (c, pos) in enumerate(gts):
            dd = np.hypot(d.xyz[0] - pos[0], d.xyz[1] - pos[1])
            if c == d.color and i not in used and dd <= bd:
                best, bd = i, dd
        if best is not None:
            used.add(best)
            pairs.append((d, gts[best][1]))
    return pairs, len(dets) - len(pairs)


def run_detect(seed, noise):
    s = build_scene(seed, gui=False)
    rgb, depth, _ = perception.capture()
    if noise:
        rgb = add_noise(rgb, seed)
    dets = perception.detect(rgb, depth)
    gts = [(o["color"], p.getBasePositionAndOrientation(o["id"])[0]) for o in s["objects"]]
    pairs, fp = match(dets, gts)
    errs = [np.hypot(d.xyz[0] - g[0], d.xyz[1] - g[1]) * 1000 for d, g in pairs]
    zerr = [(d.xyz[2] - (TABLE_Z + CUBE)) * 1000 for d, _ in pairs]
    flags = sum(d.suspicious for d in dets)
    # On failure seeds the merged blob covers 2 parts, so it cannot match a single GT within 2 cm:
    # recall counts it as missed by design; the flag is what we measure there.
    return dict(seed=seed, noise=int(noise), n_gt=len(gts), n_det=len(dets), matched=len(pairs), fp=fp,
                xy_mean_mm=np.mean(errs) if errs else np.nan, xy_max_mm=np.max(errs) if errs else np.nan,
                z_mean_mm=np.mean(zerr) if zerr else np.nan, flags=flags, errs=errs)


def run_pick(seed, noise):
    s = build_scene(seed, gui=False)
    robot, ids = s["robot"], [o["id"] for o in s["objects"]]
    picks = fails = skipped = 0
    steps0 = controller.STEPS
    failed = []
    for _ in range(2 * len(ids)):
        rgb, depth, _ = perception.capture()
        if noise:
            rgb = add_noise(rgb, seed + picks + fails)
        dets = perception.detect(rgb, depth)
        ok = [d for d in dets if not d.suspicious
              and not any(np.hypot(d.xyz[0] - x, d.xyz[1] - y) < 0.02 for x, y in failed)]
        if not ok:
            skipped = len(dets)
            break
        d = min(ok, key=lambda d: np.hypot(d.xyz[0], d.xyz[1]))
        if controller.pick_and_place(robot, d, ids):
            picks += 1
        else:
            fails += 1
            failed.append(d.xyz[:2])
    correct = 0
    for o in s["objects"]:
        x, y, _ = p.getBasePositionAndOrientation(o["id"])[0]
        bx, by = BINS[o["color"]]
        correct += abs(x - bx) < BIN_HALF and abs(y - by) < BIN_HALF
    cyc = (controller.STEPS - steps0) / 240 / max(picks, 1)
    return dict(seed=seed, noise=int(noise), n_parts=len(ids), picked=picks, grasp_fail=fails,
                skipped_suspicious=skipped, correct_bin=correct, cycle_s_sim=cyc)


def run_benchmark(seeds, mode="detect", noise=False, out="results"):
    os.makedirs(out, exist_ok=True)
    rows = [(run_detect if mode == "detect" else run_pick)(s, noise) for s in seeds]
    path = os.path.join(out, f"{mode}{'_noise' if noise else ''}.csv")
    keys = [k for k in rows[0] if k != "errs"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return path, rows


def summarize_detect(rows, label):
    normal = [r for r in rows if r["seed"] < 100]
    fail = [r for r in rows if r["seed"] >= 100]
    errs = [e for r in normal for e in r["errs"]]
    gt = sum(r["n_gt"] for r in normal)
    lines = [f"[{label}] seeds {len(normal)} normal + {len(fail)} failure",
             f"  recall {sum(r['matched'] for r in normal)}/{gt} = {sum(r['matched'] for r in normal) / gt:.1%}",
             f"  false positive {sum(r['fp'] for r in normal)}",
             f"  XY error mm: mean {np.mean(errs):.2f} median {np.median(errs):.2f} max {np.max(errs):.2f}",
             f"  Z error mm (top face vs CUBE): mean {np.nanmean([r['z_mean_mm'] for r in normal]):.2f}",
             f"  suspicious false alarm on normal seeds: {sum(r['flags'] for r in normal)}",
             f"  suspicious hit on failure seeds: {sum(r['flags'] > 0 for r in fail)}/{len(fail)}"]
    return "\n".join(lines), errs


def summarize_pick(rows, label):
    normal = [r for r in rows if r["seed"] < 100]
    fail = [r for r in rows if r["seed"] >= 100]
    n = sum(r["n_parts"] for r in normal)
    c = sum(r["correct_bin"] for r in normal)
    lines = [f"[{label}] correct-bin {c}/{n} = {c / n:.1%} (normal seeds)",
             f"  grasp failures {sum(r['grasp_fail'] for r in normal)}",
             f"  cycle time in SIM (team-chosen speed, not real robot): "
             f"{np.mean([r['cycle_s_sim'] for r in normal]):.2f} s/part"]
    if fail:
        lines.append(f"  failure seeds: correct-bin {sum(r['correct_bin'] for r in fail)}/"
                     f"{sum(r['n_parts'] for r in fail)}, suspicious detections left "
                     f"{sum(r['skipped_suspicious'] for r in fail)} (merged blobs, handed to operator)")
    return "\n".join(lines)


def plot(errs_clean, errs_noise, path="results/xy_error.png"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.boxplot([errs_clean, errs_noise])
    ax.set_xticks([1, 2], ["clean", "noise"])
    ax.set_ylabel("XY localization error (mm)")
    ax.set_title("VinSort detection error, seeds 0-19 (sim)")
    ax.axhline(5, ls="--", c="r", lw=1, label="5 mm target")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    return path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["detect", "pick", "all"], default="all")
    ap.add_argument("--quick", action="store_true", help="10 normal seeds, no noise for pick")
    a = ap.parse_args()
    normal = SEEDS_NORMAL[:10] if a.quick else SEEDS_NORMAL
    seeds = normal + SEEDS_FAILURE
    report = []
    if a.mode in ("detect", "all"):
        _, clean = run_benchmark(seeds, "detect")
        _, noisy = run_benchmark(seeds, "detect", noise=True)
        t1, e1 = summarize_detect(clean, "detect clean")
        t2, e2 = summarize_detect(noisy, "detect noise")
        report += [t1, t2, "  chart: " + plot(e1, e2)]
    if a.mode in ("pick", "all"):
        _, pk = run_benchmark(seeds, "pick")
        report.append(summarize_pick(pk, "pick clean"))
        if not a.quick:
            _, pkn = run_benchmark(normal, "pick", noise=True)
            report.append(summarize_pick(pkn, "pick noise"))
    text = "\n".join(report)
    print(text)
    with open("results/summary.txt", "w", encoding="utf-8") as f:
        f.write(text + "\n")
