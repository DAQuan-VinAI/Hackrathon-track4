"""Reproducible benchmark against engine ground truth: p.DIRECT, fixed seeds (owner: D).

    python benchmark.py               # detect (clean/light/hard) + pick (clean/light) -> results/
    python benchmark.py --quick       # 10 normal seeds, pick clean only
    python benchmark.py --mode detect

Metric definitions (copied to the slides):
- Detection recall: GT parts matched / all GT parts. Match = same color, nearest in XY, within 2 cm.
- False positive: detections not matched to any part.
- XY error (mm): ||xy_det - xy_gt||, mean / median / max over matched pairs.
- Z error (mm): z_det - (TABLE_Z + CUBE), reported separately (detection = top face, GT = center).
- Correct-bin rate: parts resting in the bin of their color after the run (scene.in_bin) / parts.
  Parts left because they were flagged suspicious are counted separately.
- Suspicious flag: on SEEDS_FAILURE count hits; on SEEDS_NORMAL count false alarms.
- Cycle time (sim): simulation steps per pick / SIM_HZ. Depends on team-chosen speeds, NOT real robot speed.

Noise modes (fixed noise seed per scene seed, reruns give the same numbers):
- light: the plan's mode, brightness x U[0.7, 1.3] + Gaussian sigma 8 (0-255).
  The cube faces have S ~220 against a threshold of 120, so this barely changes the masks.
- hard: brightness x U[0.5, 1.5], saturation x 0.6, Gaussian sigma 20, 5x5 blur on RGB,
  and Gaussian noise sigma 2 mm on the linear depth.
"""
import argparse
import csv
import os

import cv2
import numpy as np

import controller
import perception
from common import CAM, CUBE, SEEDS_FAILURE, SEEDS_NORMAL, SIM_HZ, TABLE_Z
from scene import build_scene, ground_truth, in_bin

OUT = "results"
NOISES = ("clean", "light", "hard")


def _depth_to_buffer(z):
    n, f = CAM["near"], CAM["far"]
    return (f - f * n / z) / (f - n)


def add_noise(rgb, depth, level, seed):
    """Returns a perturbed (rgb, depth). `seed` fixes the noise so reruns are identical."""
    if level == "clean":
        return rgb, depth
    rng = np.random.default_rng(1000 + seed)
    if level == "light":
        img = rgb * rng.uniform(0.7, 1.3) + rng.normal(0, 8, rgb.shape)
        return np.clip(img, 0, 255).astype(np.uint8), depth
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV).astype(np.float32)
    hsv[..., 1] *= 0.6
    img = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2RGB).astype(np.float32)
    img = img * rng.uniform(0.5, 1.5) + rng.normal(0, 20, rgb.shape)
    img = cv2.GaussianBlur(np.clip(img, 0, 255).astype(np.uint8), (5, 5), 0)
    z = perception._linear_depth(depth) + rng.normal(0, 0.002, depth.shape)
    return img, _depth_to_buffer(z).astype(depth.dtype)


def run_detect(seed, noise):
    sc = build_scene(seed, gui=False)
    controller.go_home(sc["robot"])                    # arm out of the camera view
    rgb, depth, _ = perception.capture()
    rgb, depth = add_noise(rgb, depth, noise, seed)
    dets = perception.detect(rgb, depth)
    gt = ground_truth(sc)
    pairs, fps = perception._match(dets, gt)
    trusted = [(d, g) for d, g in pairs if not d.suspicious]
    errs = [np.hypot(d.xyz[0] - g[0], d.xyz[1] - g[1]) * 1000 for d, g in trusted]
    zerr = [(d.xyz[2] - (TABLE_Z + CUBE)) * 1000 for d, _ in trusted]
    # On failure seeds the merged blob covers 2 cubes, its centroid is ~2 cm from either,
    # so recall counts it as missed by design; the flag is what we measure there.
    return dict(seed=seed, noise=noise, n_gt=len(gt), n_det=len(dets), matched=len(pairs), fp=len(fps),
                flags=sum(d.suspicious for d in dets),
                reasons="|".join(d.reason for d in dets if d.suspicious),
                xy_mean_mm=np.mean(errs) if errs else np.nan, xy_max_mm=np.max(errs) if errs else np.nan,
                z_mean_mm=np.mean(zerr) if zerr else np.nan, errs=errs)


def run_pick(seed, noise, max_fails=2):
    """Same rule as main.py AUTO: pick the nearest confident cube, stop when only suspicious ones remain."""
    sc = build_scene(seed, gui=False)
    robot, ids = sc["robot"], [o["id"] for o in sc["objects"]]
    steps = [0]
    tick = lambda: steps.__setitem__(0, steps[0] + 1)
    controller.go_home(robot)
    picks = fails = consecutive = left_suspicious = 0
    for i in range(2 * len(ids)):
        rgb, depth, _ = perception.capture()
        rgb, depth = add_noise(rgb, depth, noise, seed * 100 + i)
        dets = perception.detect(rgb, depth)
        ok = [d for d in dets if not d.suspicious]
        if not ok:
            left_suspicious = len(dets)
            break
        det = min(ok, key=lambda d: np.hypot(d.xyz[0], d.xyz[1]))
        if controller.pick_and_place(robot, det, ids, on_step=tick):
            picks, consecutive = picks + 1, 0
        else:
            fails, consecutive = fails + 1, consecutive + 1
            if consecutive >= max_fails:               # main.py hands over to the operator here
                break
    correct = sum(in_bin(o["id"], o["color"]) for o in sc["objects"])
    return dict(seed=seed, noise=noise, n_parts=len(ids), picked=picks, grasp_fail=fails,
                left_suspicious=left_suspicious, correct_bin=correct,
                cycle_s_sim=steps[0] / SIM_HZ / max(picks, 1))


def write_csv(rows, name):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, name)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, [k for k in rows[0] if k != "errs"], extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    return path


def summarize_detect(rows, label):
    normal = [r for r in rows if r["seed"] not in SEEDS_FAILURE]
    fail = [r for r in rows if r["seed"] in SEEDS_FAILURE]
    errs = [e for r in normal for e in r["errs"]]
    gt = sum(r["n_gt"] for r in normal)
    m = sum(r["matched"] for r in normal)
    lines = [f"[detect {label}] {len(normal)} normal + {len(fail)} failure seeds",
             f"  recall {m}/{gt} = {m / gt:.1%}, false positives {sum(r['fp'] for r in normal)}"]
    if errs:
        lines.append(f"  XY error mm: mean {np.mean(errs):.2f} median {np.median(errs):.2f} max {np.max(errs):.2f}")
        lines.append(f"  Z error mm (top face vs CUBE): mean {np.nanmean([r['z_mean_mm'] for r in normal]):.2f}")
    lines.append(f"  suspicious false alarms on normal seeds: {sum(r['flags'] > 0 for r in normal)}/{len(normal)}")
    if fail:
        lines.append(f"  suspicious hit on failure seeds: {sum(r['flags'] > 0 for r in fail)}/{len(fail)}")
    return "\n".join(lines), errs


def summarize_pick(rows, label):
    normal = [r for r in rows if r["seed"] not in SEEDS_FAILURE]
    fail = [r for r in rows if r["seed"] in SEEDS_FAILURE]
    n = sum(r["n_parts"] for r in normal)
    c = sum(r["correct_bin"] for r in normal)
    lines = [f"[pick {label}] correct-bin {c}/{n} = {c / n:.1%} on normal seeds",
             f"  grasp failures {sum(r['grasp_fail'] for r in normal)}",
             f"  cycle time in SIM (team-chosen speed, not a real robot): "
             f"{np.mean([r['cycle_s_sim'] for r in normal]):.2f} s/part"]
    if fail:
        lines.append(f"  failure seeds: correct-bin {sum(r['correct_bin'] for r in fail)}/"
                     f"{sum(r['n_parts'] for r in fail)}, flagged blobs left for the operator "
                     f"{sum(r['left_suspicious'] for r in fail)}")
    return "\n".join(lines)


def plot(errs_by_noise, path=os.path.join(OUT, "xy_error.png")):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = [k for k, v in errs_by_noise.items() if v]
    fig, ax = plt.subplots(figsize=(6, 4))
    ax.boxplot([errs_by_noise[k] for k in labels])
    ax.set_xticks(range(1, len(labels) + 1), labels)
    ax.set_ylabel("XY localization error (mm)")
    ax.set_title("VinSort detection error, normal seeds (sim)")
    ax.axhline(5, ls="--", c="r", lw=1, label="5 mm target")
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
    return path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["detect", "pick", "all"], default="all")
    ap.add_argument("--quick", action="store_true", help="10 normal seeds, pick clean only")
    a = ap.parse_args()
    normal = SEEDS_NORMAL[:10] if a.quick else SEEDS_NORMAL
    seeds = normal + SEEDS_FAILURE
    report = []
    if a.mode in ("detect", "all"):
        errs = {}
        for nz in NOISES:
            rows = [run_detect(s, nz) for s in seeds]
            write_csv(rows, f"detect_{nz}.csv")
            text, errs[nz] = summarize_detect(rows, nz)
            report.append(text)
        report.append(f"  chart: {plot(errs)}")
    if a.mode in ("pick", "all"):
        for nz in ("clean",) if a.quick else ("clean", "light"):
            rows = [run_pick(s, nz) for s in seeds]
            write_csv(rows, f"pick_{nz}.csv")
            report.append(summarize_pick(rows, nz))
    text = "\n".join(report)
    print(text)
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
