#!/usr/bin/env python3
import argparse
import json
import math
import statistics
import sys


def load_samples(stream):
    samples = []
    for line in stream:
        try:
            message = json.loads(line)
        except json.JSONDecodeError:
            continue
        if message.get("type") != "samples":
            continue
        for sample in message.get("samples", []):
            values = sample.get("values", {})
            if "qpos" not in values:
                continue
            samples.append({"timestamp": sample.get("timestamp"), **values})
    samples.sort(key=lambda item: item["timestamp"] or 0)
    return samples


def median_period_ms(samples):
    periods = []
    for previous, current in zip(samples, samples[1:]):
        if previous["timestamp"] is None or current["timestamp"] is None:
            continue
        period = current["timestamp"] - previous["timestamp"]
        if period > 0:
            periods.append(period)
    return statistics.median(periods) if periods else 100.0


def first_stable_index(samples, start, band_mm, count):
    run = 0
    for index in range(start, len(samples)):
        if abs(samples[index]["qpos"]) <= band_mm:
            run += 1
            if run >= count:
                return index - count + 1
        else:
            run = 0
    return None


def first_index(samples, start, stop, predicate):
    for index in range(start, stop):
        if predicate(samples[index]):
            return index
    return None


def elapsed_ms(samples, start, stop):
    if start is None or stop is None:
        return None
    first = samples[start]["timestamp"]
    last = samples[stop]["timestamp"]
    return None if first is None or last is None else last - first


def rounded(value, digits=2):
    return None if value is None else round(value, digits)


def longest_stall(samples, start, stop, args):
    best = None
    run_start = None
    for index in range(start, stop + 1):
        stalled = False
        if index < stop:
            item = samples[index]
            stalled = (
                abs(item["qpos"]) > args.band_mm
                and abs(item.get("qvel", 0.0)) <= args.stall_speed_mm_s
            )
        if stalled and run_start is None:
            run_start = index
        if not stalled and run_start is not None:
            duration = elapsed_ms(samples, run_start, index - 1) or 0
            if best is None or duration > best[0]:
                best = (duration, run_start, index)
            run_start = None
    if best is None or best[0] < args.stall_ms:
        return None
    duration, first, after = best
    window = samples[first:after]
    return {
        "durationMs": duration,
        "meanPositionMm": rounded(statistics.fmean(item["qpos"] for item in window)),
        "positionMinMm": rounded(min(item["qpos"] for item in window)),
        "positionMaxMm": rounded(max(item["qpos"] for item in window)),
        "baseCommandMinDeg": rounded(min(item.get("qbase", 0.0) for item in window)),
        "baseCommandMaxDeg": rounded(max(item.get("qbase", 0.0) for item in window)),
        "finalCommandMinDeg": rounded(min(item.get("qcmd", 0.0) for item in window)),
        "finalCommandMaxDeg": rounded(max(item.get("qcmd", 0.0) for item in window)),
    }


def analyze_episode(samples, start, settle_count, args):
    preliminary_settle = first_stable_index(samples, start, args.band_mm, settle_count)
    evaluation_stop = (
        len(samples)
        if preliminary_settle is None
        else min(len(samples), preliminary_settle + settle_count)
    )
    peak = max(range(start, evaluation_stop), key=lambda index: abs(samples[index]["qpos"]))
    peak_position = samples[peak]["qpos"]
    side = 1 if peak_position >= 0 else -1
    near_peak = [
        index
        for index in range(peak, evaluation_stop)
        if side * samples[index]["qpos"] >= abs(peak_position) - args.motion_mm
    ]
    release = near_peak[-1] if near_peak else peak
    moved = first_index(
        samples,
        release + 1,
        evaluation_stop,
        lambda item: side * item["qpos"] <= abs(peak_position) - args.motion_mm,
    )
    if moved is None:
        return {
            "startTimestamp": samples[start]["timestamp"],
            "side": "positive" if side > 0 else "negative",
            "heldPeakMm": rounded(peak_position),
            "released": False,
        }, evaluation_stop

    release = moved
    settle = first_stable_index(samples, release, args.band_mm, settle_count)
    evaluation_stop = len(samples) if settle is None else min(len(samples), settle + settle_count)
    first_band = first_index(
        samples, release, evaluation_stop, lambda item: abs(item["qpos"]) <= args.band_mm
    )
    zero_cross = first_index(
        samples,
        release,
        evaluation_stop,
        (lambda item: item["qpos"] <= 0) if side > 0 else (lambda item: item["qpos"] >= 0),
    )
    positions = [item["qpos"] for item in samples[release:evaluation_stop]]
    opposite_overshoot = max(0.0, -min(positions)) if side > 0 else max(0.0, max(positions))
    qcmd = [abs(item.get("qcmd", 0.0)) for item in samples[release:evaluation_stop]]
    adeg = [abs(item.get("adeg", 0.0)) for item in samples[release:evaluation_stop]]
    imu = [abs(item.get("imu", 0.0)) for item in samples[release:evaluation_stop]]
    stable_positions = [] if settle is None else [
        item["qpos"] for item in samples[settle:settle + settle_count]
    ]
    result = {
        "startTimestamp": samples[start]["timestamp"],
        "side": "positive" if side > 0 else "negative",
        "releasePeakMm": rounded(peak_position),
        "releaseTimestamp": samples[release]["timestamp"],
        "released": True,
        "firstBandMs": elapsed_ms(samples, release, first_band),
        "firstBandSpeedMmS": (
            rounded(samples[first_band].get("qvel", 0.0)) if first_band is not None else None
        ),
        "zeroCrossMs": elapsed_ms(samples, release, zero_cross),
        "zeroCrossSpeedMmS": (
            rounded(samples[zero_cross].get("qvel", 0.0)) if zero_cross is not None else None
        ),
        "zeroCrossBaseDeg": (
            rounded(samples[zero_cross].get("qbase", 0.0)) if zero_cross is not None else None
        ),
        "zeroCrossCommandDeg": (
            rounded(samples[zero_cross].get("qcmd", 0.0)) if zero_cross is not None else None
        ),
        "zeroCrossActualDeg": (
            rounded(samples[zero_cross].get("adeg", 0.0)) if zero_cross is not None else None
        ),
        "oppositeOvershootMm": rounded(opposite_overshoot),
        "settleMs": elapsed_ms(samples, release, settle),
        "settled": settle is not None,
        "stableMinMm": rounded(min(stable_positions)) if stable_positions else None,
        "stableMaxMm": rounded(max(stable_positions)) if stable_positions else None,
        "maxAbsCommandDeg": rounded(max(qcmd)) if qcmd else None,
        "maxAbsActualDeg": rounded(max(adeg)) if adeg else None,
        "maxAbsImuDeg": rounded(max(imu)) if imu else None,
        "visionAgeMaxMs": rounded(
            max(item.get("vage", 0.0) for item in samples[release:evaluation_stop])
        ),
        "longestStall": longest_stall(samples, release, evaluation_stop, args),
    }
    result["passes"] = bool(
        result["settled"] and result["oppositeOvershootMm"] <= args.band_mm
    )
    return result, evaluation_stop


def analyze(samples, args):
    if not samples:
        return {"sampleCount": 0, "episodes": []}
    period_ms = median_period_ms(samples)
    settle_count = max(1, math.ceil(args.settle_ms / period_ms))
    episodes = []
    index = 0
    while index < len(samples):
        if abs(samples[index]["qpos"]) < args.trigger_mm:
            index += 1
            continue
        episode, next_index = analyze_episode(samples, index, settle_count, args)
        episodes.append(episode)
        index = max(index + 1, next_index)
    return {
        "sampleCount": len(samples),
        "periodMs": rounded(period_ms, 1),
        "positionMinMm": rounded(min(item["qpos"] for item in samples)),
        "positionMaxMm": rounded(max(item["qpos"] for item in samples)),
        "visionAgeMaxMs": rounded(max(item.get("vage", 0.0) for item in samples)),
        "episodes": episodes,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trigger-mm", type=float, default=8.0)
    parser.add_argument("--motion-mm", type=float, default=1.0)
    parser.add_argument("--band-mm", type=float, default=2.0)
    parser.add_argument("--settle-ms", type=float, default=800.0)
    parser.add_argument("--stall-ms", type=float, default=200.0)
    parser.add_argument("--stall-speed-mm-s", type=float, default=12.0)
    args = parser.parse_args()
    print(json.dumps(analyze(load_samples(sys.stdin), args), indent=2))


if __name__ == "__main__":
    main()
