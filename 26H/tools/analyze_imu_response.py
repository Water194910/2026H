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
            if "qpos" not in values or "ay" not in values:
                continue
            samples.append({"timestamp": sample.get("timestamp"), **values})
    samples.sort(key=lambda item: item.get("timestamp") or 0)
    return samples


def rounded(value, digits=3):
    return None if value is None else round(value, digits)


def rms(values):
    return math.sqrt(statistics.fmean(value * value for value in values)) if values else 0.0


def channel_stats(samples, key):
    values = [item[key] for item in samples if key in item]
    if not values:
        return None
    return {
        "min": rounded(min(values)),
        "max": rounded(max(values)),
        "mean": rounded(statistics.fmean(values)),
        "rms": rounded(rms(values)),
        "last": rounded(values[-1]),
    }


def median_period_ms(samples):
    periods = []
    for previous, current in zip(samples, samples[1:]):
        first = previous.get("timestamp")
        second = current.get("timestamp")
        if first is not None and second is not None and second > first:
            periods.append(second - first)
    return statistics.median(periods) if periods else None


def time_after(samples, start, delay_ms):
    timestamp = samples[start].get("timestamp")
    if timestamp is None:
        return start
    for index in range(start, len(samples)):
        if samples[index].get("timestamp", timestamp) >= timestamp + delay_ms:
            return index
    return len(samples) - 1


def find_motion_segments(samples, threshold, merge_gap_ms):
    active = [index for index, item in enumerate(samples) if abs(item["ay"]) >= threshold]
    if not active:
        return []
    segments = []
    start = active[0]
    end = active[0]
    for index in active[1:]:
        previous_time = samples[end].get("timestamp")
        current_time = samples[index].get("timestamp")
        gap = math.inf if previous_time is None or current_time is None else current_time - previous_time
        if gap <= merge_gap_ms:
            end = index
        else:
            segments.append((start, end))
            start = index
            end = index
    segments.append((start, end))
    return segments


def peak_record(samples, index, first_timestamp):
    item = samples[index]
    expected = math.degrees(math.atan(item["ay"] / 9.80665))
    return {
        "timeMs": None if item.get("timestamp") is None else item["timestamp"] - first_timestamp,
        "ay": rounded(item["ay"]),
        "expectedPhysicalDeg": rounded(expected),
        "ffDeg": rounded(item.get("ff")),
        "targetDeg": rounded(item.get("atgt")),
        "actualDeg": rounded(item.get("adeg")),
        "targetErrorDeg": rounded(item.get("atgt", 0.0) - item.get("adeg", 0.0)),
        "positionMm": rounded(item["qpos"]),
        "velocityMmS": rounded(item.get("qvel")),
    }


def analyze_segment(samples, start, end, args, first_timestamp):
    response_end = time_after(samples, end, args.response_ms)
    window = samples[start:response_end + 1]
    motion = samples[start:end + 1]
    positive_peak = max(range(start, end + 1), key=lambda index: samples[index]["ay"])
    negative_peak = min(range(start, end + 1), key=lambda index: samples[index]["ay"])
    tracking_errors = [item.get("atgt", 0.0) - item.get("adeg", 0.0) for item in motion]
    outside = [item for item in window if abs(item["qpos"]) > args.band_mm]
    start_time = samples[start].get("timestamp")
    end_time = samples[end].get("timestamp")
    return {
        "startMs": None if start_time is None else start_time - first_timestamp,
        "motionDurationMs": None if start_time is None or end_time is None else end_time - start_time,
        "ayMin": rounded(min(item["ay"] for item in motion)),
        "ayMax": rounded(max(item["ay"] for item in motion)),
        "ffMinDeg": rounded(min(item.get("ff", 0.0) for item in motion)),
        "ffMaxDeg": rounded(max(item.get("ff", 0.0) for item in motion)),
        "targetMinDeg": rounded(min(item.get("atgt", 0.0) for item in motion)),
        "targetMaxDeg": rounded(max(item.get("atgt", 0.0) for item in motion)),
        "actualMinDeg": rounded(min(item.get("adeg", 0.0) for item in motion)),
        "actualMaxDeg": rounded(max(item.get("adeg", 0.0) for item in motion)),
        "trackingErrorMaxAbsDeg": rounded(max(abs(value) for value in tracking_errors)),
        "trackingErrorRmsDeg": rounded(rms(tracking_errors)),
        "positionStartMm": rounded(samples[start]["qpos"]),
        "positionMinMm": rounded(min(item["qpos"] for item in window)),
        "positionMaxMm": rounded(max(item["qpos"] for item in window)),
        "positionMaxAbsMm": rounded(max(abs(item["qpos"]) for item in window)),
        "outsideBandSamples": len(outside),
        "positivePeak": peak_record(samples, positive_peak, first_timestamp),
        "negativePeak": peak_record(samples, negative_peak, first_timestamp),
    }


def analyze(samples, args):
    if not samples:
        return {"sampleCount": 0, "passes": False, "reason": "no samples"}
    first_timestamp = samples[0].get("timestamp") or 0
    last_timestamp = samples[-1].get("timestamp") or first_timestamp
    outside = [item for item in samples if abs(item["qpos"]) > args.band_mm]
    ff_saturated = [item for item in samples if abs(item.get("ff", 0.0)) >= args.ff_limit_deg - 0.05]
    target_saturated = [item for item in samples if abs(item.get("atgt", 0.0)) >= args.total_limit_deg - 0.05]
    imu_ok = all(item.get("ist") == 3 for item in samples)
    age_ok = max(item.get("iage", math.inf) for item in samples) <= args.max_imu_age_ms
    error_ok = (
        max(item.get("ierr", 0.0) for item in samples) == min(item.get("ierr", 0.0) for item in samples)
        and max(item.get("ibad", 0.0) for item in samples) == min(item.get("ibad", 0.0) for item in samples)
    )
    segments = [
        analyze_segment(samples, start, end, args, first_timestamp)
        for start, end in find_motion_segments(samples, args.motion_threshold, args.merge_gap_ms)
    ]
    passes = bool(segments and not outside and imu_ok and age_ok and error_ok)
    return {
        "sampleCount": len(samples),
        "durationMs": last_timestamp - first_timestamp,
        "periodMedianMs": rounded(median_period_ms(samples), 1),
        "limits": {
            "positionBandMm": args.band_mm,
            "ffLimitDeg": args.ff_limit_deg,
            "totalLimitDeg": args.total_limit_deg,
        },
        "position": channel_stats(samples, "qpos"),
        "velocity": channel_stats(samples, "qvel"),
        "ay": channel_stats(samples, "ay"),
        "ff": channel_stats(samples, "ff"),
        "targetAngle": channel_stats(samples, "atgt"),
        "actualAngle": channel_stats(samples, "adeg"),
        "outsideBandSamples": len(outside),
        "ffSaturatedSamples": len(ff_saturated),
        "totalAngleSaturatedSamples": len(target_saturated),
        "imu": {
            "statusAlways3": imu_ok,
            "ageMaxMs": rounded(max(item.get("iage", math.inf) for item in samples)),
            "errorCounterChanged": not error_ok,
            "ierr": channel_stats(samples, "ierr"),
            "ibad": channel_stats(samples, "ibad"),
        },
        "motionSegments": segments,
        "passes": passes,
        "reason": (
            "pass"
            if passes
            else "no significant Y-axis motion"
            if not segments
            else "position exceeded band"
            if outside
            else "IMU freshness/status/error gate failed"
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--band-mm", type=float, default=10.0)
    parser.add_argument("--motion-threshold", type=float, default=0.15)
    parser.add_argument("--merge-gap-ms", type=float, default=400.0)
    parser.add_argument("--response-ms", type=float, default=1500.0)
    parser.add_argument("--max-imu-age-ms", type=float, default=25.0)
    parser.add_argument("--ff-limit-deg", type=float, default=14.0)
    parser.add_argument("--total-limit-deg", type=float, default=14.0)
    args = parser.parse_args()
    print(json.dumps(analyze(load_samples(sys.stdin), args), indent=2))


if __name__ == "__main__":
    main()
