#!/usr/bin/env python3
"""Compare motor-noise recordings with an explicit sample-rate convention."""

import argparse
import math
import wave
from pathlib import Path

import numpy as np


def load_audio(path, sample_rate):
    suffix = path.suffix.lower()
    if suffix == ".npy":
        data = np.asarray(np.load(path), dtype=np.float64)
        rate = sample_rate
    elif suffix == ".wav":
        with wave.open(str(path), "rb") as source:
            channels = source.getnchannels()
            width = source.getsampwidth()
            rate = source.getframerate()
            frames = source.readframes(source.getnframes())
        if width == 1:
            data = (np.frombuffer(frames, dtype=np.uint8).astype(np.float64) - 127.5) / 127.5
        elif width == 2:
            data = np.frombuffer(frames, dtype="<i2").astype(np.float64) / 32768.0
        elif width == 4:
            data = np.frombuffer(frames, dtype="<i4").astype(np.float64) / 2147483648.0
        else:
            raise ValueError(f"Unsupported WAV sample width: {width}")
        data = data.reshape(-1, channels).T
    else:
        raise ValueError("Input must be .npy or PCM .wav")

    if data.ndim == 1:
        data = data.reshape(1, -1)
    elif data.shape[0] > 8 and data.shape[1] <= 8:
        data = data.T
    return data, rate


def db(value):
    return 20.0 * math.log10(max(value, 1e-15))


def spectrum(signal, rate):
    centered = signal - signal.mean()
    window = np.hanning(centered.size)
    magnitude = np.abs(np.fft.rfft(centered * window))
    frequency = np.fft.rfftfreq(centered.size, 1.0 / rate)
    return frequency, magnitude


def separated_peaks(frequency, magnitude, low, high, count):
    indices = np.where((frequency >= low) & (frequency <= high))[0]
    if indices.size == 0:
        return []
    order = indices[np.argsort(magnitude[indices])[::-1]]
    selected = []
    for index in order:
        if all(abs(frequency[index] - frequency[old]) >= 2.0 for old in selected):
            selected.append(index)
        if len(selected) == count:
            break
    return [(frequency[index], magnitude[index]) for index in selected]


def print_channel(signal, rate, label):
    frequency, magnitude = spectrum(signal, rate)
    reference = max(float(magnitude.max()), 1e-15)
    print(f"{label}: rms={db(float(np.sqrt(np.mean(signal * signal)))):.1f} dBFS")
    print("  peaks 20-1200 Hz:")
    for value, level in separated_peaks(frequency, magnitude, 20.0, 1200.0, 10):
        print(f"    {value:8.2f} Hz  {20.0 * math.log10(level / reference):7.1f} dB-rel")
    print("  peaks 1-20 kHz:")
    for value, level in separated_peaks(frequency, magnitude, 1000.0, 20000.0, 8):
        print(f"    {value:8.2f} Hz  {20.0 * math.log10(level / reference):7.1f} dB-rel")

    total_power = float(np.sum(magnitude * magnitude))
    bands = [
        (20, 80),
        (80, 300),
        (300, 1000),
        (1000, 5000),
        (5000, 10000),
        (10000, 20000),
    ]
    print("  band power:")
    for low, high in bands:
        mask = (frequency >= low) & (frequency < high)
        ratio = float(np.sum(magnitude[mask] ** 2)) / total_power
        print(f"    {low:5d}-{high:5d} Hz  {10.0 * math.log10(ratio + 1e-15):7.1f} dB-rel")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument("--sample-rate", type=int, default=48000)
    args = parser.parse_args()

    for path in args.recordings:
        data, rate = load_audio(path, args.sample_rate)
        print(f"\n{path.name}: channels={data.shape[0]} rate={rate} Hz "
              f"duration={data.shape[1] / rate:.3f} s")
        mixed = data.mean(axis=0)
        print_channel(mixed, rate, "mixed")


if __name__ == "__main__":
    main()
