"""Transcribes the narration so the edit can place each picture change on the spoken word.

    ~/venvs/the-eyes-tts/bin/python scripts/transcribe_narration.py [--voice DIR] [--scenes 1,2]

Reads runs/video/voice/sceneNN.* (the same file and audio stream scripts/build_video.py
uses) and writes runs/video/voice/words.json: for each file its duration and the
recognised words with their start and end in seconds. build_video.py matches those
words to the script's; a file changed since it was transcribed is ignored there.
Prints each transcript, which is also the quickest check for a skipped or misread line.

Whisper large-v3-turbo through transformers, the whole file in one pass: decoding in
overlapping 30 s windows repeats words at the window seams.
"""
import argparse
import importlib.util
import json
import os
import subprocess

import numpy as np
import torch
from transformers import pipeline

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
spec = importlib.util.spec_from_file_location("bv", os.path.join(ROOT, "scripts/build_video.py"))
bv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bv)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--voice", default=bv.VOICE)
    ap.add_argument("--scenes", default="1,2,3,4,5,6,7,8,9,10,11,12")
    a = ap.parse_args()
    bv.VOICE = os.path.abspath(a.voice)
    path = os.path.join(bv.VOICE, "words.json")
    out = json.load(open(path)) if os.path.exists(path) else {}
    asr = pipeline("automatic-speech-recognition", model="openai/whisper-large-v3-turbo", torch_dtype=torch.float16,
                   device="cuda:0")
    for sc in [int(x) for x in a.scenes.split(",")]:
        f = bv.voice_file(sc)
        if not f:
            print(f"scene {sc:2d}: no recording")
            continue
        track = bv.voice_track(f)
        raw = subprocess.run(["ffmpeg", "-v", "error", "-i", f, "-map", f"0:a:{track}", "-ac", "1", "-ar", "16000",
                              "-f", "f32le", "-"], capture_output=True, check=True).stdout
        r = asr({"raw": np.frombuffer(raw, np.float32).copy(), "sampling_rate": 16000}, return_timestamps="word",
                generate_kwargs={"language": "en", "task": "transcribe"})
        words = [[c["text"].strip(), round(c["timestamp"][0], 3), round(c["timestamp"][1] or c["timestamp"][0], 3)]
                 for c in r["chunks"]]
        out[os.path.basename(f)] = {"duration": round(bv.duration(f), 3), "track": track, "words": words}
        print(f"scene {sc:2d} ({os.path.basename(f)}, {len(words)} words): {r['text'].strip()}\n", flush=True)
    json.dump(out, open(path, "w"), indent=1)
    print("wrote", path)


if __name__ == "__main__":
    main()
