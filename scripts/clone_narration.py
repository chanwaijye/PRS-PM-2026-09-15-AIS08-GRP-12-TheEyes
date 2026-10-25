"""A practice narration in the author's cloned voice, for rehearsing the delivery.
Never part of the submitted video: the author records the narration themselves.

    ~/venvs/the-eyes-tts/bin/python scripts/clone_narration.py REF.wav "REF TEXT" [--scenes 1,2]

REF.wav is about ten seconds of the author's own voice and REF TEXT its exact words.
Writes runs/video/voice_clone/sceneNN.wav, one per scene, and all_scenes.mp3.
Each chunk of the script (a paragraph, split at "(pause)") is generated on its own
and the chunks are joined with real silence, so the pauses of the script are
pauses here, and scripts/build_video.py can cut a rehearsal edit from them.
Numbers are written out in words for the voice model only.
"""
import argparse
import importlib.util
import os
import re
import subprocess

import numpy as np
import soundfile as sf
from num2words import num2words

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "runs/video/voice_clone")
SR = 24000
GAP_PAUSE, GAP_PARA = 1.6, 1.0

spec = importlib.util.spec_from_file_location("bv", os.path.join(ROOT, "scripts/build_video.py"))
bv = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bv)


def say(text):
    """The text as the voice model should read it."""
    t = text.replace("3D", "three D").replace(" v8 ", " vee eight ").replace("occluded-from", "occluded from")
    words = lambda x: num2words(x).replace(",", "").replace("-", " ")
    t = re.sub(r"\d+(?:,\d{3})*\.\d+", lambda m: words(float(m.group(0).replace(",", ""))), t)
    t = re.sub(r"\d+(?:,\d{3})*", lambda m: words(int(m.group(0).replace(",", ""))), t)
    return re.sub(r"\s+", " ", t)


def chunks_with_gaps(scene):
    """[(text, gap after it)]: a (pause) inside a paragraph, or the end of a paragraph."""
    out = []
    for para in bv.spoken(scene):
        parts = [c.strip() for c in para.split("(pause)")]
        for i, c in enumerate(parts):
            if not c:
                continue
            last = i == len(parts) - 1 or not any(p.strip() for p in parts[i + 1:])
            out.append((c, GAP_PARA if last else GAP_PAUSE))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("ref_text")
    ap.add_argument("--scenes", default="1,2,3,4,5,6,7,8,9,10,11,12")
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    from f5_tts.api import F5TTS
    tts = F5TTS(model="F5TTS_v1_Base")
    os.makedirs(OUT, exist_ok=True)
    done = []
    for sc in [int(x) for x in a.scenes.split(",")]:
        pieces = []
        for text, gap in chunks_with_gaps(sc):
            wav, sr, _ = tts.infer(ref_file=a.ref, ref_text=a.ref_text, gen_text=say(text), seed=a.seed,
                                   show_info=lambda *x: None, progress=None)
            assert sr == SR
            pieces += [np.asarray(wav, dtype=np.float32), np.zeros(int(gap * SR), np.float32)]
        audio = np.concatenate([np.zeros(int(0.5 * SR), np.float32)] + pieces[:-1] + [np.zeros(int(0.8 * SR), np.float32)])
        path = os.path.join(OUT, f"scene{sc:02d}.wav")
        sf.write(path, audio, SR)
        done.append(path)
        print(f"scene {sc:2d}: {len(audio) / SR:5.1f} s  {path}")
    # one file to listen to on the phone, scene after scene
    lst = os.path.join(OUT, "all.txt")
    gap = os.path.join(OUT, "gap.wav")
    sf.write(gap, np.zeros(int(2.5 * SR), np.float32), SR)
    with open(lst, "w") as fh:
        for i in range(1, 13):
            p = os.path.join(OUT, f"scene{i:02d}.wav")
            if os.path.exists(p):
                fh.write(f"file '{p}'\nfile '{gap}'\n")
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst, "-c:a", "libmp3lame",
                    "-q:a", "3", os.path.join(OUT, "all_scenes.mp3")], check=True)
    os.remove(lst)
    os.remove(gap)
    print("wrote", os.path.join(OUT, "all_scenes.mp3"))


if __name__ == "__main__":
    main()
