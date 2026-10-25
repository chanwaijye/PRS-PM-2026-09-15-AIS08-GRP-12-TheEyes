"""Builds the project video from the takes, replays, slides and narration.

    python3 scripts/build_video.py [--scenes 1,6] [--fonts DIR] [--final]

Inputs, none of them in git:
  runs/video/voice/sceneNN.*        the narration, one file per scene (any format ffmpeg reads)
  runs/video/slides/png/*.png       the slide frames (build_final_deck.py --video, then rendered)
  runs/video/retimed/*.mp4          the live console files in true time (retime_console.py)
  runs/video/replay_*.avi           replays of the scored scenes, true time
  runs/video/phone/*.mp4            the phone clips
Output: runs/video/build/sceneNN.mp4, and with --final the whole video.

The spoken lines come from ProjectReport/presentation/video_script.txt, so the
picture cues below name a sentence by its first words. A scene without its
voice file is timed from the word count at 130 words a minute and has no sound:
a rough cut of the picture.

Two kinds of scene:
  voice-led    the recording plays straight through; the picture changes at the
               sentence each cue names, its time read from where that sentence's
               words fall in the recording's speech.
  picture-led  (scenes 1 and 6) the take plays straight through; the recording is
               cut at its pauses into the script's chunks (split at paragraphs and
               at "(pause)"), and each chunk starts at its beat in the take.
"""
import argparse
import glob
import os
import re
import sqlite3
import subprocess
import tempfile
from datetime import datetime, timezone

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
V = os.path.join(ROOT, "runs/video")
SLIDE = os.path.join(V, "slides/png")
OUT = os.path.join(V, "build")
FPS, WPM = 30, 130
# The phone's first frame comes about 0.95 s after the time in its file name: measured
# on 25 Oct by correlating phone motion with cam_front's (demo_cold_e +1.0 s,
# demo_live_d +0.9 s), and checked frame by frame on demo_live_d's first tap.
PHONE_LAG = 0.95
BG = "0x14202B"
SCRIPT = os.path.join(ROOT, "ProjectReport/presentation/video_script.txt")
FINAL = os.path.join(V, "PRS-PM-2026-AIS08-GRP-12-TheEyes-Video.mp4")


# ---- the script -----------------------------------------------------------
def spoken(scene):
    """The scene's spoken paragraphs, in order."""
    body = open(SCRIPT).read().split("-" * 76)[0]
    parts = re.split(r"\n(?=Scene \d+\. )", body)
    for p in parts:
        if p.startswith(f"Scene {scene}. "):
            paras = []
            for block in p.strip().split("\n\n")[1:]:
                b = " ".join(block.split())
                if b and not b.startswith(("ON SCREEN", "NOTE")):
                    paras.append(b)
            return paras
    raise SystemExit(f"scene {scene} not in the script")


def chunks(scene):
    """Paragraphs split at (pause): what a speaker says between two long gaps."""
    out = []
    for p in spoken(scene):
        out += [c.strip() for c in p.split("(pause)") if c.strip()]
    return out


def words(t):
    return len(t.split())


# ---- the recording ----------------------------------------------------------
def voice_file(scene):
    f = sorted(glob.glob(os.path.join(V, f"voice/scene{scene:02d}.*")))
    return f[0] if f else None


def duration(path):
    r = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", path],
                       capture_output=True, text=True, check=True)
    return float(r.stdout.strip())


def speech(path, gap=0.35, noise="-38dB"):
    """Speech segments [(start, end)], from silencedetect."""
    r = subprocess.run(["ffmpeg", "-hide_banner", "-i", path, "-af", f"silencedetect=noise={noise}:d={gap}",
                        "-f", "null", "-"], capture_output=True, text=True)
    d = duration(path)
    starts = [float(x) for x in re.findall(r"silence_start: ([\d.]+)", r.stderr)]
    ends = [float(x) for x in re.findall(r"silence_end: ([\d.]+)", r.stderr)]
    segs, t = [], 0.0
    for s, e in zip(starts, ends + [d] * (len(starts) - len(ends))):
        if s > t + 0.05:
            segs.append((t, s))
        t = e
    if t < d - 0.05:
        segs.append((t, d))
    return [(a, b) for a, b in segs if b - a > 0.12]


def group(segs, k):
    """Joins fine speech segments into k chunks at the k-1 longest gaps."""
    if len(segs) < k:
        return None
    gaps = sorted(range(len(segs) - 1), key=lambda i: segs[i + 1][0] - segs[i][1], reverse=True)[:k - 1]
    cuts = sorted(gaps)
    out, a = [], 0
    for c in cuts + [len(segs) - 1]:
        out.append((segs[a][0], segs[c][1]))
        a = c + 1
    return out


def word_clock(segs, texts):
    """Maps a word position (index into the joined texts) to a time in the recording,
    spreading the words over the speech in proportion to its duration."""
    total_w = sum(words(t) for t in texts)
    total_s = sum(b - a for a, b in segs)

    def at(w):
        target = total_s * w / max(total_w, 1)
        acc = 0.0
        for a, b in segs:
            if acc + (b - a) >= target:
                return a + (target - acc)
            acc += b - a
        return segs[-1][1]
    return at


# ---- rendering --------------------------------------------------------------
class Fonts:
    def __init__(self, d):
        self.d = d

    def f(self, name):
        p = os.path.join(self.d, name) if self.d else ""
        return p if p and os.path.exists(p) else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

    @property
    def body(self):
        return self.f("IBMPlexSans-Regular.ttf")

    @property
    def bold(self):
        return self.f("SpaceGrotesk-Bold.ttf")

    @property
    def mono(self):
        return self.f("IBMPlexMono-Regular.ttf")


def run(cmd):
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode:
        raise SystemExit("ffmpeg failed:\n" + " ".join(cmd) + "\n" + r.stderr[-3000:])


class Captions:
    """drawtext filters, each from its own text file so nothing needs escaping."""

    def __init__(self, tmp, fonts):
        self.tmp, self.fonts, self.n = tmp, fonts, 0

    def __call__(self, text, x, y, size, color="white", font=None, box=False, when=None):
        self.n += 1
        p = os.path.join(self.tmp, f"cap{self.n}.txt")
        open(p, "w").write(text)
        f = font or self.fonts.body
        s = f"drawtext=fontfile={f}:textfile={p}:x={x}:y={y}:fontsize={size}:fontcolor={color}:line_spacing=8"
        if box:
            s += ":box=1:boxcolor=0x14202BCC:boxborderw=18"
        if when:
            s += f":enable='between(t,{when[0]:.3f},{when[1]:.3f})'"
        return s


def still(png, dur, out, caps=()):
    vf = ",".join(["scale=1920:1080,setsar=1", *caps, "format=yuv420p"])
    run(["ffmpeg", "-y", "-loglevel", "error", "-loop", "1", "-framerate", str(FPS), "-t", f"{dur:.3f}", "-i", png,
         "-vf", vf, "-r", str(FPS), "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-an", out])


def clip(src, start, dur, out, caps=(), console=False):
    """A video segment. A console file is letterboxed above a caption band; a phone
    clip fills the frame. Held on its last frame if the segment outruns it."""
    if console:
        geo = "scale=1590:960,pad=1920:1080:165:0:color=" + BG
    else:
        geo = "scale=1920:1080"
    vf = ",".join([geo, "setsar=1", f"tpad=stop_mode=clone:stop_duration={dur:.3f}", *caps, "format=yuv420p"])
    run(["ffmpeg", "-y", "-loglevel", "error", "-ss", f"{start:.3f}", "-i", src, "-t", f"{dur:.3f}", "-vf", vf,
         "-r", str(FPS), "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-an", out])


def concat(parts, out):
    lst = out + ".txt"
    with open(lst, "w") as fh:
        for p in parts:
            fh.write(f"file '{p}'\n")
    run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", lst, "-c", "copy", out])


def mux(video, audio_filter_inputs, out, dur):
    """Muxes picture and sound; fades the scene in and out over 0.3 s."""
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-i", video]
    for a in audio_filter_inputs["inputs"]:
        cmd += ["-i", a]
    fade = f"fade=t=in:st=0:d=0.3,fade=t=out:st={max(dur - 0.3, 0):.3f}:d=0.3"
    if audio_filter_inputs["inputs"]:
        fc = f"[0:v]{fade}[v];{audio_filter_inputs['graph']}"
        cmd += ["-filter_complex", fc, "-map", "[v]", "-map", "[a]"]
    else:
        cmd += ["-f", "lavfi", "-t", f"{dur:.3f}", "-i", "anullsrc=r=48000:cl=stereo",
                "-filter_complex", f"[0:v]{fade}[v]", "-map", "[v]", "-map", f"{1}:a"]
    cmd += ["-t", f"{dur:.3f}", "-c:v", "libx264", "-crf", "18", "-preset", "medium", "-c:a", "aac", "-b:a", "192k",
            "-ar", "48000", "-ac", "2", out]
    run(cmd)


# ---- the takes ----------------------------------------------------------
def take_t0(name):
    db = sorted(glob.glob(os.path.join(ROOT, f"runs/runtime/*_{name}/metrics.sqlite")))[-1]
    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    return con.execute("select min(stamp_ns) from frames").fetchone()[0] / 1e9


def phone_start(clip_path):
    n = os.path.basename(clip_path)
    t = datetime.strptime(n[4:19], "%Y%m%d_%H%M%S").replace(tzinfo=timezone.utc).timestamp()
    return t + int(n[19:22]) / 1000


# ---- scene specs ------------------------------------------------------------
P = lambda n: os.path.join(SLIDE, n + ".png")
RE = lambda n: os.path.join(V, f"replay_{n}.avi")
PH = lambda n: os.path.join(V, "phone", f"PXL_{n}.mp4")
REPLAY = "REPLAY of a recorded scene"

# voice-led scenes: (first words of the sentence the picture changes at, picture)
# picture: ("still", png, [captions]) | ("clip", src, src_start, console?, [captions])
VOICE_LED = {
    2: [(None, ("still", P("01_title"), []))],
    3: [(None, ("still", P("02_problem_1"), [])),
        ("A second camera can see it again", ("still", P("03_problem_2"), [])),
        ("So there are three questions", ("still", P("04_problem_3"), []))],
    4: [(None, ("clip", PH("20261010_142806282"), 0.0, False, [])),
        ("The software pairs their frames", ("clip", PH("20261010_143009327"), 0.0, False,
                                             ["Each camera is selected by its serial number, never by the order it was plugged in"])),
        ("The cameras are calibrated against this board", ("clip", PH("20261010_142827160"), 0.0, False, [])),
        ("The three objects are 3D printed", ("clip", PH("20261010_142845216"), 0.0, False, [])),
        ("The grey marker trays", ("clip", PH("20261010_142916406"), 0.0, False, []))],
    5: [(None, ("still", P("06_pipeline"), [])),
        ("The top row runs once", ("still", P("07_pipeline_lit0"), [])),
        ("A detector finds the objects", ("still", P("08_pipeline_lit1"), [])),
        ("Then FoundationPose", ("still", P("09_pipeline_lit2"), [])),
        ("The bottom row runs once", ("still", P("06_pipeline"), [])),
        ("Every pose is moved into one shared", ("still", P("10_pipeline_lit3"), [])),
        ("Estimates of the same object", ("still", P("12_pipeline_lit5"), [])),
        ("If the two views disagree", ("still", P("13_pipeline_lit6"), [])),
        ("The last stage turns geometry", ("still", P("14_pipeline_lit7"), [])),
        ("This console shows all of it", ("clip", RE("20261007_t2_clutter_a"), 3.0, True,
                                          [REPLAY + " · t2_clutter_a",
                                           "1 the two camera views   2 the table from above   3 the scene graph   4 the facts a planner reads"]))],
    7: [(None, ("still", P("18_ground_truth"), [])),
        ("I recorded 16 scenes", ("still", P("19_scenes"), [])),
        ("The score is A-D-D-S", ("still", P("18_ground_truth"), []))],
    8: [(None, ("clip", RE("20261005_t4_cube_over"), 2.0, True, [REPLAY + " · t4_cube_over: the card hides the cube from cam_over"])),
        ("In every one, the blocked camera", ("clip", RE("20261005_t4_cube_front"), 2.0, True,
                                              [REPLAY + " · t4_cube_front: the card hides the cube from cam_front"])),
        ("The fused output has it", ("still", P("20_hidden"), [])),
        ("With nothing hidden, two views", ("still", P("21_combine"), []))],
    9: [(None, ("clip", RE("20261006_t3_bottle_over_b_byscore"), 2.0, True,
                [REPLAY + " · t3_bottle_over_b under the old rule: keep the higher score"])),
        ("The fix does not use the score", ("still", P("22_wrong_camera"), [])),
        ("With that check, the two recordings", ("clip", RE("20261006_t3_bottle_over_b"), 2.0, True,
                                                 [REPLAY + " · the same scene with the depth check"])),
        ("Of about a thousand estimates", ("still", P("22_wrong_camera"), []))],
    10: [(None, ("clip", os.path.join(V, "synthetic_detector/replay_20261005_t3_cube_front.avi"), 2.0, True,
                 [REPLAY + " · synthetic-only detector: a cube on the card's red print"])),
         ("None of these false objects", ("still", P("23_not_there"), [])),
         ("The other fix was at the source", ("clip", RE("20261005_t3_cube_front"), 2.0, True,
                                              [REPLAY + " · the same scene, fine-tuned detector"])),
         ("On the sixteen scored scenes", ("still", P("23_not_there"), []))],
    11: [(None, ("clip", RE("20261007_t2_clutter_c"), 2.0, True,
                 ["What it still gets wrong · " + REPLAY + " · t2_clutter_c"])),
         ("The occluded-from fact is the weakest", ("clip", os.path.join(V, "retimed/demo_live_d.mp4"), 91.2, True,
                                                    ["What it still gets wrong · the live take at 91 s",
                                                     "For one frame the bowl is counted twice: bowl_02 and bowl_04, and the copy reads as hidden from cam_front"])),
         ("And a half-hidden object", ("still", P("24_predicates"), [])),
         ("One result surprised me", ("still", P("26_depth"), []))],
    12: [(None, ("still", P("28_output"), [])),
         ("Two cameras do see more than one", ("still", P("29_close"), []))],
}

# picture-led scenes: the take, its phone clip, the window, and each chunk's beat
PICTURE_LED = {
    1: dict(take="demo_cold_e", console=os.path.join(V, "retimed/demo_cold_e.mp4"),
            phone=PH("20261025_074919969"), window=(12.0, 47.0),
            beats=[12.5, 19.6, 23.0, 31.5], side=[], events=[]),
    6: dict(take="demo_live_d", console=os.path.join(V, "retimed/demo_live_d.mp4"),
            phone=PH("20261025_080801961"), window=(-4.0, 135.0),
            beats=[-3.5, 8.0, 14.8, 17.5, 25.5, 37.0, 42.0, 50.5, 59.5, 68.5, 71.0, 81.0, 96.5, 101.5, 120.0, 123.5, 126.5],
            side=["13.3 frame pairs a second", "over the take;", "10 with three objects", "tracked in each view"],
            events=[((15.6, 20.4), "Freeze: the pose estimator is finding a new object, about a second in each camera."),
                    ((90.9, 94.6), "For this one frame the bowl is counted twice, and the copy reads as hidden from the\n"
                                   "front camera. See: What it still gets wrong.")]),
}


def caption_band(cap, lines, boxed=False):
    """Captions for the band under a letterboxed console, or boxed over a phone clip."""
    out = []
    for i, t in enumerate(lines):
        out.append(cap(t, "165", str(984 + i * 44 - (22 if len(lines) > 1 else 0)), 30 if i == 0 else 26,
                       color="white" if i == 0 else "0xC8D3D8", box=boxed))
    return out


def build_voice_led(scene, tmp, cap, dur_out):
    texts = chunks(scene)
    vf = voice_file(scene)
    head, tail = 0.6, 0.9
    if vf:
        segs = speech(vf)
        a0, a1 = segs[0][0], segs[-1][1]
        at = word_clock(segs, texts)
        span = a1 - a0
    else:
        a0 = 0.0
        span = sum(words(t) for t in texts) / WPM * 60 + 0.6 * (len(texts) - 1)
        at = lambda w: span * w / max(sum(words(t) for t in texts), 1)
    total = head + span + tail
    joined = " ".join(texts)
    # each cue's time in the scene
    times = []
    for key, pic in VOICE_LED[scene]:
        if key is None:
            times.append(0.0)
            continue
        i = joined.find(key)
        if i < 0:
            raise SystemExit(f"scene {scene}: cue {key!r} is not in the script")
        times.append(head + at(words(joined[:i])) - a0 - 0.15)
    times.append(total)
    parts = []
    for k, (key, pic) in enumerate(VOICE_LED[scene]):
        d = max(times[k + 1] - times[k], 0.5)
        out = os.path.join(tmp, f"s{scene}_{k}.mp4")
        if pic[0] == "still":
            still(pic[1], d, out)
        else:
            _, src, s0, console, lines = pic
            clip(src, s0, d, out, caption_band(cap, lines, boxed=not console) if lines else (), console=console)
        parts.append(out)
    video = os.path.join(tmp, f"s{scene}_v.mp4")
    concat(parts, video)
    if vf:
        audio = {"inputs": [vf], "graph": f"[1:a]atrim={a0:.3f}:{a1:.3f},asetpts=PTS-STARTPTS,"
                                          f"adelay={int(head * 1000)}|{int(head * 1000)},apad[a]"}
    else:
        audio = {"inputs": [], "graph": ""}
    mux(video, audio, dur_out, total)
    return total, bool(vf)


def build_picture_led(scene, tmp, cap, fonts, dur_out):
    s = PICTURE_LED[scene]
    w0, w1 = s["window"]
    texts = chunks(scene)
    if len(texts) != len(s["beats"]):
        raise SystemExit(f"scene {scene}: {len(texts)} chunks in the script, {len(s['beats'])} beats")
    vf = voice_file(scene)
    if vf:
        segs = group(speech(vf), len(texts)) or None
        if segs is None:
            raise SystemExit(f"scene {scene}: the recording has fewer pauses than the script's {len(texts)} chunks")
    else:
        segs = None
    # chunk placement on the take's clock
    place, t_free = [], w0
    for i, b in enumerate(s["beats"]):
        d = (segs[i][1] - segs[i][0]) if segs else words(texts[i]) / WPM * 60
        st = max(b, t_free + 0.25)
        place.append((st, d))
        t_free = st + d
    end = max(w1, t_free + 1.0)
    total = end - w0
    off = phone_start(s["phone"]) - take_t0(s["take"]) - PHONE_LAG  # phone time = take time - off
    side = [cap("LIVE", "1468", "300", 46, color="0x6CCABD", font=fonts.bold),
            cap("on the rig, 25 Oct 2026", "1468", "362", 26),
            cap("one take, not replayed", "1468", "398", 26, color="0xC8D3D8")]
    for i, t in enumerate(s["side"]):
        side.append(cap(t, "1468", str(470 + i * 34), 24, color="0xC8D3D8"))
    for (a, b), t in s["events"]:
        side.append(cap(t, "24", "905", 30, when=(a - w0, b - w0)))
    # before the take's first pair the console holds its first frame; the phone was already running
    fc = (f"[0:v]trim=start={max(w0, 0):.3f}:duration={total:.3f},setpts=PTS-STARTPTS,scale=1440:869,setsar=1,"
          f"tpad=start_mode=clone:start_duration={max(-w0, 0):.3f}:stop_mode=clone:stop_duration={total:.3f}[c];"
          f"[1:v]trim=start={max(w0 - off, 0):.3f}:duration={total:.3f},setpts=PTS-STARTPTS,scale=448:252,setsar=1,"
          f"tpad=stop_mode=clone:stop_duration={total:.3f}[p];"
          f"color=c={BG}:s=1920x1080:r={FPS}:d={total:.3f}[bg];"
          f"[bg][c]overlay=16:16[b1];[b1][p]overlay=1464:16[b2];[b2]{','.join(side)},format=yuv420p[v]")
    video = os.path.join(tmp, f"s{scene}_v.mp4")
    run(["ffmpeg", "-y", "-loglevel", "error", "-i", s["console"], "-i", s["phone"], "-filter_complex", fc,
         "-map", "[v]", "-t", f"{total:.3f}", "-r", str(FPS), "-c:v", "libx264", "-crf", "18", "-preset", "medium",
         "-an", video])
    if vf:
        graph, labels = [], []
        for i, ((a, b), (st, d)) in enumerate(zip(segs, place)):
            ms = int((st - w0) * 1000)
            graph.append(f"[1:a]atrim={a:.3f}:{b:.3f},asetpts=PTS-STARTPTS,adelay={ms}|{ms}[a{i}]")
            labels.append(f"[a{i}]")
        graph.append(f"{''.join(labels)}amix=inputs={len(labels)}:normalize=0,apad[a]")
        audio = {"inputs": [vf], "graph": ";".join(graph)}
    else:
        audio = {"inputs": [], "graph": ""}
    mux(video, audio, dur_out, total)
    return total, bool(vf), [(round(st, 1), round(d, 1)) for st, d in place]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenes", default="1,2,3,4,5,6,7,8,9,10,11,12")
    ap.add_argument("--fonts", help="directory holding IBMPlexSans-Regular.ttf and SpaceGrotesk-Bold.ttf")
    ap.add_argument("--final", action="store_true", help="join all twelve scenes into the video")
    a = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    fonts = Fonts(a.fonts)
    total = 0.0
    with tempfile.TemporaryDirectory(dir=OUT) as tmp:
        cap = Captions(tmp, fonts)
        for sc in [int(x) for x in a.scenes.split(",")]:
            out = os.path.join(OUT, f"scene{sc:02d}.mp4")
            if sc in PICTURE_LED:
                d, has_voice, place = build_picture_led(sc, tmp, cap, fonts, out)
                extra = f"  chunks at {place}"
            else:
                d, has_voice = build_voice_led(sc, tmp, cap, out)
                extra = ""
            total += d
            print(f"scene {sc:2d}: {d:6.1f} s  {'voice' if has_voice else 'NO VOICE (timed from words)'}{extra}")
    print(f"total of these scenes: {total / 60:.1f} min")
    if a.final:
        parts = [os.path.join(OUT, f"scene{i:02d}.mp4") for i in range(1, 13)]
        missing = [p for p in parts if not os.path.exists(p)]
        if missing:
            raise SystemExit(f"missing {missing}")
        concat(parts, FINAL)
        print("wrote", FINAL, f"{duration(FINAL) / 60:.2f} min")


if __name__ == "__main__":
    main()
