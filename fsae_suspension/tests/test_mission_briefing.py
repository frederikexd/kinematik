# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
#
#  tests/test_mission_briefing.py — the mission-briefing engine, pinned.
# ============================================================================
"""What these tests guard, without importing Streamlit:

The mission briefing lives in streamlit_app.py, which imports Streamlit and so
can't be imported in a headless CI run. But the parts that matter here are pure
data (the per-tool copy dicts) and one pure compiler (_build_briefing). We reach
them by parsing the module's AST and either inspecting the dict literals or
exec-ing just the compiler plus its real dependencies — so these tests exercise
the ACTUAL code, not a reimplementation.

The invariants, in the order they'd silently rot:

* COVERAGE — every tab in _TAB_META has briefing copy in _BRIEF_TOOLS, a
  plain-English gloss in _BRIEF_SIMPLE, and a feature list in
  _BRIEF_TOOL_FEATURES. A new tab added without briefing copy is the classic
  regression (it happened to genesis_fc/frames/phantom_env/cost); this fails
  the moment it recurs.
* REACHABILITY — the full-car synthesis goals actually surface genesis_fc, so
  the FullCar tab is recommendable from the questionnaire rather than orphaned.
* PROFICIENCY — _build_briefing accepts and stores the proficiency axis, and a
  briefing compiles at each level (beginner/intermediate/advanced) with the
  chosen tools intact. This is the subsystem/goal/proficiency trio the upgrade
  added.
"""

import ast
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

_APP = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    "streamlit_app.py")


def _load_app_tree():
    with open(_APP, encoding="utf-8") as f:
        src = f.read()
    return src, ast.parse(src)


def _dict_keys(tree, name):
    """Keys of a top-level `name = { ... }` dict literal, or None."""
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for t in node.targets:
                if (isinstance(t, ast.Name) and t.id == name
                        and isinstance(node.value, ast.Dict)):
                    return [k.value for k in node.value.keys
                            if isinstance(k, ast.Constant)]
    return None


def _tab_ids(tree):
    keys = _dict_keys(tree, "_TAB_META")
    assert keys, "_TAB_META not found as a dict literal"
    return set(keys)


# --------------------------------------------------------------------------- #
#  Coverage — every tab is briefed at every depth
# --------------------------------------------------------------------------- #
def test_every_tab_has_full_briefing_copy():
    _src, tree = _load_app_tree()
    tabs = _tab_ids(tree)
    for dict_name in ("_BRIEF_TOOLS", "_BRIEF_SIMPLE", "_BRIEF_TOOL_FEATURES"):
        keys = set(_dict_keys(tree, dict_name) or [])
        missing = sorted(tabs - keys)
        assert not missing, f"{dict_name} is missing copy for: {missing}"


def test_new_tools_specifically_covered():
    # the four that were missing before this upgrade — a targeted guard
    _src, tree = _load_app_tree()
    for dict_name in ("_BRIEF_TOOLS", "_BRIEF_SIMPLE", "_BRIEF_TOOL_FEATURES"):
        keys = set(_dict_keys(tree, dict_name) or [])
        for tid in ("genesis_fc", "frames", "phantom_env", "cost"):
            assert tid in keys, f"{tid} absent from {dict_name}"


# --------------------------------------------------------------------------- #
#  The pure compiler — exec _build_briefing with its real dependencies
# --------------------------------------------------------------------------- #
def _load_build_briefing():
    src, tree = _load_app_tree()
    need = {"_BRIEF_PURPOSES", "_ROLE_GOALS", "_VERIFY_GOALS",
            "_FREETEXT_KEYWORDS", "_FULL_ORDER", "_TAB_META", "_BRIEF_TOOLS",
            "_BRIEF_SIMPLE", "_BRIEF_TOOL_FEATURES", "_BRIEF_GOAL_FEATURES",
            "_ROLE_LABELS", "_CAT_LABEL", "_ID_CATEGORY", "_TAB_CATEGORIES",
            "_build_briefing", "_brief_goal_options",
            "_freetext_matched_tools", "_BRIEF_PURPOSE_MAP",
            "_briefing_ordered_tools", "_briefing_feature_lines",
            "_briefing_to_text", "_briefing_offline_html"}
    segs = []
    for node in tree.body:
        names = []
        if isinstance(node, ast.Assign):
            names = [t.id for t in node.targets if isinstance(t, ast.Name)]
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target,
                                                            ast.Name):
            names = [node.target.id]
        elif isinstance(node, ast.FunctionDef):
            names = [node.name]
        if names and any(n in need for n in names):
            segs.append((node.lineno, ast.get_source_segment(src, node)))
        elif isinstance(node, ast.For):
            seg = ast.get_source_segment(src, node)
            if seg and "_ID_CATEGORY[" in seg:
                segs.append((node.lineno, seg))
    segs.sort()
    ns: dict = {}
    exec("from __future__ import annotations\n"
         + "\n".join(s for _, s in segs), ns)  # noqa: S102 — trusted repo source
    return ns


def test_build_briefing_accepts_and_stores_proficiency():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    for prof in ("beginner", "intermediate", "advanced"):
        bf = build(["suspension"], "design", [], "", style="visual",
                   proficiency=prof)
        assert bf["proficiency"] == prof
        assert bf["style"] == "visual"


def test_fullcar_goal_recommends_genesis_fc_at_every_level():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    opts = {k: (lab, ids)
            for k, lab, ids in ns["_brief_goal_options"](["powertrain"])}
    synth = next((k for k, (lab, ids) in opts.items()
                  if "genesis_fc" in ids), None)
    assert synth, "no powertrain goal surfaces genesis_fc"
    for prof in ("beginner", "intermediate", "advanced"):
        bf = build(["powertrain"], "design", [synth], "",
                   style="visual", proficiency=prof)
        assert "genesis_fc" in bf["core_tabs"], \
            f"genesis_fc not recommended at {prof}"


def test_default_proficiency_is_intermediate():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    bf = build(["suspension"], "design", [], "")   # no proficiency passed
    assert bf["proficiency"] == "intermediate"


def test_freetext_still_expands_the_toolbox():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    bf = build(["powertrain"], "design", [], "pack overheats in endurance",
               style="numbers", proficiency="advanced")
    # the note should pull in at least one energy/thermal-related tab
    assert bf["note_tabs"], "freetext note added no tools"


# --------------------------------------------------------------------------- #
#  Audio + PDF — the briefing compiles to spoken text and a PDF-ready markdown
# --------------------------------------------------------------------------- #
def test_briefing_compiles_to_text_and_speech():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    to_text = ns["_briefing_to_text"]
    bf = build(["suspension"], "design", [], "", style="visual",
               proficiency="intermediate")
    md, speech = to_text(bf)
    assert md.startswith("# KinematiK — Your Mission Briefing")
    assert "Your tool plan" in md
    assert len(speech) > 100          # a real spoken script, not empty
    # speech must be free of the markdown/glyph noise the cleaner strips
    for bad in ("**", "→", "×", "“", "”"):
        assert bad not in speech


def test_briefing_text_depth_tracks_proficiency():
    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    to_text = ns["_briefing_to_text"]
    opts = {k: (lab, ids)
            for k, lab, ids in ns["_brief_goal_options"](["powertrain"])}
    synth = next(k for k, (lab, ids) in opts.items() if "genesis_fc" in ids)

    md_beg, _ = to_text(build(["powertrain"], "design", [synth], "",
                              style="visual", proficiency="beginner"))
    md_int, _ = to_text(build(["powertrain"], "design", [synth], "",
                              style="visual", proficiency="intermediate"))
    md_adv, _ = to_text(build(["powertrain"], "design", [synth], "",
                              style="visual", proficiency="advanced"))

    # the FullCar tool is present in the PDF markdown at every level
    for md in (md_beg, md_int, md_adv):
        assert "InverseGenesis-FullCar" in md

    # beginner adds plain-English; advanced collapses the external-tool
    # comparison; intermediate keeps it and is between the two in length
    assert "In plain English" in md_beg
    assert "Why here, not MATLAB" in md_int
    assert "Why here, not MATLAB" not in md_adv
    assert len(md_beg) > len(md_int) > len(md_adv)


def test_briefing_markdown_renders_to_pdf():
    import os
    import tempfile
    from suspension import project as pj

    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    to_text = ns["_briefing_to_text"]
    bf = build(["powertrain"], "design", [], "pack overheats in endurance",
               style="visual", proficiency="beginner")
    md, _ = to_text(bf)
    out = os.path.join(tempfile.gettempdir(), "t_mission_briefing.pdf")
    pj.render_pdf(md, out)             # must not raise on any briefing content
    assert os.path.getsize(out) > 1000
    os.unlink(out)


# --------------------------------------------------------------------------- #
#  Offline audio — a self-contained, downloadable HTML player
# --------------------------------------------------------------------------- #
def test_offline_audio_html_is_self_contained_and_safe():
    import json
    import re

    ns = _load_build_briefing()
    offline = ns["_briefing_offline_html"]
    # speech with the exact characters that break naive templating
    speech = 'He said "go" & <stop> now.\nTool 1: FullCar. Why: it works.'
    html = offline(speech, "KinematiK Mission Briefing")

    assert html.startswith("<!DOCTYPE html>")
    # no unreplaced template tokens
    for tok in ("__PAYLOAD__", "__READABLE__", "__TITLE__"):
        assert tok not in html
    # the human-readable block is HTML-escaped (no raw injection)
    assert "&lt;stop&gt;" in html and "&amp;" in html
    # the JS payload is valid JSON and round-trips to the exact speech
    m = re.search(r'var text = (".*?");\n', html, re.S)
    assert m and json.loads(m.group(1)) == speech
    # truly offline: no external network references
    for bad in ("http://", "https://", "cdn", "src="):
        assert bad not in html


def test_offline_audio_html_end_to_end_all_levels():
    import json
    import re

    ns = _load_build_briefing()
    build = ns["_build_briefing"]
    to_text = ns["_briefing_to_text"]
    offline = ns["_briefing_offline_html"]
    opts = {k: (lab, ids)
            for k, lab, ids in ns["_brief_goal_options"](["powertrain"])}
    synth = next(k for k, (lab, ids) in opts.items() if "genesis_fc" in ids)

    for prof in ("beginner", "intermediate", "advanced"):
        bf = build(["powertrain"], "design", [synth], "",
                   style="visual", proficiency=prof)
        _md, speech = to_text(bf)
        html = offline(speech, "KinematiK Mission Briefing")
        assert "speechSynthesis" in html
        # the FullCar tool is in the spoken script, hence in the offline file
        assert "InverseGenesis-FullCar" in speech
        m = re.search(r'var text = (".*?");\n', html, re.S)
        assert m and json.loads(m.group(1)) == speech


# --------------------------------------------------------------------------- #
#  Downloadable MP3 (optional, best-effort) — must degrade, never crash
# --------------------------------------------------------------------------- #
def _load_mp3_fn():
    import ast as _ast
    with open(_APP, encoding="utf-8") as f:
        src = f.read()
    tree = _ast.parse(src)
    seg = next(_ast.get_source_segment(src, n) for n in tree.body
               if isinstance(n, _ast.FunctionDef)
               and n.name == "_briefing_mp3_bytes")
    ns: dict = {}
    exec("from __future__ import annotations\n" + seg, ns)  # noqa: S102
    return ns["_briefing_mp3_bytes"]


def test_mp3_empty_text_returns_none():
    f = _load_mp3_fn()
    assert f("") is None
    assert f("   ") is None


def test_mp3_missing_dependency_degrades_to_none(monkeypatch):
    # simulate gTTS not installed: import must fail inside the function, and
    # the function must return None rather than raise.
    import builtins
    _real = builtins.__import__

    def _blocked(name, *a, **k):
        if name == "gtts" or name.startswith("gtts."):
            raise ImportError("gtts not installed")
        return _real(name, *a, **k)

    monkeypatch.setattr(builtins, "__import__", _blocked)
    f = _load_mp3_fn()
    assert f("Some briefing text that would otherwise be spoken.") is None


def test_mp3_chunks_on_sentence_boundaries(monkeypatch):
    # stub gTTS so we can assert chunking without any network: every request
    # must be <=450 chars, and the fragments concatenate into one blob.
    import sys
    import types

    calls = []

    class _FakeGTTS:
        def __init__(self, text, lang="en", tld="co.uk"):
            calls.append(len(text))
            assert len(text) <= 450

        def write_to_fp(self, fp):
            fp.write(b"MP3")

    fake = types.ModuleType("gtts")
    fake.gTTS = _FakeGTTS
    monkeypatch.setitem(sys.modules, "gtts", fake)

    f = _load_mp3_fn()
    speech = " ".join(f"Sentence {i} describes a tool feature in detail."
                      for i in range(60))
    data = f(speech)
    assert data and len(calls) > 1
    assert all(c <= 450 for c in calls)


# --------------------------------------------------------------------------- #
#  Local, no-network TTS via piper — resolver + transcode, degrade cleanly
# --------------------------------------------------------------------------- #
def _load_local_audio_fns():
    import ast as _ast
    with open(_APP, encoding="utf-8") as f:
        src = f.read()
    tree = _ast.parse(src)
    names = {"_VM_PIPER_VOICE", "_vm_piper_cache_root", "_vm_piper_is_model",
             "_vm_piper_find_local", "_vm_piper_model_path",
             "_briefing_piper_wav_bytes", "_wav_bytes_to_mp3",
             "_briefing_local_audio"}
    segs = []
    for n in tree.body:
        if (isinstance(n, _ast.FunctionDef) and n.name in names) or \
           (isinstance(n, _ast.Assign)
                and any(getattr(t, "id", None) in names for t in n.targets)):
            segs.append((n.lineno, _ast.get_source_segment(src, n)))
    segs.sort()
    ns: dict = {"__file__": os.path.abspath(_APP)}
    exec("from __future__ import annotations\n"
         + "\n".join(s for _, s in segs), ns)  # noqa: S102
    return ns


def test_piper_is_model_gate():
    ns = _load_local_audio_fns()
    is_model = ns["_vm_piper_is_model"]
    assert is_model(None) is False
    assert is_model("/definitely/not/here.onnx") is False


def test_local_audio_degrades_to_none_without_model(monkeypatch):
    # no env override, no committed model, no download attempt -> clean None
    monkeypatch.delenv("KINEMATIK_PIPER_MODEL", raising=False)
    ns = _load_local_audio_fns()
    assert ns["_briefing_piper_wav_bytes"]("hello", download=False) is None
    assert ns["_briefing_local_audio"]("hello", download=False) \
        == (None, None, None)


def test_wav_to_mp3_transcode_or_wav_fallback():
    # exercises the real transcoder: with ffmpeg present -> mp3; the function
    # must never raise and must return a valid (bytes, ext, mime) triple.
    import io
    import math
    import struct
    import wave

    ns = _load_local_audio_fns()
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(22050)
        w.writeframes(b"".join(
            struct.pack("<h", int(8000 * math.sin(2 * math.pi * 440 * i / 22050)))
            for i in range(22050)))
    data, ext, mime = ns["_wav_bytes_to_mp3"](buf.getvalue())
    assert data and ext in ("mp3", "wav")
    assert mime in ("audio/mpeg", "audio/wav")


def test_wav_to_mp3_handles_empty():
    ns = _load_local_audio_fns()
    assert ns["_wav_bytes_to_mp3"](b"") == (None, None, None)


# --------------------------------------------------------------------------- #
#  Karaoke word-highlight — the spoken word is marked in the transcript
# --------------------------------------------------------------------------- #
def _load_offline_html_fn():
    import ast as _ast
    with open(_APP, encoding="utf-8") as f:
        src = f.read()
    tree = _ast.parse(src)
    seg = next(_ast.get_source_segment(src, n) for n in tree.body
               if isinstance(n, _ast.FunctionDef)
               and n.name == "_briefing_offline_html")
    ns: dict = {}
    exec("from __future__ import annotations\n" + seg, ns)  # noqa: S102
    return ns["_briefing_offline_html"]


def _capture_inapp_audio_html():
    """Capture the HTML _render_briefing_audio hands to components.html,
    by stubbing streamlit so no real Streamlit runtime is needed."""
    import ast as _ast
    import sys
    import types

    with open(_APP, encoding="utf-8") as f:
        src = f.read()
    tree = _ast.parse(src)
    seg = next(_ast.get_source_segment(src, n) for n in tree.body
               if isinstance(n, _ast.FunctionDef)
               and n.name == "_render_briefing_audio")

    captured = {}
    comp = types.ModuleType("streamlit.components.v1")
    comp.html = lambda h, height=None: captured.__setitem__("html", h)
    saved = {k: sys.modules.get(k) for k in
             ("streamlit", "streamlit.components", "streamlit.components.v1")}
    sys.modules["streamlit"] = types.ModuleType("streamlit")
    sys.modules["streamlit.components"] = types.ModuleType(
        "streamlit.components")
    sys.modules["streamlit.components.v1"] = comp
    try:
        ns: dict = {}
        exec("from __future__ import annotations\n" + seg, ns)  # noqa: S102
        ns["_render_briefing_audio"]("Word one. Word two here.", key="k")
        return captured.get("html", "")
    finally:
        for k, v in saved.items():
            if v is None:
                sys.modules.pop(k, None)
            else:
                sys.modules[k] = v


def test_inapp_player_has_word_highlight_machinery():
    html = _capture_inapp_audio_html()
    assert html
    # boundary event drives the highlight; word spans + active-word CSS exist;
    # the current word is scrolled into view
    assert "u.onboundary" in html
    assert 'className = "kkw"' in html or ".kkw" in html
    assert ".kkw.on{" in html
    assert "scrollTo(" in html


def test_offline_html_has_word_highlight_machinery():
    offline = _load_offline_html_fn()
    html = offline("Word one. Word two here.", "KinematiK Mission Briefing")
    assert "u.onboundary" in html
    assert ".kkw.on{" in html
    assert "scrollTo(" in html
    # the static escaped transcript remains as the no-JS / no-speech fallback
    assert 'id="script"' in html
