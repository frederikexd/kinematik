# ============================================================================
#  KinematiK — Formula SAE suspension & vehicle dynamics toolkit
#  Created by Frederik Thio. Copyright (c) 2026 Frederik Thio.
#  Open source. Original author: Frederik Thio, creator of KinematiK.
# ============================================================================
"""The KinematiK Guide: a chat-style helper that answers questions from KinematiK's own docs.

Why: new members ask the same things ("where do I change the mass?", "what is bump steer?", "what should I open
first?"), and the answers already exist in the app (the mission briefing's per-tab copy, the role goals, the FAQ
below). The guide puts them behind one chat box so a question gets a direct answer instead of a hunt through 40 tabs.

What it is NOT: an AI. Nothing leaves the page and nothing is generated. A question is matched against a fixed
knowledge base (keyword overlap weighted by rarity, with spelling-tolerant matching and FSAE synonyms), and the
best entry's prewritten answer is returned. When no entry fits well it says so and offers the closest questions
instead of guessing. The guide says this about itself when asked.

Pure Python, no Streamlit import: the app builds the knowledge base (FAQ below + entries made from its own tab
copy) and renders the chat; this module only matches and formats.
"""
from __future__ import annotations

import difflib
import math
import re
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------
#  Text handling
# ---------------------------------------------------------------------------
_STOP = frozenset("""
a an the and or but if of to in on at by for with from into onto about as is are was were be been being am do does
did doing have has had i me my we our you your it its this that these those there here what which who whom how why
when where can could should would will shall may might must just so than then too very also not no yes please tell
explain show give get got want wanna need know let us any some one thing things stuff kinematik app tool
""".split())

# Spelling variants and FSAE shorthand -> one canonical token (applied to single tokens after lowercasing).
_CANON = {
    "centre": "center", "centres": "center", "tyre": "tire", "tyres": "tire", "optimiser": "optimizer",
    "optimise": "optimize", "colour": "color", "behaviour": "behavior", "litre": "liter", "metre": "meter",
    "rc": "rollcenter", "rcs": "rollcenter", "llt": "loadtransfer", "arb": "antirollbar", "arbs": "antirollbar",
    "kpi": "kingpin", "sai": "kingpin", "cg": "cog", "cog": "cog", "hp": "hardpoint", "hps": "hardpoint",
    "hardpoints": "hardpoint", "pickup": "hardpoint", "pickups": "hardpoint", "braking": "brake",
    "mr": "motionratio", "toe": "toe", "fos": "safetyfactor", "ev": "electric", "ice": "combustion",
    "bom": "cost", "cfd": "cfd", "fea": "fea", "lbs": "pound", "lb": "pound", "kg": "kilogram",
    "imperial": "units", "metric": "units", "si": "units",
}
# Two-word phrases that mean one thing; matched before tokenizing.
_PHRASES = [
    (r"roll[\s-]*cent(?:er|re)s?", " rollcenter "),
    (r"load[\s-]*transfer", " loadtransfer "),
    (r"anti[\s-]*roll[\s-]*bars?", " antirollbar "),
    (r"sway[\s-]*bars?", " antirollbar "),
    (r"motion[\s-]*ratios?", " motionratio "),
    (r"bump[\s-]*steer", " bumpsteer "),
    (r"camber[\s-]*gain", " cambergain "),
    (r"king[\s-]*pin", " kingpin "),
    (r"scrub[\s-]*radius", " scrubradius "),
    (r"anti[\s-]*dive", " antidive "),
    (r"anti[\s-]*squat", " antisquat "),
    (r"cent(?:er|re)\s+of\s+gravity", " cog "),
    (r"wheel[\s-]*rate", " wheelrate "),
    (r"spring[\s-]*rates?", " springrate "),
    (r"weight[\s-]*(?:split|distribution)", " weightsplit "),
    (r"lap[\s-]*times?", " laptime "),
    (r"garbage[\s-]*in", " garbagein "),
    (r"optimum\s*k", " optimumk "),
    (r"gt[\s-]*suite", " gtsuite "),
    (r"adams(?:/car)?", " adams "),
]


def _stem(t: str) -> str:
    """Strip one common suffix, keeping at least 4 letters (so "spring" stays "spring", "springs" -> "spring")."""
    for suf, keep in (("ies", "y"), ("ing", ""), ("ed", ""), ("es", ""), ("s", "")):
        if len(t) - len(suf) >= 4 and t.endswith(suf) and not t.endswith("ss"):
            return t[: -len(suf)] + keep
    return t


def tokens(text: str) -> list[str]:
    """Lowercase, fold FSAE phrases and spelling variants, drop stop words, light stemming. Text only (unitless)."""
    s = " " + (text or "").lower() + " "
    for pat, rep in _PHRASES:
        s = re.sub(pat, rep, s)
    out = []
    for raw in re.findall(r"[a-z0-9]+", s):
        t = _CANON.get(raw, raw)
        if t in _STOP or len(t) < 2:
            continue
        out.append(_CANON.get(_stem(t), _stem(t)))
    return out


# ---------------------------------------------------------------------------
#  Knowledge base
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Entry:
    """One answerable question. `answer` is markdown and may hold {placeholders} the app fills from live state
    (see Guide.reply's `context`). `tabs` are tab ids the answer points to; `related` are entry ids offered as
    follow-up questions."""
    id: str
    title: str
    answer: str
    keywords: tuple = ()
    tabs: tuple = ()
    related: tuple = ()


@dataclass
class Reply:
    text: str
    tabs: list = field(default_factory=list)
    suggestions: list = field(default_factory=list)   # entry titles offered as next questions
    entry_id: str | None = None
    confident: bool = True


class _Ctx(dict):
    def __missing__(self, k):
        return "{" + k + "}"


_GREET = re.compile(r"^\s*(hi+|hey+|hello+|yo|hiya|howdy|good (morning|afternoon|evening)|sup)\b[\s!.?]*$", re.I)
_THANKS = re.compile(r"\b(thanks?|thank you|thx|ty|cheers|appreciate it|awesome|great|perfect|nice|cool)\b", re.I)
_WHO = re.compile(r"\b(who|what) are you\b|\bare you (an? )?(ai|bot|robot|human|real|chatgpt|gpt|llm|claude)\b"
                  r"|\b(ai|chatgpt|gpt|llm)\b", re.I)
_ABOUT = re.compile(r"^\W*(what(\s+is|'s|s)?|tell me about|explain)\s+(kinematik|this(\s+(app|tool|thing))?)"
                    r"\W*$", re.I)
_HELP = re.compile(r"^\s*(help|\?|what can you do|what can i ask( you)?|menu|options)\s*[?!.]*\s*$", re.I)


class Guide:
    """Matches a question against the knowledge base. Build once per knowledge base; `reply` is cheap."""

    STRONG = 0.42      # normalized score above which the best entry is answered outright
    WEAK = 0.18        # below this nothing is close enough even to suggest

    def __init__(self, entries, starters=()):
        self.entries = list(entries)
        self.by_id = {e.id: e for e in self.entries}
        self.starters = list(starters)          # entry ids offered before the first question
        self._fields = []
        df: dict[str, int] = {}
        for e in self.entries:
            strong = set(tokens(e.title)) | set(tokens(" ".join(e.keywords)))
            weak = set(tokens(e.answer)) - strong
            self._fields.append((strong, weak))
            for t in strong | weak:
                df[t] = df.get(t, 0) + 1
        n = max(1, len(self.entries))
        self._idf = {t: math.log(1 + n / c) for t, c in df.items()}
        self._vocab = sorted(self._idf)

    # -- matching ---------------------------------------------------------
    def _resolve(self, toks):
        """Map unknown tokens to the closest known one (typos), keep known ones; drop the rest."""
        out = []
        for t in toks:
            if t in self._idf:
                out.append(t)
            elif len(t) >= 4:
                m = difflib.get_close_matches(t, self._vocab, n=1, cutoff=0.85)
                if m:
                    out.append(m[0])
        return out

    def rank(self, question: str):
        """[(normalized score 0..1, unitless; entry)] best first. The score is the share of the question's (rarity-weighted)
        words that the entry covers: in its title or keywords at full weight, only in its answer at 0.35."""
        q = self._resolve(tokens(question))
        if not q:
            return []
        q = list(dict.fromkeys(q))
        total = sum(self._idf.get(t, 0.0) for t in q) or 1.0
        scored = []
        for order, (e, (strong, weak)) in enumerate(zip(self.entries, self._fields)):
            s = sum(self._idf[t] * (1.0 if t in strong else 0.35 if t in weak else 0.0) for t in q)
            if s > 0:
                scored.append((s / total, order, e))
        # ties go to the earlier entry: the hand-written FAQ comes before the app-made entries
        scored.sort(key=lambda p: (-p[0], p[1]))
        return [(sc, e) for sc, _o, e in scored]

    # -- replies ------------------------------------------------------------
    def _titles(self, ids):
        return list(dict.fromkeys(self.by_id[i].title for i in ids if i in self.by_id))

    def welcome(self) -> Reply:
        """The opening message and starter questions (text only, unitless)."""
        return Reply("Hi! I'm the **KinematiK Guide**. Ask me where something is, what a term means, or what to "
                     "open first for your subteam.", suggestions=self._titles(self.starters), entry_id=None)

    def reply(self, question: str, context: dict | None = None) -> Reply:
        """The answer to one question (text only, unitless). `context` fills {placeholders} in answers with live
        values, already formatted with their units by the app."""
        q = (question or "").strip()
        ctx = _Ctx(context or {})
        if not q:
            return self.welcome()
        if _HELP.match(q):
            return Reply("I can help with:\n- **Where things are**: \"where do I change the mass?\"\n"
                         "- **What terms mean**: \"what is bump steer?\"\n"
                         "- **What to do first**: \"what should I open first?\"\n"
                         "- **What a tab is for**: \"what is the Roll tab for?\"",
                         suggestions=self._titles(self.starters), entry_id="_help")
        if _GREET.match(q):
            w = self.welcome()
            w.entry_id = "_greet"
            return w
        if _ABOUT.match(q) and "what_is" in self.by_id:
            return self.reply_to_title(self.by_id["what_is"].title, context)
        if _WHO.search(q):
            return Reply("I'm KinematiK's built-in guide, not an AI. I match your question against KinematiK's own "
                         "docs and give you the prewritten answer, so nothing you type leaves this page, and when "
                         "I don't know, I say so.", suggestions=self._titles(self.starters), entry_id="_who")
        ranked = self.rank(q)
        if not ranked and _THANKS.search(q):
            return Reply("Anytime! Ask me something else whenever you're stuck.",
                         suggestions=self._titles(self.starters), entry_id="_thanks")
        if not ranked or ranked[0][0] < self.WEAK:
            return Reply("I don't have an answer for that one yet. Try asking it another way, or pick one of "
                         "these:", suggestions=self._titles(self.starters), entry_id=None, confident=False)
        best_s, best = ranked[0]
        close = [e for s, e in ranked[1:4] if s >= self.WEAK]
        tied = len(ranked) > 1 and ranked[1][0] >= best_s - 0.02 and best_s < 0.75
        if best_s < self.STRONG or tied:      # too weak, or two answers fit equally: ask, don't pick one
            return Reply("I'm not sure I understood. Did you mean one of these?",
                         suggestions=list(dict.fromkeys([best.title] + [e.title for e in close]))[:3],
                         entry_id=None, confident=False)
        follow = [i for i in best.related if i in self.by_id and i != best.id]
        follow += [e.id for e in close if e.id not in follow and e.id != best.id]
        return Reply(best.answer.format_map(ctx), tabs=list(best.tabs),
                     suggestions=self._titles(follow[:3]), entry_id=best.id, confident=True)

    def reply_to_title(self, title: str, context: dict | None = None) -> Reply:
        """A suggestion chip was clicked: answer that entry exactly, without re-matching (text only, unitless)."""
        for e in self.entries:
            if e.title == title:
                follow = [i for i in e.related if i in self.by_id and i != e.id][:3]
                return Reply(e.answer.format_map(_Ctx(context or {})), tabs=list(e.tabs),
                             suggestions=self._titles(follow), entry_id=e.id)
        return self.reply(title, context)


def stream_words(text: str):
    """Yield `text` a word at a time (with the whitespace that followed it), for a typing effect. Unitless."""
    for m in re.finditer(r"\S+\s*", text):
        yield m.group(0)


# ---------------------------------------------------------------------------
#  The FAQ (app-independent answers). Tab ids match the app's _TAB_META.
# ---------------------------------------------------------------------------
FAQ = [
    Entry("start", "What should I do first?",
          "{start_plan}",
          ("start", "begin", "first", "new", "where start", "getting started", "lost", "confused", "onboard",
           "what now", "next step", "order"),
          related=("change_mass", "change_geometry", "what_is")),
    Entry("what_is", "What is KinematiK?",
          "KinematiK is an open-source **pre-validation studio** for Formula Student teams. It works in the hour "
          "before you open ANSYS, Adams/Car or GT-SUITE: you check a design here in seconds, catch wrong inputs "
          "(garbage-in) early, and only take the finalists into the heavy tools. It doesn't replace them; it makes "
          "sure what reaches them is worth simulating.",
          ("about", "purpose", "what does it do", "overview", "pre-validation", "prevalidation", "studio"),
          related=("accuracy", "start", "replace_adams")),
    Entry("change_mass", "Where do I change the car's mass, CG height or weight split?",
          "Open **🚗 Car basics** at the top of 🎚️ Roll & Load Transfer (it's also in Setup Optimiser, Brakes, Track "
          "Testing and ⚖️ Weight & Handover). It's one shared value: change it in any of those tabs and every tab "
          "updates.\n\nRight now: **{car_basics}**.",
          ("mass", "weight", "heavy", "kilogram", "pound", "cog", "height", "weightsplit", "front", "rear",
           "distribution", "change", "edit", "set", "car basics"),
          tabs=("roll",), related=("change_springs", "units", "my_car")),
    Entry("change_springs", "Where do I set spring rates and anti-roll bars?",
          "Open **🔩 Springs & anti-roll bars** at the top of 🎚️ Roll & Load Transfer (also in 🎛️ Setup "
          "Optimiser). Pick **My spring rates (recommended)** to enter coil springs and bars, or **A roll stiffness "
          "I type in** if you only know the axle totals.\n\nRight now: **{springs}**.",
          ("spring", "springrate", "antirollbar", "roll stiffness", "stiffness", "damper", "coil", "bar"),
          tabs=("roll",), related=("motion_ratio", "change_mass", "setup")),
    Entry("change_geometry", "Where do I edit the suspension geometry (hardpoints)?",
          "Open **✏️ Edit suspension geometry** at the top of 📐 Kinematics. Pick the suspension type, then edit "
          "each pickup point's x / y / z in the table (x toward the rear, y to the right, z up). Static camber and "
          "toe, the pushrod & rocker, and an importer are in the tabs next to the points.\n\nRight now: "
          "**{geometry}**.",
          ("hardpoint", "geometry", "point", "coordinate", "wishbone", "edit", "change", "suspension", "upright",
           "chassis", "move", "xyz", "topology", "type"),
          tabs=("kinematics",), related=("import", "reset_geometry", "frames")),
    Entry("import", "How do I import hardpoints from OptimumK, Excel or CSV?",
          "In 📐 Kinematics → **✏️ Edit suspension geometry → 📥 Import**, upload your OptimumK export, Excel or "
          "CSV file. KinematiK recognizes the point names, converts units and axis conventions, shows you what it "
          "matched, and you apply it with one button.",
          ("import", "upload", "optimumk", "excel", "csv", "xlsx", "file", "load", "spreadsheet"),
          tabs=("kinematics",), related=("change_geometry", "frames")),
    Entry("reset_geometry", "How do I reset the geometry to the example?",
          "In 📐 Kinematics → **✏️ Edit suspension geometry**, press **↺ Reset every point**. Every pickup point goes "
          "back to the example front corner (your edits are lost, so save the project first if you need them).",
          ("reset", "undo", "default", "example", "start over", "restore", "original"),
          tabs=("kinematics",), related=("save", "change_geometry")),
    Entry("units", "How do I switch between metric and US units?",
          "Use the **Units** switch at the top of the sidebar. Every input and result switches between metric (mm, "
          "kg, N) and US (in, lb, lbf); the model always computes in SI underneath, so nothing changes but the "
          "display.",
          ("units", "inch", "mm", "millimeter", "imperial", "metric", "convert", "pound", "kilogram", "us"),
          related=("change_mass",)),
    Entry("frames", "Which axis convention does KinematiK use?",
          "x points toward the **rear**, y to the **right**, z **up** (ISO-style, z up). That is not SAE J670, where "
          "z points down. To convert points from another convention, use ✅ Checks & Integration → 🧭 Frames & "
          "Datums, or let the importer do it.",
          ("axis", "axes", "frame", "convention", "coordinate", "sae", "iso", "datum", "origin", "direction",
           "which way", "positive", "negative", "sign"),
          tabs=("frames",), related=("import", "change_geometry")),
    Entry("save", "How do I save my work?",
          "Scroll to **Save / load your work** at the bottom of 🏁 Track Testing and press **💾 Save project "
          "(.json)**. One file captures the whole session; load it back from the same place.",
          ("save", "export", "download", "keep", "lose", "json", "project", "session", "backup", "load"),
          tabs=("laptime",), related=("reset_geometry", "preview")),
    Entry("preview", "How does the 5-minute preview work?",
          "Without an account you get **one 5-minute preview per browser**, with the full app. It survives a page "
          "refresh. Nothing you do in the preview is saved to any team's workspace; when it ends, the sign-up "
          "screen appears.",
          ("preview", "trial", "account", "sign up", "signup", "login", "minute", "free", "register", "timer"),
          related=("save", "what_is")),
    Entry("accuracy", "How accurate is KinematiK?",
          "Each module is benchmarked against established tools, with the scripts and data in the repo:\n"
          "- **Suspension**: matches MSC Adams View within 0.00003° on three corners, Lotus within 0.009° (camber) "
          "and Applied Racing Dynamics within 0.004° (caster, KPI).\n"
          "- **Powertrain**: NREL's FASTSim within 0.04% on identical inputs; five published GT-SUITE studies "
          "within 5% on the papers' inputs; measured dyno and race data within 1–4% on average.\n\n"
          "What it doesn't show: that a built car matches its hardpoints. That still needs a K&C rig and testing.",
          ("accurate", "accuracy", "validated", "validation", "benchmark", "trust", "correct", "reliable", "adams",
           "lotus", "ard", "fastsim", "gtsuite", "simscape", "error", "proof", "credible"),
          tabs=("validation",), related=("replace_adams", "garbage")),
    Entry("replace_adams", "Does KinematiK replace Adams, ANSYS or GT-SUITE?",
          "No. KinematiK sits **upstream** of them. Use it to sweep options, check inputs and pick a finalist in "
          "seconds; then validate that finalist in Adams, ANSYS or GT-SUITE. The heavy tool then confirms a design "
          "instead of debugging a typo.",
          ("replace", "instead", "adams", "ansys", "gtsuite", "matlab", "optimumk", "simulink", "versus", "vs",
           "compare", "alternative"),
          related=("what_is", "accuracy")),
    Entry("garbage", "What does a plausibility warning mean?",
          "It means an input looks wrong for a Formula Student car, for example a mass entered in pounds, an "
          "efficiency typed as 92 instead of 0.92, or an inverted gear ratio. KinematiK flags it at entry, before "
          "the number spreads into every result. Check the value and its units; if it's really right, carry on.",
          ("plausibility", "warning", "warn", "garbagein", "wrong", "suspicious", "error", "check", "flag",
           "input", "typo"),
          related=("accuracy", "units")),
    Entry("my_car", "What are my car's current numbers?",
          "Your car right now:\n- 🚗 **Car basics**: {car_basics}\n- 🔩 **Springs & bars**: {springs}\n"
          "- ✏️ **Suspension geometry**: {geometry}\n\nThe sidebar's **Your car** panel shows how many of them are "
          "still example values, and what each of your edits did to the car.",
          ("current", "now", "my car", "summary", "values", "numbers", "settings", "status", "inputs"),
          related=("change_mass", "change_springs", "change_geometry")),
    Entry("license", "Is KinematiK free and open source?",
          "Yes. KinematiK is open source under the **GNU AGPL v3**. The code, tests and benchmark data are on "
          "GitHub (frederikexd/kinematik); if you use it in published work, the repo's CITATION file says how to "
          "cite it.",
          ("free", "open source", "license", "licence", "agpl", "github", "cite", "citation", "cost money", "price",
           "source code"),
          related=("what_is",)),
    Entry("feedback", "How do I report a bug or give feedback?",
          "Open an issue on GitHub (frederikexd/kinematik), or message Frederik Thio, KinematiK's author, on "
          "LinkedIn. Screenshots and the steps to reproduce help a lot.",
          ("bug", "issue", "broken", "feedback", "report", "contact", "suggest", "feature request", "crash", "author"),
          related=("license",)),
    Entry("undo", "How do I undo a change?",
          "Every edit you make shows up in the sidebar under **What your changes did**, with what it did to roll "
          "and load transfer. Press **↶** next to one to undo it: only the inputs that change touched go back, so "
          "your later edits stay.",
          ("undo", "revert", "go back", "mistake", "previous", "history", "changes", "what changed", "ripple",
           "effect", "what did"),
          related=("my_car", "reset_geometry")),
    Entry("setup", "How do I find a good spring and anti-roll bar setup?",
          "Open 🎛️ **Setup Optimiser**. It searches springs, bars, CG height and weight split for the best balance, "
          "and **Apply to the car** writes the result into 🚗 Car basics and 🔩 Springs, where every tab reads it.",
          ("setup", "optimize", "optimizer", "tune", "tuning", "best", "balance"),
          tabs=("setup",), related=("change_springs", "understeer")),
    # --- glossary ---------------------------------------------------------
    Entry("bump_steer", "What is bump steer?",
          "**Bump steer** is the toe change a wheel gets as it moves up and down. Too much and the car steers itself "
          "over bumps and under braking. It comes mostly from where the tie-rod's inner and outer points sit "
          "relative to the wishbones; see the toe curve in 📐 Kinematics.",
          ("bumpsteer", "toe change", "toe curve", "steer"), tabs=("kinematics",),
          related=("camber_gain", "change_geometry")),
    Entry("camber_gain", "What is camber gain?",
          "**Camber gain** is how much the wheel's camber changes as the suspension compresses (or as the car rolls). "
          "Some negative gain keeps the tire flatter on the road in a corner. It's set by the wishbone lengths and "
          "angles; see it in 📐 Kinematics.",
          ("cambergain", "camber", "tilt", "lean"), tabs=("kinematics",), related=("roll_center", "bump_steer")),
    Entry("roll_center", "What is a roll center?",
          "The **roll center** is the point the car's body rolls about at each axle, found from the wishbone "
          "geometry. Its height splits lateral load transfer between the links and the springs, and how much it "
          "moves (migrates) with travel changes how predictable the car feels. See 🎚️ Roll & Load Transfer.",
          ("rollcenter", "roll", "migration", "height"), tabs=("roll",), related=("load_transfer", "camber_gain")),
    Entry("load_transfer", "What is lateral load transfer?",
          "In a corner, weight shifts from the inside tires to the outside ones. How that **load transfer** splits "
          "between the front and rear axle decides which end loses grip first (understeer or oversteer). Roll "
          "stiffness, roll center heights and CG height set it; see 🎚️ Roll & Load Transfer.",
          ("loadtransfer", "lateral", "llt", "weight transfer"), tabs=("roll",),
          related=("understeer", "roll_center")),
    Entry("understeer", "What are understeer and oversteer?",
          "**Understeer**: the front tires run out of grip first and the car goes wider than you steer. "
          "**Oversteer**: the rear lets go first and the back steps out. The front/rear split of lateral load "
          "transfer is the main lever; check the balance in 🎚️ Roll & Load Transfer.",
          ("understeer", "oversteer", "balance", "push", "loose", "handling"), tabs=("roll",),
          related=("load_transfer", "setup")),
    Entry("motion_ratio", "What is a motion ratio?",
          "The **motion ratio** is how far the spring moves per unit of wheel travel. Wheel rate ≈ spring rate × "
          "motion ratio², so a small change in the rocker makes a big change at the wheel. Turn on **Pushrod & "
          "rocker** in the geometry editor to get the real one from your rocker.",
          ("motionratio", "wheelrate", "rocker", "bellcrank", "pushrod", "pullrod", "installation ratio"),
          tabs=("kinematics",), related=("change_springs",)),
    Entry("kingpin", "What are kingpin inclination (KPI), caster and scrub radius?",
          "**KPI**: how far the steering axis leans inward, seen from the front. **Caster**: how far it leans "
          "back, seen from the side; it gives self-centering and camber when steering. **Scrub radius**: the "
          "distance at the ground between the steering axis and the tire's center; it sets how much braking and "
          "bumps tug the steering. All three are in 📐 Kinematics.",
          ("kingpin", "caster", "scrubradius", "scrub", "steering axis", "trail", "mechanical trail"),
          tabs=("kinematics",), related=("bump_steer",)),
    Entry("anti", "What are anti-dive and anti-squat?",
          "**Anti-dive** is how much the suspension geometry resists the nose diving under braking; **anti-squat** "
          "resists the rear squatting under acceleration. Both come from the side-view angles of the wishbones, "
          "given as a percentage.",
          ("antidive", "antisquat", "dive", "squat", "pitch", "braking", "acceleration"),
          tabs=("kinematics",), related=("roll_center",)),
]

STARTERS = ("start", "change_mass", "bump_steer", "accuracy")


def app_entries(tab_meta, brief_simple, brief_tools, role_goals, role_labels):
    """Entries made from the app's own copy: "What is <tab> for?" from the mission briefing's per-tab text, and one
    entry per role goal listing the tabs to open in order. Arguments are the app's _TAB_META, _BRIEF_SIMPLE,
    _BRIEF_TOOLS, _ROLE_GOALS and _ROLE_LABELS. Text only (unitless)."""
    out = []
    for tid, (em, label) in tab_meta.items():
        simple, tool = brief_simple.get(tid, ""), brief_tools.get(tid)
        if not simple and not tool:
            continue
        parts = [simple] if simple else []
        if tool:
            parts += [tool[0], f"**Why here first:** {tool[2]}"]
        out.append(Entry(f"tab:{tid}", f"What is {em} {label} for?", "\n\n".join(p for p in parts if p),
                         keywords=(label, tid, "tab"), tabs=(tid,)))
    for role, goals in role_goals.items():
        for gkey, glabel, ids in goals:
            known = [i for i in ids if i in tab_meta]
            steps = "\n".join(f"{n}. **{tab_meta[i][0]} {tab_meta[i][1]}**"
                              + (f": {brief_tools[i][0]}" if i in brief_tools else "")
                              for n, i in enumerate(known, 1))
            out.append(Entry(f"goal:{gkey}", glabel, f"Open these, in order:\n{steps}",
                             keywords=(role_labels.get(role, role), role), tabs=tuple(known[:1])))
    return out


def build_guide(extra_entries=(), starters=STARTERS) -> Guide:
    """The FAQ plus app-made entries (tab and goal answers). App entries never override a FAQ entry, and an entry
    whose title is already taken is dropped (the same goal can be listed under several subteams). Unitless."""
    out = list(FAQ)
    ids, titles = {e.id for e in out}, {e.title for e in out}
    for e in extra_entries:
        if e.id not in ids and e.title not in titles:
            out.append(e)
            ids.add(e.id)
            titles.add(e.title)
    return Guide(out, starters=starters)
