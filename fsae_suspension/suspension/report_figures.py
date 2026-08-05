# ============================================================================
#  KinematiK — suspension/report_figures.py
#  Turn a captured plotly figure spec into a print-ready PNG for the PDF
#  reports. Two renderers, tried in order, then an honest give-up:
#
#    1. kaleido  — plotly's own exporter. Pixel-identical to what the member
#                  saw on screen. Kaleido v1+ drives a headless Chrome, which
#                  is present on a dev machine but usually NOT on Streamlit
#                  Cloud, so its availability is probed once and cached.
#    2. matplotlib — re-plots the trace data. Pure wheel, no browser, no system
#                  package, so it works everywhere the app deploys. Covers the
#                  2-D trace types KinematiK actually documents (scatter, line,
#                  bar, heatmap, contour).
#    3. None     — the caller keeps the existing text description ("Camber vs
#                  wheel travel · 1 series"). A report that silently drops a
#                  figure is worse than one that names it.
#
#  Everything here is defensive: a documentation path must never raise into a
#  feature, and must never claim a figure it did not actually draw.
# ============================================================================
"""Rasterize captured plotly figure specs for the calculation-report PDFs.

The app captures figures as plain plotly *specs* (nested dicts) at render time —
cheap, picklable, and safe to hold in session state. Rasterization happens here,
lazily, only when a member actually asks for a PDF. That keeps the per-rerun
cost of documentation at zero, which is the whole reason the capture layer is
allowed to be always-on.

Screen theme vs print theme
---------------------------
KinematiK's on-screen ``PLOT_LAYOUT`` is dark (near-black plot area, pale grey
type). Dropped unchanged onto a white A4 page that renders as a black rectangle
with invisible axis labels, and eats a toner cartridge. :func:`to_print_theme`
recolours the spec for paper — white ground, dark type, light grid — and
darkens any series colour too pale to read on white, while keeping each series
distinguishable from the others.
"""

from __future__ import annotations

import copy
import math

#: Hard cap on samples per trace. A lap-time or transient trace can carry tens
#: of thousands of points; at report DPI anything past a couple of thousand is
#: sub-pixel detail that costs memory and render time and shows up nowhere.
MAX_POINTS_PER_TRACE = 2000

#: Default figure box, in points, sized to the A4 text column used by
#: project.render_pdf (A4 width less 18 mm margins each side).
DEFAULT_WIDTH = 760
DEFAULT_HEIGHT = 380

# Print palette. Backgrounds and furniture only — series colours are preserved
# from the original figure (merely darkened when unreadable) so a chart in the
# report is recognisably the chart the member was looking at.
_PRINT_PAPER = "#ffffff"
_PRINT_PLOT = "#fbfbfc"
_PRINT_INK = "#1a1f24"
_PRINT_GRID = "#dfe4e9"
_PRINT_ZERO = "#aab4bd"

#: Relative luminance above which a colour is too pale to read on white.
#: KinematiK's accents are tuned for a near-black UI: the cyan (#37e0d0, lum
#: ~0.59) and amber (#ffb02e, ~0.52) are vivid on screen and washed out on
#: paper. 0.30 is roughly a 4.5:1 contrast ratio against white — the WCAG AA
#: text threshold — which is a sane bar for a 2 px line someone will read off a
#: printed design-review pack.
_MAX_PRINT_LUMINANCE = 0.30

_kaleido_state = None          # None = unprobed, True/False = probe result


# --------------------------------------------------------------------------- #
#  Colour helpers
# --------------------------------------------------------------------------- #
def _parse_color(c):
    """(r, g, b) floats in 0..1 for '#rgb', '#rrggbb' or 'rgb[a](...)'. None if
    the colour is a name, a colourscale reference, or anything unparseable —
    callers then leave it exactly as it was."""
    if not isinstance(c, str):
        return None
    s = c.strip().lower()
    try:
        if s.startswith("#"):
            h = s[1:]
            if len(h) == 3:
                h = "".join(ch * 2 for ch in h)
            if len(h) != 6:
                return None
            return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
        if s.startswith("rgb"):
            inner = s[s.index("(") + 1:s.index(")")]
            parts = [p.strip() for p in inner.split(",")[:3]]
            return tuple(min(255.0, max(0.0, float(p))) / 255.0 for p in parts)
    except Exception:
        return None
    return None


def _to_hex(rgb):
    return "#" + "".join(f"{int(round(v * 255)):02x}" for v in rgb)


def _luminance(rgb):
    """Relative luminance (WCAG), used only to decide 'too pale for paper'."""
    def _lin(v):
        return v / 12.92 if v <= 0.03928 else ((v + 0.055) / 1.055) ** 2.4
    r, g, b = (_lin(v) for v in rgb)
    return 0.2126 * r + 0.7152 * g + 0.0722 * b


def darken_for_print(color):
    """Darken a screen colour just enough to read on white, preserving hue.

    Scales the RGB triple down until its luminance clears the print threshold,
    so the amber stays amber and the cyan stays cyan — they simply stop
    disappearing into the page. Returns the input untouched when it is already
    dark enough or cannot be parsed.
    """
    rgb = _parse_color(color)
    if rgb is None:
        return color
    lum = _luminance(rgb)
    if lum <= _MAX_PRINT_LUMINANCE:
        return color
    # Binary search the scale factor; ~12 iterations is plenty for 8-bit output.
    lo, hi = 0.0, 1.0
    for _ in range(12):
        mid = (lo + hi) / 2
        if _luminance(tuple(v * mid for v in rgb)) > _MAX_PRINT_LUMINANCE:
            hi = mid
        else:
            lo = mid
    return _to_hex(tuple(v * lo for v in rgb))


# --------------------------------------------------------------------------- #
#  Spec normalisation
# --------------------------------------------------------------------------- #
#: plotly dtype code -> numpy dtype string, for the binary array encoding below.
_BDATA_DTYPES = {
    "f8": "<f8", "f4": "<f4",
    "i1": "|i1", "i2": "<i2", "i4": "<i4", "i8": "<i8",
    "u1": "|u1", "u2": "<u2", "u4": "<u4", "u8": "<u8",
}


def _decode_bdata(v):
    """Decode plotly's base64 binary array encoding, or return None.

    Since plotly 6, ``Figure.to_dict()`` no longer hands back plain lists for
    numpy-backed traces. It returns ``{"dtype": "f8", "bdata": "<base64>"}``
    (plus ``"shape"`` for 2-D data such as a heatmap's z). That is compact and
    fine for the browser, but it means anything that walks the spec expecting
    numbers gets a dict instead — which is exactly how a chart ends up silently
    missing from a report on a modern plotly while still "capturing" fine.

    Decoded here rather than avoided by asking plotly for lists, because the
    encoding is chosen inside plotly's serialiser and the knob to disable it has
    moved between releases; decoding what we are given does not depend on that.
    """
    if not (isinstance(v, dict) and "bdata" in v and "dtype" in v):
        return None
    try:
        import base64
        import numpy as _np
        dt = _BDATA_DTYPES.get(str(v.get("dtype")))
        if dt is None:
            return None
        raw = v["bdata"]
        arr = _np.frombuffer(
            base64.b64decode(raw) if isinstance(raw, str) else raw, dtype=dt)
        shape = v.get("shape")
        if shape:
            if isinstance(shape, str):
                shape = [int(p) for p in shape.replace(",", " ").split()]
            try:
                arr = arr.reshape(tuple(int(s) for s in shape))
            except Exception:
                pass
        return arr.tolist()
    except Exception:
        return None


def _decimate(seq):
    """Stride-sample a long sequence down to MAX_POINTS_PER_TRACE, always
    keeping the final point so the curve still ends where the data ends."""
    try:
        n = len(seq)
    except Exception:
        return seq
    if n <= MAX_POINTS_PER_TRACE:
        return seq
    step = max(1, n // MAX_POINTS_PER_TRACE)
    out = list(seq[::step])
    if out and out[-1] is not seq[n - 1]:
        out.append(seq[n - 1])
    return out


def compact_spec(fig):
    """A plain-dict, size-bounded copy of a plotly figure, safe for session state.

    Accepts a ``go.Figure`` or an already-plain dict. Numpy arrays and pandas
    series are converted to lists so the result stays picklable and does not
    pin large backing buffers alive for the rest of the session.
    """
    try:
        spec = fig.to_dict() if hasattr(fig, "to_dict") else copy.deepcopy(fig)
    except Exception:
        return None
    if not isinstance(spec, dict):
        return None

    def _listify(v):
        if v is None or isinstance(v, (str, bool, int, float)):
            return v
        decoded = _decode_bdata(v)        # plotly>=6 base64 binary arrays
        if decoded is not None:
            v = decoded
        elif hasattr(v, "tolist"):        # numpy array / pandas series
            try:
                v = v.tolist()
            except Exception:
                return None
        if isinstance(v, (list, tuple)):
            return _decimate([_listify(x) for x in v])
        return v

    out_traces = []
    for tr in (spec.get("data") or [])[:12]:      # a report figure past 12
        if not isinstance(tr, dict):              # series is unreadable anyway
            continue
        clean = {}
        for k, v in tr.items():
            if k in ("x", "y", "z", "text", "customdata", "error_x", "error_y"):
                clean[k] = _listify(v)
            elif isinstance(v, dict):
                clean[k] = {kk: _listify(vv) for kk, vv in v.items()}
            else:
                clean[k] = _listify(v)
        out_traces.append(clean)
    return {"data": out_traces, "layout": spec.get("layout") or {}}


def to_print_theme(spec):
    """Recolour a captured (dark-themed) spec for a white page. Never raises."""
    try:
        out = copy.deepcopy(spec)
    except Exception:
        return spec
    lay = out.setdefault("layout", {})
    lay["paper_bgcolor"] = _PRINT_PAPER
    lay["plot_bgcolor"] = _PRINT_PLOT
    lay["template"] = None                 # drop any dark template underneath

    font = lay.setdefault("font", {})
    font["color"] = _PRINT_INK
    font.setdefault("size", 12)
    # The UI font is JetBrains Mono, which the renderer will not have. Let it
    # fall back to a stack that exists on any box rather than to a tofu grid.
    font["family"] = "DejaVu Sans, Helvetica, Arial, sans-serif"

    for axis in ("xaxis", "yaxis", "xaxis2", "yaxis2"):
        a = lay.get(axis)
        if not isinstance(a, dict):
            continue
        a["gridcolor"] = _PRINT_GRID
        a["zerolinecolor"] = _PRINT_ZERO
        a["linecolor"] = _PRINT_ZERO
        a["color"] = _PRINT_INK

    lgd = lay.get("legend")
    if isinstance(lgd, dict):
        lgd["bgcolor"] = "rgba(255,255,255,0.85)"
        lgd["bordercolor"] = _PRINT_GRID

    for tr in out.get("data") or []:
        if not isinstance(tr, dict):
            continue
        line = tr.get("line")
        if isinstance(line, dict) and line.get("color"):
            line["color"] = darken_for_print(line["color"])
        marker = tr.get("marker")
        if isinstance(marker, dict) and isinstance(marker.get("color"), str):
            marker["color"] = darken_for_print(marker["color"])
        if isinstance(tr.get("fillcolor"), str):
            tr["fillcolor"] = darken_for_print(tr["fillcolor"])
    return out


# --------------------------------------------------------------------------- #
#  Renderer 1 — kaleido (exact, needs Chrome)
# --------------------------------------------------------------------------- #
def kaleido_available():
    """Probe kaleido ONCE and cache the answer.

    Kaleido v1 raises ChromeNotFoundError at render time, not import time, so
    importing it successfully proves nothing. We render a throwaway 2-point
    figure to find out for real. Probing every chart on every export would cost
    a browser-launch attempt per figure on exactly the deployments where it can
    never work, which is the slowest possible way to produce a fallback.
    """
    global _kaleido_state
    if _kaleido_state is not None:
        return _kaleido_state
    _kaleido_state = False
    try:
        import plotly.graph_objects as go
        probe = go.Figure(go.Scatter(x=[0, 1], y=[0, 1]))
        data = probe.to_image(format="png", width=80, height=60)
        _kaleido_state = bool(data)
    except Exception:
        _kaleido_state = False
    return _kaleido_state


def _via_kaleido(spec, width, height, scale):
    try:
        import plotly.graph_objects as go
        fig = go.Figure(spec)
        fig.update_layout(width=width, height=height)
        return fig.to_image(format="png", width=width, height=height,
                            scale=scale)
    except Exception:
        return None


# --------------------------------------------------------------------------- #
#  Renderer 2 — matplotlib (portable re-plot)
# --------------------------------------------------------------------------- #
def _axis_title(lay, key):
    a = lay.get(key)
    if not isinstance(a, dict):
        return ""
    t = a.get("title")
    if isinstance(t, dict):
        return str(t.get("text") or "")
    return str(t or "")


def _fig_title(lay):
    t = lay.get("title")
    if isinstance(t, dict):
        return str(t.get("text") or "")
    return str(t or "")


def _mpl_dash(dash):
    return {"dot": ":", "dash": "--", "dashdot": "-.",
            "longdash": (0, (8, 4)), "longdashdot": (0, (8, 4, 2, 4))
            }.get(str(dash or "").lower(), "-")


def _via_matplotlib(spec, width, height, scale):
    """Re-plot the captured traces with matplotlib.

    Deliberately covers the 2-D families KinematiK documents and no more.
    Anything else (3-D surfaces, meshes, the full-car model) returns None so
    the report falls back to naming the figure instead of drawing a wrong or
    empty one — a misleading chart in a design-review document is worse than a
    line of text saying the chart exists.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return None

    lay = spec.get("layout") or {}
    traces = [t for t in (spec.get("data") or []) if isinstance(t, dict)]
    if not traces:
        return None
    if any(str(t.get("type", "")).lower() in
           ("scatter3d", "surface", "mesh3d", "cone", "volume", "isosurface")
           for t in traces):
        return None

    dpi = 100
    fig, ax = plt.subplots(figsize=(width / dpi, height / dpi), dpi=dpi * scale)
    fig.patch.set_facecolor(_PRINT_PAPER)
    ax.set_facecolor(_PRINT_PLOT)

    drew = False
    labelled = False
    for tr in traces:
        ttype = str(tr.get("type", "scatter")).lower()
        name = tr.get("name")
        try:
            if ttype in ("heatmap", "contour", "histogram2d"):
                z = tr.get("z")
                if not z:
                    continue
                if ttype == "contour":
                    cs = ax.contourf(z, levels=14)
                else:
                    cs = ax.imshow(z, aspect="auto", origin="lower")
                fig.colorbar(cs, ax=ax, fraction=0.046, pad=0.03)
                drew = True
                continue

            x = tr.get("x")
            y = tr.get("y")
            if y is None:
                continue
            if x is None:
                x = list(range(len(y)))
            n = min(len(x), len(y))
            x, y = x[:n], y[:n]
            if n == 0:
                continue

            line = tr.get("line") if isinstance(tr.get("line"), dict) else {}
            marker = tr.get("marker") if isinstance(tr.get("marker"), dict) else {}
            color = darken_for_print(line.get("color")
                                     or (marker.get("color")
                                         if isinstance(marker.get("color"), str)
                                         else None))
            lw = float(line.get("width") or 2.0) * 0.85   # plotly px -> mpl pt

            if ttype == "bar":
                ax.bar(x, y, color=color, label=name)
            else:
                mode = str(tr.get("mode") or "lines").lower()
                if "lines" in mode or mode == "none":
                    ax.plot(x, y, color=color, linewidth=lw,
                            linestyle=_mpl_dash(line.get("dash")), label=name)
                    if "markers" in mode:
                        ax.plot(x, y, linestyle="none", marker="o",
                                markersize=3.2, color=color)
                elif "markers" in mode:
                    ax.plot(x, y, linestyle="none", marker="o", markersize=3.6,
                            color=color, label=name)
                else:
                    ax.plot(x, y, color=color, linewidth=lw, label=name)
            drew = True
            if name:
                labelled = True
        except Exception:
            continue        # one bad trace must not lose the whole figure

    if not drew:
        plt.close(fig)
        return None

    title = _fig_title(lay)
    if title:
        ax.set_title(title, color=_PRINT_INK, fontsize=12, pad=8)
    ax.set_xlabel(_axis_title(lay, "xaxis"), color=_PRINT_INK, fontsize=10)
    ax.set_ylabel(_axis_title(lay, "yaxis"), color=_PRINT_INK, fontsize=10)
    for key, setter in (("xaxis", ax.set_xscale), ("yaxis", ax.set_yscale)):
        a = lay.get(key)
        if isinstance(a, dict) and str(a.get("type", "")) == "log":
            try:
                setter("log")
            except Exception:
                pass
    ax.grid(True, color=_PRINT_GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for spine in ax.spines.values():
        spine.set_color(_PRINT_ZERO)
    ax.tick_params(colors=_PRINT_INK, labelsize=9)
    if labelled and len(traces) > 1:
        ax.legend(fontsize=8, framealpha=0.85, facecolor="white",
                  edgecolor=_PRINT_GRID)

    import io
    buf = io.BytesIO()
    try:
        fig.tight_layout(pad=0.8)
        fig.savefig(buf, format="png", facecolor=_PRINT_PAPER)
    except Exception:
        plt.close(fig)
        return None
    plt.close(fig)
    return buf.getvalue()


# --------------------------------------------------------------------------- #
#  Public entry point
# --------------------------------------------------------------------------- #
def figure_png(spec, *, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, scale=2,
               prefer_kaleido=True):
    """PNG bytes for a captured figure spec, or None if it cannot be drawn.

    Tries kaleido (exact) then matplotlib (portable). Returning None is a
    supported outcome, not an error: the caller keeps the figure's text
    description so the document still records that the chart exists.
    """
    if not isinstance(spec, dict) or not (spec.get("data") or []):
        return None
    try:
        printable = to_print_theme(spec)
    except Exception:
        printable = spec
    if prefer_kaleido and kaleido_available():
        png = _via_kaleido(printable, width, height, scale)
        if png:
            return png
    return _via_matplotlib(printable, width, height, scale)


def renderer_name():
    """Which renderer will be used, for the report's own provenance footer."""
    if kaleido_available():
        return "kaleido"
    try:
        import matplotlib                              # noqa: F401
        return "matplotlib"
    except Exception:
        return "none"
