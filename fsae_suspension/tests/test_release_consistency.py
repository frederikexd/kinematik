"""One release number, stated the same everywhere a user or a citation manager reads it."""
import re
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_release_number_is_the_same_in_packaging_citation_and_changelog():
    package = tomllib.loads((ROOT / "fsae_suspension" / "pyproject.toml").read_text(encoding="utf-8"))["project"]
    cff = re.search(r"^version:\s*([\d.]+)", (ROOT / "CITATION.cff").read_text(encoding="utf-8"), re.M).group(1)
    log = re.search(r"^## (\d+\.\d+\.\d+) ", (ROOT / "CHANGELOG.md").read_text(encoding="utf-8"), re.M).group(1)
    assert package["version"] == cff == log


def test_readme_states_the_python_the_package_requires():
    package = tomllib.loads((ROOT / "fsae_suspension" / "pyproject.toml").read_text(encoding="utf-8"))["project"]
    need = package["requires-python"].lstrip(">=")
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    assert f"Python {need} or newer" in readme
    assert "github.com/<" not in readme                       # a clone command that can be pasted
