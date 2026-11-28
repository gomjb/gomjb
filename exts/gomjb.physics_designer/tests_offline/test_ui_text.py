import pathlib

UI_DIR = pathlib.Path(__file__).resolve().parents[1] / "gomjb" / "physics_designer" / "ui"


def test_ui_sources_are_ascii():
    """Isaac Sim's default UI font has no Hangul (or arrow / degree) glyphs: they render as '?'."""
    for path in UI_DIR.glob("*.py"):
        for n, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            assert line.isascii(), f"{path.name}:{n}: {line.strip()}"
