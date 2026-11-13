"""Where motion tables are cached (Isaac Sim runtime)."""
from __future__ import annotations


def table_cache() -> str:
    """Kit's cache shared by all apps of the install (like its shader cache): a table built in the
    GUI serves a batch run of the same model, and the other way round."""
    import carb.tokens

    return carb.tokens.get_tokens_interface().resolve("${omni_cache}/gomjb.kinematics/motion_tables")
