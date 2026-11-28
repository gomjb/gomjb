"""Motion expressions in the RecurDyn / Adams function style, e.g. ``STEP(TIME, 0, 0, 1, 90)``.

Only arithmetic, numbers, ``TIME``, ``PI`` and the functions below are allowed (names are case
insensitive); anything else is rejected when the expression is compiled, so an expression stored
in a USD file cannot run code. Trigonometric functions take radians.
"""
from __future__ import annotations

import ast
import math
from typing import Callable


def step(x: float, x0: float, h0: float, x1: float, h1: float) -> float:
    """Cubic step (RecurDyn / Adams STEP): h0 before x0, h1 after x1, smooth (zero slope) between."""
    if x <= x0:
        return h0
    if x >= x1:
        return h1
    a = (x - x0) / (x1 - x0)
    return h0 + (h1 - h0) * a * a * (3.0 - 2.0 * a)


FUNCTIONS = {
    "STEP": step, "SIN": math.sin, "COS": math.cos, "TAN": math.tan, "ASIN": math.asin, "ACOS": math.acos,
    "ATAN": math.atan, "ATAN2": math.atan2, "ABS": abs, "SQRT": math.sqrt, "EXP": math.exp, "LOG": math.log,
    "MIN": min, "MAX": max, "SIGN": lambda a, b: math.copysign(abs(a), b), "MOD": math.fmod,
}
CONSTANTS = {"PI": math.pi}
_NODES = (ast.Expression, ast.BinOp, ast.UnaryOp, ast.Call, ast.Name, ast.Load, ast.Constant, ast.Add, ast.Sub,
          ast.Mult, ast.Div, ast.Pow, ast.Mod, ast.USub, ast.UAdd)


def compile_expression(text: str) -> Callable[[float], float]:
    """``text`` -> f(time). Raises ValueError for anything outside the expression language."""
    try:
        tree = ast.parse(text.strip(), mode="eval")
    except SyntaxError as exc:
        raise ValueError(f"invalid expression: {exc.msg}") from None
    for node in ast.walk(tree):
        if not isinstance(node, _NODES):
            raise ValueError(f"not allowed in an expression: {type(node).__name__}")
        if isinstance(node, ast.Constant) and not isinstance(node.value, (int, float)):
            raise ValueError("only numbers are allowed as constants")
        if isinstance(node, ast.Call):
            if not isinstance(node.func, ast.Name) or node.func.id.upper() not in FUNCTIONS or node.keywords:
                raise ValueError(f"unknown function: {ast.unparse(node.func)}")
        elif isinstance(node, ast.Name) and node.id.upper() not in FUNCTIONS:
            if node.id.upper() not in CONSTANTS and node.id.upper() != "TIME":
                raise ValueError(f"unknown name: {node.id}")
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            node.id = node.id.upper()  # case insensitive, as in RecurDyn
    code = compile(tree, "<motion>", "eval")
    names = {**FUNCTIONS, **CONSTANTS}

    def f(t: float) -> float:
        return float(eval(code, {"__builtins__": {}}, dict(names, TIME=t)))  # noqa: S307 - whitelisted nodes and names

    try:
        f(0.0)  # wrong argument counts and domain errors show up when the motion is set, not mid-run
    except (TypeError, ValueError, ZeroDivisionError, OverflowError) as exc:
        raise ValueError(f"expression fails at TIME = 0: {exc}") from None
    return f
