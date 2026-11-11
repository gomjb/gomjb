"""Minimal stand-ins for omni.ui / omni.usd / omni.kit.commands / carb.

They let the Kit-facing code (window callbacks, undo commands) run against a real
``usd-core`` stage in plain pytest. Only the API surface used by this extension is faked.
"""
import sys
import types


class _Model:
    def __init__(self, value=None):
        self._v = value
        self._cbs = []

    def set_value(self, v):
        self._v = v
        for cb in self._cbs:
            cb(self)

    def add_value_changed_fn(self, fn):
        self._cbs.append(fn)

    as_float = property(lambda self: float(self._v))
    as_int = property(lambda self: int(self._v))
    as_bool = property(lambda self: bool(self._v))
    as_string = property(lambda self: str(self._v))


class _ComboModel:
    def __init__(self, idx):
        self._m = _Model(idx)
        self._cbs = []

    def get_item_value_model(self, *_):
        return self._m

    def add_item_changed_fn(self, fn):
        self._cbs.append(fn)

    def select(self, idx):
        self._m.set_value(idx)
        for cb in self._cbs:
            cb(self, None)


class _Widget:
    registry = []

    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs
        self.model = kwargs.get("model")
        self.visible = True
        _Widget.registry.append(self)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def set_build_fn(self, fn):
        self._build_fn = fn
        fn()

    def rebuild(self):
        self._build_fn()

    def destroy(self):
        pass


class Button(_Widget):
    pass


class ComboBox(_Widget):
    def __init__(self, idx, *items, **kw):
        super().__init__(*items, **kw)
        self.items = items
        self.model = _ComboModel(idx)


class Window(_Widget):
    def __init__(self, title, **kw):
        super().__init__(title, **kw)
        self.frame = _Widget()


def install():
    ui = types.ModuleType("omni.ui")
    for name in ("VStack", "HStack", "ScrollingFrame", "CollapsableFrame", "Frame", "Label", "Spacer", "Separator",
                 "FloatDrag", "IntDrag", "StringField", "CheckBox"):
        setattr(ui, name, type(name, (_Widget,), {}))
    ui.Button, ui.ComboBox, ui.Window = Button, ComboBox, Window
    ui.SimpleFloatModel = ui.SimpleIntModel = ui.SimpleBoolModel = ui.SimpleStringModel = _Model
    ui.Alignment = types.SimpleNamespace(CENTER=0)
    ui.ScrollBarPolicy = types.SimpleNamespace(SCROLLBAR_ALWAYS_OFF=0)
    ui.Workspace = types.SimpleNamespace(set_show_window_fn=lambda *a: None)

    class _Selection:
        paths = []

        def get_selected_prim_paths(self):
            return list(self.paths)

        def set_selected_prim_paths(self, paths, _=True):
            self.paths = list(paths)

    class _Stream:
        def create_subscription_to_pop(self, fn, name=""):
            return object()

    class _Context:
        stage = None
        selection = _Selection()

        def get_stage(self):
            return self.stage

        def get_selection(self):
            return self.selection

        def get_stage_event_stream(self):
            return _Stream()

    ctx = _Context()
    usd = types.ModuleType("omni.usd")
    usd.get_context = lambda name="": ctx
    usd.StageEventType = types.SimpleNamespace(SELECTION_CHANGED=1, OPENED=2, CLOSING=3)

    commands = types.ModuleType("omni.kit.commands")
    commands._registry, commands.undo_stack, commands.redo_stack = {}, [], []

    class Command:
        pass

    def execute(name, **kwargs):
        cmd = commands._registry[name](**kwargs)
        result = cmd.do()
        commands.undo_stack.append(cmd)
        commands.redo_stack.clear()
        return True, result

    def undo():
        cmd = commands.undo_stack.pop()
        cmd.undo()
        commands.redo_stack.append(cmd)

    def redo():
        cmd = commands.redo_stack.pop()
        cmd.do()
        commands.undo_stack.append(cmd)

    commands.Command = Command
    commands.register = lambda cls: commands._registry.__setitem__(cls.__name__, cls)
    commands.unregister = lambda cls: commands._registry.pop(cls.__name__, None)
    commands.execute, commands.undo, commands.redo = execute, undo, redo

    app = types.ModuleType("omni.kit.app")
    app.get_app = lambda: types.SimpleNamespace(get_update_event_stream=lambda: _Stream())
    carb = types.ModuleType("carb")
    carb.log_info = carb.log_warn = carb.log_error = lambda *a: None
    ext = types.ModuleType("omni.ext")
    ext.IExt = object

    class XYPlot:
        """isaacsim.gui.components.XYPlot stand-in: remembers the data it was given."""

        def __init__(self, label, x_data=None, y_data=None, legends=None, plot_colors=None, x_min=None,
                     x_max=None, y_min=None, y_max=None, **kwargs):
            self.x_data, self.y_data, self.legends = x_data or [], y_data or [], legends or []
            self.colors = [list(c) for c in plot_colors or []]
            self.x_min, self.x_max, self.y_min, self.y_max = x_min, x_max, y_min, y_max

    isaacsim = types.ModuleType("isaacsim")
    gui = types.ModuleType("isaacsim.gui")
    components = types.ModuleType("isaacsim.gui.components")
    components.XYPlot = XYPlot
    isaacsim.gui, gui.components = gui, components

    omni = types.ModuleType("omni")
    kit = types.ModuleType("omni.kit")
    omni.ui, omni.usd, omni.kit, omni.ext = ui, usd, kit, ext
    kit.commands, kit.app = commands, app
    sys.modules.update({"omni": omni, "omni.ui": ui, "omni.usd": usd, "omni.kit": kit, "omni.kit.commands": commands,
                        "omni.kit.app": app, "omni.ext": ext, "carb": carb, "isaacsim": isaacsim,
                        "isaacsim.gui": gui, "isaacsim.gui.components": components})
    return ctx, commands
