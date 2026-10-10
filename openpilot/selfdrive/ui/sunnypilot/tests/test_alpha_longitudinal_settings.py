import ast
from pathlib import Path


ROOT = Path(__file__).parents[4]


def _method_source(path: Path, class_name: str, method_name: str) -> str:
  source = path.read_text()
  tree = ast.parse(source)
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == class_name)
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == method_name)
  return ast.get_source_segment(source, method) or ""


def test_startup_clear_is_guarded_by_offroad_state():
  method = _method_source(ROOT / "selfdrive/ui/sunnypilot/ui_state.py", "UIStateSP", "_enforce_constraints")
  tree = ast.parse(method)
  guarded_removals = []
  for node in ast.walk(tree):
    if isinstance(node, ast.If) and ast.unparse(node.test) == "not self.started":
      guarded_removals.extend(ast.unparse(child) for child in node.body)
  assert any("AlphaLongitudinalEnabled" in line and ".remove(" in line for line in guarded_removals)


def test_unsupported_car_still_clears_alpha_long():
  method = _method_source(ROOT / "selfdrive/ui/sunnypilot/ui_state.py", "UIStateSP", "_enforce_constraints")
  assert "if not CP.alphaLongitudinalAvailable:" in method
  assert 'self.params.remove("AlphaLongitudinalEnabled")' in method


def test_alpha_toggle_is_cruise_owned_offroad_and_confirmation_gated():
  alpha = (ROOT / "selfdrive/ui/sunnypilot/layouts/settings/alpha_longitudinal_toggles.py").read_text()
  cruise = (ROOT / "selfdrive/ui/sunnypilot/layouts/settings/cruise.py").read_text()
  assert "enabled=ui_state.is_offroad" in alpha
  assert "ConfirmDialog" in alpha and "DialogResult.CONFIRM" in alpha
  assert "AlphaLongitudinalToggles()" in cruise
  assert "*self._alpha_long.items" in cruise
