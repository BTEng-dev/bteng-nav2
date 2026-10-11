"""Tests for the bteng-nav2 command line interface."""

import pytest

from bteng_nav2 import cli

EXAMPLES = "examples/trees"
PARAMS = "examples/config/nav2_params.yaml"


# ── parser ──────────────────────────────────────────────────────────────────────

def test_run_defaults():
    args = cli.build_parser().parse_args(["run", "trees/"])
    assert args.trees == ["trees/"]
    assert args.params is None
    assert args.timeout == 60.0
    assert args.tick_interval == 0.05
    assert args.bb == []
    assert args.dry_run is False
    assert args.plain is False
    assert args.log == "INFO"


def test_run_accepts_several_trees_and_repeated_bb():
    args = cli.build_parser().parse_args(
        ["run", "a.xml", "b.xml", "-p", "p.yaml", "--bb", "x=1", "--bb", "y=2"]
    )
    assert args.trees == ["a.xml", "b.xml"]
    assert args.params == "p.yaml"
    assert args.bb == ["x=1", "y=2"]


def test_a_command_is_required():
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args([])


def test_unknown_command_exits():
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(["fly"])


# ── nodes ───────────────────────────────────────────────────────────────────────

def test_nodes_lists_tags(capsys):
    assert cli.main(["nodes"]) == 0
    out = capsys.readouterr().out
    assert "NavigateToPose" in out
    assert "NavigateToPoseNode" in out          # the alias is shown too
    assert "XML tags" in out


def test_nodes_long_shows_endpoints(capsys):
    assert cli.main(["nodes", "--long"]) == 0
    assert "/navigate_to_pose" in capsys.readouterr().out


# ── run ─────────────────────────────────────────────────────────────────────────

def test_run_dry_run_over_the_examples(capsys):
    assert cli.main(["run", EXAMPLES, "-p", PARAMS, "--dry-run", "--plain"]) == 0
    assert "Summary" in capsys.readouterr().out


def test_run_reports_a_missing_params_file():
    assert cli.main(["run", f"{EXAMPLES}/01_navigate_to_pose.xml", "-p", "nope.yaml"]) == 1


def test_run_without_params_uses_the_default_when_present(monkeypatch, tmp_path):
    """No -p and a ./params.yaml next door: pick it up, the way a project layout
    expects."""
    seen = {}

    def fake(trees, config_path, **kw):
        seen["config"] = config_path
        return 0

    monkeypatch.setattr(cli, "_DEFAULT_PARAMS", str(tmp_path / "params.yaml"))
    (tmp_path / "params.yaml").write_text("ros: {namespace: /r1}\n")
    monkeypatch.setattr("bteng_nav2.runtime.run_trees", fake)
    assert cli.main(["run", "trees/"]) == 0
    assert seen["config"] == str(tmp_path / "params.yaml")


def test_run_without_params_and_no_default_passes_none(monkeypatch, tmp_path):
    seen = {}

    def fake(trees, config_path, **kw):
        seen["config"] = config_path
        return 0

    monkeypatch.setattr(cli, "_DEFAULT_PARAMS", str(tmp_path / "absent.yaml"))
    monkeypatch.setattr("bteng_nav2.runtime.run_trees", fake)
    assert cli.main(["run", "trees/"]) == 0
    assert seen["config"] is None


def test_run_forwards_every_flag(monkeypatch):
    captured = {}

    def fake(trees, config_path, **kw):
        captured.update(trees=trees, config_path=config_path, **kw)
        return 0

    monkeypatch.setattr("bteng_nav2.runtime.run_trees", fake)
    cli.main([
        "run", "trees/", "-p", "p.yaml", "--bb", "x=1", "--namespace", "/robot1",
        "--timeout", "12.5", "--tick-interval", "0.02", "--plain", "--log", "DEBUG",
    ])
    assert captured["presets"] == ["x=1"]
    assert captured["namespace"] == "/robot1"
    assert captured["timeout"] == 12.5
    assert captured["tick_interval"] == 0.02
    assert captured["pretty"] is False
    assert captured["dry_run"] is False


def test_run_exit_code_is_propagated(monkeypatch):
    monkeypatch.setattr("bteng_nav2.runtime.run_trees", lambda *a, **kw: 1)
    assert cli.main(["run", "trees/"]) == 1
