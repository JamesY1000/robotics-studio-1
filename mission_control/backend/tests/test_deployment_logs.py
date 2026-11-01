"""Terminal viewing must preserve startup logs and survive backend reconnection."""
from mission_control.deployment_logs import DeploymentLogTail


def test_new_deployment_waits_for_file_and_follows_appends(tmp_path):
    path = tmp_path / "ros.log"
    deployment = {"log_path": str(path)}
    tail = DeploymentLogTail(None)
    assert tail.read(deployment) == ""
    path.write_text("mission ready\n")
    assert tail.read(deployment).endswith("mission ready\n")
    assert tail.read(deployment) == ""
    with path.open("a") as log:
        log.write("ROI reached\n")
    assert tail.read(deployment) == "ROI reached\n"


def test_reconnect_skips_history_but_new_session_includes_startup(tmp_path):
    path = tmp_path / "old.log"
    path.write_text("old mission history\n")
    deployment = {"log_path": str(path)}
    tail = DeploymentLogTail(deployment)
    assert "old mission history" not in tail.read(deployment)
    with path.open("a") as log:
        log.write("still navigating\n")
    assert tail.read(deployment) == "still navigating\n"
    new_path = tmp_path / "new.log"
    new_path.write_text("starting new robots\n")
    assert tail.read({"log_path": str(new_path)}).endswith("starting new robots\n")


def test_reads_are_bounded_and_truncated_log_can_resume(tmp_path):
    path = tmp_path / "ros.log"
    deployment = {"log_path": str(path)}
    tail = DeploymentLogTail(None)
    path.write_text("x" * 70000)
    assert tail.read(deployment).endswith("x" * 65536)
    assert tail.read(deployment) == "x" * (70000 - 65536)
    path.write_text("restarted\n")
    assert tail.read(deployment) == "restarted\n"
