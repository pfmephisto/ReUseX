# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""The background job runner: one at a time, killable, and honest about it.

`rux create mesh` outlives any MCP call, so mutating commands are submitted
rather than awaited.  These tests drive real subprocesses — the whole point of
the module is process lifetime, which a mock cannot exercise.
"""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from rux_mcp.jobs import MAX_OUTPUT_CHARS, JobBusy, JobNotFound, JobRunner, JobState
from rux_mcp.runner import ProjectMissingError, RuxNotFoundError, RuxRunner


@pytest.fixture
def script(tmp_path: Path):
    """Write an executable stand-in for ``rux`` and return its path."""

    def make(body: str, name: str = "rux") -> Path:
        path = tmp_path / name
        path.write_text("#!/bin/sh\n" + body)
        path.chmod(0o755)
        return path

    return make


@pytest.fixture
def jobs_for(project: Path, tmp_path: Path):
    """Build a :class:`JobRunner` around a given fake ``rux`` script."""

    def make(binary: Path, timeout: float = 30.0) -> JobRunner:
        runner = RuxRunner(project=project, binary=str(binary), timeout=timeout)
        return JobRunner(runner, log_dir=tmp_path / "joblogs")

    return make


def finished(runner: JobRunner, job_id: str, seconds: float = 15.0):
    job = runner.wait(job_id, seconds)
    assert job.state in {
        JobState.succeeded,
        JobState.failed,
        JobState.cancelled,
        JobState.timed_out,
    }, f"job never finished: {job.state}"
    return job


# -- happy path -----------------------------------------------------------


def test_a_successful_job_records_its_output_and_exit_status(script, jobs_for):
    runner = jobs_for(script('echo "planes: 38 found"\n'))
    job = runner.submit("create planes", ["create", "planes"], 30.0)
    job = finished(runner, job.id)

    assert job.state == JobState.succeeded
    assert job.returncode == 0
    assert "planes: 38 found" in job.output()
    assert job.summary()["done"] is True
    assert job.summary()["elapsed_seconds"] >= 0


def test_the_full_output_is_also_written_to_disk(script, jobs_for):
    runner = jobs_for(script('echo "line one"\necho "line two"\n'))
    job = finished(runner, runner.submit("info", ["info"], 30.0).id)
    assert "line two" in job.log_path.read_text()


def test_the_project_flag_precedes_the_subcommand(script, jobs_for, project):
    calls = script('printf "%s\\n" "$*"\n')
    runner = jobs_for(calls)
    job = finished(runner, runner.submit("create rooms", ["create", "rooms"], 30.0).id)
    assert job.output().strip() == f"-p {project} create rooms"


def test_stderr_is_interleaved_with_stdout(script, jobs_for):
    runner = jobs_for(script('echo out\necho err >&2\n'))
    job = finished(runner, runner.submit("info", ["info"], 30.0).id)
    assert "out" in job.output() and "err" in job.output()


def test_colour_escapes_do_not_reach_the_agent(script, jobs_for):
    runner = jobs_for(script('printf "\\033[31mred\\033[0m\\n"\n'))
    job = finished(runner, runner.submit("info", ["info"], 30.0).id)
    assert job.output().strip() == "red"


# -- failure --------------------------------------------------------------


def test_a_failing_job_keeps_its_status_and_says_so(script, jobs_for):
    runner = jobs_for(script('echo "Error: no cloud named cloud"\nexit 3\n'))
    job = finished(runner, runner.submit("create planes", ["create", "planes"], 30.0).id)

    assert job.state == JobState.failed
    assert job.returncode == 3
    assert "status 3" in job.error
    assert "no cloud named cloud" in job.output()


def test_a_missing_project_is_refused_before_a_job_exists(
    script, tmp_path, project
):
    runner = RuxRunner(
        project=tmp_path / "gone.rux", binary=str(script("exit 0")), timeout=5
    )
    jobs = JobRunner(runner, log_dir=tmp_path / "joblogs")
    with pytest.raises(ProjectMissingError):
        jobs.submit("info", ["info"], 30.0)
    assert jobs.list() == []


def test_a_missing_binary_is_refused_before_a_job_exists(project, tmp_path):
    runner = RuxRunner(project=project, binary="rux-does-not-exist")
    jobs = JobRunner(runner, log_dir=tmp_path / "joblogs")
    with pytest.raises(RuxNotFoundError):
        jobs.submit("info", ["info"], 30.0)
    assert jobs.list() == []


def test_an_unknown_job_id_names_the_way_out(script, jobs_for):
    runner = jobs_for(script("exit 0"))
    with pytest.raises(JobNotFound) as excinfo:
        runner.get("nope")
    assert "list_jobs()" in excinfo.value.args[0]


# -- the single slot ------------------------------------------------------


def test_a_second_job_is_refused_while_one_runs(script, jobs_for):
    """sqlite takes one writer; a queue would hide that from the agent."""
    runner = jobs_for(script("sleep 3\n"))
    first = runner.submit("create mesh", ["create", "mesh"], 30.0)
    with pytest.raises(JobBusy) as excinfo:
        runner.submit("create planes", ["create", "planes"], 30.0)

    message = str(excinfo.value)
    assert first.id in message
    assert "create mesh" in message
    assert "runs at a time" in message
    assert "cancel_job()" in message

    runner.cancel(first.id)


def test_the_slot_is_free_again_once_a_job_finishes(script, jobs_for):
    runner = jobs_for(script("exit 0"))
    finished(runner, runner.submit("info", ["info"], 30.0).id)
    assert runner.active_job is None
    second = finished(runner, runner.submit("log", ["log"], 30.0).id)
    assert second.state == JobState.succeeded
    assert len(runner.list()) == 2


def test_the_slot_is_free_again_after_a_failure(script, jobs_for):
    runner = jobs_for(script("exit 1"))
    finished(runner, runner.submit("info", ["info"], 30.0).id)
    assert runner.active_job is None
    runner.submit("info", ["info"], 30.0)  # would raise JobBusy if it were not


def test_list_is_newest_first_and_capped(script, jobs_for):
    runner = jobs_for(script("exit 0"))
    labels = []
    for index in range(4):
        label = f"info-{index}"
        labels.append(label)
        finished(runner, runner.submit(label, ["info"], 30.0).id)
    assert [job.label for job in runner.list(2)] == labels[::-1][:2]


# -- cancellation and timeouts -------------------------------------------


def test_cancelling_kills_the_process(script, jobs_for):
    runner = jobs_for(script("sleep 30\n"))
    job = runner.submit("create mesh", ["create", "mesh"], 60.0)
    # Give the thread a moment to actually spawn the child.
    for _ in range(100):
        if job._process is not None:
            break
        time.sleep(0.01)
    cancelled = runner.cancel(job.id)
    assert cancelled.state == JobState.cancelled
    assert runner.active_job is None


def test_cancelling_a_finished_job_is_a_no_op(script, jobs_for):
    runner = jobs_for(script("exit 0"))
    job = finished(runner, runner.submit("info", ["info"], 30.0).id)
    assert runner.cancel(job.id).state == JobState.succeeded


def test_an_over_budget_job_is_killed_even_while_silent(script, jobs_for):
    """A watchdog, not a deadline in the read loop: `create mesh` goes quiet."""
    runner = jobs_for(script("sleep 30\n"))
    job = runner.submit("create mesh", ["create", "mesh"], 0.4)
    job = finished(runner, job.id, seconds=20)
    assert job.state == JobState.timed_out
    assert "timeout_seconds" in job.error
    assert runner.active_job is None


# -- output handling ------------------------------------------------------


def test_output_is_readable_while_the_job_is_still_running(script, jobs_for):
    runner = jobs_for(script('echo "stage 1 of 2"\nsleep 5\n'))
    job = runner.submit("create mesh", ["create", "mesh"], 30.0)
    for _ in range(200):
        if "stage 1 of 2" in job.output():
            break
        time.sleep(0.02)
    else:  # pragma: no cover - only on a pathologically slow machine
        pytest.fail("progress output never became visible")
    assert not job.summary()["done"]
    runner.cancel(job.id)


def test_a_flood_of_output_is_capped(script, jobs_for):
    runner = jobs_for(script("i=0\nwhile [ $i -lt 4000 ]; do\n"
                             '  echo "0123456789012345678901234567890123456789"\n'
                             "  i=$((i+1))\ndone\n"))
    job = finished(runner, runner.submit("create clouds", ["create", "clouds"], 60.0).id)
    assert job.state == JobState.succeeded
    assert len(job.output(tail_chars=0)) <= MAX_OUTPUT_CHARS + 128
    # ...but the untruncated log is still on disk.
    assert len(job.log_path.read_text()) > MAX_OUTPUT_CHARS


def test_output_tail_marks_that_it_is_a_tail(script, jobs_for):
    runner = jobs_for(script('echo "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"\n'))
    job = finished(runner, runner.submit("info", ["info"], 30.0).id)
    assert job.output(tail_chars=10).startswith("...\n")


def test_wait_returns_early_when_the_job_is_slow(script, jobs_for):
    runner = jobs_for(script("sleep 5\n"))
    job = runner.submit("create mesh", ["create", "mesh"], 30.0)
    started = time.time()
    polled = runner.wait(job.id, 0.2)
    assert time.time() - started < 3
    assert polled.state == JobState.running
    runner.cancel(job.id)
