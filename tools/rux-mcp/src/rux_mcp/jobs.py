# SPDX-FileCopyrightText: 2025 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later

"""Background execution for the `rux` commands that take minutes.

An MCP tool call is a request/response round trip, and `rux create mesh` or
`rux optimize` can run for the better part of an hour — far longer than any
client will hold a call open.  So a mutating command is **submitted**, not
awaited: the tool returns a job id immediately, and the agent polls.
``wait_seconds`` lets a caller block for the short stages that finish in a
breath, which keeps the common case a single round trip.

One job runs at a time. ``ProjectDB`` is sqlite and explicitly not
thread-safe, and two writers on one project is corruption, so a second submit
is refused while a job is in flight rather than queued behind it — the agent
should know it is waiting.
"""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

from rux_mcp.runner import RuxError, RuxRunner, strip_ansi

#: How much of a job's output is kept in memory for `job_output`.  The full
#: log is always on disk; this only bounds what a poll can cost.
MAX_OUTPUT_CHARS = 40_000


class JobState(str):
    """String constants, so a job dict serialises without a converter."""

    queued = "queued"
    running = "running"
    succeeded = "succeeded"
    failed = "failed"
    cancelled = "cancelled"
    timed_out = "timed_out"


TERMINAL = frozenset(
    {JobState.succeeded, JobState.failed, JobState.cancelled, JobState.timed_out}
)


def terminate_tree(process: subprocess.Popen, grace: float = 10.0) -> None:
    """Stop ``process`` and everything it started.

    Signalling the process alone is not enough: `rux create dense` shells out
    to OpenMVS and `rux create mesh` to a solver, and a surviving grandchild
    keeps the job's stdout pipe open — the reader would then block until the
    grandchild finished, long after the job was reported dead.  The job is put
    in its own process group at spawn time (``start_new_session``) precisely so
    the whole tree can be signalled here.
    """
    if process.poll() is not None:
        return
    for sig, wait_for in ((signal.SIGTERM, grace), (signal.SIGKILL, 5.0)):
        try:
            os.killpg(os.getpgid(process.pid), sig)
        except (ProcessLookupError, PermissionError):  # pragma: no cover
            # No group (or not ours): fall back to the process itself.
            process.kill() if sig == signal.SIGKILL else process.terminate()
        try:
            process.wait(timeout=wait_for)
            return
        except subprocess.TimeoutExpired:
            continue


class JobBusy(RuntimeError):
    """Another job is already running against this project."""


class JobNotFound(KeyError):
    """No job with that id."""


@dataclass
class Job:
    """One `rux` invocation and everything an agent can learn about it."""

    id: str
    label: str
    command: list[str]
    timeout: float
    log_path: Path
    state: str = JobState.queued
    submitted_at: float = field(default_factory=time.time)
    started_at: float | None = None
    finished_at: float | None = None
    returncode: int | None = None
    error: str | None = None
    #: Captured output, kept as arriving chunks so a poll can read progress
    #: while the command is still running.  Trimmed from the front once the
    #: total passes :data:`MAX_OUTPUT_CHARS`.
    _chunks: list[str] = field(default_factory=list, repr=False)
    _chunk_chars: int = field(default=0, repr=False)
    _buffer_lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _done: threading.Event = field(default_factory=threading.Event, repr=False)
    _process: subprocess.Popen | None = field(default=None, repr=False)
    #: Set by :meth:`JobRunner.cancel` so a job cancelled between submit and
    #: exec never starts, rather than being reported dead and running anyway.
    _cancelled: threading.Event = field(default_factory=threading.Event, repr=False)

    @property
    def elapsed(self) -> float:
        """Seconds spent on this job.

        Measured from the start of the process, or from submission while the
        worker thread has not reached it yet — a job reported as busy must be
        able to say *how* busy without a ``None`` in the message.
        """
        return (self.finished_at or time.time()) - (self.started_at or self.submitted_at)

    def append(self, text: str) -> None:
        """Record a chunk of output, discarding the oldest once over budget."""
        cleaned = strip_ansi(text)
        with self._buffer_lock:
            self._chunks.append(cleaned)
            self._chunk_chars += len(cleaned)
            while self._chunk_chars > MAX_OUTPUT_CHARS and len(self._chunks) > 1:
                self._chunk_chars -= len(self._chunks.pop(0))

    def summary(self) -> dict[str, Any]:
        """The compact status an agent polls for."""
        data: dict[str, Any] = {
            "job_id": self.id,
            "command": self.label,
            "state": self.state,
            "done": self.state in TERMINAL,
            "argv": self.command,
            "log_path": str(self.log_path),
            "elapsed_seconds": round(self.elapsed, 1),
        }
        if self.returncode is not None:
            data["exit_status"] = self.returncode
        if self.error:
            data["error"] = self.error
        return data

    def output(self, tail_chars: int = 4000) -> str:
        """The tail of the combined stdout/stderr captured so far."""
        with self._buffer_lock:
            text = "".join(self._chunks).strip()
        if tail_chars <= 0 or len(text) <= tail_chars:
            return text
        return "...\n" + text[-tail_chars:]


class JobRunner:
    """Runs one `rux` command at a time and remembers what happened."""

    def __init__(self, runner: RuxRunner, log_dir: Path) -> None:
        self._runner = runner
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._jobs: dict[str, Job] = {}
        self._order: list[str] = []
        self._active: str | None = None

    # -- queries ---------------------------------------------------------

    @property
    def active_job(self) -> Job | None:
        with self._lock:
            return self._jobs.get(self._active) if self._active else None

    def get(self, job_id: str) -> Job:
        with self._lock:
            job = self._jobs.get(job_id)
        if job is None:
            raise JobNotFound(
                f"no job {job_id!r}. Call list_jobs() for this session's jobs."
            )
        return job

    def list(self, limit: int = 20) -> list[Job]:
        with self._lock:
            ids = self._order[-limit:] if limit > 0 else list(self._order)
            return [self._jobs[job_id] for job_id in reversed(ids)]

    # -- lifecycle -------------------------------------------------------

    def submit(
        self, label: str, args: Sequence[str], timeout: float
    ) -> Job:
        """Start ``rux <args>`` in the background and return its job."""
        self._runner.check_project()
        command = self._runner.argv(args)

        with self._lock:
            if self._active is not None:
                running = self._jobs[self._active]
                raise JobBusy(
                    f"job {running.id} ({running.label}) is still running after "
                    f"{running.elapsed:.0f}s. One rux command runs at a time — "
                    "the project database is sqlite and cannot take two "
                    "writers. Poll job_status(), or cancel_job() first."
                )
            job_id = uuid.uuid4().hex[:12]
            job = Job(
                id=job_id,
                label=label,
                command=command,
                timeout=timeout,
                log_path=self._log_dir / f"{job_id}.log",
            )
            self._jobs[job_id] = job
            self._order.append(job_id)
            self._active = job_id

        thread = threading.Thread(
            target=self._run, args=(job,), name=f"rux-job-{job_id}", daemon=True
        )
        thread.start()
        return job

    def wait(self, job_id: str, seconds: float) -> Job:
        """Block up to ``seconds`` for a job to finish, then return it."""
        job = self.get(job_id)
        if seconds > 0:
            job._done.wait(seconds)
        return job

    def cancel(self, job_id: str) -> Job:
        """Terminate a running job. Finished jobs are returned unchanged."""
        job = self.get(job_id)
        if job.state in TERMINAL:
            return job
        # Set first: a job cancelled in the window between submit() and exec
        # has no process to signal, and must not be allowed to start.
        job._cancelled.set()
        process = job._process
        if process is not None:
            terminate_tree(process)
        job._done.wait(15)
        if job.state not in TERMINAL:  # pragma: no cover - kill raced the thread
            job.state = JobState.cancelled
            job.error = "cancelled before rux reported an exit status"
        return job

    # -- worker ----------------------------------------------------------

    def _run(self, job: Job) -> None:
        job.started_at = time.time()
        if job._cancelled.is_set():
            job.state = JobState.cancelled
            job.error = "cancelled before rux was started"
            self._retire(job)
            return
        job.state = JobState.running
        try:
            with job.log_path.open("w", encoding="utf-8") as log:
                # rux logs to stdout, so stderr is merged rather than kept
                # apart: the interleaved order is what a human would have seen.
                process = subprocess.Popen(  # noqa: S603 - fixed argv, no shell
                    job.command,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    errors="replace",
                    # Own process group, so cancel/timeout can reach the whole
                    # tree — see terminate_tree().
                    start_new_session=True,
                )
                job._process = process
                assert process.stdout is not None
                if job._cancelled.is_set():  # cancelled during the spawn
                    terminate_tree(process)

                # A watchdog rather than a deadline check inside the read
                # loop: a stage that goes quiet for an hour still has to be
                # killed on time, and `rux create mesh` is quiet while the
                # MIP solver runs.
                def _expire() -> None:
                    if process.poll() is None:
                        job.state = JobState.timed_out
                        job.error = (
                            f"rux exceeded its {job.timeout:g}s budget and was "
                            "killed. Raise timeout_seconds if the stage is "
                            "genuinely this slow."
                        )
                        terminate_tree(process, grace=5.0)

                watchdog = threading.Timer(job.timeout, _expire)
                watchdog.daemon = True
                watchdog.start()
                try:
                    for line in process.stdout:
                        log.write(line)
                        job.append(line)
                    returncode = process.wait()
                finally:
                    watchdog.cancel()

            job.returncode = returncode
            if job.state == JobState.timed_out:
                pass
            elif returncode == 0:
                job.state = JobState.succeeded
            elif returncode < 0:
                job.state = JobState.cancelled
                job.error = f"rux was terminated by signal {-returncode}"
            else:
                job.state = JobState.failed
                job.error = f"rux exited with status {returncode}"
        except FileNotFoundError as exc:
            job.state = JobState.failed
            job.error = f"could not start rux: {exc}"
        except Exception as exc:  # pragma: no cover - defensive
            job.state = JobState.failed
            job.error = f"{type(exc).__name__}: {exc}"
        finally:
            self._retire(job)

    def _retire(self, job: Job) -> None:
        """Free the single job slot and wake anything waiting on the job."""
        job.finished_at = time.time()
        with self._lock:
            if self._active == job.id:
                self._active = None
        job._done.set()


__all__ = [
    "Job",
    "JobBusy",
    "JobNotFound",
    "JobRunner",
    "JobState",
    "MAX_OUTPUT_CHARS",
    "RuxError",
    "TERMINAL",
    "terminate_tree",
]
