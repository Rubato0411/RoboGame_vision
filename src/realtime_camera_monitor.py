from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Lock, Thread
from time import monotonic

from .image_source import FramePacket, ImageSource
from .stream_health import StreamHealth, StreamHealthMonitor


@dataclass(frozen=True)
class CameraSnapshot:
    role: str
    packet: FramePacket | None
    health: StreamHealth
    error: str | None


class CameraMonitorWorker:
    """Read one camera in its own thread so another camera cannot block it."""

    def __init__(self, role: str, source: ImageSource,
                 health_monitor: StreamHealthMonitor) -> None:
        self.role = role
        self.source = source
        self.health_monitor = health_monitor
        self._stop = Event()
        self._lock = Lock()
        self._packet: FramePacket | None = None
        self._error: str | None = None
        self._thread = Thread(target=self._run, name=f"camera-monitor-{role}", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                packet = self.source.read()
                now = monotonic()
                if packet is None:
                    with self._lock:
                        self.health_monitor.observe_failure(now)
                        self._error = "camera returned no frame"
                    break
                with self._lock:
                    self.health_monitor.observe(packet, now)
                    self._packet = packet
                    self._error = None
            except Exception as exc:  # Boundary thread must expose failure to dashboard.
                with self._lock:
                    self.health_monitor.observe_failure(monotonic())
                    self._error = f"{type(exc).__name__}: {exc}"
                break

    def snapshot(self, now: float | None = None) -> CameraSnapshot:
        current = monotonic() if now is None else now
        with self._lock:
            return CameraSnapshot(self.role, self._packet,
                                  self.health_monitor.check(current), self._error)

    def stop(self, join_timeout_s: float = 2.0) -> None:
        self._stop.set()
        self._thread.join(timeout=join_timeout_s)
        self.source.release()


class RealtimeCameraMonitor:
    def __init__(self, workers: list[CameraMonitorWorker]) -> None:
        if not workers:
            raise ValueError("at least one camera worker is required")
        if len({worker.role for worker in workers}) != len(workers):
            raise ValueError("camera worker roles must be unique")
        self.workers = workers

    def start(self) -> None:
        for worker in self.workers:
            worker.start()

    def snapshots(self) -> tuple[CameraSnapshot, ...]:
        now = monotonic()
        return tuple(worker.snapshot(now) for worker in self.workers)

    def stop(self) -> None:
        for worker in self.workers:
            worker.stop()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()
