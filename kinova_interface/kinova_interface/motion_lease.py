"""Process-wide motion arbitration for middleware nodes on one robot host."""

from __future__ import annotations

import errno
import os
import threading


class MotionLeaseError(RuntimeError):
    """Raised when the local motion lease cannot be opened or locked."""


class MotionLease:
    """Hold an advisory, non-blocking OS lock while a node owns robot motion.

    The MTC and legacy motion servers are separate ROS processes. A regular
    ``threading.Lock`` cannot arbitrate between them, while an OS file lock is
    released automatically if its owning process exits. All command-producing
    nodes must run on the same host and use the same ``path``.
    """

    def __init__(self, path: str, owner: str):
        if not isinstance(path, str) or not os.path.isabs(path):
            raise MotionLeaseError("motion lease path must be absolute")
        if not isinstance(owner, str) or not owner.strip():
            raise MotionLeaseError("motion lease owner must be non-empty")
        self._path = path
        self._owner = owner.strip()
        self._stream = None
        self._guard = threading.Lock()

    @property
    def held(self) -> bool:
        with self._guard:
            return self._stream is not None

    def acquire(self, operation: str) -> bool:
        """Try to atomically acquire the lease; return ``False`` if it is busy."""

        with self._guard:
            if self._stream is not None:
                return True

            stream = None
            try:
                stream = open(self._path, "a+b")
                self._lock(stream)
            except BlockingIOError:
                if stream is not None:
                    stream.close()
                return False
            except OSError as exc:
                if stream is not None:
                    stream.close()
                if exc.errno in (errno.EACCES, errno.EAGAIN):
                    return False
                raise MotionLeaseError(
                    f"cannot acquire motion lease {self._path}: {exc}"
                ) from exc

            details = (
                f"pid={os.getpid()} owner={self._owner} "
                f"operation={operation.strip() or 'unspecified'}\n"
            ).encode("utf-8")
            # Windows locks byte 0. Preserve that sentinel and keep metadata
            # outside the locked range; resizing/replacing byte 0 can make a
            # later LK_UNLCK fail even though this process owns the lease.
            stream.seek(1 if os.name == "nt" else 0)
            stream.truncate()
            stream.write(details)
            stream.flush()
            self._stream = stream
            return True

    def release(self) -> None:
        """Release a held lease. Calling this when idle is harmless."""

        with self._guard:
            if self._stream is None:
                return
            stream = self._stream
            self._stream = None
            try:
                self._unlock(stream)
            finally:
                stream.close()

    @staticmethod
    def _lock(stream) -> None:
        if os.name == "nt":
            import msvcrt

            stream.seek(0)
            if not stream.read(1):
                stream.write(b"\0")
                stream.flush()
            stream.seek(0)
            try:
                msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
            except OSError as exc:
                raise BlockingIOError(exc.errno, str(exc)) from exc
        else:
            import fcntl

            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

    @staticmethod
    def _unlock(stream) -> None:
        if os.name == "nt":
            # Closing the file descriptor in ``release()`` releases Windows
            # byte-range locks. Explicit LK_UNLCK is unreliable after another
            # handle has made a failed non-blocking lock attempt.
            return
        else:
            import fcntl

            fcntl.flock(stream.fileno(), fcntl.LOCK_UN)

    def __enter__(self):
        if not self.acquire("context manager"):
            raise MotionLeaseError(f"motion lease is already held: {self._path}")
        return self

    def __exit__(self, _exc_type, _exc, _traceback):
        self.release()
