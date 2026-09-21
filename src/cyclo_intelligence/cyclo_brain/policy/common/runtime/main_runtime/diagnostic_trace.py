"""Opt-in, bounded runtime capture. Disk writes never run on the control thread."""

import json
import logging
import os
from pathlib import Path
import queue
import threading
import time


class DiagnosticTrace:
    MAX_BYTES = 100 * 1024 * 1024

    def __init__(self, directory=None):
        directory = os.environ.get('POLICY_DIAGNOSTIC_TRACE_DIR', '') if directory is None else directory
        self.enabled = bool(directory)
        self.dropped = 0
        self._queue = queue.Queue(maxsize=2048)
        self._closing = threading.Event()
        self._thread = None
        self.path = None
        if self.enabled:
            try:
                root = Path(directory)
                root.mkdir(parents=True, exist_ok=True)
                self.path = root / f'runtime-{time.time_ns()}-{os.getpid()}.jsonl'
                self._thread = threading.Thread(target=self._write, daemon=True)
                self._thread.start()
            except OSError:
                self.enabled = False
                logging.getLogger(__name__).exception('Could not enable diagnostic trace')

    def record(self, event, **fields):
        if not self.enabled:
            return
        row = dict(event=event, monotonic_s=time.monotonic(), wall_s=time.time(),
                   dropped=self.dropped, **fields)
        try:
            self._queue.put_nowait(row)
        except queue.Full:
            self.dropped += 1

    def close(self):
        self.enabled = False
        self._closing.set()
        if self._thread:
            self._thread.join(timeout=2)

    def record_rejection(self, **fields):
        """Keep one failure snapshot even after high-rate capture hits its cap.

        Called by the inference-request worker after the loop has stopped,
        never by the publishing tick. One atomically replaced file per runtime
        bounds disk use and keeps the last complete rejection available.
        """
        if self.path is None:
            return
        target = self.path.with_name(self.path.stem + '-last_rejection.json')
        temporary = target.with_suffix('.tmp')
        try:
            payload = json.dumps(dict(event='chunk_rejected', wall_s=time.time(), **fields),
                                 allow_nan=False, separators=(',', ':'))
            if len(payload.encode()) > 4 * 1024 * 1024:
                raise ValueError('Rejection snapshot exceeds 4 MB')
            temporary.write_text(payload + '\n')
            temporary.replace(target)
        except (OSError, TypeError, ValueError):
            logging.getLogger(__name__).exception('Could not capture rejected chunk')

    def _write(self):
        size = 0
        try:
            with self.path.open('x') as output:
                while not self._closing.is_set() or not self._queue.empty():
                    try:
                        row = self._queue.get(timeout=.2)
                    except queue.Empty:
                        output.flush()
                        continue
                    line = json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n'
                    size += len(line.encode())
                    if size > self.MAX_BYTES:
                        output.write(json.dumps({'event': 'trace_size_limit', 'dropped': self.dropped}) + '\n')
                        self.enabled = False
                        return
                    output.write(line)
        except (OSError, TypeError, ValueError):
            self.enabled = False
            logging.getLogger(__name__).exception('Diagnostic trace stopped')
