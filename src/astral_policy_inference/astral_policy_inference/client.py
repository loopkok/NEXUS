"""Self-contained websocket policy client (no ``openpi_client`` dependency).

Mirrors ``openpi_client/websocket_client_policy.py`` so it is wire-compatible
with both ``serve.py`` and the VLA/openpi ``serve_policy.py``:

* connect -> the server first sends packed metadata (``_wait_for_server``);
* ``infer(obs)`` -> pack -> send -> recv; a *string* response is an error;
* ``reset()`` is a no-op (matching openpi's client);
* ``close()`` actually closes the socket.

Serialization uses :mod:`astral_policy_inference.protocol` (the vendored
``__ndarray__`` format), so the package needs no external install beyond the
light ``websockets`` dependency.
"""

from __future__ import annotations

from typing import Optional

from astral_policy_inference import protocol as _proto


class WebsocketClient:
    """A blocking websocket client speaking the policy-inference protocol."""

    def __init__(self, host: str = "127.0.0.1", port: int = 8001, api_key: Optional[str] = None):
        self.host = host
        self.port = int(port)
        self.api_key = api_key
        self._packer = _proto.Packer()
        self._ws, self._server_metadata = self._wait_for_server()

    def get_server_metadata(self) -> dict:
        return self._server_metadata

    def _wait_for_server(self):
        import time

        import websockets.sync.client

        url = f"ws://{self.host}:{self.port}"
        while True:
            try:
                conn = websockets.sync.client.connect(url)
                metadata = _proto.unpackb(conn.recv())
                return conn, metadata
            except (OSError, ConnectionError):
                # server not up yet — retry every 2s
                time.sleep(2.0)

    def infer(self, obs: dict) -> dict:
        data = self._packer.pack(obs)
        self._ws.send(data)
        response = self._ws.recv()
        if isinstance(response, str):
            # server sends a string on error — treat as RuntimeError
            raise RuntimeError(f"Error in inference server:\n{response}")
        return _proto.unpackb(response)

    def reset(self) -> None:
        pass

    def close(self) -> None:
        ws = getattr(self, "_ws", None)
        if ws is not None:
            try:
                ws.close()
            except Exception:  # noqa: BLE001
                pass
            self._ws = None
