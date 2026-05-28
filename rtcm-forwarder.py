#!/usr/bin/env python3
"""Forward RTCM correction data from a serial GNSS receiver."""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import json
import logging
import signal
import sys
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import serial_asyncio
import yaml
from serial import SerialException


LOGGER = logging.getLogger("rtcm-forwarder")
RTCM_VALIDATE_CRC = True
NO_RTCM_WARNING_SECONDS = 10.0


class ConfigError(ValueError):
    """Raised when the YAML configuration is missing or invalid."""


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baudrate: int
    config_port: str
    config_baudrate: int


@dataclass(frozen=True)
class TcpConfig:
    enabled: bool
    host: str
    port: int


@dataclass(frozen=True)
class NtripConfig:
    enabled: bool
    host: str
    port: int
    mountpoint: str
    password: str
    identifier: str
    reconnect_seconds: float


@dataclass(frozen=True)
class WebConfig:
    enabled: bool
    host: str
    port: int


@dataclass(frozen=True)
class AppConfig:
    serial: SerialConfig
    tcp: TcpConfig
    ntrip: NtripConfig
    web: WebConfig


def _as_bool(value: Any, field_name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "1", "on"}:
            return True
        if normalized in {"false", "no", "0", "off"}:
            return False
    raise ConfigError(f"{field_name} must be true or false")


def _as_int(value: Any, field_name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field_name} must be an integer") from exc


def _as_float(value: Any, field_name: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ConfigError(f"{field_name} must be a number") from exc


def _as_str(value: Any, field_name: str) -> str:
    if value is None:
        raise ConfigError(f"{field_name} is required")
    text = str(value).strip()
    if not text:
        raise ConfigError(f"{field_name} cannot be empty")
    return text


def _port(value: Any, field_name: str) -> int:
    port = _as_int(value, field_name)
    if port < 1 or port > 65535:
        raise ConfigError(f"{field_name} must be between 1 and 65535")
    return port


def load_config(path: Path) -> AppConfig:
    if not path.exists():
        raise ConfigError(f"Config file not found: {path}")

    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ConfigError(f"Could not parse YAML config {path}: {exc}") from exc

    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ConfigError("Config file must contain a YAML mapping at the top level")

    serial_raw = raw.get("serial") or {}
    tcp_raw = raw.get("tcp") or {}
    ntrip_raw = raw.get("ntrip") or {}
    web_raw = raw.get("web") or {}
    if not isinstance(serial_raw, dict):
        raise ConfigError("serial must be a mapping")
    if not isinstance(tcp_raw, dict):
        raise ConfigError("tcp must be a mapping")
    if not isinstance(ntrip_raw, dict):
        raise ConfigError("ntrip must be a mapping")
    if not isinstance(web_raw, dict):
        raise ConfigError("web must be a mapping")

    serial = SerialConfig(
        port=_as_str(serial_raw.get("port", "/dev/ttyACM0"), "serial.port"),
        baudrate=_as_int(serial_raw.get("baudrate", 115200), "serial.baudrate"),
        config_port=_as_str(
            serial_raw.get("config_port", "/dev/ttyUSB0"),
            "serial.config_port",
        ),
        config_baudrate=_as_int(
            serial_raw.get("config_baudrate", 115200),
            "serial.config_baudrate",
        ),
    )
    if serial.baudrate <= 0:
        raise ConfigError("serial.baudrate must be greater than 0")
    if serial.config_baudrate <= 0:
        raise ConfigError("serial.config_baudrate must be greater than 0")

    tcp = TcpConfig(
        enabled=_as_bool(tcp_raw.get("enabled", True), "tcp.enabled"),
        host=_as_str(tcp_raw.get("host", "0.0.0.0"), "tcp.host"),
        port=_port(tcp_raw.get("port", 2101), "tcp.port"),
    )

    ntrip_enabled = _as_bool(ntrip_raw.get("enabled", True), "ntrip.enabled")
    ntrip = NtripConfig(
        enabled=ntrip_enabled,
        host=_as_str(ntrip_raw.get("host"), "ntrip.host") if ntrip_enabled else str(ntrip_raw.get("host", "")).strip(),
        port=_port(ntrip_raw.get("port", 2101), "ntrip.port"),
        mountpoint=_as_str(ntrip_raw.get("mountpoint"), "ntrip.mountpoint") if ntrip_enabled else str(ntrip_raw.get("mountpoint", "")).strip(),
        password=_as_str(ntrip_raw.get("password"), "ntrip.password") if ntrip_enabled else str(ntrip_raw.get("password", "")).strip(),
        identifier=str(ntrip_raw.get("identifier", "raspberry-pi-rtcm")).strip() or "raspberry-pi-rtcm",
        reconnect_seconds=_as_float(ntrip_raw.get("reconnect_seconds", 5), "ntrip.reconnect_seconds"),
    )
    if ntrip.reconnect_seconds <= 0:
        raise ConfigError("ntrip.reconnect_seconds must be greater than 0")

    web = WebConfig(
        enabled=_as_bool(web_raw.get("enabled", True), "web.enabled"),
        host=_as_str(web_raw.get("host", "0.0.0.0"), "web.host"),
        port=_port(web_raw.get("port", 8080), "web.port"),
    )

    return AppConfig(serial=serial, tcp=tcp, ntrip=ntrip, web=web)


def crc24q(data: bytes) -> int:
    """Return the RTCM3 CRC-24Q checksum for header and payload bytes."""
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
            crc &= 0xFFFFFF
    return crc


class RtcmFrameExtractor:
    """Extract complete RTCM3 frames from an arbitrary serial byte stream."""

    def __init__(self, validate_crc: bool = True) -> None:
        self.validate_crc = validate_crc
        self.buffer = bytearray()
        self.discarded_bytes = 0
        self.bad_crc_frames = 0

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames: list[bytes] = []

        while True:
            preamble_index = self.buffer.find(b"\xd3")
            if preamble_index < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break

            if preamble_index > 0:
                self.discarded_bytes += preamble_index
                del self.buffer[:preamble_index]

            if len(self.buffer) < 3:
                break

            if self.buffer[1] & 0xFC:
                self.discarded_bytes += 1
                del self.buffer[0]
                continue

            payload_length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            frame_length = 3 + payload_length + 3
            if len(self.buffer) < frame_length:
                break

            frame = bytes(self.buffer[:frame_length])
            if self.validate_crc:
                expected_crc = int.from_bytes(frame[-3:], "big")
                actual_crc = crc24q(frame[:-3])
                if actual_crc != expected_crc:
                    self.bad_crc_frames += 1
                    self.discarded_bytes += 1
                    del self.buffer[0]
                    continue

            frames.append(frame)
            del self.buffer[:frame_length]

        return frames


WEBSOCKET_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
WEB_CONSOLE_HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RTCM Forwarder Console</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101316;
      --panel: #171c21;
      --panel-2: #1f252b;
      --border: #333c45;
      --text: #edf1f4;
      --muted: #9da8b2;
      --accent: #42b883;
      --warn: #f4c430;
      --error: #ff6b6b;
      --input: #0c0f12;
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      min-height: 100vh;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }

    header {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 14px 18px;
      border-bottom: 1px solid var(--border);
      background: #13181d;
    }

    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 650;
      letter-spacing: 0;
    }

    .socket-state {
      color: var(--muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 13px;
      white-space: nowrap;
    }

    main {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 14px;
      padding: 14px;
      height: calc(100vh - 55px);
    }

    section {
      display: grid;
      grid-template-rows: auto minmax(180px, 1fr) auto;
      min-width: 0;
      min-height: 0;
      border: 1px solid var(--border);
      border-radius: 8px;
      background: var(--panel);
      overflow: hidden;
    }

    .console-head {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 10px 12px;
      border-bottom: 1px solid var(--border);
      background: var(--panel-2);
    }

    h2 {
      margin: 0;
      font-size: 14px;
      font-weight: 650;
      letter-spacing: 0;
    }

    .serial-state {
      color: var(--muted);
      font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      font-size: 12px;
      overflow: hidden;
      text-overflow: ellipsis;
      white-space: nowrap;
      text-align: right;
    }

    .serial-state[data-state="connected"] {
      color: var(--accent);
    }

    .serial-state[data-state="connecting"],
    .serial-state[data-state="retrying"] {
      color: var(--warn);
    }

    .serial-state[data-state="error"],
    .serial-state[data-state="write error"] {
      color: var(--error);
    }

    pre {
      margin: 0;
      padding: 12px;
      min-height: 0;
      overflow: auto;
      background: #090b0d;
      color: #e7ecef;
      font: 13px/1.45 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
    }

    form {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto auto auto;
      gap: 8px;
      padding: 10px;
      border-top: 1px solid var(--border);
      background: var(--panel-2);
    }

    input,
    select,
    button {
      min-height: 36px;
      border: 1px solid var(--border);
      border-radius: 6px;
      background: var(--input);
      color: var(--text);
      font: 13px ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
    }

    input {
      min-width: 0;
      padding: 0 10px;
    }

    select {
      padding: 0 8px;
    }

    button {
      padding: 0 12px;
      cursor: pointer;
    }

    button:hover,
    input:focus,
    select:focus {
      border-color: var(--accent);
      outline: none;
    }

    @media (max-width: 850px) {
      header {
        align-items: flex-start;
        flex-direction: column;
      }

      main {
        grid-template-columns: 1fr;
        height: auto;
        min-height: calc(100vh - 85px);
      }

      section {
        min-height: 42vh;
      }

      form {
        grid-template-columns: minmax(0, 1fr) auto;
      }

      select {
        grid-column: 1;
      }
    }
  </style>
</head>
<body>
  <header>
    <h1>RTCM Forwarder Console</h1>
    <div id="socket-state" class="socket-state">connecting</div>
  </header>
  <main>
    <section>
      <div class="console-head">
        <h2>GNSS Receiver</h2>
        <div class="serial-state" data-status="gnss">offline</div>
      </div>
      <pre data-log="gnss"></pre>
      <form data-form="gnss">
        <input data-input="gnss" autocomplete="off" spellcheck="false">
        <select data-ending="gnss">
          <option value="crlf">CRLF</option>
          <option value="lf">LF</option>
          <option value="cr">CR</option>
          <option value="none">None</option>
        </select>
        <button type="submit">Send</button>
        <button type="button" data-clear="gnss">Clear</button>
      </form>
    </section>
    <section>
      <div class="console-head">
        <h2>ESP32 Config</h2>
        <div class="serial-state" data-status="esp32">offline</div>
      </div>
      <pre data-log="esp32"></pre>
      <form data-form="esp32">
        <input data-input="esp32" autocomplete="off" spellcheck="false">
        <select data-ending="esp32">
          <option value="crlf">CRLF</option>
          <option value="lf">LF</option>
          <option value="cr">CR</option>
          <option value="none">None</option>
        </select>
        <button type="submit">Send</button>
        <button type="button" data-clear="esp32">Clear</button>
      </form>
    </section>
  </main>
  <script>
    const MAX_CHARS = 120000;
    const suffixes = { crlf: "\\r\\n", lf: "\\n", cr: "\\r", none: "" };
    const socketState = document.getElementById("socket-state");
    const channels = {};
    let socket = null;

    for (const id of ["gnss", "esp32"]) {
      channels[id] = {
        log: document.querySelector(`[data-log="${id}"]`),
        form: document.querySelector(`[data-form="${id}"]`),
        input: document.querySelector(`[data-input="${id}"]`),
        ending: document.querySelector(`[data-ending="${id}"]`),
        status: document.querySelector(`[data-status="${id}"]`),
        clear: document.querySelector(`[data-clear="${id}"]`)
      };

      channels[id].form.addEventListener("submit", (event) => {
        event.preventDefault();
        sendLine(id);
      });
      channels[id].clear.addEventListener("click", () => {
        channels[id].log.textContent = "";
        channels[id].input.focus();
      });
      channels[id].log.addEventListener("click", () => channels[id].input.focus());
    }

    function setSocketState(text) {
      socketState.textContent = text;
    }

    function setSerialState(channel, state, detail) {
      const status = channels[channel]?.status;
      if (!status) return;
      status.dataset.state = state;
      status.textContent = detail ? `${state} ${detail}` : state;
    }

    function append(channel, text) {
      const log = channels[channel]?.log;
      if (!log) return;
      const nearBottom = log.scrollHeight - log.scrollTop - log.clientHeight < 24;
      log.textContent += text;
      if (log.textContent.length > MAX_CHARS) {
        log.textContent = log.textContent.slice(-MAX_CHARS);
      }
      if (nearBottom) {
        log.scrollTop = log.scrollHeight;
      }
    }

    function sendLine(channel) {
      if (!socket || socket.readyState !== WebSocket.OPEN) return;
      const entry = channels[channel];
      const data = entry.input.value + (suffixes[entry.ending.value] ?? "");
      socket.send(JSON.stringify({ type: "input", channel, data }));
      entry.input.value = "";
      entry.input.focus();
    }

    function connect() {
      const scheme = window.location.protocol === "https:" ? "wss" : "ws";
      socket = new WebSocket(`${scheme}://${window.location.host}/ws`);
      setSocketState("connecting");

      socket.addEventListener("open", () => setSocketState("connected"));
      socket.addEventListener("message", (event) => {
        let message;
        try {
          message = JSON.parse(event.data);
        } catch {
          return;
        }

        if (message.type === "data") {
          append(message.channel, message.text || "");
        } else if (message.type === "notice") {
          append(message.channel, message.text || "");
        } else if (message.type === "status") {
          setSerialState(message.channel, message.state || "offline", message.detail || "");
        }
      });
      socket.addEventListener("close", () => {
        setSocketState("reconnecting");
        for (const id of Object.keys(channels)) {
          setSerialState(id, "web offline", "");
        }
        window.setTimeout(connect, 1500);
      });
    }

    connect();
  </script>
</body>
</html>
"""


def _console_text(data: bytes) -> str:
    text: list[str] = []
    index = 0
    while index < len(data):
        byte = data[index]
        if byte == 13:
            text.append("\n")
            if index + 1 < len(data) and data[index + 1] == 10:
                index += 2
                continue
        elif byte == 10:
            text.append("\n")
        elif byte == 9:
            text.append("\t")
        elif 32 <= byte <= 126:
            text.append(chr(byte))
        else:
            text.append(f"\\x{byte:02x}")
        index += 1
    return "".join(text)


def _websocket_accept(key: str) -> str:
    digest = hashlib.sha1((key + WEBSOCKET_GUID).encode("ascii")).digest()
    return base64.b64encode(digest).decode("ascii")


def _websocket_frame(payload: bytes, opcode: int = 1) -> bytes:
    header = bytearray([0x80 | opcode])
    length = len(payload)
    if length < 126:
        header.append(length)
    elif length <= 0xFFFF:
        header.extend((126, *length.to_bytes(2, "big")))
    else:
        header.extend((127, *length.to_bytes(8, "big")))
    return bytes(header) + payload


async def _read_websocket_frame(reader: asyncio.StreamReader) -> tuple[int, bytes]:
    header = await reader.readexactly(2)
    opcode = header[0] & 0x0F
    masked = bool(header[1] & 0x80)
    length = header[1] & 0x7F
    if length == 126:
        length = int.from_bytes(await reader.readexactly(2), "big")
    elif length == 127:
        length = int.from_bytes(await reader.readexactly(8), "big")
    if length > 65536:
        raise RuntimeError("WebSocket frame is too large")

    mask = await reader.readexactly(4) if masked else b""
    payload = await reader.readexactly(length) if length else b""
    if masked:
        payload = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    return opcode, payload


class WebSocketClient:
    def __init__(
        self,
        server: WebConsoleServer,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        self.server = server
        self.reader = reader
        self.writer = writer
        self.write_lock = asyncio.Lock()
        self.closed = False

    async def run(self) -> None:
        try:
            while True:
                opcode, payload = await _read_websocket_frame(self.reader)
                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    await self.send_raw(payload, opcode=0xA)
                    continue
                if opcode != 0x1:
                    continue
                await self.server.handle_client_message(
                    self,
                    payload.decode("utf-8", errors="replace"),
                )
        except (ConnectionError, OSError, asyncio.IncompleteReadError, RuntimeError):
            pass
        finally:
            await self.server.remove_client(self)

    async def send_json(self, message: dict[str, Any]) -> bool:
        payload = json.dumps(message, separators=(",", ":")).encode("utf-8")
        return await self.send_raw(payload, opcode=0x1)

    async def send_raw(self, payload: bytes, opcode: int = 0x1) -> bool:
        if self.closed or self.writer.is_closing():
            return False
        try:
            async with self.write_lock:
                self.writer.write(_websocket_frame(payload, opcode=opcode))
                await asyncio.wait_for(self.writer.drain(), timeout=2)
        except (ConnectionError, OSError, asyncio.TimeoutError):
            return False
        return True

    async def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if not self.writer.is_closing():
            with suppress(ConnectionError, OSError, asyncio.TimeoutError):
                self.writer.write(_websocket_frame(b"", opcode=0x8))
                await asyncio.wait_for(self.writer.drain(), timeout=1)
            self.writer.close()
            with suppress(ConnectionError, OSError, asyncio.TimeoutError):
                await asyncio.wait_for(self.writer.wait_closed(), timeout=1)


class WebConsoleServer:
    def __init__(self, config: WebConfig) -> None:
        self.config = config
        self.server: asyncio.AbstractServer | None = None
        self.clients: set[WebSocketClient] = set()
        self.serial_writers: dict[str, asyncio.StreamWriter | None] = {
            "gnss": None,
            "esp32": None,
        }
        self.serial_status: dict[str, tuple[str, str]] = {
            "gnss": ("offline", ""),
            "esp32": ("offline", ""),
        }

    async def start(self) -> None:
        if not self.config.enabled:
            LOGGER.info("Web console is disabled")
            return

        self.server = await asyncio.start_server(
            self._handle_http,
            self.config.host,
            self.config.port,
        )
        sockets = self.server.sockets or []
        addresses = ", ".join(str(sock.getsockname()) for sock in sockets)
        if not addresses:
            addresses = f"{self.config.host}:{self.config.port}"
        LOGGER.info("Web console listening on http://%s", addresses)

    async def stop(self) -> None:
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()
            self.server = None

        for client in tuple(self.clients):
            await self.remove_client(client)

    async def set_serial_state(
        self,
        channel: str,
        writer: asyncio.StreamWriter | None,
        state: str,
        detail: str = "",
    ) -> None:
        if channel not in self.serial_writers:
            return
        self.serial_writers[channel] = writer
        self.serial_status[channel] = (state, detail)
        await self.broadcast(
            {
                "type": "status",
                "channel": channel,
                "state": state,
                "detail": detail,
            }
        )

    async def broadcast_serial_data(self, channel: str, data: bytes) -> None:
        if not data:
            return
        await self.broadcast(
            {
                "type": "data",
                "channel": channel,
                "text": _console_text(data),
            }
        )

    async def broadcast(self, message: dict[str, Any]) -> None:
        if not self.config.enabled or not self.clients:
            return
        for client in tuple(self.clients):
            sent = await client.send_json(message)
            if not sent:
                await self.remove_client(client)

    async def handle_client_message(
        self,
        client: WebSocketClient,
        text: str,
    ) -> None:
        try:
            message = json.loads(text)
        except json.JSONDecodeError:
            return

        if not isinstance(message, dict) or message.get("type") != "input":
            return
        channel = message.get("channel")
        if channel not in self.serial_writers:
            return
        data = message.get("data", "")
        if not isinstance(data, str):
            return
        if len(data) > 4096:
            await client.send_json(
                {
                    "type": "notice",
                    "channel": channel,
                    "text": "[input too large]\n",
                }
            )
            return

        writer = self.serial_writers[channel]
        if writer is None or writer.is_closing():
            await client.send_json(
                {
                    "type": "notice",
                    "channel": channel,
                    "text": "[serial port is not connected]\n",
                }
            )
            return

        try:
            writer.write(data.encode("utf-8", errors="replace"))
            await asyncio.wait_for(writer.drain(), timeout=2)
        except (ConnectionError, OSError, asyncio.TimeoutError) as exc:
            await self.set_serial_state(channel, None, "write error", str(exc))

    async def remove_client(self, client: WebSocketClient) -> None:
        self.clients.discard(client)
        await client.close()

    async def _handle_http(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        try:
            request = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=5)
            request_text = request.decode("latin1", errors="replace")
            request_line, headers = self._parse_http_request(request_text)
            method, raw_path, _version = request_line
            path = raw_path.split("?", 1)[0]

            if self._is_websocket_request(path, headers):
                await self._handle_websocket(headers, reader, writer)
                return

            if method == "GET" and path in {"/", "/index.html"}:
                await self._send_response(
                    writer,
                    "200 OK",
                    WEB_CONSOLE_HTML.encode("utf-8"),
                    "text/html; charset=utf-8",
                )
            else:
                await self._send_response(
                    writer,
                    "404 Not Found",
                    b"Not found\n",
                    "text/plain; charset=utf-8",
                )
        except (asyncio.IncompleteReadError, asyncio.TimeoutError, ValueError):
            with suppress(ConnectionError, OSError):
                await self._send_response(
                    writer,
                    "400 Bad Request",
                    b"Bad request\n",
                    "text/plain; charset=utf-8",
                )
        finally:
            if not writer.is_closing():
                writer.close()
                with suppress(ConnectionError, OSError, asyncio.TimeoutError):
                    await asyncio.wait_for(writer.wait_closed(), timeout=1)

    def _parse_http_request(
        self,
        request_text: str,
    ) -> tuple[tuple[str, str, str], dict[str, str]]:
        lines = request_text.split("\r\n")
        request_parts = lines[0].split()
        if len(request_parts) != 3:
            raise ValueError("Bad HTTP request line")
        headers: dict[str, str] = {}
        for line in lines[1:]:
            if not line or ":" not in line:
                continue
            name, value = line.split(":", 1)
            headers[name.strip().lower()] = value.strip()
        return (request_parts[0], request_parts[1], request_parts[2]), headers

    def _is_websocket_request(self, path: str, headers: dict[str, str]) -> bool:
        connection = headers.get("connection", "").lower()
        upgrade = headers.get("upgrade", "").lower()
        return path == "/ws" and "upgrade" in connection and upgrade == "websocket"

    async def _handle_websocket(
        self,
        headers: dict[str, str],
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        key = headers.get("sec-websocket-key")
        if not key:
            await self._send_response(
                writer,
                "400 Bad Request",
                b"Missing WebSocket key\n",
                "text/plain; charset=utf-8",
            )
            return

        response = (
            "HTTP/1.1 101 Switching Protocols\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Accept: {_websocket_accept(key)}\r\n"
            "\r\n"
        )
        writer.write(response.encode("ascii"))
        await writer.drain()

        client = WebSocketClient(self, reader, writer)
        self.clients.add(client)
        for channel, (state, detail) in self.serial_status.items():
            await client.send_json(
                {
                    "type": "status",
                    "channel": channel,
                    "state": state,
                    "detail": detail,
                }
            )
        await client.run()

    async def _send_response(
        self,
        writer: asyncio.StreamWriter,
        status: str,
        body: bytes,
        content_type: str,
    ) -> None:
        response = (
            f"HTTP/1.1 {status}\r\n"
            f"Content-Type: {content_type}\r\n"
            f"Content-Length: {len(body)}\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii") + body
        writer.write(response)
        await writer.drain()


class TcpBroadcaster:
    def __init__(self, config: TcpConfig) -> None:
        self.config = config
        self.clients: set[asyncio.StreamWriter] = set()
        self.server: asyncio.AbstractServer | None = None

    async def start(self) -> None:
        if not self.config.enabled:
            LOGGER.info("TCP forwarding is disabled")
            return

        self.server = await asyncio.start_server(
            self._handle_client,
            self.config.host,
            self.config.port,
        )
        sockets = self.server.sockets or []
        addresses = ", ".join(str(sock.getsockname()) for sock in sockets)
        if not addresses:
            addresses = f"{self.config.host}:{self.config.port}"
        LOGGER.info("TCP server listening on %s", addresses)

    async def stop(self) -> None:
        if self.server is not None:
            self.server.close()
            await self.server.wait_closed()
            self.server = None

        for writer in tuple(self.clients):
            await self._close_client(writer)

    async def broadcast(self, data: bytes) -> None:
        if not self.config.enabled or not self.clients:
            return

        for writer in tuple(self.clients):
            try:
                writer.write(data)
                await asyncio.wait_for(writer.drain(), timeout=2)
            except (ConnectionError, OSError, asyncio.TimeoutError):
                peer = writer.get_extra_info("peername")
                LOGGER.warning("Dropping slow or disconnected TCP client %s", peer)
                await self._close_client(writer)

    async def _handle_client(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        peer = writer.get_extra_info("peername")
        self.clients.add(writer)
        LOGGER.info("TCP client connected: %s", peer)
        try:
            while await reader.read(1024):
                pass
        except (ConnectionError, OSError):
            pass
        finally:
            LOGGER.info("TCP client disconnected: %s", peer)
            await self._close_client(writer)

    async def _close_client(self, writer: asyncio.StreamWriter) -> None:
        self.clients.discard(writer)
        writer.close()
        with suppress(ConnectionError, OSError, asyncio.TimeoutError):
            await asyncio.wait_for(writer.wait_closed(), timeout=2)


class NtripPublisher:
    def __init__(self, config: NtripConfig) -> None:
        self.config = config
        self.queue: asyncio.Queue[bytes] = asyncio.Queue(maxsize=100)
        self.task: asyncio.Task[None] | None = None

    async def start(self) -> None:
        if not self.config.enabled:
            LOGGER.info("NTRIP publishing is disabled")
            return
        self.task = asyncio.create_task(self._run(), name="ntrip-publisher")

    async def stop(self) -> None:
        if self.task is None:
            return
        self.task.cancel()
        with suppress(asyncio.CancelledError):
            await self.task
        self.task = None

    def publish(self, data: bytes) -> None:
        if not self.config.enabled:
            return

        if self.queue.full():
            with suppress(asyncio.QueueEmpty):
                self.queue.get_nowait()

        with suppress(asyncio.QueueFull):
            self.queue.put_nowait(data)

    async def _run(self) -> None:
        while True:
            writer: asyncio.StreamWriter | None = None
            try:
                LOGGER.info(
                    "Connecting to NTRIP caster %s:%s mountpoint %s",
                    self.config.host,
                    self.config.port,
                    self.config.mountpoint,
                )
                reader, writer = await asyncio.open_connection(
                    self.config.host,
                    self.config.port,
                )
                await self._send_source_request(reader, writer)
                self._drop_queued_data()
                LOGGER.info("NTRIP source connected")

                while True:
                    data = await self.queue.get()
                    writer.write(data)
                    await writer.drain()
            except asyncio.CancelledError:
                raise
            except Exception as exc:  # noqa: BLE001 - keep the forwarder alive.
                LOGGER.warning(
                    "NTRIP publisher disconnected: %s; reconnecting in %.1f seconds",
                    exc,
                    self.config.reconnect_seconds,
                )
            finally:
                if writer is not None:
                    writer.close()
                    with suppress(ConnectionError, OSError, asyncio.TimeoutError):
                        await asyncio.wait_for(writer.wait_closed(), timeout=2)

            await asyncio.sleep(self.config.reconnect_seconds)

    async def _send_source_request(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        mountpoint = self.config.mountpoint.lstrip("/")
        request = (
            f"SOURCE {self.config.password} /{mountpoint}\r\n"
            f"Source-Agent: NTRIP {self.config.identifier}\r\n"
            "\r\n"
        )
        writer.write(request.encode("ascii", errors="replace"))
        await writer.drain()

        try:
            response = await asyncio.wait_for(reader.readline(), timeout=5)
        except asyncio.TimeoutError:
            LOGGER.info("NTRIP caster did not send an immediate response; continuing")
            return

        response_text = response.decode("latin1", errors="replace").strip()
        upper_response = response_text.upper()
        if "200" in upper_response or upper_response.startswith("OK"):
            LOGGER.info("NTRIP caster accepted source: %s", response_text)
            return

        raise RuntimeError(f"NTRIP caster rejected source: {response_text}")

    def _drop_queued_data(self) -> None:
        dropped = 0
        while True:
            try:
                self.queue.get_nowait()
                dropped += 1
            except asyncio.QueueEmpty:
                break
        if dropped:
            LOGGER.info("Dropped %s stale RTCM chunks before NTRIP publishing", dropped)


async def run_config_serial_console(
    config: SerialConfig,
    web: WebConsoleServer,
) -> None:
    reconnect_seconds = 3.0
    try:
        while True:
            writer: asyncio.StreamWriter | None = None
            try:
                detail = f"{config.config_port} @ {config.config_baudrate}"
                await web.set_serial_state("esp32", None, "connecting", detail)
                reader, writer = await serial_asyncio.open_serial_connection(
                    url=config.config_port,
                    baudrate=config.config_baudrate,
                )
                await web.set_serial_state("esp32", writer, "connected", detail)
                LOGGER.info(
                    "Reading ESP32 config serial from %s at %s baud",
                    config.config_port,
                    config.config_baudrate,
                )

                while True:
                    data = await reader.read(4096)
                    if not data:
                        raise RuntimeError(
                            f"Config serial port {config.config_port} closed"
                        )
                    await web.broadcast_serial_data("esp32", data)
            except asyncio.CancelledError:
                raise
            except (SerialException, OSError, RuntimeError) as exc:
                LOGGER.warning(
                    "ESP32 config serial unavailable on %s: %s; retrying in %.1f seconds",
                    config.config_port,
                    exc,
                    reconnect_seconds,
                )
                await web.set_serial_state("esp32", None, "retrying", str(exc))
                await asyncio.sleep(reconnect_seconds)
            finally:
                if writer is not None:
                    writer.close()
                    with suppress(ConnectionError, OSError, asyncio.TimeoutError, AttributeError):
                        await asyncio.wait_for(writer.wait_closed(), timeout=2)
    finally:
        await web.set_serial_state("esp32", None, "offline", "stopped")


async def run_forwarder(config: AppConfig) -> None:
    tcp = TcpBroadcaster(config.tcp)
    ntrip = NtripPublisher(config.ntrip)
    web = WebConsoleServer(config.web)
    rtcm = RtcmFrameExtractor(validate_crc=RTCM_VALIDATE_CRC)
    forwarded_frames = 0
    serial_writer: asyncio.StreamWriter | None = None
    config_serial_task: asyncio.Task[None] | None = None

    await tcp.start()
    await ntrip.start()
    await web.start()
    if config.web.enabled:
        config_serial_task = asyncio.create_task(
            run_config_serial_console(config.serial, web),
            name="esp32-config-serial",
        )

    try:
        try:
            await web.set_serial_state(
                "gnss",
                None,
                "connecting",
                f"{config.serial.port} @ {config.serial.baudrate}",
            )
            serial_reader, serial_writer = await serial_asyncio.open_serial_connection(
                url=config.serial.port,
                baudrate=config.serial.baudrate,
            )
        except (SerialException, OSError) as exc:
            await web.set_serial_state("gnss", None, "error", str(exc))
            raise RuntimeError(
                f"Could not open serial port {config.serial.port}: {exc}"
            ) from exc
        await web.set_serial_state(
            "gnss",
            serial_writer,
            "connected",
            f"{config.serial.port} @ {config.serial.baudrate}",
        )

        LOGGER.info(
            "Reading RTCM from %s at %s baud",
            config.serial.port,
            config.serial.baudrate,
        )
        LOGGER.info("RTCM3 frame filtering is enabled")

        loop = asyncio.get_running_loop()
        last_valid_frame_time = loop.time()
        last_no_frame_warning = loop.time()
        last_warning_discarded_bytes = 0
        last_warning_bad_crc_frames = 0
        rtcm_warning_active = False
        while True:
            data = await serial_reader.read(4096)
            if not data:
                raise RuntimeError(f"Serial port {config.serial.port} closed")

            await web.broadcast_serial_data("gnss", data)
            frames = rtcm.feed(data)
            if frames:
                for frame in frames:
                    await tcp.broadcast(frame)
                    ntrip.publish(frame)

                if forwarded_frames == 0:
                    LOGGER.info("First valid RTCM3 frame received")
                elif rtcm_warning_active:
                    LOGGER.info("Valid RTCM3 frames resumed")
                    rtcm_warning_active = False
                forwarded_frames += len(frames)
                last_valid_frame_time = loop.time()
                continue

            now = loop.time()
            no_frame_seconds = now - last_valid_frame_time
            should_warn = (
                no_frame_seconds >= NO_RTCM_WARNING_SECONDS
                and now - last_no_frame_warning >= NO_RTCM_WARNING_SECONDS
            )
            if should_warn:
                discarded_since_last_warning = (
                    rtcm.discarded_bytes - last_warning_discarded_bytes
                )
                bad_crc_since_last_warning = (
                    rtcm.bad_crc_frames - last_warning_bad_crc_frames
                )
                LOGGER.warning(
                    "Serial data is arriving, but no valid RTCM3 frames were found "
                    "in the last %.0f seconds. Ignored %s non-RTCM bytes and %s "
                    "bad-CRC frame candidates during that period.",
                    no_frame_seconds,
                    discarded_since_last_warning,
                    bad_crc_since_last_warning,
                )
                rtcm_warning_active = True
                last_no_frame_warning = now
                last_warning_discarded_bytes = rtcm.discarded_bytes
                last_warning_bad_crc_frames = rtcm.bad_crc_frames
    finally:
        if config_serial_task is not None:
            config_serial_task.cancel()
            with suppress(asyncio.CancelledError):
                await config_serial_task
        if serial_writer is not None:
            await web.set_serial_state("gnss", None, "offline", "stopped")
            serial_writer.close()
            with suppress(ConnectionError, OSError, asyncio.TimeoutError, AttributeError):
                await asyncio.wait_for(serial_writer.wait_closed(), timeout=2)
        else:
            await web.set_serial_state("gnss", None, "offline", "stopped")
        await web.stop()
        await ntrip.stop()
        await tcp.stop()


async def async_main(args: argparse.Namespace) -> int:
    config = load_config(Path(args.config))
    if args.check_config:
        print(f"Config OK: {args.config}")
        return 0

    forwarder_task = asyncio.create_task(run_forwarder(config), name="rtcm-forwarder")
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        with suppress(NotImplementedError):
            loop.add_signal_handler(sig, forwarder_task.cancel)

    try:
        await forwarder_task
    except asyncio.CancelledError:
        LOGGER.info("Shutdown requested")
        return 0
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Forward RTCM corrections from a serial GNSS receiver to TCP and NTRIP.",
    )
    parser.add_argument(
        "--config",
        default="config.yaml",
        help="Path to YAML config file (default: config.yaml)",
    )
    parser.add_argument(
        "--check-config",
        action="store_true",
        help="Validate the config file and exit without opening serial/TCP/NTRIP.",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"),
        help="Logging level (default: INFO)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    try:
        return asyncio.run(async_main(args))
    except ConfigError as exc:
        LOGGER.error("Config error: %s", exc)
        return 2
    except RuntimeError as exc:
        LOGGER.error("%s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
