#!/usr/bin/env python3
"""Forward RTCM correction data from a serial GNSS receiver."""

from __future__ import annotations

import argparse
import asyncio
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
class AppConfig:
    serial: SerialConfig
    tcp: TcpConfig
    ntrip: NtripConfig


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
    if not isinstance(serial_raw, dict):
        raise ConfigError("serial must be a mapping")
    if not isinstance(tcp_raw, dict):
        raise ConfigError("tcp must be a mapping")
    if not isinstance(ntrip_raw, dict):
        raise ConfigError("ntrip must be a mapping")

    serial = SerialConfig(
        port=_as_str(serial_raw.get("port", "/dev/ttyACM0"), "serial.port"),
        baudrate=_as_int(serial_raw.get("baudrate", 115200), "serial.baudrate"),
    )
    if serial.baudrate <= 0:
        raise ConfigError("serial.baudrate must be greater than 0")

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

    return AppConfig(serial=serial, tcp=tcp, ntrip=ntrip)


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


async def run_forwarder(config: AppConfig) -> None:
    tcp = TcpBroadcaster(config.tcp)
    ntrip = NtripPublisher(config.ntrip)
    rtcm = RtcmFrameExtractor(validate_crc=RTCM_VALIDATE_CRC)
    forwarded_frames = 0
    serial_writer = None

    await tcp.start()
    await ntrip.start()

    try:
        try:
            serial_reader, serial_writer = await serial_asyncio.open_serial_connection(
                url=config.serial.port,
                baudrate=config.serial.baudrate,
            )
        except (SerialException, OSError) as exc:
            raise RuntimeError(
                f"Could not open serial port {config.serial.port}: {exc}"
            ) from exc

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
        if serial_writer is not None:
            serial_writer.close()
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
