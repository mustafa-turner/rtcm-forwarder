# RTCM Forwarder

A small Raspberry Pi Python service that reads RTCM correction data from a GNSS
receiver on `/dev/ttyACM0`, broadcasts the stream over TCP, and publishes the
same stream to an NTRIP caster mountpoint.

## What it does

- Reads binary RTCM data from a serial device such as `/dev/ttyACM0`.
- Ignores NMEA text and forwards only valid RTCM3 frames by default.
- Starts a local TCP server so one or more clients can receive the stream.
- Connects to an NTRIP caster as a source/server using:

  ```text
  SOURCE <password> /<mountpoint>
  ```

- Reconnects to the NTRIP caster if the network or caster drops.

## Raspberry Pi Setup

Install Python, virtualenv support, and tools:

```bash
sudo apt update
sudo apt install -y python3 python3-venv python3-pip
```

Allow the `pi` user to access USB serial devices:

```bash
sudo usermod -a -G dialout pi
```

Log out and back in, or reboot:

```bash
sudo reboot
```

After reboot, confirm the GNSS receiver appears:

```bash
ls -l /dev/ttyACM0
```

## Confirm The Receiver Outputs RTCM

If you see readable lines like this from the serial port:

```text
$GNGGA,123033.00,,,,,0,00,99.99,,,,,,*78
$GNRMC,123034.00,V,,,,,,,280526,,,N,V*15
```

that is NMEA, not RTCM. The sample above also shows no valid GNSS fix:
`GGA` fix quality is `0`, satellite count is `00`, and `RMC` status is `V`.

For this forwarder to be useful as an NTRIP source, configure the receiver as a
base station and enable RTCM3 output on the USB or UART port connected to the Pi.
Common RTCM3 messages for a multi-GNSS base include station coordinates
`1005` or `1006`, GPS MSM `1074` or `1077`, GLONASS MSM `1084` or `1087`,
Galileo MSM `1094` or `1097`, BeiDou MSM `1124` or `1127`, and GLONASS bias
`1230`.

The exact setup depends on the receiver. For a u-blox RTK receiver, this is
usually done in u-center, PyGPSClient, or `ubxtool` by setting survey-in or fixed
base coordinates and enabling RTCM3 messages on the output port.

RTCM is binary, so it will not look like readable text in `screen`. A quick check
is:

```bash
xxd -g 1 -l 64 /dev/ttyACM0
```

Valid RTCM3 frames start with byte `d3`. Seeing only `$GN...`, `$GP...`,
`$GL...`, `$GA...`, or `$GB...` sentences means the receiver is still outputting
NMEA only.

## Install

Go to this project directory:

```bash
cd /home/pi/rtcm-forwarder
```

Create and activate a virtual environment:

```bash
python3 -m venv .venv
source .venv/bin/activate
```

Upgrade pip and install the Python dependencies:

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

Create your local config:

```bash
cp config.example.yaml config.yaml
nano config.yaml
```

Edit the NTRIP caster settings:

```yaml
ntrip:
  enabled: true
  host: caster.example.com
  port: 2101
  mountpoint: MOUNTPOINT
  password: source-password
  identifier: raspberry-pi-rtcm
  reconnect_seconds: 5
```

`config.yaml` is ignored by git so you do not publish your caster password.

## Run Manually

Validate the config:

```bash
.venv/bin/python rtcm-forwarder.py --config config.yaml --check-config
```

Start the forwarder:

```bash
.venv/bin/python rtcm-forwarder.py --config config.yaml
```

From another device on the network, test the TCP stream:

```bash
nc <pi-ip-address> 2101 > rtcm.bin
```

Stop the manual run with `Ctrl+C`.

## Run On Boot With systemd

Copy the service file:

```bash
sudo cp rtcm-forwarder.service /etc/systemd/system/
sudo systemctl daemon-reload
```

Enable and start it:

```bash
sudo systemctl enable --now rtcm-forwarder
```

Check status:

```bash
systemctl status rtcm-forwarder
```

Follow logs:

```bash
journalctl -u rtcm-forwarder -f
```

Restart after changing `config.yaml`:

```bash
sudo systemctl restart rtcm-forwarder
```

Disable it:

```bash
sudo systemctl disable --now rtcm-forwarder
```

## Configuration

```yaml
serial:
  port: /dev/ttyACM0
  baudrate: 115200

tcp:
  enabled: true
  host: 0.0.0.0
  port: 2101

ntrip:
  enabled: true
  host: caster.example.com
  port: 2101
  mountpoint: MOUNTPOINT
  password: source-password
  identifier: raspberry-pi-rtcm
  reconnect_seconds: 5
```

Set `tcp.enabled` or `ntrip.enabled` to `false` if you only want one output.
The forwarder always validates RTCM3 frames and ignores non-RTCM serial data, so
NMEA text is not sent to your TCP clients or NTRIP caster.

## Troubleshooting

If `/dev/ttyACM0` does not exist, unplug and reconnect the GNSS receiver, then
check recent kernel messages:

```bash
dmesg | tail -50
```

If the service cannot read `/dev/ttyACM0`, confirm the user has serial access:

```bash
groups pi
```

If the NTRIP caster rejects the connection, check `host`, `port`, `mountpoint`,
and `password` in `config.yaml`.

If the forwarder logs `no valid RTCM3 frames`, the serial port is receiving data,
but no validated RTCM3 frame has arrived recently. Reconfigure the receiver to
output RTCM3 on the Pi-connected port, or check whether RTCM output has stopped.
