# Host Setup

`robstride_driver` talks to the motor through Linux SocketCAN or through the official RobStride USB-CAN module (serial). No ROS installation is required.

Sections 2–5 apply to SocketCAN adapters; see section 7 for the RobStride USB-CAN module.

## 1. Install tools

```bash
sudo apt update
sudo apt install can-utils          # candump, cansend, ip link helpers
sudo apt install build-essential cmake
sudo apt install libgtest-dev       # optional: unit tests
```

## 2. Bring up the CAN interface

With a SocketCAN adapter (e.g. Canable with candleLight firmware) plugged in, a `can0` network interface appears. Configure it for the RobStride bit rate (1 Mbps):

```bash
sudo ip link set can0 down          # in case it was already up
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Verify:

```bash
ip -details link show can0          # state should be UP, bitrate 1000000
```

## 3. Check communication with the motor

With the motor powered:

```bash
candump can0
```

Then send an enable frame from another terminal (motor id 0x01, host id 0xFD):

```bash
cansend can0 0300FD01#0000000000000000
```

You should see a feedback frame (identifier starting with `02`) from the motor. Stop the motor again:

```bash
cansend can0 0400FD01#0000000000000000
```

## 4. Persistent configuration (optional)

### systemd-networkd

Create `/etc/systemd/network/80-can.network`:

```ini
[Match]
Name=can0

[CAN]
BitRate=1M
```

Then:

```bash
sudo systemctl enable --now systemd-networkd
```

### udev rename (multiple adapters)

To give a specific adapter a stable name, match its serial number:

```bash
udevadm info -a /sys/class/net/can0 | grep -i serial
```

`/etc/udev/rules.d/70-can.rules`:

```
SUBSYSTEM=="net", ACTION=="add", ATTRS{serial}=="<serial>", NAME="can_robstride"
```

## 5. Testing without hardware (vcan)

The library can be exercised against a virtual CAN interface:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

Point the driver at `vcan0` and observe outgoing frames with `candump vcan0`. Note that without a motor no response frames arrive, so commands will end with a `TimeoutError` — useful for verifying encoding, not full round trips. Full protocol round trips are covered by the unit tests (`ctest`), which use a mock CAN interface.

## 6. Build and run the example

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
./build/examples/velocity_control can0 1 2.0 3.0
```

The example switches motor id 1 into velocity mode, runs it at 2 rad/s for 3 seconds while printing feedback, then stops and disables it.

## 7. RobStride USB-CAN module (serial)

The official RobStride USB-CAN module uses a CH340 USB-serial bridge (mainline kernel driver), so it appears as `/dev/ttyUSB*` when plugged in — no `ip link` setup is involved.

Grant your user access to the serial port:

```bash
sudo usermod -aG dialout $USER      # takes effect after re-login
# or, for the current session only:
sudo chmod 666 /dev/ttyUSB0
```

Check the DIP switches on the module: switch 1 must be OFF (ON enters boot mode), switch 2 ON connects the built-in 120 Ω terminator.

Then pass the device path instead of a CAN interface name:

```bash
./build/examples/velocity_control /dev/ttyUSB0 1 2.0 3.0
```

`candump`/`cansend` do not work with this module; use the example (or `AtSerialCanInterface` directly) for bus debugging.
