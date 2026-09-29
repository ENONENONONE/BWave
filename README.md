# BWave — agnostic build (for DeCLARE)

ESP32-C6 CSI sensor firmware and body interpreter with no network credentials,
site addresses, or personal identity compiled in. Every deployment-specific
value is provisioned per device.

## What is empty by default

| Setting | Default | Where to set it |
|---|---|---|
| WiFi SSID / password | empty | NVS `bwave/ssid`, `bwave/password` |
| Aggregator IP | empty | NVS `bwave/target_ip` |
| Aggregator port | 5005 | NVS `bwave/target_port` or Kconfig |
| Node ID | from MAC | NVS `bwave/node_id` or Kconfig |
| BWave position | 0 (unassigned) | NVS `bwave/position` or Kconfig `BWAVE_POSITION` |
| Identity hash | 32 zero bytes | NVS `bwave/hash` (32-byte blob) |
| BLE name | `BWAVE` | Kconfig `BWAVE_BLE_NAME` |

With BWave position 0, every identity field in the packet, BLE beacon and
`identity` command is 0. With a position `n`, the node derives
element `3^n mod 257`, fold element `element^-1`, ray `n mod 12`,
fold dlog `256 - n`.

## Build

```bash
cd firmware
idf.py set-target esp32c6
idf.py build
```

## Flashing over DeCLARE (ESP32-C6)

Write the app only — this keeps DeCLARE's bootloader, partition table,
NVS (`csi_cfg`) and SPIFFS intact:

```bash
esptool.py --chip esp32c6 write_flash 0x10000 build/bwave.bin
```

or `idf.py -p <port> app-flash`. The partition layouts agree on NVS
(`0x9000`, 24K) and the app offset (`0x10000`); BWave (~1.43 MB) fits
DeCLARE's 1920K app slot and never touches the internal storage partition.

BWave reads DeCLARE's settings as a read-only fallback, and anything in the
`bwave` namespace overrides them:

| BWave value | from `csi_cfg` |
|---|---|
| SSID / password | `ssid`, `password` |
| Aggregator IP / port | `target_ip`, `target_port` |
| BWave position | `obs_pos` (only when `obs_hash` is set) |

`node_id` is not taken from `csi_cfg` (DeCLARE ignores it too); it comes
from the MAC unless `bwave/node_id` is set. Not supported on ESP32-S3.

**Do not use the NVS-image method below on a DeCLARE device** — writing an
image at `0x9000` replaces the whole NVS partition and erases `csi_cfg`.

## Provision a blank device

Copy `provision.example.csv` to `provision.csv` (git-ignored), fill it in, then:

```bash
python $IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py generate provision.csv provision.bin 0x6000
esptool.py --chip esp32c6 write_flash 0x9000 provision.bin
```

`0x9000` / `0x6000` match the `nvs` row in `firmware/partitions.csv`.
Alternatively put values in a local `firmware/sdkconfig` via `idf.py menuconfig`
→ *BWave Configuration*; that file is git-ignored.

## Interpreter

```bash
BWAVE_UDP_PORT=5005 BWAVE_HTTP_PORT=8210 python3 interpreter/bwave_interp.py
```
