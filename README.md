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

## Resonance measurements (tier 2)

The vital-sign pipeline treats breathing and heartbeat as tuned circuits
(Radiotron Designer's Handbook ch. 8-9). About once a second it sends a
48-byte packet, magic `0xC511000F`, and answers the `resonance` command:

| Field | Meaning |
|---|---|
| `br` / `hr` `f0`, `q` | Spectral peak of the CSI phase and Q = f0 / (f2 - f1) at the half-power (70.7%) points. High Q = steady rhythm, low Q = irregular or noisy |
| `res_limited` | The peak is as narrow as the 256-sample (12.8 s) window can resolve, so Q is a lower bound. Breathing Q tops out near 2.2 for this reason |
| `br_decrement` / `hr_decrement` | Logarithmic decrement per cycle, ln(i / i'). About 0 for a sustained rhythm, > 0 when decaying (for example settling after movement). Exact Q = sqrt(pi^2/delta^2 + 1/4) |
| `am_br`, `am_depth` | The same breathing peak measured from amplitude (AM), and modulation depth m |
| `pm_index` | Peak phase deviation in radians in the breathing band (PM) |
| `am_pm_agree` | AM and PM breathing peaks are within 10% (or one resolution bin) of each other |

A peak counts as valid only when its power is at least 10x the median over
the scan. On white noise that gives about 1.5% false peaks.

The breathing and heart band-pass filters are designed by centre frequency
and Q, with 1-4 identical stages in cascade. Overall selectivity is
(1 + Q^2 Y^2)^(-n/2), where Y = f/f0 - f0/f. By default the filters have
one stage, with half-power points exactly at the band edges: breathing
0.1-0.5 Hz (f0 0.224 Hz, Q 0.56), heart 0.8-2.0 Hz (f0 1.265 Hz, Q 1.05).
Q is per stage, so adding stages narrows the band.

| Setting | Default | Where to set it |
|---|---|---|
| Breathing / heart Q x100 | 0 (band default) | NVS `bwave/br_q_x100`, `bwave/hr_q_x100` (u16) or Kconfig |
| Breathing / heart stages | 1 | NVS `bwave/br_stages`, `bwave/hr_stages` (u8) or Kconfig |

To retune at runtime (not saved across reboots), send
`{"cmd":"filter","br_q":2.0,"br_stages":2}`. A field left out keeps its
current value, and a Q of 0 restores the band default.

Host tests for the maths and the DSP pipeline (C compiler and libm only):

```bash
firmware/test/host/run.sh
```

## Interpreter

```bash
BWAVE_UDP_PORT=5005 BWAVE_HTTP_PORT=8210 python3 interpreter/bwave_interp.py
```

Resonance data is served at `/api/v1/resonance`. `/api/v1/history/{br_q,hr_q,am_depth}`
gives the recent history of each value.
