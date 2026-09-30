#!/usr/bin/env python3
"""BWave Interpreter — body-mapping CSI interpreter for BWave node 180.

UDP 5005 listener, parses all BWave packet types, serves HTTP/WS on port 8210.
Separate from DeClare's interpreter_exp.py — this is body-only.

Packet magics (0xC511xxxx LE):
  0x01 = CSI raw frame (26-byte hdr + IQ)
  0x02 = Vitals (32 bytes)
  0x03 = Feature vector (48 bytes)
  0x08 = Amplitude baseline (8 + n_sc*2)
  0x09 = Brainwave bands (24 bytes)
  0x0A = Multi-person vitals
  0x0E = Doppler velocity (40 bytes)
  0x0F = Resonance: Q, decrement, AM/PM (48 bytes)
"""

import asyncio
import json
import logging
import math
import os
import struct
import time
from collections import deque

from aiohttp import web

log = logging.getLogger("bwave_interp")
logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")

UDP_PORT = int(os.environ.get("BWAVE_UDP_PORT", "5005"))
HTTP_PORT = int(os.environ.get("BWAVE_HTTP_PORT", "8210"))
BWAVE_NODE = int(os.environ.get("BWAVE_NODE_ID", "0"))

MAGIC_CSI      = 0xC5110001
MAGIC_VITALS   = 0xC5110002
MAGIC_FEATURE  = 0xC5110003
MAGIC_BASELINE = 0xC5110008
MAGIC_BRAIN    = 0xC5110009
MAGIC_PERSON   = 0xC511000A
MAGIC_DOPPLER  = 0xC511000E
MAGIC_RESONANCE = 0xC511000F

HISTORY_LEN = 300


class BWaveState:
    def __init__(self):
        self.node_id = BWAVE_NODE
        self.packet_count = 0
        self.csi_count = 0
        self.last_update = 0.0

        self.vitals = {}
        self.brainwave = {}
        self.feature = {}
        self.doppler = {}
        self.persons = []
        self.baseline = {}
        self.resonance = {}

        self.amplitudes = []
        self.phases = []
        self.n_subcarriers = 0

        self.hr_history = deque(maxlen=HISTORY_LEN)
        self.br_history = deque(maxlen=HISTORY_LEN)
        self.motion_history = deque(maxlen=HISTORY_LEN)
        self.presence_history = deque(maxlen=HISTORY_LEN)
        self.delta_history = deque(maxlen=HISTORY_LEN)
        self.theta_history = deque(maxlen=HISTORY_LEN)
        self.alpha_history = deque(maxlen=HISTORY_LEN)
        self.amp_history = deque(maxlen=60)
        self.br_q_history = deque(maxlen=HISTORY_LEN)
        self.hr_q_history = deque(maxlen=HISTORY_LEN)
        self.am_depth_history = deque(maxlen=HISTORY_LEN)

        self.ws_clients = set()

    def parse_packet(self, data):
        if len(data) < 4:
            return
        magic = struct.unpack_from("<I", data, 0)[0]
        self.packet_count += 1
        self.last_update = time.time()

        if magic == MAGIC_CSI:
            self._parse_csi(data)
        elif magic == MAGIC_VITALS:
            self._parse_vitals(data)
        elif magic == MAGIC_FEATURE:
            self._parse_feature(data)
        elif magic == MAGIC_BASELINE:
            self._parse_baseline(data)
        elif magic == MAGIC_BRAIN:
            self._parse_brain(data)
        elif magic == MAGIC_PERSON:
            self._parse_person(data)
        elif magic == MAGIC_DOPPLER:
            self._parse_doppler(data)
        elif magic == MAGIC_RESONANCE:
            self._parse_resonance(data)

    def _parse_csi(self, data):
        if len(data) < 26:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return
        if not self.node_id:
            self.node_id = node_id
            log.info("Auto-detected node_id=%d", node_id)

        n_sub = struct.unpack_from("<H", data, 6)[0]
        rssi = struct.unpack_from("<b", data, 16)[0]
        noise = struct.unpack_from("<b", data, 17)[0]
        radio = data[19]

        iq_data = data[26:]
        if len(iq_data) < n_sub * 2:
            return

        self.csi_count += 1
        self.n_subcarriers = n_sub

        amps = []
        phases = []
        for k in range(n_sub):
            i_val = struct.unpack_from("<b", iq_data, 2 * k)[0]
            q_val = struct.unpack_from("<b", iq_data, 2 * k + 1)[0]
            amp = math.sqrt(i_val * i_val + q_val * q_val)
            phase = math.atan2(q_val, i_val)
            amps.append(round(amp, 2))
            phases.append(round(phase, 4))

        self.amplitudes = amps
        self.phases = phases
        self.amp_history.append({"t": time.time(), "amps": amps})

    def _parse_vitals(self, data):
        if len(data) < 32:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return

        flags = data[5]
        br_raw = struct.unpack_from("<H", data, 6)[0]
        hr_raw = struct.unpack_from("<I", data, 8)[0]
        rssi = struct.unpack_from("<b", data, 12)[0]
        n_persons = data[13]
        die_temp = struct.unpack_from("<b", data, 14)[0]
        motion = struct.unpack_from("<f", data, 16)[0]
        presence = struct.unpack_from("<f", data, 20)[0]
        ts_ms = struct.unpack_from("<I", data, 24)[0]

        br = br_raw / 100.0
        hr = hr_raw / 10000.0

        now = time.time()
        self.vitals = {
            "node_id": node_id,
            "breathing_rate": round(br, 2),
            "heartrate": round(hr, 2),
            "rssi": rssi,
            "n_persons": n_persons,
            "die_temp_c": die_temp,
            "motion_energy": round(motion, 4),
            "presence_score": round(presence, 4),
            "presence_detected": bool(flags & 1),
            "fall_detected": bool(flags & 2),
            "timestamp_ms": ts_ms,
            "updated": now,
        }

        if br > 0:
            self.br_history.append({"t": now, "v": round(br, 2)})
        if hr > 0:
            self.hr_history.append({"t": now, "v": round(hr, 2)})
        self.motion_history.append({"t": now, "v": round(motion, 4)})
        self.presence_history.append({"t": now, "v": round(presence, 4)})

        self._broadcast("vitals", self.vitals)

    def _parse_brain(self, data):
        if len(data) < 24:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return

        ts_ms = struct.unpack_from("<H", data, 6)[0]
        delta = struct.unpack_from("<f", data, 8)[0]
        theta = struct.unpack_from("<f", data, 12)[0]
        alpha = struct.unpack_from("<f", data, 16)[0]

        now = time.time()
        self.brainwave = {
            "delta": round(delta, 6),
            "theta": round(theta, 6),
            "alpha": round(alpha, 6),
            "timestamp_ms": ts_ms,
            "updated": now,
        }
        self.delta_history.append({"t": now, "v": round(delta, 6)})
        self.theta_history.append({"t": now, "v": round(theta, 6)})
        self.alpha_history.append({"t": now, "v": round(alpha, 6)})

        self._broadcast("brainwave", self.brainwave)

    def _parse_feature(self, data):
        if len(data) < 48:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return

        seq = struct.unpack_from("<H", data, 6)[0]
        ts_us = struct.unpack_from("<q", data, 8)[0]
        features = list(struct.unpack_from("<8f", data, 16))

        self.feature = {
            "seq": seq,
            "timestamp_us": ts_us,
            "presence": round(features[0], 4),
            "motion": round(features[1], 4),
            "breathing_norm": round(features[2], 4),
            "heartrate_norm": round(features[3], 4),
            "phase_variance": round(features[4], 4),
            "person_count_norm": round(features[5], 4),
            "fall_risk": round(features[6], 4),
            "rssi_norm": round(features[7], 4),
            "updated": time.time(),
        }

    def _parse_doppler(self, data):
        if len(data) < 40:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return

        n_carriers = data[5]
        ts_ms = struct.unpack_from("<H", data, 6)[0]
        velocities = []
        for i in range(min(n_carriers, 8)):
            v = struct.unpack_from("<f", data, 8 + i * 4)[0]
            velocities.append(round(v, 4))

        self.doppler = {
            "n_carriers": n_carriers,
            "velocity": velocities,
            "timestamp_ms": ts_ms,
            "updated": time.time(),
        }
        self._broadcast("doppler", self.doppler)

    def _parse_resonance(self, data):
        if len(data) < 48:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return

        flags = data[5]
        ts_ms = struct.unpack_from("<H", data, 6)[0]
        (br_f0, br_q, hr_f0, hr_q, br_dec, hr_dec,
         am_depth, pm_index, am_f0, am_q) = struct.unpack_from("<10f", data, 8)

        def q_from_decrement(d):
            # exact for a damped tuned circuit: Q^2 = pi^2/delta^2 + 1/4
            return round(math.sqrt(math.pi ** 2 / d ** 2 + 0.25), 2) if d > 0 else None

        def peak(f0, q, valid, res_limited):
            return {
                "valid": valid,
                "f0_hz": round(f0, 4),
                "bpm": round(f0 * 60.0, 2),
                "q": round(q, 2),
                "res_limited": res_limited,
            }

        now = time.time()
        self.resonance = {
            "breathing": peak(br_f0, br_q, bool(flags & 0x01), bool(flags & 0x08)),
            "heart": peak(hr_f0, hr_q, bool(flags & 0x02), bool(flags & 0x10)),
            "am_breathing": peak(am_f0, am_q, bool(flags & 0x04), bool(flags & 0x40)),
            "br_decrement": round(br_dec, 4),
            "br_q_decrement": q_from_decrement(br_dec),
            "hr_decrement": round(hr_dec, 4),
            "hr_q_decrement": q_from_decrement(hr_dec),
            "am_depth": round(am_depth, 4),
            "pm_index": round(pm_index, 4),
            "am_pm_agree": bool(flags & 0x20),
            "timestamp_ms": ts_ms,
            "updated": now,
        }

        if flags & 0x01:
            self.br_q_history.append({"t": now, "v": round(br_q, 2)})
        if flags & 0x02:
            self.hr_q_history.append({"t": now, "v": round(hr_q, 2)})
        self.am_depth_history.append({"t": now, "v": round(am_depth, 4)})

        self._broadcast("resonance", self.resonance)

    def _parse_person(self, data):
        if len(data) < 8:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return

        n_persons = data[5]
        ts_ms = struct.unpack_from("<H", data, 6)[0]
        persons = []
        offset = 8
        for p in range(min(n_persons, 4)):
            if offset + 6 > len(data):
                break
            br = struct.unpack_from("<H", data, offset)[0] / 100.0
            hr = struct.unpack_from("<H", data, offset + 2)[0] / 100.0
            sc_idx = data[offset + 4]
            active = data[offset + 5]
            if active:
                persons.append({
                    "id": p,
                    "breathing_rate": round(br, 2),
                    "heartrate": round(hr, 2),
                    "subcarrier_idx": sc_idx,
                })
            offset += 6

        self.persons = persons
        self._broadcast("persons", {"persons": persons, "timestamp_ms": ts_ms})

    def _parse_baseline(self, data):
        if len(data) < 8:
            return
        node_id = data[4]
        if self.node_id and node_id != self.node_id:
            return

        die_temp = struct.unpack_from("<b", data, 5)[0]
        n_sc = struct.unpack_from("<H", data, 6)[0]
        if len(data) < 8 + n_sc * 2:
            return

        baseline = []
        for k in range(n_sc):
            val = struct.unpack_from("<H", data, 8 + k * 2)[0] / 100.0
            baseline.append(round(val, 2))

        self.baseline = {
            "n_subcarriers": n_sc,
            "die_temp_c": die_temp,
            "amplitudes": baseline,
            "updated": time.time(),
        }
        self._broadcast("baseline", self.baseline)

    def _broadcast(self, msg_type, payload):
        if not self.ws_clients:
            return
        msg = json.dumps({"type": msg_type, "data": payload})
        dead = set()
        for ws in self.ws_clients:
            try:
                asyncio.ensure_future(ws.send_str(msg))
            except Exception:
                dead.add(ws)
        self.ws_clients -= dead

    def summary(self):
        return {
            "node_id": self.node_id,
            "packet_count": self.packet_count,
            "csi_count": self.csi_count,
            "n_subcarriers": self.n_subcarriers,
            "last_update": self.last_update,
            "vitals": self.vitals,
            "brainwave": self.brainwave,
            "feature": self.feature,
            "doppler": self.doppler,
            "persons": self.persons,
            "baseline_n_sc": self.baseline.get("n_subcarriers", 0),
            "resonance": self.resonance,
        }


state = BWaveState()


class UDPProtocol(asyncio.DatagramProtocol):
    def datagram_received(self, data, addr):
        state.parse_packet(data)


async def handle_summary(request):
    return web.json_response(state.summary())


async def handle_vitals(request):
    return web.json_response(state.vitals or {"error": "no data"})


async def handle_brainwave(request):
    return web.json_response(state.brainwave or {"error": "no data"})


async def handle_amplitudes(request):
    return web.json_response({
        "node_id": state.node_id,
        "n_subcarriers": state.n_subcarriers,
        "amplitudes": state.amplitudes,
        "phases": state.phases,
    })


async def handle_feature(request):
    return web.json_response(state.feature or {"error": "no data"})


async def handle_doppler(request):
    return web.json_response(state.doppler or {"error": "no data"})


async def handle_persons(request):
    return web.json_response({"persons": state.persons})


async def handle_baseline(request):
    return web.json_response(state.baseline or {"error": "no data"})


async def handle_resonance(request):
    return web.json_response(state.resonance or {"error": "no data"})


async def handle_history(request):
    which = request.match_info.get("series", "hr")
    histories = {
        "hr": state.hr_history,
        "br": state.br_history,
        "motion": state.motion_history,
        "presence": state.presence_history,
        "delta": state.delta_history,
        "theta": state.theta_history,
        "alpha": state.alpha_history,
        "br_q": state.br_q_history,
        "hr_q": state.hr_q_history,
        "am_depth": state.am_depth_history,
    }
    h = histories.get(which)
    if h is None:
        return web.json_response({"error": "unknown series", "available": list(histories.keys())})
    return web.json_response({"series": which, "data": list(h)})


async def handle_ws(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    state.ws_clients.add(ws)
    log.info("WS client connected (%d total)", len(state.ws_clients))

    try:
        async for msg in ws:
            pass
    finally:
        state.ws_clients.discard(ws)
        log.info("WS client disconnected (%d remaining)", len(state.ws_clients))
    return ws


@web.middleware
async def cors_middleware(request, handler):
    if request.method == "OPTIONS":
        resp = web.Response(status=204)
    else:
        resp = await handler(request)
    resp.headers["Access-Control-Allow-Origin"] = "*"
    resp.headers["Access-Control-Allow-Headers"] = "*"
    resp.headers["Access-Control-Allow-Methods"] = "GET, OPTIONS"
    return resp


async def start_udp(app):
    loop = asyncio.get_event_loop()
    transport, protocol = await loop.create_datagram_endpoint(
        UDPProtocol, local_addr=("0.0.0.0", UDP_PORT)
    )
    app["udp_transport"] = transport
    log.info("UDP listener bound :%d", UDP_PORT)


async def cleanup_udp(app):
    t = app.get("udp_transport")
    if t:
        t.close()


def main():
    app = web.Application(middlewares=[cors_middleware])
    app.router.add_get("/api/v1/summary", handle_summary)
    app.router.add_get("/api/v1/vitals", handle_vitals)
    app.router.add_get("/api/v1/brainwave", handle_brainwave)
    app.router.add_get("/api/v1/amplitudes", handle_amplitudes)
    app.router.add_get("/api/v1/feature", handle_feature)
    app.router.add_get("/api/v1/doppler", handle_doppler)
    app.router.add_get("/api/v1/persons", handle_persons)
    app.router.add_get("/api/v1/baseline", handle_baseline)
    app.router.add_get("/api/v1/resonance", handle_resonance)
    app.router.add_get("/api/v1/history/{series}", handle_history)
    app.router.add_get("/ws", handle_ws)

    app.on_startup.append(start_udp)
    app.on_cleanup.append(cleanup_udp)

    log.info("BWave Interpreter — node=%d UDP=%d HTTP=%d", state.node_id, UDP_PORT, HTTP_PORT)
    web.run_app(app, host="0.0.0.0", port=HTTP_PORT)


if __name__ == "__main__":
    main()
