#!/usr/bin/env python3
"""stub_sdr.py - a world-driven SDR module (TASK-P5-08).

Mirrors the real de_sdr contract (src/sdr/sdr_main.cpp): a de.cap/1
advert for namespace "sdr" with actions scan{f_start, f_end,
threshold_dbm, dwell_ms, gain} + stop, state {scanning, f_start, f_end,
peak_dbm, peak_freq} and the sdr.signal event {freq, rssi_dbm, pos,
radius_m, snr_db}.

Instead of a SoapySDR spectrum the stub synthesizes each sweep from the
world: every entity with tx_dbm contributes a log-distance RSSI at its
freq_hz (the world's sdr model: ref_dbm_1m + tx - 10*n*log10(d)), plus a
seeded noise floor. The peak goes through the same debounce + throttle
state machine as CSDRSignalDetector, so sdr.signal fires only while a
transmitter is genuinely inside the modelled range - and only after
debounce_sweeps consecutive hits. radius_m is the same
sdrEstimateRadiusM inversion, clamped [50, 2000].

Config (injected by the world loader):
    signal_detect: {ref_power_dbm, path_loss_n, debounce_sweeps}
    sdr_scan:      {dwell_ms}
    stub:          {world, seed, unit, pose_port}
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from stub_base import StubBase  # noqa: E402
import world as world_mod  # noqa: E402
import geo_projection as gp  # noqa: E402

# the de_sdr signal_detect defaults (src/sdr/sdr_main.cpp)
REF_POWER_DBM = -30.0
PATH_LOSS_N = 2.7
DEBOUNCE_SWEEPS = 3
THROTTLE_S = 3.0
RISE_DB = 6.0
DWELL_MS = 500
RADIUS_MIN_M = 50.0
RADIUS_MAX_M = 2000.0

ADVERT = {
    "schema": "de.cap/1",
    "ns": "sdr",
    "module": "stub_sdr",
    "ver": "0.1.0",
    "actions": {
        "scan": {
            "desc": "Sweep [f_start, f_end] and report detected signals "
                    "as sdr.signal area events",
            "params": {
                "f_start": {"type": "number", "unit": "Hz", "min": 1},
                "f_end": {"type": "number", "unit": "Hz", "min": 1},
                "threshold_dbm": {"type": "number", "unit": "dBm",
                                  "default": -70},
                "dwell_ms": {"type": "int", "unit": "ms", "min": 0,
                             "default": DWELL_MS},
                "gain": {"type": ["number", "string"], "optional": True,
                         "desc": "gain dB or \"auto\""},
            },
            "timeout_s": 10,
        },
        "stop": {"desc": "Stop the running scan sweep", "timeout_s": 10},
    },
    "events": {
        "signal": {
            "desc": "Spectrum peak above threshold, debounced",
            "payload": {
                "freq": {"type": "number", "unit": "Hz"},
                "rssi_dbm": {"type": "number", "unit": "dBm"},
                "pos": {"type": "geo"},
                "radius_m": {"type": "number", "unit": "m"},
                "snr_db": {"type": "number", "unit": "dB"},
            },
        },
    },
    "state": {
        "scanning": {"type": "bool"},
        "f_start": {"type": "number", "unit": "Hz"},
        "f_end": {"type": "number", "unit": "Hz"},
        "peak_dbm": {"type": "number", "unit": "dBm"},
        "peak_freq": {"type": "number", "unit": "Hz"},
    },
}


def estimate_radius_m(rssi_dbm, ref_power_dbm, path_loss_n):
    """sdrEstimateRadiusM - free-space path-loss inversion, clamped."""
    if path_loss_n <= 0.0:
        return RADIUS_MAX_M
    d = 10.0 ** ((ref_power_dbm - rssi_dbm) / (10.0 * path_loss_n))
    return min(RADIUS_MAX_M, max(RADIUS_MIN_M, d))


class Detector:
    """CSDRSignalDetector: strongest bin, debounce by same-bin cluster,
    per-bin throttle unless the rssi rose rise_db over the last fired."""

    def __init__(self):
        self.threshold_dbm = -70.0
        self.debounce_sweeps = DEBOUNCE_SWEEPS
        self.throttle_s = THROTTLE_S
        self.rise_db = RISE_DB
        self.reset()

    def configure(self, threshold_dbm, debounce_sweeps):
        self.threshold_dbm = threshold_dbm
        self.debounce_sweeps = max(1, debounce_sweeps)
        self.reset()

    def reset(self):
        self.consec = 0
        self.candidate_freq_hz = 0.0
        self.last_fired = {}           # bin -> (t_s, rssi)

    def process_sweep(self, freq_min_hz, freq_step_hz, bins_dbm, now_s):
        """-> (freq_hz, rssi_dbm, snr_db) when an event should fire."""
        if not bins_dbm or freq_step_hz <= 0.0:
            self.consec = 0
            return None
        peak_i = max(range(len(bins_dbm)), key=lambda i: bins_dbm[i])
        peak_dbm = bins_dbm[peak_i]
        peak_freq = freq_min_hz + (peak_i + 0.5) * freq_step_hz
        if peak_dbm < self.threshold_dbm:
            self.consec = 0
            return None
        if self.consec > 0 and \
                abs(peak_freq - self.candidate_freq_hz) <= freq_step_hz:
            self.consec += 1
        else:
            self.consec = 1
        self.candidate_freq_hz = peak_freq
        if self.consec < self.debounce_sweeps:
            return None
        key = round(peak_freq / freq_step_hz)
        last = self.last_fired.get(key)
        if last is not None:
            cooled = (now_s - last[0]) >= self.throttle_s
            risen = peak_dbm >= last[1] + self.rise_db
            if not cooled and not risen:
                return None
        self.last_fired[key] = (now_s, peak_dbm)
        srt = sorted(bins_dbm)
        noise_dbm = srt[len(srt) // 2]
        return (peak_freq, peak_dbm, peak_dbm - noise_dbm)


class StubSDR(StubBase):
    MODULE_CLASS = "gen"
    MODULE_VERSION = "0.1.0"

    def __init__(self, argv):
        super().__init__(argv, "stub_sdr")
        sd = self.cfg.get("signal_detect") or {}
        self.ref_power_dbm = float(sd.get("ref_power_dbm", REF_POWER_DBM))
        self.path_loss_n = float(sd.get("path_loss_n", PATH_LOSS_N))
        self.debounce_sweeps = int(sd.get("debounce_sweeps",
                                          DEBOUNCE_SWEEPS))
        self.dwell_ms = int((self.cfg.get("sdr_scan") or {})
                            .get("dwell_ms", DWELL_MS))
        sdr = (self.world.sensors["sdr"] if self.world
               else dict(world_mod.SDR_DEFAULTS))
        self.noise_db = float(sdr.get("noise_db", 3.0))
        self.bin_hz = float(sdr.get("bin_hz", 25000.0))
        self.sweep_s = float(sdr.get("sweep_s", 2.0))
        self.detector = Detector()
        self.scan = None               # {f_start, f_end, threshold_dbm}
        self.peak_dbm = 0.0
        self.peak_freq = 0.0
        self._last_sweep = -1

    # ----------------------------------------------------------- invokes

    def on_invoke(self, ns, act, params):
        """handler(ns, act, params) -> (data, err), as the real module."""
        if act == "scan":
            f_start = params.get("f_start")
            f_end = params.get("f_end")
            if f_start is None or f_end is None:
                return None, "scan needs f_start and f_end"
            if f_end <= f_start:
                return None, "f_end must be greater than f_start"
            if self.scan is not None:
                return None, ("a scan is already running - "
                              "invoke sdr.stop first")
            if "gain" in params and params["gain"] is not None \
                    and not isinstance(params["gain"], (int, float)) \
                    and params["gain"] != "auto":
                return None, "gain must be a number or \"auto\""
            thr = float(params.get("threshold_dbm", -70.0))
            self.detector.configure(thr, self.debounce_sweeps)
            self.scan = {"f_start": float(f_start), "f_end": float(f_end),
                         "threshold_dbm": thr}
            self.log.log("SCAN", self.scan, ms=self.clock_ms())
            self.state({"scanning": True, "f_start": self.scan["f_start"],
                        "f_end": self.scan["f_end"]})
            return {"f_start": self.scan["f_start"],
                    "f_end": self.scan["f_end"]}, None
        if act == "stop":
            was = self.scan is not None
            self.scan = None
            self.detector.reset()
            self.state({"scanning": False})
            self.log.log("STOP", {"stopped": was}, ms=self.clock_ms())
            return {"stopped": was}, None
        return None, f"unknown action '{act}'"

    # -------------------------------------------------------- databus out

    def state(self, changed, full=False):
        self.log.log("STATE", changed, ms=self.clock_ms())
        if self.facade is not None:
            self.facade.publish_state("sdr", changed, full)

    def event(self, payload):
        self.log.log("EVENT", {"d": "sdr.signal", "ed": payload},
                     ms=self.clock_ms())
        if self.facade is not None:
            self.facade.fire_event("sdr", "signal", payload)

    # ------------------------------------------------------------- model

    def tick(self, t_s, _frame20):
        if self.world is None or self.scan is None:
            return
        sweep = int(t_s / self.sweep_s)
        if sweep == self._last_sweep:
            return
        self._last_sweep = sweep
        pose = self.pose_feed.pose()
        if pose is None:
            return
        f_start, f_end = self.scan["f_start"], self.scan["f_end"]
        n_bins = max(1, int((f_end - f_start) / self.bin_hz))
        n_bins = min(n_bins, 4096)
        # noise floor: one seeded draw per bin per sweep
        r = self.rng("sdr", self.unit, sweep)
        bins = [-100.0 + r.gauss(0.0, self.noise_db)
                for _ in range(n_bins)]
        # each transmitter inside the band lifts its nearest bin by the
        # log-distance RSSI at the unit's current range
        for e in self.world.rf_entities():
            f = e.freq_hz if e.freq_hz is not None else \
                float(self.world.sensors["sdr"]["freq_hz"])
            if not (f_start <= f <= f_end):
                continue
            lat, lng, _alt = e.pos_at(t_s, self.world)
            d = max(1.0, gp.geo_distance_m(pose["lat"], pose["lng"],
                                           lat, lng))
            rssi = self.tx_ref(e) - 10.0 * self.path_loss_n * \
                math.log10(d)
            bi = int((f - f_start) / self.bin_hz)
            if 0 <= bi < n_bins:
                bins[bi] = max(bins[bi], rssi)
        det = self.detector.process_sweep(f_start, self.bin_hz, bins, t_s)
        if det is None:
            return
        freq, rssi, snr = det
        self.peak_dbm, self.peak_freq = round(rssi, 1), freq
        self.event({"freq": freq, "rssi_dbm": round(rssi, 1),
                    "pos": [pose["lat"], pose["lng"],
                            round(pose["alt_m"], 1)],
                    "radius_m": round(estimate_radius_m(
                        rssi, self.ref_power_dbm, self.path_loss_n), 0),
                    "snr_db": round(snr, 1)})
        self.state({"peak_dbm": self.peak_dbm,
                    "peak_freq": self.peak_freq})
        # truth row: the emitter that owns the peak bin (or "clutter")
        tid = "clutter"
        for e in self.world.rf_entities():
            f = e.freq_hz if e.freq_hz is not None else \
                float(self.world.sensors["sdr"]["freq_hz"])
            if abs(f - freq) <= self.bin_hz:
                tid = e.id
                break
        self.truth.append({"src": "sdr", "tid": tid, "cls": "rf",
                           "freq": freq, "rssi_dbm": round(rssi, 1),
                           "lat": pose["lat"], "lng": pose["lng"],
                           "sent": True}, ms=self.clock_ms())

    def tx_ref(self, e):
        """RSSI at 1 m for this emitter: the world's ref_dbm_1m + tx_dbm.
        (ref_power_dbm in signal_detect is the radius calibration - kept
        identical to tx_ref(strongest tx) by the world loader.)"""
        sdr = self.world.sensors["sdr"]
        return float(sdr["ref_dbm_1m"]) + float(e.tx_dbm or 0.0)


def main(argv):
    s = StubSDR(argv)
    if s.selftest:
        # replay: a scan covering the world band starts at t=0 unless the
        # pose rows carry their own invokes
        s._vt_ms = 0
        s.scan = None
        if s.world is not None:
            sdr = s.world.sensors["sdr"]
            f0 = float(sdr["freq_hz"]) - 20 * s.bin_hz
            s.detector.configure(float(sdr.get("threshold_dbm", -70.0)),
                                 s.debounce_sweeps)
            s.scan = {"f_start": f0,
                      "f_end": float(sdr["freq_hz"]) + 20 * s.bin_hz,
                      "threshold_dbm": float(sdr.get("threshold_dbm",
                                                    -70.0))}
            s.log.log("SCAN", s.scan, ms=0)
        s.run_selftest()
        return 0
    s.connect(adverts=[__import__("json").dumps(ADVERT)])
    s.state({"scanning": False, "f_start": 0, "f_end": 0,
             "peak_dbm": 0, "peak_freq": 0}, full=True)
    s.run()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
