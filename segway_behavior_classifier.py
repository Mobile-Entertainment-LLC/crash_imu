"""
Segway behavior classifier - IMU + GPS + Barometer.
Writes CSV + MCAP. No ROS, no Docker. Open MCAP in Foxglove Studio.

Sensors recorded:
  - IMU  (LSM6DSL accel+gyro)  at 100 Hz
  - Baro (BMP388 temp/press)   at 10 Hz
  - GPS  (uBlox CAM-M8 NMEA)   at 1 Hz
"""

import argparse
import csv
import json
import math
import os
import struct
import subprocess
import threading
import time
import wave
from datetime import datetime

import IMU
import BMP388
import gps as gpsd_module


ACCEL_SCALE_G_PER_LSB  = 0.000244
GYRO_SCALE_DPS_PER_LSB = 0.00875
G_TO_MS2               = 9.80665
DPS_TO_RADS            = math.pi / 180.0

TRIGGER_ACCEL_G     = 2.0
TRIGGER_JERK_GPS    = 25.0
TRIGGER_GYRO_DPS    = 60.0
TRIGGER_ROLL_DEG    = 30.0

EVENT_WINDOW_SEC    = 1.0
FALL_ROLL_DEG       = 30.0
CRASH_PEAK_ACCEL_G  = 4.0
CRASH_PEAK_JERK_GPS = 40.0
HARD_STOP_PEAK_G    = 0.7
HARD_STOP_PEAK_JERK = 10.0

IDLE_ACCEL_BAND  = 0.12
IDLE_GYRO_DPS    = 3.0

BEEP_PATH    = "/tmp/crash_beep.wav"
AUDIO_DEVICE = "plughw:2,0"

OUTPUT_DIR = os.path.expanduser("~/crash_imu/recordings")


def make_beep_wav(path, freq=800, duration=1.0, sample_rate=44100):
    n = int(sample_rate * duration)
    fade = int(sample_rate * 0.01)
    with wave.open(path, "w") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        for i in range(n):
            f = 1.0
            if i < fade:        f = i / fade
            elif i > n - fade:  f = (n - i) / fade
            v = int(32767 * 0.6 * f * math.sin(2 * math.pi * freq * i / sample_rate))
            w.writeframes(struct.pack("<h", v))


def play_beep_async():
    try:
        subprocess.Popen(
            ["aplay", "-q", "-D", AUDIO_DEVICE, BEEP_PATH],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        pass


def read_imu():
    ax = IMU.readACCx() * ACCEL_SCALE_G_PER_LSB
    ay = IMU.readACCy() * ACCEL_SCALE_G_PER_LSB
    az = IMU.readACCz() * ACCEL_SCALE_G_PER_LSB
    gx = IMU.readGYRx() * GYRO_SCALE_DPS_PER_LSB
    gy = IMU.readGYRy() * GYRO_SCALE_DPS_PER_LSB
    gz = IMU.readGYRz() * GYRO_SCALE_DPS_PER_LSB
    return ax, ay, az, gx, gy, gz


class GpsReader:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = {
            "fix": "NO_FIX", "lat": None, "lon": None,
            "alt_m": None, "speed_mps": None, "sats_used": 0, "ts": 0.0,
        }
        self._running = False
        self._thread = None
        self._session = None

    def start(self):
        try:
            self._session = gpsd_module.gps(mode=gpsd_module.WATCH_ENABLE)
        except Exception as e:
            print(f"[GPS] Could not connect to gpsd: {e}. GPS disabled.")
            self._session = None
            return False
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[GPS] Connected to gpsd")
        return True

    def _loop(self):
        while self._running:
            try:
                report = self._session.next()
                if report.get("class") == "TPV":
                    fix_modes = {0: "NO_FIX", 1: "NO_FIX", 2: "2D", 3: "3D"}
                    with self._lock:
                        self._latest["fix"] = fix_modes.get(report.get("mode", 0), "NO_FIX")
                        self._latest["lat"] = report.get("lat")
                        self._latest["lon"] = report.get("lon")
                        self._latest["alt_m"] = report.get("altMSL")
                        self._latest["speed_mps"] = report.get("speed")
                        self._latest["ts"] = time.time()
                elif report.get("class") == "SKY":
                    sats_used = sum(1 for s in report.get("satellites", []) if s.get("used"))
                    with self._lock:
                        self._latest["sats_used"] = sats_used
            except StopIteration:
                break
            except Exception:
                time.sleep(0.5)

    def latest(self):
        with self._lock:
            return dict(self._latest)

    def stop(self):
        self._running = False


class McapWriter:
    IMU_SCHEMA = {
        "title": "ImuSample", "type": "object",
        "properties": {
            "timestamp_unix": {"type": "number"},
            "linear_acceleration": {"type": "object", "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}}},
            "angular_velocity": {"type": "object", "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}}},
            "accel_mag_g": {"type": "number"}, "gyro_mag_dps": {"type": "number"},
            "roll_deg": {"type": "number"}, "jerk_gps": {"type": "number"},
            "live_state": {"type": "string"},
        },
    }
    BARO_SCHEMA = {
        "title": "Baro", "type": "object",
        "properties": {
            "timestamp_unix": {"type": "number"},
            "temperature_c": {"type": "number"},
            "pressure_hpa": {"type": "number"},
            "altitude_m": {"type": "number"},
        },
    }
    GPS_SCHEMA = {
        "title": "Gps", "type": "object",
        "properties": {
            "timestamp_unix": {"type": "number"},
            "fix": {"type": "string"},
            "latitude": {"type": ["number", "null"]},
            "longitude": {"type": ["number", "null"]},
            "altitude_m": {"type": ["number", "null"]},
            "speed_mps": {"type": ["number", "null"]},
            "sats_used": {"type": "integer"},
        },
    }
    VERDICT_SCHEMA = {
        "title": "Verdict", "type": "object",
        "properties": {
            "verdict": {"type": "string"},
            "peak_accel_g": {"type": "number"},
            "peak_jerk_gps": {"type": "number"},
            "peak_gyro_dps": {"type": "number"},
            "peak_roll_deg": {"type": "number"},
        },
    }

    def __init__(self, path):
        from mcap.writer import Writer
        self._file = open(path, "wb")
        self._writer = Writer(self._file)
        self._writer.start(profile="", library="segway-classifier-2.0")

        def add_channel(name, schema, topic):
            sid = self._writer.register_schema(
                name=name, encoding="jsonschema",
                data=json.dumps(schema).encode())
            return self._writer.register_channel(
                schema_id=sid, topic=topic, message_encoding="json")

        self._imu_ch  = add_channel("ImuSample", self.IMU_SCHEMA, "/imu/data")
        self._baro_ch = add_channel("Baro", self.BARO_SCHEMA, "/baro")
        self._gps_ch  = add_channel("Gps", self.GPS_SCHEMA, "/gps")
        self._v_ch    = add_channel("Verdict", self.VERDICT_SCHEMA, "/segway/verdict")
        self._seq = 0

    def _write(self, ch, t, msg):
        ts = int(t * 1e9)
        self._writer.add_message(
            channel_id=ch, log_time=ts, publish_time=ts,
            data=json.dumps(msg).encode(), sequence=self._seq)
        self._seq += 1

    def write_imu(self, t, ax, ay, az, gx, gy, gz,
                  accel_mag, gyro_mag, roll, jerk_gps, live_state):
        self._write(self._imu_ch, t, {
            "timestamp_unix": t,
            "linear_acceleration": {
                "x": ax * G_TO_MS2, "y": ay * G_TO_MS2, "z": az * G_TO_MS2},
            "angular_velocity": {
                "x": gx * DPS_TO_RADS, "y": gy * DPS_TO_RADS, "z": gz * DPS_TO_RADS},
            "accel_mag_g": accel_mag, "gyro_mag_dps": gyro_mag,
            "roll_deg": roll, "jerk_gps": jerk_gps, "live_state": live_state,
        })

    def write_baro(self, t, temp, press, alt):
        self._write(self._baro_ch, t, {
            "timestamp_unix": t, "temperature_c": temp,
            "pressure_hpa": press, "altitude_m": alt,
        })

    def write_gps(self, t, fix, lat, lon, alt, speed, sats):
        self._write(self._gps_ch, t, {
            "timestamp_unix": t, "fix": fix,
            "latitude": lat, "longitude": lon,
            "altitude_m": alt, "speed_mps": speed,
            "sats_used": sats,
        })

    def write_verdict(self, t, verdict, p_a, p_j, p_g, p_r):
        self._write(self._v_ch, t, {
            "verdict": verdict, "peak_accel_g": p_a, "peak_jerk_gps": p_j,
            "peak_gyro_dps": p_g, "peak_roll_deg": p_r})

    def close(self):
        self._writer.finish()
        self._file.close()


class IdleDetector:
    def __init__(self, timeout_sec, accel_band=IDLE_ACCEL_BAND,
                 gyro_threshold=IDLE_GYRO_DPS):
        self.timeout = timeout_sec
        self.accel_band = accel_band
        self.gyro_thresh = gyro_threshold
        self._last_move = time.time()
        self._is_idle = False

    def update(self, accel_mag, gyro_mag):
        moving = (abs(accel_mag - 1.0) > self.accel_band or
                  gyro_mag > self.gyro_thresh)
        if moving:
            if self._is_idle:
                self._is_idle = False
                print("\n[IDLE] Movement detected - resuming.")
            self._last_move = time.time()
        else:
            if not self._is_idle and time.time() - self._last_move > self.timeout:
                self._is_idle = True
                print(f"\n[IDLE] No movement for {self.timeout // 60} min - pausing.")
        return self._is_idle


class EventClassifier:
    def __init__(self, sample_dt):
        self.sample_dt = sample_dt
        self.prev_accel_mag = None
        self.in_event = False
        self.event_start = 0.0
        self.peak_accel = self.peak_jerk = 0.0
        self.peak_gyro = self.peak_roll = 0.0

    def step(self, ax, ay, az, gx, gy, gz):
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
        gyro_mag = math.sqrt(gx*gx + gy*gy + gz*gz)
        roll = abs(math.degrees(math.atan2(ay, az)))
        if roll > 90:
            roll = 180 - roll
        jerk_gps = 0.0
        if self.prev_accel_mag is not None:
            jerk_gps = abs(accel_mag - self.prev_accel_mag) / self.sample_dt
        self.prev_accel_mag = accel_mag

        m = {"accel_mag": accel_mag, "gyro_mag": gyro_mag,
             "roll": roll, "jerk_gps": jerk_gps}
        now = time.time()

        if self.in_event:
            self.peak_accel = max(self.peak_accel, accel_mag)
            self.peak_jerk = max(self.peak_jerk, jerk_gps)
            self.peak_gyro = max(self.peak_gyro, gyro_mag)
            self.peak_roll = max(self.peak_roll, roll)
            if now - self.event_start < EVENT_WINDOW_SEC:
                return "WATCHING", None, m
            v = self._classify_window(end_roll=roll)
            peaks = (self.peak_accel, self.peak_jerk, self.peak_gyro, self.peak_roll)
            self._reset_event()
            return "GOOD", (v, peaks), m

        if (accel_mag > TRIGGER_ACCEL_G or jerk_gps > TRIGGER_JERK_GPS or
                gyro_mag > TRIGGER_GYRO_DPS or roll > TRIGGER_ROLL_DEG):
            self.in_event = True
            self.event_start = now
            self.peak_accel = accel_mag
            self.peak_jerk = jerk_gps
            self.peak_gyro = gyro_mag
            self.peak_roll = roll
            return "WATCHING", None, m

        return "GOOD", None, m

    def _classify_window(self, end_roll):
        if end_roll > FALL_ROLL_DEG:
            return "FALL"
        if self.peak_accel > CRASH_PEAK_ACCEL_G and self.peak_jerk > CRASH_PEAK_JERK_GPS:
            return "CRASH"
        if self.peak_accel > HARD_STOP_PEAK_G and self.peak_jerk > HARD_STOP_PEAK_JERK:
            return "HARD_STOP"
        return "NORMAL"

    def _reset_event(self):
        self.in_event = False
        self.peak_accel = self.peak_jerk = 0.0
        self.peak_gyro = self.peak_roll = 0.0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hz", type=int, default=100)
    p.add_argument("--idle-min", type=float, default=5.0)
    p.add_argument("--no-csv", action="store_true")
    p.add_argument("--no-mcap", action="store_true")
    p.add_argument("--no-sound", action="store_true")
    p.add_argument("--no-baro", action="store_true")
    p.add_argument("--no-gps", action="store_true")
    args = p.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = os.path.join(OUTPUT_DIR, f"ride_{stamp}.csv")
    mcap_path = os.path.join(OUTPUT_DIR, f"ride_{stamp}.mcap")

    if not args.no_sound:
        if not os.path.exists(BEEP_PATH):
            print("Generating crash alert tone...")
            make_beep_wav(BEEP_PATH)
        print("Crash alert sound enabled.")

    IMU.detectIMU()
    if IMU.BerryIMUversion == 99:
        raise SystemExit("No BerryIMU detected.")
    IMU.initIMU()
    print(f"BerryIMU v{IMU.BerryIMUversion} initialised.")

    bmp = None
    if not args.no_baro:
        try:
            bmp = BMP388.BMP388()
            print("BMP388 barometer initialised.")
        except Exception as e:
            print(f"[BARO] Init failed: {e}. Continuing without baro.")
            bmp = None

    gps_reader = None
    if not args.no_gps:
        gps_reader = GpsReader()
        if not gps_reader.start():
            gps_reader = None

    print(f"Target rate : {args.hz} Hz")
    print(f"Idle timeout: {args.idle_min} min")

    log_file = log_writer = None
    if not args.no_csv:
        log_file = open(csv_path, "w", newline="")
        log_writer = csv.writer(log_file)
        log_writer.writerow([
            "time", "ax", "ay", "az", "gx", "gy", "gz",
            "accel_mag", "gyro_mag", "roll", "jerk_gps",
            "temp_c", "pressure_hpa", "altitude_m",
            "gps_fix", "lat", "lon", "gps_alt", "speed_mps", "sats",
            "live_state", "verdict"])
        print(f"Writing CSV  -> {csv_path}")

    mcap_writer = None
    if not args.no_mcap:
        mcap_writer = McapWriter(mcap_path)
        print(f"Writing MCAP -> {mcap_path}")

    classifier = EventClassifier(sample_dt=1.0 / args.hz)
    idle_detect = IdleDetector(timeout_sec=int(args.idle_min * 60))

    period = 1.0 / args.hz
    sc = 0
    t_start = time.time()
    counts = {"NORMAL": 0, "HARD_STOP": 0, "CRASH": 0, "FALL": 0}

    last_baro_t = 0.0
    last_gps_t = 0.0
    baro_period = 0.1
    gps_period = 1.0
    cached_baro = (None, None, None)
    cached_gps = None

    print(f"\n{'STATE':<12} {'|a|(g)':>7} {'|w|(dps)':>9} "
          f"{'roll':>6} {'jerk(g/s)':>10} {'recording':>10}")
    print("-" * 60)

    try:
        while True:
            t0 = time.time()
            ax, ay, az, gx, gy, gz = read_imu()
            live_state, evt, m = classifier.step(ax, ay, az, gx, gy, gz)

            is_idle = idle_detect.update(m["accel_mag"], m["gyro_mag"])
            recording = not is_idle

            if bmp is not None and t0 - last_baro_t >= baro_period:
                try:
                    cached_baro = bmp.read_all()
                except Exception:
                    cached_baro = (None, None, None)
                last_baro_t = t0
                if recording and mcap_writer and cached_baro[0] is not None:
                    mcap_writer.write_baro(t0, *cached_baro)

            if gps_reader is not None and t0 - last_gps_t >= gps_period:
                cached_gps = gps_reader.latest()
                last_gps_t = t0
                if recording and mcap_writer and cached_gps:
                    mcap_writer.write_gps(t0,
                        cached_gps["fix"], cached_gps["lat"], cached_gps["lon"],
                        cached_gps["alt_m"], cached_gps["speed_mps"],
                        cached_gps["sats_used"])

            verdict = None
            if evt is not None and recording:
                verdict, peaks = evt
                pa, pj, pg, pr = peaks
                counts[verdict] += 1
                print(f">>> EVENT: {verdict}  "
                      f"(peak |a|={pa:.2f}g, peak jerk={pj:.1f} g/s) <<<")
                if verdict in ("CRASH", "FALL") and not args.no_sound:
                    play_beep_async()

            if recording:
                if mcap_writer is not None:
                    mcap_writer.write_imu(t0, ax, ay, az, gx, gy, gz,
                        m["accel_mag"], m["gyro_mag"],
                        m["roll"], m["jerk_gps"], live_state)
                    if verdict is not None:
                        mcap_writer.write_verdict(t0, verdict, pa, pj, pg, pr)

                if log_writer is not None:
                    bt, bp, ba = cached_baro
                    g = cached_gps or {"fix": "-", "lat": None, "lon": None,
                                       "alt_m": None, "speed_mps": None, "sats_used": 0}
                    log_writer.writerow([
                        t0, ax, ay, az, gx, gy, gz,
                        m["accel_mag"], m["gyro_mag"], m["roll"], m["jerk_gps"],
                        bt or "", bp or "", ba or "",
                        g["fix"], g["lat"] or "", g["lon"] or "",
                        g["alt_m"] or "", g["speed_mps"] or "", g["sats_used"],
                        live_state, verdict or ""])

            if sc % max(1, args.hz // 10) == 0:
                rec_label = "RECORDING" if recording else "PAUSED"
                print(f"{live_state:<12} {m['accel_mag']:>7.2f} "
                      f"{m['gyro_mag']:>9.1f} {m['roll']:>6.1f} "
                      f"{m['jerk_gps']:>10.1f} {rec_label:>10}")

            sc += 1
            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)
    except KeyboardInterrupt:
        total = time.time() - t_start
        rate = sc / total if total > 0 else 0
        print(f"\nStopped. {sc} samples in {total:.1f}s (actual {rate:.1f} Hz).")
        print("Verdicts:")
        for v, c in counts.items():
            print(f"  {v:<10} {c}")
    finally:
        if gps_reader is not None:
            gps_reader.stop()
        if log_file is not None:
            log_file.close()
            print(f"CSV saved : {csv_path}")
        if mcap_writer is not None:
            mcap_writer.close()
            print(f"MCAP saved: {mcap_path}")


if __name__ == "__main__":
    main()