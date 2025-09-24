#!/usr/bin/env python3
import os, cv2, math, time, serial, threading, requests
import numpy as np
from collections import deque

# ====================== CONFIG ======================
# Inference (Roboflow Hosted API)
USE_ROBOFLOW_CLOUD = True
ROBOFLOW_API_KEY   = os.getenv("ROBOFLOW_API_KEY", "YOUR_API_KEY")
ROBOFLOW_MODEL     = os.getenv("ROBOFLOW_MODEL",   "workspace/model")
ROBOFLOW_VERSION   = int(os.getenv("ROBOFLOW_VERSION", "1"))
CONF_THRESH        = float(os.getenv("CONF_THRESH", "0.45"))

# Camera (fixed nadir)
CAM_INDEX     = 0
FRAME_WIDTH   = 1280
FRAME_HEIGHT  = 720
H_FOV_DEG     = 62.0   # set to your lens’ measured FOV
V_FOV_DEG     = 48.8

# Telemetry (MAVLink/DroneKit)
USE_DRONEKIT = True
MAVLINK_CONNECTION = os.getenv("MAVLINK", "udp:127.0.0.1:14550")
MAX_TILT_DEG = 7.0        # skip frames if |roll| or |pitch| exceeds this

# HC-12 radio (TX only, rover listens at 9600)
USE_HC12 = True
HC12_PORT = os.getenv("HC12_PORT", "/dev/ttyAMA0")  # or /dev/ttyUSB0
HC12_BAUD = 9600
HC12_TX_INTERVAL_SEC = 0.12

# Auto-transmit when near home/rover
AUTO_TX_WHEN_NEAR_HOME = True
HOME_DETECT_METHOD = "first_fix"   # "first_fix" or "dronekit_home"
HOME_RADIUS_M = 20.0
ALT_LAND_THRESH_M = 5.0

# Detection storage / dedup
MIN_SEP_M = 3.0
MERGE_RADIUS_M = 6.0
MIN_BOX_AREA_PCT = 0.002
SMOOTHING_SEC = 0.8

# Debug / logs
SAVE_CSV = True
CSV_PATH = os.getenv("CSV_PATH", "stagnant_hits.csv")

# ====================== TELEMETRY ======================
class Telemetry:
    def __init__(self):
        self.lat = None
        self.lon = None
        self.alt_agl_m = None
        self.yaw_deg = 0.0
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.home_lat = None
        self.home_lon = None

tm = Telemetry()

def connect_dronekit():
    from dronekit import connect
    v = connect(MAVLINK_CONNECTION, wait_ready=True, timeout=60)
    return v

def _set_home_if_needed(vehicle):
    if HOME_DETECT_METHOD == "dronekit_home" and vehicle.home_location:
        tm.home_lat = float(vehicle.home_location.lat)
        tm.home_lon = float(vehicle.home_location.lon)

def telemetry_thread(vehicle):
    first_fix_set_home = False
    # simple yaw low-pass to reduce jitter
    yaw_alpha = 0.25
    yaw_smoothed = None
    while True:
        try:
            loc_rel = vehicle.location.global_relative_frame
            att = vehicle.attitude
            hdg = getattr(vehicle, "heading", None)

            if loc_rel and loc_rel.lat is not None and loc_rel.lon is not None:
                tm.lat = float(loc_rel.lat)
                tm.lon = float(loc_rel.lon)
                tm.alt_agl_m = None if loc_rel.alt is None else float(loc_rel.alt)
                if HOME_DETECT_METHOD == "first_fix" and not first_fix_set_home:
                    tm.home_lat, tm.home_lon = tm.lat, tm.lon
                    first_fix_set_home = True

            # roll/pitch from radians -> degrees
            if att:
                tm.roll_deg  = math.degrees(att.roll or 0.0)
                tm.pitch_deg = math.degrees(att.pitch or 0.0)

            # prefer compass heading if available
            if hdg is not None:
                new_yaw = float(hdg)
            else:
                new_yaw = (math.degrees(att.yaw) + 360.0) % 360.0 if att and att.yaw is not None else tm.yaw_deg

            if yaw_smoothed is None:
                yaw_smoothed = new_yaw
            # unwrap heading to avoid 359->0 jump
            dy = ((new_yaw - yaw_smoothed + 540) % 360) - 180
            yaw_smoothed = (yaw_smoothed + yaw_alpha * dy) % 360
            tm.yaw_deg = yaw_smoothed

            if HOME_DETECT_METHOD == "dronekit_home" and (tm.home_lat is None or tm.home_lon is None):
                _set_home_if_needed(vehicle)

        except Exception:
            pass
        time.sleep(0.05)

# ====================== GEODESY ======================
def haversine_m(lat1, lon1, lat2, lon2):
    R = 6378137.0
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat/2)**2 + math.cos(math.radians(lat1))*math.cos(math.radians(lat2))*math.sin(dLon/2)**2
    return 2*R*math.atan2(math.sqrt(a), math.sqrt(1-a))

def add_ne_offset_to_latlon(lat, lon, dn, de):
    if lat is None or lon is None:
        return None, None
    R = 6378137.0
    dlat = dn / R
    dlon = de / (R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)

def pixels_to_ground_offset(cx_px, cy_px, img_w, img_h, alt_m, yaw_deg, h_fov_deg, v_fov_deg):
    """Strict nadir projection; requires small roll/pitch (enforced by MAX_TILT_DEG)."""
    if alt_m is None or alt_m <= 0:
        return 0.0, 0.0
    h_fov = math.radians(h_fov_deg); v_fov = math.radians(v_fov_deg)
    width_ground  = 2.0 * alt_m * math.tan(h_fov/2.0)
    height_ground = 2.0 * alt_m * math.tan(v_fov/2.0)
    m_per_px_x = width_ground / img_w
    m_per_px_y = height_ground / img_h

    dx_px = (cx_px - img_w/2.0)
    dy_px = (cy_px - img_h/2.0)

    # camera frame: +x right, +y up (invert image y)
    x_cam = dx_px * m_per_px_x
    y_cam = -dy_px * m_per_px_y

    yaw = math.radians(yaw_deg)
    de = x_cam * math.sin(yaw) + y_cam * math.cos(yaw)
    dn = x_cam * math.cos(yaw) - y_cam * math.sin(yaw)
    return dn, de

# ====================== INFERENCE ======================
def roboflow_infer(image_bgr):
    _, jpg = cv2.imencode(".jpg", image_bgr)
    r = requests.post(
        f"https://detect.roboflow.com/{ROBOFLOW_MODEL}/{ROBOFLOW_VERSION}",
        params={"api_key": ROBOFLOW_API_KEY, "confidence": CONF_THRESH},
        files={"file": ("frame.jpg", jpg.tobytes(), "image/jpeg")},
        timeout=10,
    )
    r.raise_for_status()
    dets = []
    for p in r.json().get("predictions", []):
        dets.append({
            "cx": p["x"], "cy": p["y"], "w": p["width"], "h": p["height"],
            "conf": float(p.get("confidence", 0.0)),
            "cls": p.get("class", "stagnant_water")
        })
    return dets

# ====================== DEDUP / STORAGE ======================
class HitStore:
    def __init__(self, min_sep_m=MIN_SEP_M, merge_radius_m=MERGE_RADIUS_M):
        self.points = []
        self.min_sep = min_sep_m
        self.merge_r = merge_radius_m

    def add(self, lat, lon, conf, ts=None):
        if lat is None or lon is None: return
        if ts is None: ts = time.time()
        # merge near
        for p in self.points:
            if haversine_m(p["lat"], p["lon"], lat, lon) <= self.merge_r:
                n = p["count"]
                p["lat"] = (p["lat"]*n + lat) / (n+1)
                p["lon"] = (p["lon"]*n + lon) / (n+1)
                p["count"] += 1
                p["conf_max"] = max(p["conf_max"], conf)
                p["last_ts"] = ts
                return
        # drop too-close duplicates
        for p in self.points:
            if haversine_m(p["lat"], p["lon"], lat, lon) < self.min_sep:
                return
        self.points.append({"lat": lat, "lon": lon, "count": 1, "conf_max": conf, "last_ts": ts})

    def as_sorted(self):
        return sorted(self.points, key=lambda p: (p["conf_max"], p["last_ts"]), reverse=True)

    def __len__(self):
        return len(self.points)

# ====================== RADIO TX ======================
class HC12TX:
    def __init__(self, port, baud):
        self.port = port
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()

    def open(self):
        if self.ser: return
        import serial
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(0.2)

    def close(self):
        if self.ser:
            try: self.ser.close()
            except: pass
        self.ser = None

    def send_geo(self, lat, lon):
        line = f"GEO,{lat:.6f},{lon:.6f}\n"
        with self.lock:
            self.ser.write(line.encode("ascii"))
            self.ser.flush()

# ====================== MAIN LOOP ======================
def main():
    # DroneKit / telemetry
    vehicle = None
    if USE_DRONEKIT:
        try:
            vehicle = connect_dronekit()
            _set_home_if_needed(vehicle)
            threading.Thread(target=telemetry_thread, args=(vehicle,), daemon=True).start()
            print("[DRONE] DroneKit connected.")
        except Exception as e:
            print("[DRONE] DroneKit connect failed:", e)

    # Camera
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 15)

    # HC-12
    radio = None
    if USE_HC12:
        try:
            radio = HC12TX(HC12_PORT, HC12_BAUD)
            radio.open()
            print("[DRONE] HC-12 opened:", HC12_PORT)
        except Exception as e:
            print("[DRONE] HC-12 open failed:", e)
            radio = None

    hits = HitStore()
    area_min = MIN_BOX_AREA_PCT * FRAME_WIDTH * FRAME_HEIGHT
    smooth = deque()
    last_tx = 0

    print("[DRONE] Scanning with fixed-nadir camera... 'T' to transmit, ESC to quit.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.03); continue

            # ENFORCE NADIR ASSUMPTION: skip tilted frames
            if (abs(tm.roll_deg) > MAX_TILT_DEG) or (abs(tm.pitch_deg) > MAX_TILT_DEG):
                # optional: show HUD but skip inference
                hud = f"TILT! roll:{tm.roll_deg:>5.1f} pitch:{tm.pitch_deg:>5.1f}  (skip)"
                cv2.putText(frame, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,255), 2)
                cv2.imshow("stagnant_scan", frame)
                if (cv2.waitKey(1) & 0xFF) == 27: break
                continue

            # inference
            try:
                dets = roboflow_infer(frame) if USE_ROBOFLOW_CLOUD else []
            except Exception as e:
                print("[DRONE] Inference error:", e)
                dets = []

            dets = [d for d in dets if (d["w"]*d["h"]) >= area_min]

            ts = time.time()
            outs = []
            for d in dets:
                dn, de = pixels_to_ground_offset(
                    d["cx"], d["cy"], FRAME_WIDTH, FRAME_HEIGHT,
                    tm.alt_agl_m, tm.yaw_deg, H_FOV_DEG, V_FOV_DEG
                )
                lat, lon = add_ne_offset_to_latlon(tm.lat, tm.lon, dn, de)
                if lat is None or lon is None: continue
                outs.append({"lat": lat, "lon": lon, "conf": d["conf"]})
                hits.add(lat, lon, d["conf"], ts=ts)

            smooth.append((ts, outs))
            while smooth and (ts - smooth[0][0] > SMOOTHING_SEC):
                smooth.pop()  # we don't use smoothing much; keep last only

            # Draw detections and HUD
            for d in dets:
                x1 = int(d["cx"] - d["w"]/2); y1 = int(d["cy"] - d["h"]/2)
                x2 = int(d["cx"] + d["w"]/2); y2 = int(d["cy"] + d["h"]/2)
                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(frame, f"{d['cls']} {d['conf']:.2f}", (x1, max(0,y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

            hud = f"Fix:{'Y' if (tm.lat is not None) else 'N'} Alt:{tm.alt_agl_m if tm.alt_agl_m is not None else '-'}m "\
                  f"Yaw:{tm.yaw_deg:>5.1f} R:{tm.roll_deg:>5.1f} P:{tm.pitch_deg:>5.1f} Hits:{len(hits)}"
            cv2.putText(frame, hud, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,255), 2)
            cv2.imshow("stagnant_scan", frame)

            # Auto-transmit when near home / low altitude
            should_tx = False
            if AUTO_TX_WHEN_NEAR_HOME and tm.home_lat and tm.lat:
                dist_home = haversine_m(tm.lat, tm.lon, tm.home_lat, tm.home_lon)
                alt_ok = (tm.alt_agl_m is not None and tm.alt_agl_m <= ALT_LAND_THRESH_M)
                if dist_home <= HOME_RADIUS_M or alt_ok:
                    should_tx = True

            k = cv2.waitKey(1) & 0xFF
            if k in (ord('t'), ord('T')):
                should_tx = True
            if k == 27:
                break

            if should_tx and radio and (time.time() - last_tx > 3.0) and len(hits) > 0:
                pts = hits.as_sorted()
                print(f"[DRONE] Transmitting {len(pts)} GEO targets...")
                # Optional CSV log
                if SAVE_CSV:
                    try:
                        import csv, os
                        write_header = not os.path.exists(CSV_PATH)
                        with open(CSV_PATH, "a", newline="") as f:
                            w = csv.writer(f)
                            if write_header:
                                w.writerow(["timestamp","lat","lon","count","conf_max"])
                            for p in pts:
                                w.writerow([int(time.time()), f"{p['lat']:.7f}", f"{p['lon']:.7f}", p["count"], f"{p['conf_max']:.3f}"])
                    except Exception as e:
                        print("[DRONE] CSV write error:", e)
                try:
                    for p in pts:
                        radio.send_geo(p["lat"], p["lon"])
                        time.sleep(HC12_TX_INTERVAL_SEC)
                    print("[DRONE] Transmission complete.")
                except Exception as e:
                    print("[DRONE] HC-12 TX error:", e)
                last_tx = time.time()

    finally:
        cap.release(); cv2.destroyAllWindows()
        try:
            if vehicle: vehicle.close()
        except: pass
        try:
            if radio: radio.close()
        except: pass

if __name__ == "__main__":
    main()
