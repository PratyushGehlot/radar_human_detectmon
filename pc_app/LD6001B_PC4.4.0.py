# ====================================================================
# RadarParser   --> reads serial → gives frames
# HumanRenderer --> draws grids, boxes, overlays
# Track         -->
# HumanLogic    -->
# RadarApp      --> tracking + clustering + posture + fall + update()
# ====================================================================

import sys, serial, re, numpy as np, time, json
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from sklearn.cluster import DBSCAN
from scipy.optimize import linear_sum_assignment

# ================= MACROS =================
GRID_X = 6.0
GRID_Y = 6.0
GRID_Z = 3.0
GRID_STEP = 0.5
SERIAL_PORT = "COM5"
BAUDRATE = 115200

# ================= Detection Settings =================
EPS = 0.55
MIN_SAMPLES = 5
HUMAN_CONF_THRESHOLD = 0.3
CONF_POINT_MIN = 0.4

# Posture thresholds (will be calibrated)
STANDING_Z = 1.8
SITTING_Z = 1.1
LYING_Z   = 0.5

# ================= Tracking Settings =================
DT = 0.03
MAX_TRACKS = 3

PROCESS_NOISE_POS = 0.04
PROCESS_NOISE_VEL = 0.15

MEAS_NOISE_POS = 0.04
MEAS_NOISE_Z =  0.08#0.12 # smaller → faster response

GATE_THRESH = 6.0#2.5
DEATH_MISSES = int(1.2/DT)
SMOOTH_LAG = 5

fall_detected = False
fall_time = None

# ================= Confidence =================
#   Term	Meaning	            Weight
#   SNR	    RF quality	        35%
#   ABS	    target reflectivity	45%
#   DPK	    detection peak	    20%
# ==============================================
def point_confidence(snr, abs_val, dpk):
    snr = max(snr, 0)
    abs_val = max(abs_val, 0)
    dpk = max(dpk, 0)

    snr_c = min(snr / 40.0, 1.0)
    abs_c = min(abs_val / 15.0, 1.0)
    dpk_c = min(dpk / 10.0, 1.0)

    return 0.35 * snr_c + 0.45 * abs_c + 0.20 * dpk_c


# ================= RadarParser → UART point cloud decode =================
class RadarParser:
    def __init__(self, ser):
        self.ser = ser
        self.frame_points = []
        self.z_max = 0.0
        self.point_re = re.compile(
            r"x=([-0-9.]+),y=([-0-9.]+),z=([-0-9.]+),v=([-0-9.]+),"
            r"snr=([-0-9.]+),abs=([-0-9.]+),dpk=([-0-9.]+)"
        )

    def f(self, x):
        try: return float(x)
        except: return 0.0

    # def parse_point(self, line):
    #     m = self.point_re.search(line)
    #     if not m:
    #         return None
    #
    #     x = self.f(m.group(1))
    #     y = self.f(m.group(2))
    #     z = self.f(m.group(3))
    #     v = self.f(m.group(4))
    #     snr = self.f(m.group(5))
    #     abs_val = self.f(m.group(6))
    #     dpk = self.f(m.group(7))
    #
    #     x = x + GRID_X/2
    #     y = y + GRID_Y/2
    #     z = GRID_Z - z
    #
    #     return [x, y, z, snr, abs_val, dpk, v]

    def update(self):
        frame_done = False
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="ignore").strip()

            if "-----PointNum" in line:
                frame_done = True
                break

            m = self.point_re.search(line)
            if not m:
                continue

            x=self.f(m.group(1)); y=self.f(m.group(2))
            z=self.f(m.group(3)); v=self.f(m.group(4))
            snr=self.f(m.group(5)); abs_val=self.f(m.group(6))
            dpk=self.f(m.group(7))

            x += GRID_X/2
            y += GRID_Y/2
            z = GRID_Z - z

            # 🔥 ADD CONFIDENCE HERE
            conf = point_confidence(snr, abs_val, dpk)

            # Append as a row: x, y, z, snr, abs, dpk, v, conf
            self.frame_points.append([x, y, z, snr, abs_val, dpk, v, conf])

        return frame_done

    def get_frame(self):
        if not self.frame_points:
            return None, 0.0
        pts = np.array(self.frame_points, dtype=float)
        if pts.ndim == 1:
            pts = pts.reshape(1, -1)

        z_max = np.max(pts[:, 2]) if pts.shape[0] > 0 else 0.0
        self.frame_points = []  # clear for next frame
        self.z_max = z_max
        return pts, z_max

    def close(self):
        if self.ser:
            self.ser.close()


# ================= HumanRenderer =================
class HumanRenderer:
    def __init__(self, view, parent_widget=None):
        self.view = view
        self.parent_widget = parent_widget
        self.human_boxes = []
        self.human_labels = []
        self.info_items = []
        # default fonts
        self.font_big = QtGui.QFont('Helvetica', 18, QtGui.QFont.Bold)
        self.font_label = QtGui.QFont('Helvetica', 10)

    # ----------------- GRID -----------------
    def add_corner_grids(self, size=GRID_X, height=GRID_Z, step=GRID_STEP, color=(0.2,0.2,0.2,0.6), width=1):
        coords_x = np.arange(0, size+1e-6, step)
        coords_y = np.arange(0, GRID_Y+1e-6, step)
        coords_z = np.arange(0, height+1e-6, step)
        for x in coords_x:
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[x,0,0],[x,GRID_Y,0]]), color=color, width=width))
        for y in coords_y:
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,y,0],[size,y,0]]), color=color, width=width))
        for x in coords_x:
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[x,0,0],[x,0,height]]), color=color, width=width))
        for y in coords_y:
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,y,0],[0,y,height]]), color=color, width=width))
        for z in coords_z:
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,z],[size,0,z]]), color=color, width=width))
            self.view.addItem(gl.GLLinePlotItem(pos=np.array([[0,0,z],[0,GRID_Y,z]]), color=color, width=width))

    # ----------------- AXIS LABELS -----------------
    def add_3d_axis_labels(self, size=GRID_X, height=GRID_Z):
        axis_font = QtGui.QFont('Helvetica', 14)
        tick_font = QtGui.QFont('Helvetica', 11)

        # axis labels
        for text,pos in [("X (m)", (size+0.5,0,0)),
                         ("Y (m)", (0,GRID_Y+0.5,0)),
                         ("Z (m)", (0,0,height+0.5))]:
            self.view.addItem(gl.GLTextItem(pos=pos, text=text, color=(255,255,255,255), font=axis_font))

        # tick numbers
        for i in range(int(size)+1):
            self.view.addItem(gl.GLTextItem(pos=(i,0,0), text=f"{i}", color=(200,200,200,255), font=tick_font))
        for i in range(int(GRID_Y)+1):
            self.view.addItem(gl.GLTextItem(pos=(0,i,0), text=f"{i}", color=(200,200,200,255), font=tick_font))
        for i in range(int(height)+1):
            self.view.addItem(gl.GLTextItem(pos=(0,0,i), text=f"{i}", color=(200,200,200,255), font=tick_font))

    # ----------------- TRIAD -----------------
    def draw_world_triad(self, origin=None):
        if origin is None:
            origin = np.array([GRID_X/2, GRID_Y/2, GRID_Z])
        colors = {'x':(1,0,0,0.5),'y':(0,1,0,0.5),'z':(0,0,1,0.5)}
        axes = {'x':np.array([1,0,0]),'y':np.array([0,1,0]),'z':np.array([0,0,-1])}
        for k in axes:
            seg = np.array([origin, origin + axes[k]*1.0])
            self.view.addItem(gl.GLLinePlotItem(pos=seg, color=colors[k], width=3))

    # ----------------- LEGEND -----------------
    def add_color_legend(self):
        if self.parent_widget is None:
            return
        legend = QtWidgets.QLabel("Legend: Red=SNR  Green=Abs  Blue=Dpk  | Human tracks labeled")
        legend.setStyleSheet("color: white; font-size: 14px; background-color: rgba(0,0,0,150); padding: 4px;")
        legend.setAlignment(QtCore.Qt.AlignCenter)
        layout = QtWidgets.QVBoxLayout(self.parent_widget)
        layout.addWidget(legend, alignment=QtCore.Qt.AlignBottom | QtCore.Qt.AlignCenter)
        self.parent_widget.setLayout(layout)

    # ----------------- BOXES -----------------
    def draw_3d_box(self, center, size=(0.5,0.5,1.0), color=(0,1,1,1), width=2):
        cx, cy, cz = center
        sx, sy, sz = size
        corners = np.array([
            [cx-sx/2, cy-sy/2, cz-sz/2],
            [cx+sx/2, cy-sy/2, cz-sz/2],
            [cx+sx/2, cy+sy/2, cz-sz/2],
            [cx-sx/2, cy+sy/2, cz-sz/2],
            [cx-sx/2, cy-sy/2, cz+sz/2],
            [cx+sx/2, cy-sy/2, cz+sz/2],
            [cx+sx/2, cy+sy/2, cz+sz/2],
            [cx-sx/2, cy+sy/2, cz+sz/2],
        ])
        edges = [(0,1),(1,2),(2,3),(3,0),
                 (4,5),(5,6),(6,7),(7,4),
                 (0,4),(1,5),(2,6),(3,7)]
        items=[]
        for i,j in edges:
            seg = np.array([corners[i], corners[j]])
            item = gl.GLLinePlotItem(pos=seg, color=color, width=width)
            self.view.addItem(item)
            items.append(item)
        self.human_boxes.extend(items)
        return items

    # ----------------- LABELS -----------------
    def add_label(self, pos, text, color=(255,255,0,255), font=None):
        font = font or self.font_label
        text_item = gl.GLTextItem(pos=pos, text=text, color=color, font=font)
        self.view.addItem(text_item)
        self.human_labels.append(text_item)
        return text_item

    # ----------------- OVERLAY -----------------
    def add_overlay_text(self, pos, text, color=(255,255,255,255), font=None):
        font = font or self.font_big
        text_item = gl.GLTextItem(pos=pos, text=text, color=color, font=font)
        self.view.addItem(text_item)
        self.info_items.append(text_item)
        return text_item

    # ----------------- CLEAR ALL -----------------
    def clear(self):
        for item in self.human_boxes + self.human_labels + self.info_items:
            self.view.removeItem(item)
        self.human_boxes.clear()
        self.human_labels.clear()
        self.info_items.clear()

    def add_overlay(self, *args, **kwargs):
        return self.add_overlay_text(*args, **kwargs)


# ================= Tracking =================
class Track:
    def __init__(self, tid, z0):
        self.x = np.zeros(6)
        self.P = np.eye(6)*0.5
        self.tid = tid
        self.misses = 0
        self.history = []
        self.avg_z = z0
        self.posture = "UNKNOWN"
        self.score = 0.0
        self.last_update_ts = time.time()
        self.fall_start_time = None
        self.fall_confirmed = False
        self.age = 0

    def F(self, dt):
        F = np.eye(6); F[0,3]=dt; F[1,4]=dt; F[2,5]=dt; return F
    def Q(self, dt): return np.diag([PROCESS_NOISE_POS]*3 + [PROCESS_NOISE_VEL]*3) * dt
    def H(self): H=np.zeros((3,6)); H[0,0]=1; H[1,1]=1; H[2,2]=1; return H
    def R(self): return np.diag([MEAS_NOISE_POS, MEAS_NOISE_POS, MEAS_NOISE_Z])

    def predict(self, dt):
        F = self.F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q(dt)
        self.history.append(self.x.copy())
        if len(self.history) > 50: self.history.pop(0)
        self.age += 1

    def update(self, z):
        H = self.H(); R = self.R()
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P
        self.misses = 0
        #self.avg_z = 0.8*self.avg_z + 0.2*z[2]
        self.avg_z = 0.9 * self.avg_z + 0.1 * z[2]
        self.last_update_ts = time.time()

    def gate_cost(self, z):
        H = self.H();
        R = self.R()
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            val = float(y.T @ np.linalg.inv(S) @ y)
            if not np.isfinite(val):
                return 1e6
            return val
        except:
            return 1e6

    def smoothed_state(self, lag=SMOOTH_LAG):
        if len(self.history) == 0:
            return self.x.copy()
        if len(self.history) < lag:
            return np.mean(self.history, axis=0)
        w = np.linspace(0.2, 1.0, len(self.history[-lag:]))
        w /= w.sum()
        return np.average(self.history[-lag:], axis=0, weights=w)


# ================= HumanLogic =================
class HumanLogic:
    def __init__(self):
        self.calibration = None

    def merge_clusters(self,detections, horiz_thresh=0.4, vert_thresh=1.5):
        """
        Merge fragmented clusters that belong to the same human.
        Returns merged detections with centroid and full-body extents.
        """
        merged = []
        used = set()
        for i, d1 in enumerate(detections):
            if i in used: continue
            cx1, cy1, z1 = d1["centroid"]
            group = [d1]
            for j, d2 in enumerate(detections[i + 1:], i + 1):
                if j in used: continue
                cx2, cy2, z2 = d2["centroid"]
                dx, dy = abs(cx1 - cx2), abs(cy1 - cy2)
                dz = abs(z1 - z2)
                if dx < horiz_thresh and dy < horiz_thresh and dz < vert_thresh:
                    group.append(d2)
                    used.add(j)

            # merge group into one detection
            all_centroids = np.array([g["centroid"] for g in group])
            merged_centroid = np.mean(all_centroids, axis=0)

            # compute full-body extents
            all_z = [g["centroid"][2] for g in group]
            z_min, z_max = min(all_z), max(all_z)
            extent_z = max(z_max - z_min, 1.0)  # ensure at least 1m tall
            pts = np.array([g["centroid"] for g in group])
            extent_x = max(0.4, np.std(pts[:, 0]) * 3)
            extent_y = max(0.4, np.std(pts[:, 1]) * 3)

            merged.append({
                "centroid": merged_centroid,
                "extent": (extent_x, extent_y, extent_z),
                "z_min": z_min,
                "z_max": z_max
            })
        return merged

    def posture_from_track(self, trk, extent_z):
        z = trk.smoothed_state()[2]
        hist = np.array([s[2] for s in trk.history[-SMOOTH_LAG:]]) if len(trk.history) >= 2 else np.array([z])

        # --- FALL candidate detection ---
        if len(hist) >= 3:
            dz = hist[-1] - hist[0]
            dt = len(hist) * DT
            drop_rate = dz / dt
            if drop_rate < -0.25 and extent_z < 0.7 and z < 0.8:
                return "FALL"

        # --- Normal posture classification ---
        if extent_z > 1.2 and z > STANDING_Z * 0.9: return "STANDING"
        if 0.6 < extent_z < 1.1 and z > LYING_Z: return "SITTING"
        if z <= LYING_Z: return "LYING"
        return "UNKNOWN"

    def capture_posture(self, label, duration=10.0):
        print(f"Please {label} under the radar for {duration} seconds...")
        start = time.time()
        zs = []
        frame_points.clear()  # clear old points
        while time.time() - start < duration:
            if frame_points:
                pts = np.array(frame_points)
                xyz = pts[:, :3]
                confs = [point_confidence(s, a, d) for _, _, z, s, a, d, v, _, in frame_points]
                mask = np.array(confs) >= CONF_POINT_MIN
                if mask.sum() >= MIN_SAMPLES:
                    centroid_z = np.mean(xyz[mask, 2])
                    if time.time() - start >= 7.0:  # only last 3 seconds
                        zs.append(centroid_z)
            QtWidgets.QApplication.processEvents()
        return float(np.median(zs)) if zs else None

    def interactive_calibration(self):
        global STANDING_Z, SITTING_Z, LYING_Z
        try:
            with open("calibration.json", "r") as f:
                calib = json.load(f)
                STANDING_Z = calib["standing"]
                SITTING_Z = calib["sitting"]
                LYING_Z = calib["lying"]
                print(f"Loaded calibration: Standing={STANDING_Z:.2f}, Sitting={SITTING_Z:.2f}, Lying={LYING_Z:.2f}")
                use_saved = input("Use saved calibration? (y/n): ").strip().lower()
                if use_saved == "y":
                    return
        except:
            pass

        stand_z = capture_posture("STAND")
        sit_z = capture_posture("SIT")
        lie_z = capture_posture("LIE DOWN")

        if stand_z: STANDING_Z = stand_z
        if sit_z:   SITTING_Z = sit_z
        if lie_z:   LYING_Z = lie_z

        print(f"Calibration complete: Standing={STANDING_Z:.2f}, Sitting={SITTING_Z:.2f}, Lying={LYING_Z:.2f}")
        calib = {"standing": STANDING_Z, "sitting": SITTING_Z, "lying": LYING_Z}
        with open("calibration.json", "w") as f:
            json.dump(calib, f)


# ================= RadarApp → pipeline + calling =================
class RadarApp:
    def __init__(self, parser, renderer, logic):
        self.parser = parser
        self.renderer = renderer
        self.logic = logic
        self.tracks = []
        self.next_tid = 1
        self.fall_detected = False
        self.fall_time = None
        self.last_time = None
        self.presence_start = None

    def update(self):

        # ---------- SERIAL ----------
        if not self.parser.update():
            return

        pts, z_max = self.parser.get_frame()
        if pts is None:
            return

        # ---------- COMPUTE CONFIDENCE ----------
        snr = pts[:, 3]
        abs_val = pts[:, 4]
        dpk = pts[:, 5]
        confs = pts[:, 7]  # confidence
        mask = confs >= CONF_POINT_MIN
        # ---------- SCATTER ----------
        scatter.setData(pos=pts[:, :3], size=6)

        # ---------- CLEAR RENDER ----------
        self.renderer.clear()

        # ---------- CLUSTER ----------
        detections = []

        if mask.sum() >= MIN_SAMPLES:
            pts_masked = pts[mask]
            pts2d = pts_masked[:, :2]

            db = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(pts2d)

            for lbl in set(db.labels_):
                if lbl == -1:
                    continue

                cluster_points = pts_masked[db.labels_ == lbl]

                # Filter points by confidence again if needed (mostly redundant)
                cluster_points_conf = cluster_points[cluster_points[:, 7] >= CONF_POINT_MIN]
                if len(cluster_points_conf) < MIN_SAMPLES:
                    continue

                # calculate cluster centroid
                cx, cy = cluster_points[:, 0].mean(), cluster_points[:, 1].mean()
                avg_z = cluster_points_conf[:, 2].mean()

                # Full-body extents
                z_min, z_max = cluster_points_conf[:, 2].min(), cluster_points_conf[:, 2].max()
                extent_z = max(z_max - z_min, 1.0)
                extent_x = max(0.4, np.std(cluster_points_conf[:, 0]) * 3)
                extent_y = max(0.4, np.std(cluster_points_conf[:, 1]) * 3)

                detections.append({
                    "centroid": np.array([cx, cy, avg_z]),
                    "extent": (extent_x, extent_y, extent_z),
                    "z_min": z_min,
                    "z_max": z_max
                })

            detections = self.logic.merge_clusters(detections)

        # ---------- TRACK PREDICT ----------
        now = time.time()
        for t in self.tracks:
            dt = min(DT, now - t.last_update_ts)
            t.predict(dt)

            if t.age > 10 and t.posture == "FALL_CONFIRMED":
                print("Emergency confirmed human down")

        # ---------- ASSOC ----------
        if detections:
            C = np.zeros((len(self.tracks), len(detections)))

            for i, t in enumerate(self.tracks):
                for j, d in enumerate(detections):
                    dist = np.linalg.norm(t.x[:2] - d["centroid"][:2])
                    C[i, j] = t.gate_cost(d["centroid"]) + 0.3 * dist

            C = np.nan_to_num(C, nan=1e6, posinf=1e6, neginf=1e6)
            ri, ci = linear_sum_assignment(C)

            used_t = set()
            used_d = set()

            for r, c in zip(ri, ci):
                if C[r, c] <= GATE_THRESH:
                    self.tracks[r].update(detections[c]["centroid"])
                    used_t.add(r)
                    used_d.add(c)

            for j, d in enumerate(detections):
                if j not in used_d:
                    if len(self.tracks) < MAX_TRACKS:
                        t = Track(self.next_tid, d["centroid"][2])
                        t.x[:3] = d["centroid"]
                        self.tracks.append(t)
                        self.next_tid += 1

            self.tracks = [t for t in self.tracks if t.misses < DEATH_MISSES]
        else:
            for t in self.tracks:
                t.misses += 1
            self.tracks = [t for t in self.tracks if t.misses < DEATH_MISSES]

        # ---------- MERGE CLOSE TRACKS ----------
        merged = []
        for t in self.tracks:
            keep = True
            for m in merged:
                if np.linalg.norm(t.x[:2] - m.x[:2]) < 0.4:
                    keep = False
                    break
            if keep:
                merged.append(t)
        self.tracks = merged

        # ---------- DRAW ----------
        for t in self.tracks:
            s = t.smoothed_state()
            cx, cy, cz = s[0], s[1], s[2]

            extent_z = 1.0
            for d in detections:
                if np.linalg.norm(d["centroid"][:2] - [cx, cy]) < 0.3:
                    extent_z = d["extent"][2]

            posture = self.logic.posture_from_track(t, extent_z)

            # ---- FALL confirmation ----
            if posture == "FALL":
                if t.fall_start_time is None:
                    t.fall_start_time = time.time()
                elif time.time() - t.fall_start_time > 1.0:
                    t.posture = "FALL_CONFIRMED"
                    t.fall_confirmed = True
                    print(f"⚠ FALL CONFIRMED Track {t.tid}")
            else:
                t.fall_start_time = None
                t.fall_confirmed = False
                t.posture = posture

            # ---- COLOR SELECTION ----
            if t.posture == "FALL_CONFIRMED":
                color = (1, 0, 0, 1)
            elif t.posture == "STANDING":
                color = (0, 1, 0, 1)
            elif t.posture == "SITTING":
                color = (0, 0, 1, 1)
            elif t.posture == "LYING":
                color = (1, 1, 0, 1)
            else:
                color = (1, 1, 1, 1)

            # ---- DRAW ----
            self.renderer.draw_3d_box((cx, cy, cz), (0.6, 0.6, extent_z), color=color)
            self.renderer.add_label((cx, cy, cz + extent_z / 2 + 0.3), f"H{t.tid}:{t.posture}")

        # ---------- OVERLAY ----------
        now = time.time()
        fps = 0 if self.last_time is None else 1 / (now - self.last_time)
        self.last_time = now

        if self.tracks:
            if self.presence_start is None:
                self.presence_start = now
            dur = now - self.presence_start
            htxt = f"Human present {int(dur // 60)}:{int(dur % 60):02d}"
        else:
            self.presence_start = None
            htxt = "No human present"

        self.renderer.add_overlay((0.2, 0.2, GRID_Z + 0.5), f"Points:{len(pts)} FPS:{fps:.1f}")
        self.renderer.add_overlay((0.2, 1.0, GRID_Z + 0.5), htxt)
        self.renderer.add_overlay((0.2, 2.0, GRID_Z + 0.5), f"MaxZ={z_max:.2f}", color=(0, 255, 0, 255))


# ================= Calibration =================
frame_points = []
tracks = []
next_tid = 1
human_boxes = []
human_labels = []

# ===================== APPLICATION =====================

# --- Logic (calibration first) ---
logic = HumanLogic()
logic.interactive_calibration()

# --- Qt App ---
app = QtWidgets.QApplication(sys.argv)
pg.setConfigOptions(antialias=True)

main_widget = QtWidgets.QWidget()
layout = QtWidgets.QVBoxLayout(main_widget)

view = gl.GLViewWidget()
view.setWindowTitle("3D Radar Point Cloud + Human Tracking")
view.setBackgroundColor('k')
view.opts['distance'] = 15
view.orbit(30,30)

layout.addWidget(view)
main_widget.setGeometry(100,100,1200,850)
main_widget.show()

# --- Renderer ---
renderer = HumanRenderer(view, main_widget)
renderer.add_corner_grids()
renderer.add_3d_axis_labels()
renderer.draw_world_triad()
renderer.add_color_legend()

# --- Scatter ---
scatter = gl.GLScatterPlotItem(size=6, pxMode=True)
view.addItem(scatter)

# --- Overlay text ---
info_text = gl.GLTextItem(
    pos=(0.2, 0.2, GRID_Z+0.2),
    text="",
    color=(255,255,255,255),
    font=QtGui.QFont('Helvetica', 12)
)
view.addItem(info_text)

# ================= SERIAL ==================
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)

# ================= OBJECTS =================
parser = RadarParser(ser)
radar_app = RadarApp(parser=parser,renderer=renderer,logic=logic)

# ================= TIMER ==================
timer = QtCore.QTimer()
timer.timeout.connect(radar_app.update)
timer.start(int(DT*1000))

# ================= EXIT ===================
def close_app():
    parser.close()

app.aboutToQuit.connect(close_app)
sys.exit(app.exec_())
