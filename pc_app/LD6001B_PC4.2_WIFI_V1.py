import sys, serial, re, numpy as np, time, socket, threading, json
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from sklearn.cluster import DBSCAN


# print("Initializing radar...")
# ser.write(b"AT+STOP\n"); time.sleep(0.2)
# ser.write(b"AT+PROG=02\n"); time.sleep(0.2)
# ser.write(b"AT+DEBUG=0\n"); time.sleep(0.2)
# ser.write(b"AT+HEATIME=60\n"); time.sleep(0.2)
# ser.write(b"AT+SENS=3\n"); time.sleep(0.2)
# ser.write(b"AT+START\n"); time.sleep(0.5)
# print("Radar ready")

# ================= MODE SELECT =================
# Set MODE to "WIFI" or "UART"
MODE = "WIFI"

# ================= MACROS =================
GRID_X = 6.0
GRID_Y = 6.0
GRID_Z = 3.0
GRID_STEP = 0.5

# UART settings
SERIAL_PORT = "COM5"
BAUDRATE = 115200

# WiFi settings
WIFI_HOST = "192.168.4.1"
WIFI_PORT = 3333

point_re = re.compile(
    r"x=([-0-9.]+),y=([-0-9.]+),z=([-0-9.]+),v=([-0-9.]+),"
    r"snr=([-0-9.]+),abs=([-0-9.]+),dpk=([-0-9.]+)"
)

def f(x):
    try: return float(x)
    except: return 0.0

# ================= Detection Settings =================
EPS = 0.55
MIN_SAMPLES = 5
HUMAN_CONF_THRESHOLD = 0.3
V_MOVE_THRESHOLD = 0.05
FALL_V_THRESHOLD = -0.3
STANDING_Z = 1.0
SITTING_Z = 0.6
LYING_Z = 0.35
CALIBRATION_FILE = "calibration.json"

def load_calibration():
    global STANDING_Z, SITTING_Z, LYING_Z
    try:
        with open(CALIBRATION_FILE, "r") as cf:
            calib = json.load(cf)
            STANDING_Z = calib.get("standing", STANDING_Z)
            SITTING_Z = calib.get("sitting", SITTING_Z)
            LYING_Z = calib.get("lying", LYING_Z)
            print(f"Loaded calibration: Standing={STANDING_Z:.2f}, Sitting={SITTING_Z:.2f}, Lying={LYING_Z:.2f}")
            use_saved = input("Use saved calibration? (y/n): ").strip().lower()
            if use_saved == "y":
                return True
    except FileNotFoundError:
        print("No calibration file found, using defaults.")
    except Exception as e:
        print(f"Calibration load error: {e}")
    return False

def save_calibration():
    calib = {"standing": STANDING_Z, "sitting": SITTING_Z, "lying": LYING_Z}
    with open(CALIBRATION_FILE, "w") as cf:
        json.dump(calib, cf)
    print(f"Saved calibration: Standing={STANDING_Z:.2f}, Sitting={SITTING_Z:.2f}, Lying={LYING_Z:.2f}")

if not load_calibration():
    print(f"Using defaults: Standing={STANDING_Z:.2f}, Sitting={SITTING_Z:.2f}, Lying={LYING_Z:.2f}")
    ans = input("Save these as calibration? (y/n): ").strip().lower()
    if ans == "y":
        save_calibration()

def point_confidence(snr, abs_val, dpk):
    snr_c = min(snr / 40.0, 1.0)
    abs_c = min(abs_val / 15.0, 1.0)
    dpk_c = min(dpk / 10.0, 1.0)
    return 0.45 * snr_c + 0.40 * abs_c + 0.15 * dpk_c

# ================= Helpers =================
def add_corner_grids(view, size=GRID_X, height=GRID_Z, step=GRID_STEP, color=(0.2,0.2,0.2,0.6), width=1):
    coords_x = np.arange(0, size+1e-6, step)
    coords_y = np.arange(0, GRID_Y+1e-6, step)
    coords_z = np.arange(0, height+1e-6, step)
    items = []
    for x in coords_x:
        pts = np.array([[x, 0, 0],[x, GRID_Y, 0]])
        item = gl.GLLinePlotItem(pos=pts, color=color, width=width); view.addItem(item); items.append(item)
    for y in coords_y:
        pts = np.array([[0, y, 0],[size, y, 0]])
        item = gl.GLLinePlotItem(pos=pts, color=color, width=width); view.addItem(item); items.append(item)
    for x in coords_x:
        pts = np.array([[x, 0, 0],[x, 0, height]])
        item = gl.GLLinePlotItem(pos=pts, color=color, width=width); view.addItem(item); items.append(item)
    for y in coords_y:
        pts = np.array([[0, y, 0],[0, y, height]])
        item = gl.GLLinePlotItem(pos=pts, color=color, width=width); view.addItem(item); items.append(item)
    for z in coords_z:
        pts = np.array([[0, 0, z],[size, 0, z]])
        item = gl.GLLinePlotItem(pos=pts, color=color, width=width); view.addItem(item); items.append(item)
        pts = np.array([[0, 0, z],[0, GRID_Y, z]])
        item = gl.GLLinePlotItem(pos=pts, color=color, width=width); view.addItem(item); items.append(item)
    return items

def add_3d_axis_labels(view, size=GRID_X, height=GRID_Z):
    axis_font = QtGui.QFont('Helvetica', 14)
    tick_font = QtGui.QFont('Helvetica', 11)
    items = []
    for text,pos in [("X (m)", (size+0.5,0,0)),
                     ("Y (m)",(0, GRID_Y+0.5,0)),
                     ("Z (m)",(0,0,height+0.5))]:
        item = gl.GLTextItem(pos=pos, text=text, color=(255,255,255,255), font=axis_font)
        view.addItem(item); items.append(item)
    for i in range(int(size)+1):
        item = gl.GLTextItem(pos=(i,0,0), text=f"{i}", color=(200,200,200,255), font=tick_font)
        view.addItem(item); items.append(item)
    for i in range(int(GRID_Y)+1):
        item = gl.GLTextItem(pos=(0,i,0), text=f"{i}", color=(200,200,200,255), font=tick_font)
        view.addItem(item); items.append(item)
    for i in range(int(height)+1):
        item = gl.GLTextItem(pos=(0,0,i), text=f"{i}", color=(200,200,200,255), font=tick_font)
        view.addItem(item); items.append(item)
    return items

def draw_world_triad(view, origin=np.array([GRID_X/2, GRID_Y/2, GRID_Z])):
    colors={'x':(1,0,0,0.5),'y':(0,1,0,0.5),'z':(0,0,1,0.5)}
    axes={'x':np.array([1,0,0]),'y':np.array([0,1,0]),'z':np.array([0,0,-1])}
    for k in axes:
        seg=np.array([origin,origin+axes[k]*1.0])
        item=gl.GLLinePlotItem(pos=seg,color=colors[k],width=3)
        view.addItem(item)

def add_color_legend(parent_widget):
    legend = QtWidgets.QLabel("Legend:  Red=SNR   Green=Abs   Blue=Dpk   | Human clusters labeled")
    legend.setStyleSheet("color: white; font-size: 14px; background-color: rgba(0,0,0,150); padding: 4px;")
    legend.setAlignment(QtCore.Qt.AlignCenter)
    layout = QtWidgets.QVBoxLayout(parent_widget)
    layout.addWidget(legend, alignment=QtCore.Qt.AlignBottom | QtCore.Qt.AlignCenter)
    parent_widget.setLayout(layout)
    return legend

# ================= QT APP =================
app = QtWidgets.QApplication(sys.argv)
pg.setConfigOptions(antialias=True)

main_widget = QtWidgets.QWidget()
layout = QtWidgets.QVBoxLayout(main_widget)

view = gl.GLViewWidget()
view.setWindowTitle("3D Radar Point Cloud + Human Detection")
view.setBackgroundColor('k')
view.opts['distance'] = 15
view.orbit(30,30)
layout.addWidget(view)

legend = add_color_legend(main_widget)

main_widget.setGeometry(100,100,1200,850)
main_widget.show()

add_corner_grids(view, size=GRID_X, height=GRID_Z, step=GRID_STEP)
add_3d_axis_labels(view, size=GRID_X, height=GRID_Z)
draw_world_triad(view)

scatter = gl.GLScatterPlotItem(size=6, pxMode=True)
view.addItem(scatter)

# ================= CONNECTION ==================
wifi_line_buffer = []
wifi_partial = ""
wifi_lock = threading.Lock()

if MODE == "UART":
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
elif MODE == "WIFI":
    WIFI_RETRY_COUNT = 10
    WIFI_RETRY_DELAY = 3
    sock = None
    print(f"Connecting to ESP32 at {WIFI_HOST}:{WIFI_PORT}...")
    print(f"Make sure your PC is connected to the 'RadarSensor' WiFi network.")
    for attempt in range(1, WIFI_RETRY_COUNT + 1):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            sock.connect((WIFI_HOST, WIFI_PORT))
            sock.setblocking(False)
            print(f"Connected to ESP32 via WiFi (attempt {attempt})")
            break
        except (OSError, socket.timeout) as e:
            print(f"Attempt {attempt}/{WIFI_RETRY_COUNT} failed: {e}")
            sock.close()
            sock = None
            if attempt < WIFI_RETRY_COUNT:
                print(f"Retrying in {WIFI_RETRY_DELAY}s...")
                time.sleep(WIFI_RETRY_DELAY)
    if sock is None:
        print("ERROR: Could not connect to ESP32. Is your PC on the 'RadarSensor' WiFi?")
        sys.exit(1)

frame_points = []
frame_count = 0
z_buffer, v_buffer = [], []

def draw_3d_box(view, center, size=(0.5,0.5,1.0), color=(1,1,0,1), width=2):
    """Draws a wireframe box centered at `center` with given size."""
    cx, cy, cz = center
    sx, sy, sz = size
    # 8 corners
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
    # edges (pairs of indices)
    edges = [
        (0,1),(1,2),(2,3),(3,0),  # bottom
        (4,5),(5,6),(6,7),(7,4),  # top
        (0,4),(1,5),(2,6),(3,7)   # verticals
    ]
    items=[]
    for i,j in edges:
        seg = np.array([corners[i], corners[j]])
        item = gl.GLLinePlotItem(pos=seg, color=color, width=width)
        view.addItem(item)
        items.append(item)
    return items


overlay_font = QtGui.QFont('Helvetica', 16, QtGui.QFont.Bold)
overlay_items = []
last_frame_time = None
presence_start = None

# ================= UPDATE LOOP ============
# keep global lists of overlay items
human_boxes = []
human_labels = []

def read_lines():
    if MODE == "UART":
        lines = []
        while ser.in_waiting:
            lines.append(ser.readline().decode(errors="ignore").strip())
        return lines
    elif MODE == "WIFI":
        global wifi_partial
        lines = []
        try:
            data = sock.recv(4096)
            if data:
                wifi_partial += data.decode(errors="ignore")
                while "\n" in wifi_partial:
                    line, wifi_partial = wifi_partial.split("\n", 1)
                    lines.append(line.strip())
        except BlockingIOError:
            pass
        except OSError:
            pass
        return lines

def add_overlay(pos, text, color=(255,255,255,255)):
    item = gl.GLTextItem(pos=pos, text=text, color=color, font=overlay_font)
    view.addItem(item)
    overlay_items.append(item)

def clear_overlays():
    for item in overlay_items:
        view.removeItem(item)
    overlay_items.clear()

def update():
    global frame_points, frame_count, STANDING_Z, SITTING_Z, V_MOVE_THRESHOLD, FALL_V_THRESHOLD
    global human_boxes, human_labels, last_frame_time, presence_start
    try:
        for line in read_lines():
            if "-----PointNum" in line:
                if frame_points:
                    pts = np.array(frame_points)
                    xyz = pts[:, :3]
                    snr = pts[:, 3]
                    abs_val = pts[:, 4]
                    dpk = pts[:, 5]
                    vels = pts[:, 6]

                    # clear previous overlays
                    for item in human_boxes: view.removeItem(item)
                    for lbl in human_labels: view.removeItem(lbl)
                    human_boxes.clear()
                    human_labels.clear()
                    clear_overlays()

                    # scatter coloring
                    snr_norm = np.clip(snr / snr.max(), 0, 1) if snr.max() > 0 else snr
                    abs_norm = np.clip(abs_val / abs_val.max(), 0, 1) if abs_val.max() > 0 else abs_val
                    dpk_norm = np.clip(dpk / dpk.max(), 0, 1) if dpk.max() > 0 else dpk
                    colors = np.zeros((len(xyz), 4))
                    colors[:,0] = np.clip(snr_norm*1.8,0,1)
                    colors[:,1] = np.clip(abs_norm*1.8,0,1)
                    colors[:,2] = np.clip(dpk_norm*1.8,0,1)
                    colors[:,3] = 1.0
                    scatter.setData(pos=xyz, color=colors, size=6)

                    # detection
                    confs = [point_confidence(s,a,d) for s,a,d in zip(snr,abs_val,dpk)]
                    filt = [(x,y,c) for (x,y),c in zip(xyz[:,:2],confs) if c>=0.4]
                    human_count = 0
                    if len(filt) >= MIN_SAMPLES:
                        pts2d = np.array([(x,y) for x,y,_ in filt])
                        conf_arr = np.array([c for _,_,c in filt])
                        db = DBSCAN(eps=EPS, min_samples=MIN_SAMPLES).fit(pts2d)
                        labels = db.labels_
                        hid=1
                        for lbl in set(labels):
                            if lbl==-1: continue
                            idx = labels==lbl
                            cluster = pts2d[idx]
                            cluster_conf = 0.6*np.mean(conf_arr[idx])+0.4*min(len(cluster)/15.0,1.0)
                            if cluster_conf < HUMAN_CONF_THRESHOLD: continue
                            cx,cy = cluster.mean(axis=0)
                            cluster_points_idx = [i for i,(xx,yy) in enumerate(xyz[:,:2])
                                                  if np.isclose(xx,cluster[:,0],atol=0.05).any()
                                                  and np.isclose(yy,cluster[:,1],atol=0.05).any()]
                            cluster_zs = [xyz[i,2] for i in cluster_points_idx]
                            cluster_vs = [vels[i] for i in cluster_points_idx]
                            avg_z = np.mean(cluster_zs)
                            avg_v = np.mean(cluster_vs)

                            # posture
                            if avg_z>=STANDING_Z: posture="STANDING"
                            elif avg_z>=SITTING_Z: posture="SITTING"
                            elif avg_z>=LYING_Z: posture="LYING"
                            else: posture="FALL"

                            # draw box
                            x_extent = np.ptp(cluster[:,0])+0.5
                            y_extent = np.ptp(cluster[:,1])+0.5
                            z_extent = max(np.ptp(cluster_zs),1.0)
                            box_items = draw_3d_box(view,
                                                    center=(cx,cy,avg_z),
                                                    size=(x_extent,y_extent,z_extent),
                                                    color=(0,1,1,1),
                                                    width=2)
                            human_boxes.extend(box_items)

                            # label
                            label_pos = (cx,cy,avg_z+z_extent/2+0.3)
                            text_item = gl.GLTextItem(pos=label_pos,
                                                      text=f"H{hid}: {posture}",
                                                      color=(255,255,0,255),
                                                      font=QtGui.QFont('Helvetica',10))
                            view.addItem(text_item)
                            human_labels.append(text_item)
                            hid+=1
                            human_count+=1

                    # FPS
                    now = time.time()
                    fps = 0 if last_frame_time is None else 1.0 / max(now - last_frame_time, 1e-6)
                    last_frame_time = now

                    n = len(pts)
                    z_max = np.max(xyz[:,2])
                    avg_snr = np.mean(snr)
                    avg_abs = np.mean(abs_val)
                    avg_dpk = np.mean(dpk)
                    avg_conf = np.mean([point_confidence(s,a,d) for s,a,d in zip(snr,abs_val,dpk)])

                    # human presence text
                    has_human = human_count > 0
                    if has_human:
                        if presence_start is None:
                            presence_start = now
                        dur = now - presence_start
                        htxt = f"Human present {int(dur//60)}:{int(dur%60):02d}"
                    else:
                        presence_start = None
                        htxt = "No human present"

                    # 3D overlays
                    add_overlay((0.2, 0.2, GRID_Z+0.5), f"Points:{n}  FPS:{fps:.1f}")
                    add_overlay((0.2, 1.0, GRID_Z+0.5), htxt)
                    add_overlay((0.2, 2.0, GRID_Z+0.5), f"MaxZ={GRID_Z-z_max:.2f}", color=(0,255,0,255))
                    add_overlay((0.2, 3.0, GRID_Z+0.5),
                        f"Avg SNR={avg_snr:.2f}  ABS={avg_abs:.2f}  DPK={avg_dpk:.2f}  Conf={avg_conf:.2f}",
                        color=(255,200,0,255))

                    frame_count+=1
                    frame_points.clear()
                continue

            m = point_re.search(line)
            if not m: continue
            x=f(m.group(1)); y=f(m.group(2)); z=f(m.group(3)); v=f(m.group(4))
            snr=f(m.group(5)); abs_val=f(m.group(6)); dpk=f(m.group(7))
            x=x+GRID_X/2; y=y+GRID_Y/2; z=GRID_Z-z
            frame_points.append([x,y,z,snr,abs_val,dpk,v])
    except Exception as e:
        pass


# ================= TIMER ==================
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(30)


# ================= EXIT ===================
def close():
    if MODE == "UART":
        ser.close()
    elif MODE == "WIFI":
        sock.close()


app.aboutToQuit.connect(close)
sys.exit(app.exec_())
