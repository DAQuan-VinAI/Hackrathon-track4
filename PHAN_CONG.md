# VinSort: Phân công team 4 người

> **Đề tài:** Cánh tay robot phân loại linh kiện bằng thị giác, điều khiển lai Tự động và Cử chỉ tay
> **Đơn vị Vin:** Vin Robotics × VinFast (khâu phân loại linh kiện trên dây chuyền lắp ráp)
> **Hackathon:** Vin Perception Hackathon, Track 4 Capstone
> **Mốc thời gian** tính từ lúc phát đề (0:00). Build 0:15–2:15, demo từ 2:30.

---

## 0. Ràng buộc từ đề bài (đọc trước khi làm)

| Ràng buộc | Ảnh hưởng tới team |
|---|---|
| **Chỉ có 120 phút build** | Cắt phạm vi: **không làm YOLO**, benchmark khoảng 20 seed, teleop bản tối giản |
| **Mốc kiểm tra 1:15** | Phải có **một vòng chạy đầu-cuối** (camera → phát hiện → gắp → thả), dù còn thô |
| **Dùng ít nhất 2 trụ cột** | Nói sâu 3 trụ cột chính **Hình học, Bất định, Bằng chứng**; nhắc thêm Biểu diễn, Thời gian (mục 8) |
| **Một số đo, một ca lỗi** | Số đo: **sai số định vị XY (mm)** và **tỉ lệ gắp đúng thùng**. Ca lỗi: **hai khối cùng màu nằm sát nhau** |
| **Giám khảo đưa đầu vào tại chỗ** | Giám khảo **kéo vật bằng chuột** trong cửa sổ PyBullet, bấm `n` để thêm vật ngẫu nhiên, hoặc **dùng tay mình** chạy teleop |
| **Dữ liệu an toàn** | Chỉ dùng tay của thành viên team, không quay hay lưu ảnh người khác |
| **Video dự phòng** | Bản thô quay lúc 1:45, bản chính quay 2:15–2:30 |

**Nguyên tắc vàng:** chạy được thô từ đầu đến cuối trước, đẹp sau. Mỗi người code trong **file riêng của mình**.

**Câu chuyện của sản phẩm (cả team nói giống nhau):** robot tự phân loại khi chắc chắn. Khi không chắc (ví dụ hai linh kiện dính nhau), robot **không đoán bừa** mà dừng lại và chuyển cho người vận hành điều khiển bằng cử chỉ tay, không cần chạm vào máy. Đây là lý do có chế độ lai, và là chỗ trụ cột **Bất định** nằm.

---

## 1. Bản này sửa gì so với bản trước

| # | Vấn đề ở bản trước | Sửa thế nào | Ai |
|---|---|---|---|
| 1 | AUTO và TELEOP là hai demo rời nhau, chưa có lý do để "lai" | Nối lại: vật bị đánh dấu `suspicious` → robot bỏ qua → HUD báo "CẦN NGƯỜI VẬN HÀNH" → TELEOP. Ca lỗi được demo trực tiếp | A, B, C |
| 2 | B phải tự viết scene tạm, trùng việc với A | A push `scene.py` tối giản lúc **0:30**; B dùng luôn | A, B |
| 3 | D chỉ chạy benchmark được sau khi `pick_and_place` xong | Tách 2 giai đoạn: **benchmark phát hiện** (chỉ cần `detect`, xong trước 1:15) rồi **benchmark gắp** | D |
| 4 | `TABLE_Z = 0.625` (bàn `table.urdf`) trong khi Panda đặt ở z = 0, vật dễ ngoài tầm với | Bỏ bàn, đặt vật trên plane: `TABLE_Z = 0.0` | A |
| 5 | Thùng màu đỏ/xanh nằm trong khung hình sẽ bị `detect` nhận là vật (false positive) | Thùng đặt **ngoài khung hình camera**. Robot về HOME (ra khỏi khung) trước khi chụp | A, B |
| 6 | `p.getClosestPoints(robot, obj, 0.03, 11)`: link 11 của Panda là điểm ảo, có thể không có collision shape nên trả về rỗng | Tìm vật gần nhất bằng **khoảng cách giữa vị trí link 11 và tâm vật** (mục 6A) | A |
| 7 | Constraint với `childFramePosition=[0,0,0.05]` làm vật "nhảy" vào tay | Tính pose tương đối lúc gắp rồi mới tạo constraint (mục 6A) | A |
| 8 | Phím `A`/`S`/`D`/`W` trùng phím tắt của cửa sổ PyBullet (wireframe, AABB...) | Đọc phím từ cửa sổ OpenCV (HUD) bằng `cv2.waitKey`, tắt phím tắt PyBullet | A, C |
| 9 | "Ctrl + kéo chuột" để kéo vật: trong PyBullet GUI, Ctrl + kéo là **xoay camera**; kéo vật là **chuột trái kéo thẳng** | Sửa hướng dẫn cho giám khảo. **Thử lại trên máy demo** | A |
| 10 | Ảnh `getCameraImage` là RGB, OpenCV mặc định BGR → đỏ và xanh dương bị đổi chỗ | Dùng `cv2.COLOR_RGB2HSV`; chỉ đổi sang BGR khi `imshow` | B |
| 11 | GUI và `p.DIRECT` có thể render màu khác nhau → ngưỡng HSV chỉnh ở GUI có thể hỏng ở benchmark | Cả hai đều dùng `renderer=p.ER_TINY_RENDERER` | B, D |
| 12 | `xyz` từ depth là **mặt trên** khối, còn ground truth là **tâm** khối → sai số z luôn lệch nửa cạnh | Sai số định vị tính trên **XY**. z báo riêng, so với `CUBE` | B, D |
| 13 | Thời gian chu kỳ trong sim do team tự đặt (số bước, tốc độ) nhưng được dùng như năng suất thật | Ghi rõ trên slide: năng suất robot thật là **giả định**, không lấy từ sim | D |
| 14 | `mediapipe` bản mới (≥ 1.0) đã bỏ `mp.solutions.hands` (đã kiểm tra trong wheel 1.1.1) | Ghim `mediapipe==0.10.21` (đã kiểm tra có `solutions/hands.py`) | C |
| 15 | `pybullet` không có wheel cho Mac arm64 / Python 3.11 (pip không tìm thấy bản binary) | Cài **trước buổi**: pip sẽ build từ source, cần Xcode Command Line Tools | Cả team |
| 16 | `REPORT.md` mục 2 vẫn ghi YOLO + ArUco + webcam cho vật, khác với kế hoạch này | D sửa `REPORT.md` theo kiến trúc mục 2 dưới đây | D |
| 17 | Không có phương án cắt khi trễ | Mỗi người có mục **"Nếu trễ thì cắt"** | Cả team |

---

## 2. Sơ đồ hệ thống và người phụ trách

```
┌────────────────────────────────────────────┐      ┌────────────────────────────────────────────┐
│ [A] scene.py  ·  thế giới mô phỏng         │      │ [C] teleop.py  ·  tay người vận hành       │
│ PyBullet: Panda, 6 khối 3 màu, 3 thùng     │      │ webcam thật → MediaPipe Hands (21 mốc)     │
│ build_scene(seed) → ground truth cho D     │      │ mốc 9 → xy · cỡ bàn tay → z · kẹp vùng     │
│ giám khảo kéo vật bằng chuột               │      │ chụm ngón 4–8, hai ngưỡng → grip, EMA      │
└──────────────────────┬─────────────────────┘      │ mất tay → robot giữ nguyên (fail-safe)     │
                       │                            └──────────────────────┬─────────────────────┘
                       │ ảnh camera ảo                                     │ xyz, grip, latency_ms
                       ▼                                                   │
┌────────────────────────────────────────────┐                             │
│ [B] perception.py  ·  camera ảo            │                             │
│ capture() → RGB + depth (nhìn thẳng xuống) │                             │
│ detect(): HSV → mask → contour → tâm       │                             │
│ pixel_to_world(): inv(P·V) + depth → xyz   │                             │
│ kiểm tra: diện tích, tỉ lệ cạnh, độ cao    │                             │
│ → đánh dấu suspicious + lý do              │                             │
└──────────────────────┬─────────────────────┘                             │
                       │ list[Detection]                                   │
                       ▼                                                   ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────┐
│ [A] main.py  ·  bộ điều phối chế độ + HUD (cửa sổ OpenCV)                                      │
│ AUTO  : gắp vật bình thường, BỎ QUA vật suspicious                                             │
│         chỉ còn vật suspicious → HUD "CẦN NGƯỜI VẬN HÀNH" → TELEOP                             │
│ TELEOP: mỗi frame gọi teleop_step(); bấm a → quét lại → AUTO                                   │
│ phím  : a AUTO · t TELEOP · r quét lại · n thêm vật · q thoát                                  │
└────────────────────────────────────────────────┬───────────────────────────────────────────────┘
                                                 │ xyz, yaw, grip
                                                 ▼
┌────────────────────────────────────────────────────────────────────────────────────────────────┐
│ [A] controller.py  ·  điều khiển cánh tay                                                      │
│ move_ee() chạy đến đích / servo_ee() từng frame: IK tại link 11                                │
│ grasp() / release(): constraint, giả lập giác hút                                              │
│ pick_and_place(): HOME → APPROACH → DESCEND → GRASP → LIFT → MOVE_BIN → RELEASE                │
└────────────────────────────────────────────────────────────────────────────────────────────────┘
                                                 ↺ góc khớp → stepSimulation() → quay lại scene.py

┌────────────────────────────────────────────────────────────────────────────────────────────────┐
│ [D] benchmark.py  ·  chạy riêng, p.DIRECT, không GUI                                           │
│ dùng lại build_scene + capture + detect + pick_and_place                                       │
│ seed 0–19 (bình thường) + seed 100–104 (ca lỗi), có / không nhiễu                              │
│ so với ground truth của engine → CSV → bảng, biểu đồ → slide, REPORT.md                        │
└────────────────────────────────────────────────────────────────────────────────────────────────┘
```

**Quy tắc AUTO ↔ TELEOP (A cài trong `main.py`):**

```
AUTO:
  về HOME → capture → detect
  ok  = các Detection không suspicious
  nếu có ok        → gắp vật gần nhất trong ok, thả vào BINS[color], lặp lại
  nếu chỉ còn suspicious → HUD "CẦN NGƯỜI VẬN HÀNH: <reason>" → chuyển TELEOP
  nếu không còn gì → HUD "XONG", chờ r (quét lại) hoặc n (thêm vật)
TELEOP:
  mỗi frame: teleop_step()  (C viết)
  bấm a → về HOME → quét lại → AUTO
```

---

## 3. Tổng quan phân vai

Thành viên: **Đoàn Anh Quân, Hoàng Đức Dũng, Trần Thị Lan, Đặng Thái Anh**.
Gợi ý chọn: **A** là người quen PyBullet và git nhất, máy demo nên là máy của A. **B** mạnh OpenCV và hình học camera. **C** có webcam ổn định, chịu thử nhiều. **D** cẩn thận với số liệu, nói tốt.

| Vai | Người | File phụ trách | Trụ cột chính | Đầu ra cuối cùng | Phần nói khi demo |
|---|---|---|---|---|---|
| **A: Sim & Điều khiển** (trưởng nhóm, người ghép code) | **Đoàn Anh Quân** | `common.py`, `scene.py`, `controller.py`, `main.py` | Hình học (IK), Biểu diễn (FSM) | Demo chạy 3 lần liên tiếp không lỗi | Demo AUTO |
| **B: Perception** | **Hoàng Đức Dũng** | `perception.py` | Hình học, Bất định | `detect()` sai số XY < 5 mm (mục tiêu), cờ `suspicious` bắt được ca lỗi | Giải thích khung camera, sai số |
| **C: Teleop & Tương tác** | **Đặng Thái Anh** | `teleop.py` | Thời gian, Bất định | Điều khiển gắp 1 vật bằng tay, fail-safe, phương án bàn phím | Demo ca lỗi → TELEOP, mời giám khảo thử |
| **D: Bằng chứng & Thuyết trình** | **Trần Thị Lan** | `benchmark.py`, slide, video, `REPORT.md` | Bằng chứng | CSV + bảng số + 1 biểu đồ, slide 6 trang, video trên máy và USB | Mở đầu, số đo, ca lỗi, impact |

---

## 4. Hợp đồng giao tiếp (chốt 0:00–0:15)

Cả team **thống nhất file này và chữ ký hàm trước khi code**. A push `common.py` kèm **stub** (hàm giả trả dữ liệu cố định) cho mọi hàm trong bảng, để ai cũng import và test được ngay.

```python
# common.py  (A push before 0:15, everyone imports from here)
from dataclasses import dataclass

COLORS = ["red", "green", "blue"]                 # 3 part types, one bin per color
CUBE = 0.04                                       # cube edge length (m)
TABLE_Z = 0.0                                     # objects rest on the ground plane, no table.urdf
WORKSPACE = dict(x=(0.35, 0.75), y=(-0.25, 0.25), z=(0.03, 0.35))  # spawn area and teleop safe box
BINS = {"red": (0.25, -0.50), "green": (0.40, -0.50), "blue": (0.55, -0.50)}  # xy, outside camera view
RELEASE_Z = 0.20                                  # drop height above a bin
EE_LINK = 11                                      # panda_grasptarget

# Overhead camera: B tunes it, then frozen at 0:30
CAM = dict(eye=(0.55, 0.0, 1.0), target=(0.55, 0.0, 0.0), up=(1, 0, 0),
           fov=30, W=640, H=480, near=0.1, far=2.0)

SEEDS_NORMAL = list(range(0, 20))                 # random layout, min spacing 8 cm
SEEDS_FAILURE = list(range(100, 105))             # two same-color cubes touching

@dataclass
class Detection:
    color: str            # "red" | "green" | "blue"
    uv: tuple             # pixel centroid (u, v)
    xyz: tuple            # world coordinate of the top face center (m)
    yaw: float            # rotation (rad), optional for a suction gripper
    area: int             # pixel area
    suspicious: bool = False
    reason: str = ""      # "area" | "aspect" | "depth" | ""
```

Với camera trên (cao 1.0 m, FOV dọc 30°), khung hình phủ khoảng x ∈ [0.28, 0.82], y ∈ [−0.36, 0.36], tức khoảng 1.1 mm/pixel. Đây là số tính từ hình học, **chưa kiểm chứng**; B kiểm lại bằng ảnh thật lúc 0:30.

| Hàm | Người viết | Người dùng | Có bản thật lúc |
|---|---|---|---|
| `build_scene(seed, n_objects=6, gui=True) -> dict` trả `{robot, objects: [{id, color}], bins}` | A | B, C, D | 0:30 |
| `capture() -> (rgb, depth, seg)` | B | A, D | 0:30 |
| `detect(rgb, depth) -> list[Detection]` | B | A, D | 0:50 (chưa có xyz), 1:05 (đủ) |
| `move_ee(robot, xyz, yaw=0, max_steps=240, realtime=False) -> bool` chạy đến đích, trả False nếu không tới | A | A, D | 0:45 |
| `servo_ee(robot, xyz, n_steps=8)` không chặn, gọi mỗi frame teleop | A | C | 1:05 |
| `grasp(robot, object_ids) -> cid or None`, `release(cid)` | A | A, C | 0:45 |
| `pick_and_place(robot, det: Detection, object_ids, realtime=False) -> bool` | A | main, D | 0:55 |
| `HandTracker().read() -> (xyz or None, grip, frame_bgr, latency_ms)` | C | C, main | 1:15 |
| `KeyboardTracker().read(key=-1)` cùng chữ ký, phương án dự phòng; phím `i/j/k/l`, `u/o`, `space` (xem `common.py`) | C | main | 2:00 |
| `teleop_step(robot, tracker, object_ids, state) -> state` | C | main | 1:40 |
| `run_benchmark(seeds, mode="detect" or "pick", noise=False) -> csv_path` | D | slide | 1:15 / 1:45 |

**Quy ước:** tọa độ **mét**, góc **radian**, hệ tọa độ thế giới PyBullet (x ra xa robot, z hướng lên, gốc robot tại (0, 0, 0)). Màu ảnh **RGB** trong mọi hàm, chỉ đổi sang BGR ngay trước `cv2.imshow`.

---

## 5. Lịch trình và mốc bàn giao

### 5.1 Timeline từng người

```
    0:00     0:15     0:30     0:45     1:00     1:15     1:30     1:45     2:00     2:15     2:30
A   [  chốt ][ scene ][  controller ][ main E2E ][   HUD+teleop   ][ hybrid+ổn định ][  test ]
B   [  chốt ][capture][  detect  ][ px→3D ][ghép][kiểm tra tin cậy][   hỗ trợ đo    ][tập Q&A]
C   [  chốt ][MediaPipe ][  ánh xạ  ][ grip+EMA ][  ghép+latency  ][ fail-safe+phím ][thử cam]
D   [  chốt ][  slide 1–2  ][    bench detect   ][bench pick+noise][  ca lỗi+slide  ][ video ]
                                                 ▲ 1:15 E2E                          ▲ 2:15 FREEZE
```

### 5.2 Mốc bàn giao (ai đưa gì cho ai, lúc nào)

| Giờ | Từ → Đến | Bàn giao | Nếu trễ |
|---|---|---|---|
| 0:15 | A → cả team | `common.py` + stub mọi hàm, repo pull về chạy được | Không ai được code trước khi có file này |
| 0:30 | A → B, D | `scene.py`: `build_scene(seed)` chạy cả GUI và DIRECT | B, D dùng scene 3 khối trên plane tự load trong `__main__`, không viết thêm |
| 0:30 | B → A, D | `CAM` đã chốt trong `common.py`, ảnh `capture()` thấy đủ khối, không thấy thùng | |
| 0:45 | A → D | `move_ee`, `grasp`, `release` chạy với tọa độ thật từ engine | |
| 0:50 | B → D | `detect()` bản đầu (đúng màu, đúng tâm pixel) | D làm khung CSV với stub |
| 0:55 | A → D | `pick_and_place()` gắp được vật bằng tọa độ thật | |
| 1:05 | B → A, D | `detect()` đủ `xyz` | A ghép bằng tọa độ thật để vẫn có vòng E2E lúc 1:15 |
| 🔴 **1:15** | A | **Vòng đầu-cuối chạy được** (mốc kiểm tra) | Xem mục 5.4 |
| 1:15 | C → A | `HandTracker.read()` chạy độc lập, có grip ổn định | |
| 1:15 | D → cả team | CSV benchmark phát hiện 20 seed | |
| 1:40 | B → A, D | Cờ `suspicious` hoạt động trên seed 100–104 | |
| 1:40 | C → A | `teleop_step()` gắp được 1 vật | A ghép phím `t` với bản đang có |
| **1:45** | A + D | **Video dự phòng bản thô** (AUTO chạy được) | |
| 1:45 | D → cả team | Bảng số benchmark gắp + nhiễu | |
| 2:00 | D → cả team | Slide 3–6 có số thật | |
| 🔴 **2:15** | A | **Đóng băng code**, tag `freeze` | Không sửa tính năng nữa, chỉ sửa lỗi crash |
| 2:30 | D | Video chính trên máy demo và USB, slide xong | |

### 5.3 Đồng bộ nhanh (2 phút, đứng nói)

Ở các mốc **0:30, 0:45, 1:15, 1:45, 2:15**: mỗi người nói 3 ý: *xong gì, đang kẹt gì, cần ai giúp*. A ghi lại quyết định cắt bớt (nếu có).

### 5.4 Nếu đến 1:15 vẫn chưa ghép được

A demo phần gắp bằng tọa độ thật từ engine, B demo `detect()` riêng trên cửa sổ OpenCV. Sau đó **cả B và A** ngồi ghép đến khi chạy, C và D giữ việc của mình. **Không ai bắt đầu tính năng mới** khi vòng đầu-cuối chưa chạy.

---

## 6. Việc cụ thể của từng người

Mỗi người có: mục tiêu, đầu ra bàn giao, mốc thời gian, hướng dẫn kỹ thuật, bẫy hay gặp, phần cắt nếu trễ, câu hỏi cần trả lời được.

### 🅰️ Người A: Sim & Điều khiển (trưởng nhóm, người ghép)

**Mục tiêu:** robot gắp được vật tại `(x, y, z)` bất kỳ trong `WORKSPACE` và thả vào đúng thùng; ghép mọi module thành một chương trình chạy ổn định.

**Đầu ra bàn giao:**
- [ ] `scene.py`: `build_scene(seed)` sinh cùng một bố cục với cùng seed, chạy được cả `p.GUI` và `p.DIRECT`
- [ ] `controller.py`: `move_ee`, `servo_ee`, `grasp`, `release`, `pick_and_place`
- [ ] `main.py`: vòng AUTO ↔ TELEOP theo quy tắc mục 2, HUD, phím `a/t/r/n/q`
- [ ] Demo chạy 3 lần liên tiếp trên máy demo không lỗi

| Thời gian | Việc | Xong khi |
|---|---|---|
| 0:00–0:15 | Chốt hợp đồng, tạo repo, `common.py`, stub, `requirements.txt` | Cả team pull về và `python -c "import common"` chạy |
| 0:15–0:30 | `scene.py`: plane, Panda (`franka_panda/panda.urdf`, `useFixedBase=True`, gốc (0,0,0)), 6 khối `CUBE` 3 màu đặt ngẫu nhiên theo seed với khoảng cách tối thiểu 8 cm; seed 100–104 đặt 2 khối cùng màu sát nhau. 3 thùng tại `BINS` (khung xám thấp hoặc ô vuông phẳng) | `python scene.py --seed 0` mở GUI, vật nằm yên sau 240 bước. **Push lúc 0:30** |
| 0:30–0:45 | `move_ee` (IK), `grasp`/`release` (constraint). Đặt camera GUI nhìn từ sau robot bằng `resetDebugVisualizerCamera` để trái/phải màn hình khớp với tay người | Đưa tay robot tới 5 điểm trong `WORKSPACE`, sai số < 5 mm, gắp và nhấc được 1 vật |
| 0:45–0:55 | `pick_and_place`: máy trạng thái HOME → APPROACH (trên vật 10 cm) → DESCEND → GRASP → LIFT → MOVE_BIN → RELEASE (`RELEASE_Z`) → HOME | Gắp 6/6 vật seed 0 bằng tọa độ thật từ engine |
| 0:55–1:15 | `main.py` AUTO: HOME → `capture` → `detect` → gắp vật gần nhất → lặp | ✅ **Mốc 1:15: vòng đầu-cuối** |
| 1:15–1:45 | HUD: cửa sổ OpenCV hiện ảnh camera ảo + khung phát hiện + chế độ + số vật đã phân loại. Phím đọc bằng `cv2.waitKey`. Ghép `teleop_step` của C vào phím `t`. Phím `n` thêm một khối ngẫu nhiên | Chuyển AUTO ↔ TELEOP 5 lần liên tiếp không crash. 1:45 quay video thô cùng D |
| 1:45–2:15 | Quy tắc lai: chỉ còn vật suspicious → HUD đỏ "CẦN NGƯỜI VẬN HÀNH" → TELEOP. Sửa lỗi, chỉnh tốc độ cho dễ xem. **Đóng băng 2:15** | Chạy kịch bản demo mục 7 trọn vẹn 3 lần |
| 2:15–2:30 | Chạy thử trên máy demo, đóng hết chương trình khác, cắm sạc | Sẵn sàng lên |

**Hướng dẫn kỹ thuật:**

```python
ARM_JOINTS = list(range(7))

def solve_ik(robot, xyz, yaw=0.0):
    orn = p.getQuaternionFromEuler([np.pi, 0, yaw])            # gripper pointing down
    q = p.calculateInverseKinematics(robot, EE_LINK, xyz, orn,
                                     maxNumIterations=100, residualThreshold=1e-4)
    return q[:7]       # IK returns all movable joints (7 arm + 2 fingers); keep the arm only

def move_ee(robot, xyz, yaw=0.0, max_steps=240, tol=0.005, realtime=False):
    p.setJointMotorControlArray(robot, ARM_JOINTS, p.POSITION_CONTROL,
                                targetPositions=solve_ik(robot, xyz, yaw), forces=[87] * 7)
    for _ in range(max_steps):
        p.stepSimulation()
        if realtime:
            time.sleep(1 / 240)
        if np.linalg.norm(np.subtract(p.getLinkState(robot, EE_LINK)[0], xyz)) < tol:
            return True
    return False       # unreachable or too slow: caller decides, never assume success
```

```python
def grasp(robot, object_ids, radius=0.04):
    ee_pos, ee_orn = p.getLinkState(robot, EE_LINK)[:2]
    dist = lambda o: np.linalg.norm(np.subtract(p.getBasePositionAndOrientation(o)[0], ee_pos))
    obj = min(object_ids, key=dist)
    if dist(obj) > radius:
        return None
    # keep the current relative pose so the object does not jump into the gripper
    obj_pos, obj_orn = p.getBasePositionAndOrientation(obj)
    inv_pos, inv_orn = p.invertTransform(ee_pos, ee_orn)
    rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, obj_pos, obj_orn)
    cid = p.createConstraint(robot, EE_LINK, obj, -1, p.JOINT_FIXED, [0, 0, 0],
                             rel_pos, [0, 0, 0], rel_orn)
    p.changeConstraint(cid, maxForce=100)
    return cid

def release(cid):
    if cid is not None:
        p.removeConstraint(cid)
```

- Chọn vật để gắp bằng **khoảng cách tới link 11**, không lấy ID từ bộ phát hiện. Như vậy perception vẫn phải làm thật: `detect` sai vị trí thì `grasp` trả `None`, và D đếm được lỗi đó.
- `servo_ee` cho teleop: đặt mục tiêu IK rồi chỉ chạy `n_steps = 240 / FPS_webcam` (khoảng 8 bước ở 30 FPS) rồi trả về, không chờ tới đích.
- Tắt phím tắt PyBullet: `p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)`. Ẩn panel bên: `p.COV_ENABLE_GUI, 0`.
- **HOME phải nằm ngoài khung hình camera** (ví dụ EE tại khoảng (0.15, 0.40, 0.45)). Kiểm tra bằng ảnh `capture()` lúc HOME: không thấy robot.
- Thả vật từ `RELEASE_Z` (rơi xuống thùng), tránh hạ thấp sát mép tầm với.
- Khi demo: `realtime=True`. Khi benchmark (D gọi): `realtime=False`.

**Bẫy hay gặp:** IK cho tư thế lạ ở rìa vùng làm việc → thêm `restPoses` gần tư thế HOME. Vật bị hất khi robot đi ngang → APPROACH luôn ở độ cao 10 cm trở lên. Kéo vật bằng chuột chỉ hoạt động khi đang `stepSimulation()`, nên vòng chờ ở chế độ "XONG" vẫn phải step.

**Nếu trễ thì cắt:** bỏ `yaw` (giác hút không cần), bỏ phím `n`, HUD chỉ còn dòng chữ chế độ.

**Câu hỏi cần trả lời được:**
- *Vì sao gắp bằng constraint thay vì ma sát?* Mô phỏng tiếp xúc trong PyBullet không ổn định; giác hút là loại tay gắp phổ biến cho chi tiết phẳng. Đổi lại, tỉ lệ gắp trong sim **không đo được độ bám thật**.
- *IK giải thế nào, khi nào thất bại?* IK số lặp (damped least squares) trong PyBullet; thất bại khi điểm ngoài tầm với hoặc gần điểm kỳ dị. `move_ee` trả `False` khi không tới đích trong 240 bước.
- *Vì sao bỏ qua vật nghi ngờ thay vì gắp thử?* Gắp sai thùng tốn hơn chờ người: lỗi phân loại đi tiếp xuống dây chuyền.

---

### 🅱️ Người B: Perception

**Mục tiêu:** từ ảnh camera ảo, trả về danh sách `Detection` có **tọa độ 3D chính xác** và **biết khi nào mình không chắc**.

**Đầu ra bàn giao:**
- [ ] `capture()`, `detect()`, `pixel_to_world()` trong `perception.py`
- [ ] Sai số XY trung bình < 5 mm trên seed 0–4 (mục tiêu, cần đo)
- [ ] Cờ `suspicious` bật trên seed 100–104 và **không bật** trên seed 0–19 (0 false alarm là mục tiêu)
- [ ] 3 ảnh debug cho slide ca lỗi: RGB, mask, contour bị gộp

| Thời gian | Việc | Xong khi |
|---|---|---|
| 0:00–0:15 | Cùng A chốt `CAM` trong `common.py` | |
| 0:15–0:30 | `capture()` dùng scene của A (hoặc scene tạm 3 khối trên plane trong `__main__` nếu A chưa push). Kiểm tra vùng phủ của camera | Lưu `debug/cap.png` thấy đủ 6 khối, không thấy thùng và robot. **Chốt `CAM` lúc 0:30** |
| 0:30–0:50 | `detect()`: `cv2.COLOR_RGB2HSV`, `inRange` từng màu (đỏ cần **2 khoảng hue**: 0–10 và 170–180), `morphologyEx` open, `findContours`, lọc theo diện tích tối thiểu, `moments` lấy tâm, `minAreaRect` lấy góc | Seed 0–4: số phát hiện = số vật, đúng màu, không có phát hiện thừa |
| 0:50–1:05 | `pixel_to_world()` (công thức dưới). Lấy depth tại tâm, nên lấy **trung vị depth trong vùng 5×5 quanh tâm** để tránh mép vật | Sai số XY so với `getBasePositionAndOrientation` < 5 mm trên seed 0 |
| 1:05–1:15 | Giao cho A ghép, sửa lỗi ghép | ✅ Mốc 1:15 |
| 1:15–1:40 | **Lớp kiểm tra độ tin cậy** (xem dưới) | Seed 100–104 bị đánh dấu; seed 0–19 không bị. HUD hiện lý do |
| 1:40–2:15 | Đưa ảnh debug ca lỗi cho D. Kiểm tra `detect` chạy với chế độ nhiễu của D. Viết 3 dòng giải thích pipeline cho slide 2 | D có ảnh và số sai số |
| 2:15–2:30 | Tập trả lời câu hỏi phần mình và phần sai số | |

**Hướng dẫn kỹ thuật:**

```python
def capture():
    view = p.computeViewMatrix(CAM["eye"], CAM["target"], CAM["up"])
    proj = p.computeProjectionMatrixFOV(CAM["fov"], CAM["W"] / CAM["H"], CAM["near"], CAM["far"])
    _, _, rgba, depth, seg = p.getCameraImage(CAM["W"], CAM["H"], view, proj,
                                              renderer=p.ER_TINY_RENDERER)   # same renderer in GUI and DIRECT
    H, W = CAM["H"], CAM["W"]
    rgb = np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8)              # RGB, not BGR
    return rgb, np.reshape(depth, (H, W)), np.reshape(seg, (H, W))

def pixel_to_world(u, v, d, view, proj, W, H):
    # +0.5: use the pixel center, not its corner
    ndc = np.array([(2 * (u + 0.5) - W) / W, (H - 2 * (v + 0.5)) / H, 2 * d - 1, 1.0])
    P = np.array(proj).reshape(4, 4, order="F")
    V = np.array(view).reshape(4, 4, order="F")
    pw = np.linalg.inv(P @ V) @ ndc
    return pw[:3] / pw[3]
```

**Lớp kiểm tra độ tin cậy** (mỗi phát hiện, bật cờ ở điều kiện đầu tiên đúng):

| Kiểm tra | Điều kiện nghi ngờ | `reason` | Ý nghĩa |
|---|---|---|---|
| Diện tích | `area > 1.6 × A_ref` | `"area"` | Có thể là 2 khối dính nhau |
| Tỉ lệ cạnh | cạnh dài / cạnh ngắn của `minAreaRect` > 1.5 | `"aspect"` | Khối vuông mà ra hình chữ nhật dài → gộp |
| Độ cao | `abs(z − (TABLE_Z + CUBE)) > 0.01` | `"depth"` | Không nằm trên mặt bàn (bị chồng, bị nghiêng) |

`A_ref` là diện tích trung vị của các khối ở seed 0–4, **đo một lần rồi ghi hằng số**, không đoán. Ngưỡng 1.6 và 1.5 là điểm khởi đầu; chỉnh bằng dữ liệu seed 0–19 và 100–104, ghi lại lý do chọn.

**Bẫy hay gặp:** `up` của `computeViewMatrix` song song với hướng nhìn (camera nhìn thẳng xuống mà `up=(0,0,1)`) → ma trận hỏng, nên dùng `up=(1,0,0)`. Khối ở rìa khung hình lộ mặt bên do phối cảnh, làm tâm lệch vài mm. **Không dùng ảnh `seg` để phát hiện vật**; ảnh này chỉ là ground truth cho D.

**Nếu trễ thì cắt:** bỏ `yaw`, bỏ kiểm tra tỉ lệ cạnh (giữ diện tích và độ cao).

**Câu hỏi cần trả lời được:**
- *Vì sao HSV thay vì RGB?* Hue tách khỏi độ sáng, nên bền hơn khi ánh sáng thay đổi. D có số so sánh có/không nhiễu để chứng minh.
- *Vì sao không dùng homography?* Homography chỉ đúng khi mọi điểm nằm trên một mặt phẳng. Sim cho sẵn depth nên nghịch đảo mô hình chiếu là đúng bản chất và dùng được cho vật có chiều cao.
- *Sai số đến từ đâu?* Độ phân giải pixel (khoảng 1.1 mm/pixel theo tính toán), tâm lệch do phối cảnh, depth ở mép vật.
- *Camera thật thì khác gì?* Cần hiệu chuẩn để có ma trận nội tại K, hệ số méo và phép biến đổi tay–mắt (hand-eye calibration); màu thật bị ảnh hưởng bởi ánh sáng nhà xưởng.

---

### 🅲 Người C: Teleop & Tương tác

**Mục tiêu:** bàn tay trước webcam thật điều khiển cánh tay robot ảo, an toàn khi mất tín hiệu; đây là **đầu vào để giám khảo tự thử** và là cách xử lý ca lỗi trong câu chuyện lai.

**Đầu ra bàn giao:**
- [ ] `HandTracker` và `KeyboardTracker` cùng chữ ký `read()`
- [ ] `teleop_step()` gọi `servo_ee`, `grasp`, `release` của A
- [ ] Số đo: độ trễ (ms) và FPS, hiện trên HUD và đưa cho D
- [ ] Kịch bản 30 giây hướng dẫn giám khảo

| Thời gian | Việc | Xong khi |
|---|---|---|
| Trước buổi | Cài `mediapipe==0.10.21`, cấp quyền Camera cho Terminal / VS Code trên macOS, chạy thử webcam | Thấy khung xương bàn tay |
| 0:15–0:35 | `HandTracker`: mở webcam, lật ảnh như gương, `mp.solutions.hands.Hands(max_num_hands=1, model_complexity=0)`, vẽ 21 mốc | Khung xương realtime, in FPS (ghi số đo được) |
| 0:35–0:55 | Ánh xạ mốc 9 `(x, y)` ảnh → `(x, y)` robot trong `WORKSPACE`. Cỡ bàn tay (khoảng cách mốc 0–9) → z: tay to (gần camera) thì hạ xuống. Kẹp mọi giá trị trong `WORKSPACE` | Đưa tay sang phải màn hình → robot sang phải trong cửa sổ PyBullet (thống nhất chiều với A) |
| 0:55–1:15 | Phát hiện gắp bằng hai ngưỡng + làm mượt EMA (code dưới) | Chụm tay 10 lần → đúng 10 lần đổi trạng thái, không nhấp nháy |
| 1:15–1:40 | `teleop_step()`: đọc tay → `servo_ee` → grip lên thì `grasp`, grip xuống thì `release`. Giới hạn tốc độ 1.5 cm mỗi frame. Đo độ trễ | Điều khiển gắp 1 vật và thả vào thùng bằng tay |
| 1:40–2:15 | **Fail-safe:** không thấy tay → robot giữ nguyên, **không thả vật**. `KeyboardTracker` theo đúng `common.py`: `i`/`k` tiến/lùi, `j`/`l` trái/phải, `u`/`o` lên/xuống, `space` gắp/thả. **Không dùng `a`/`t`/`r`/`n`/`q`**: đã dành cho `main.py` (`a` = AUTO, `q` = thoát). Tập kịch bản cho giám khảo | Rút tay ra → robot đứng yên ngay frame đó; bàn phím điều khiển được |
| 2:15–2:30 | Thử webcam dưới ánh sáng phòng demo, chỉnh ngưỡng nếu cần | |

**Hướng dẫn kỹ thuật:**

```python
# inside HandTracker.read()
hand = dist(lm[0], lm[9])                        # hand size, scale-invariant reference
ratio = dist(lm[4], lm[8]) / hand
if self.grip and ratio > 0.40:                   # two thresholds: no flicker near the boundary
    self.grip = False
elif not self.grip and ratio < 0.25:
    self.grip = True
self.pos = 0.7 * self.pos + 0.3 * raw_xyz        # EMA: smoother but adds lag
```

- **Độ trễ:** `t0` ngay sau `cap.read()` trả frame, `t1` ngay sau khi gửi lệnh `servo_ee`. `latency_ms = (t1 − t0) × 1000`. Số này **không gồm** độ trễ phần cứng camera (không đo được bằng cách này); nói rõ khi trình bày.
- **Chuyển vào TELEOP:** robot không được nhảy ngay tới vị trí tay. Giới hạn 1.5 cm mỗi frame lo việc này.
- Một tay, một người: `max_num_hands=1` để tay người khác lọt vào khung không chiếm quyền điều khiển.

**Bẫy hay gặp:** quên lật ảnh → trái phải ngược. Ánh sáng ngược (cửa sổ sau lưng) làm MediaPipe mất tay. `cv2.waitKey` chỉ có một chỗ đọc phím (trong `main.py` của A); C nhận phím qua tham số, không gọi `waitKey` riêng.

**Nếu trễ thì cắt:** bỏ điều khiển z bằng cỡ bàn tay, dùng z cố định 10 cm, khi chụm tay thì robot tự hạ xuống gắp rồi nhấc lên.

**Câu hỏi cần trả lời được:**
- *Vì sao EMA, đánh đổi là gì?* Giảm rung nhưng tăng độ trễ. Hệ số 0.3 chọn sau khi thử (ghi lại đã thử những giá trị nào). Kalman tốt hơn khi có mô hình chuyển động, nhưng EMA một tham số là đủ cho 2 giờ.
- *Vì sao hai ngưỡng?* Một ngưỡng thì khi tỉ lệ dao động quanh ngưỡng, robot gắp/thả liên tục.
- *Độ trễ bao nhiêu thì chấp nhận được?* Nêu số đo được; khoảng 100 ms thường được coi là ngưỡng cảm giác "tức thì", đây là quy tắc tham khảo, chưa kiểm chứng trên người dùng của team.
- *Mất tay thì robot làm gì?* Giữ nguyên vị trí và giữ vật (fail-safe). Thả vật khi mất tín hiệu có thể rơi vào chỗ sai.

---

### 🅳 Người D: Bằng chứng & Thuyết trình

**Mục tiêu:** có **số đo kiểm chứng được**, **một ca lỗi có phân tích nguyên nhân**, **ước lượng impact có ghi giả định**, slide, `REPORT.md` và video dự phòng.

**Đầu ra bàn giao:**
- [ ] `benchmark.py` hai chế độ `detect` và `pick`, có cờ `noise`
- [ ] `results/detect.csv`, `results/pick.csv`, 1 biểu đồ
- [ ] Slide 6 trang, `REPORT.md` điền số thật và sửa mục kiến trúc
- [ ] Video khoảng 2 phút trên máy demo và USB

| Thời gian | Việc | Xong khi |
|---|---|---|
| 0:00–0:15 | Mở slide. Ghi **định nghĩa số đo** (bảng dưới) vào đầu `benchmark.py` để cả team đọc cùng một nghĩa | Cả team đồng ý định nghĩa |
| 0:15–0:40 | Slide 1 (bài toán, người dùng) và slide 2 (sơ đồ mục 2). Viết khung impact với công thức và ô giả định | Slide 1–2 xong |
| 0:40–1:15 | `run_benchmark(mode="detect")`: với mỗi seed: `build_scene(seed, gui=False)`, chờ vật nằm yên, `capture`, `detect`, ghép với ground truth, ghi CSV. Dùng stub cho đến khi B giao `detect` lúc 0:50 | CSV 20 seed có cột sai số. **1:15 gửi số cho team** |
| 1:15–1:45 | `run_benchmark(mode="pick")`: gắp hết vật mỗi seed bằng `pick_and_place(realtime=False)`. Chạy lại cả hai chế độ với `noise=True`. 1:45 quay video thô cùng A | Bảng số + 1 biểu đồ (sai số XY, sạch vs nhiễu) |
| 1:45–2:15 | **Ca lỗi:** chạy seed 100–104, lấy ảnh debug của B, viết nguyên nhân và cách khắc phục. Hoàn thiện slide 3–6. Sửa `REPORT.md` mục 2 (kiến trúc) và điền số | Slide xong lúc 2:00, `REPORT.md` xong lúc 2:15 |
| 2:15–2:30 | **Quay video chính** (AUTO, giám khảo kéo vật, ca lỗi → TELEOP). Chép vào USB. Cả team tập nói một lượt, bấm giờ | Video có ở 2 nơi, thuyết trình ≤ 7 phút trước hỏi đáp |

**Định nghĩa số đo** (ghi nguyên văn lên slide):

| Số đo | Cách tính |
|---|---|
| Recall phát hiện | Số vật ground truth được ghép / tổng số vật. Ghép: cùng màu, gần nhất theo XY, trong bán kính 2 cm |
| False positive | Số phát hiện không ghép được với vật nào |
| Sai số định vị XY (mm) | ‖xy phát hiện − xy ground truth‖, báo trung bình, trung vị, lớn nhất trên các cặp đã ghép |
| Sai số z (mm) | z phát hiện − (TABLE_Z + CUBE), báo riêng vì phát hiện cho mặt trên còn ground truth là tâm |
| Tỉ lệ gắp đúng thùng | Số vật nằm trong vùng thùng đúng màu sau khi thả / số vật trong scene. Vật bị bỏ qua vì suspicious tính riêng |
| Cờ suspicious | Trên seed 100–104: số lần bật đúng. Trên seed 0–19: số lần bật nhầm (false alarm) |
| Thời gian chu kỳ (sim) | Số bước mô phỏng mỗi lần gắp / 240. **Phụ thuộc tham số team tự đặt**, không phải tốc độ robot thật |

**Chế độ nhiễu:** trước khi gọi `detect`, nhân độ sáng ảnh với hệ số ngẫu nhiên trong [0.7, 1.3] và cộng nhiễu Gauss σ = 8 (thang 0–255), cố định seed nhiễu.

**Bảng ước lượng impact** (điền sau benchmark; dòng nào là giả định phải ghi chữ **giả định** trên slide):

```
Tỉ lệ phân loại đúng   = [đo, benchmark pick]                      → đo trong sim
Năng suất robot        = [GIẢ ĐỊNH: chu kỳ gắp-thả của robot công nghiệp, ghi nguồn nếu có]
Số chi tiết đúng / giờ = năng suất × tỉ lệ phân loại đúng           → ước lượng
Giờ công giải phóng    = số ca × số giờ mỗi ca × số trạm            → [GIẢ ĐỊNH]
Rủi ro triển khai      : sim-to-real gap, vật bóng/trong suốt, vật chồng nhau, người và robot chung vùng làm việc
Hướng mở rộng          : YOLO train trên dữ liệu tổng hợp từ sim → camera thật hiệu chuẩn → robot thật
```

> ⚠️ **Không bịa số liệu về VinFast.** Thứ gì chưa đo được thì ghi là "giả định" và nói rõ khi thuyết trình. Giám khảo đánh giá cao sự trung thực (tiêu chí Bằng chứng).

**Bẫy hay gặp:** chưa chờ vật nằm yên đã chụp (vật còn rơi) → chạy 240 bước sau `build_scene`. Quên cố định seed nhiễu → chạy lại ra số khác. Chạy benchmark bằng GUI rất chậm → luôn `p.DIRECT`.

**Nếu trễ thì cắt:** giảm còn 10 seed, bỏ chế độ nhiễu cho benchmark gắp (giữ cho benchmark phát hiện), chỉ có bảng, không biểu đồ.

**Câu hỏi cần trả lời được:**
- *Đo bằng cách nào, có kiểm chứng lại được không?* Cố định seed, đối chiếu ground truth của engine; ai chạy `python benchmark.py` cũng ra cùng kết quả.
- *Kết quả này chưa chứng minh được điều gì?* Chưa chứng minh trên camera thật, ánh sáng thật, vật thật, hay độ bám của tay gắp thật (vì gắp bằng constraint).
- *Impact dựa trên gì?* Chỉ tỉ lệ đúng là đo; năng suất và số giờ công là giả định, đã ghi rõ.

---

## 7. Kịch bản thuyết trình 10 phút

Một câu chuyện liền mạch: **tự động khi chắc chắn, chuyển cho người khi không chắc.**

| Phút | Nội dung | Người nói | Màn hình |
|---|---|---|---|
| 0:00–1:00 | **Bài toán:** phân loại linh kiện thủ công chậm, dễ nhầm; robot truyền thống "mù", cần vật nằm đúng chỗ. **Người dùng:** kỹ thuật viên trạm phân loại, người vận hành robot | D | Slide 1–2 |
| 1:00–2:45 | **AUTO:** robot nhìn, gắp, phân loại. **Mời giám khảo kéo một vật** sang chỗ khác (chuột trái), bấm `r`, robot vẫn tìm thấy và gắp. B chỉ vào cửa sổ camera: khung, tọa độ | A, B | Màn hình demo |
| 2:45–5:00 | **Ca lỗi trực tiếp:** bấm `n` nhiều lần hoặc mở seed 100, hai khối cùng màu sát nhau → HUD đỏ "CẦN NGƯỜI VẬN HÀNH" → C điều khiển bằng tay gắp một khối ra. Bấm `a`, robot làm tiếp. **Mời giám khảo thử tay.** Rút tay ra → robot đứng yên | C | Màn hình demo |
| 5:00–7:00 | **Bằng chứng:** sai số X mm, gắp đúng Y/Z vật, kết quả khi có nhiễu, false alarm của cờ suspicious. Nguyên nhân ca lỗi. Trụ cột. Impact và giới hạn | D (B trả lời phần sai số) | Slide 3–6 |
| 7:00–10:00 | **Hỏi đáp:** mỗi người trả lời phần của mình | Cả team | |

**Nếu demo hỏng:** D mở video ngay, nói rõ "đây là video quay lúc 2:15", A khởi động lại chương trình song song.

**Sáu slide:**
1. Bài toán và người dùng (Vin Robotics × VinFast)
2. Kiến trúc: sơ đồ mục 2, ghi trụ cột ở từng khối
3. Số đo: bảng và 1 biểu đồ, kèm định nghĩa
4. Ca lỗi: ảnh RGB / mask / contour gộp, nguyên nhân, hệ thống phản ứng thế nào, cách khắc phục (distance transform + watershed, hoặc dùng cạnh trong ảnh depth)
5. Impact: công thức, đâu là đo, đâu là giả định; hướng mở rộng; rủi ro
6. Giới hạn và đường sim-to-real

---

## 8. Bản đồ trụ cột (tiêu chí 25 điểm: "Kiến thức nằm ở đâu?")

Giám khảo chấm "đúng bản chất và giải thích vì sao chọn cách đó **thay vì phương án khác**". Nói sâu 3 trụ cột chính, nhắc ngắn 2 trụ cột phụ.

| Trụ cột | Nằm ở đâu | Người | Vì sao chọn, thay vì gì |
|---|---|---|---|
| **Hình học** (chính) | Pixel + depth → 3D bằng nghịch đảo `P·V`. IK cho cánh tay | B, A | Robot cần tọa độ thế giới. Chọn mô hình chiếu đầy đủ **thay vì homography** vì homography chỉ đúng trên một mặt phẳng |
| **Bất định** (chính) | Cờ suspicious (diện tích, tỉ lệ cạnh, độ cao) → chuyển cho người. Hai ngưỡng gắp. Mất tay thì đứng yên | B, A, C | **Thay vì gắp mọi phát hiện**: gắp sai đi tiếp xuống dây chuyền, tốn hơn chờ người vài giây |
| **Bằng chứng** (chính) | Benchmark theo seed, so ground truth, có/không nhiễu, ca lỗi dựng có chủ đích, false alarm của cờ | D | **Thay vì "chạy thử thấy ổn"**: ai chạy lại cũng ra cùng số |
| Biểu diễn (phụ) | `Detection(color, xyz, suspicious)`, máy trạng thái 7 trạng thái, bàn tay 21 mốc | B, A, C | Biểu diễn gọn giúp tách nhận thức, quyết định và điều khiển |
| Thời gian (phụ) | Độ trễ ảnh → lệnh, FPS teleop, EMA, sim 240 Hz đồng bộ với webcam 30 FPS | C, A | EMA một tham số **thay vì Kalman**: đủ cho bài toán, dễ giải thích đánh đổi |

---

## 9. Quy tắc làm việc nhóm

- **Repo:** mỗi người chỉ sửa file của mình. Chỉ A sửa `main.py` và `common.py`. Ai cần đổi chữ ký hàm thì **báo cả team trước**.
- **Git:** `git pull --rebase` trước mỗi lần push, commit nhỏ và thường. A tag `e2e` khi đạt mốc 1:15 và `freeze` lúc 2:15 để luôn quay lại được bản chạy.
- **Kẹt quá 10 phút thì báo ngay.** Kẹt quá 20 phút thì áp dụng mục "Nếu trễ thì cắt" của mình.
- **Một nguồn phím, một nguồn cửa sổ:** chỉ `main.py` gọi `cv2.waitKey` và `cv2.imshow`.
- **Không lưu ảnh webcam** ra đĩa (dữ liệu an toàn). Ảnh debug chỉ lấy từ camera ảo.

---

## 10. Checklist

### 10.1 Trước buổi (tối hôm trước, nếu BTC cho phép)

- [ ] Mọi máy: Python 3.11, `xcode-select --install` (Mac), rồi `pip install -r requirements.txt`. `pybullet` sẽ build từ source trên Mac arm64, mất vài phút (chưa đo chính xác)
- [ ] `requirements.txt` ghim phiên bản (đã kiểm tra giải được phụ thuộc trên Mac arm64, Python 3.11, trừ `pybullet` build từ source):
  ```
  pybullet==3.2.7
  mediapipe==0.10.21        # >=1.0 removed mp.solutions
  opencv-python==4.11.0.86
  numpy==1.26.4             # mediapipe 0.10.21 needs numpy<2
  matplotlib
  ```
- [ ] Máy demo: chạy thử cửa sổ PyBullet GUI **cùng lúc** với cửa sổ `cv2.imshow` và webcam (A, C)
- [ ] macOS: cấp quyền Camera cho Terminal / VS Code (C)
- [ ] Thử kéo vật bằng chuột trong PyBullet GUI, ghi lại thao tác đúng cho giám khảo (A)

### 10.2 Trước khi lên demo (theo đề bài)

- [ ] Demo chạy trên đúng máy sẽ dùng, đã thử webcam ở phòng demo (A, C)
- [ ] Có video dự phòng trên máy và trên USB (D)
- [ ] Có số đo, cả team nói được nó đo như thế nào (D, B)
- [ ] Có ca lỗi kèm nguyên nhân, và hệ thống phản ứng đúng (D, B)
- [ ] Chỉ ra được ít nhất 2 trụ cột và vị trí của chúng trong hệ thống (cả team, mục 8)
- [ ] Không dùng dữ liệu thật của người ngoài team (C)
- [ ] Mỗi thành viên trả lời được vì sao team chọn cách làm này (cả team)
- [ ] Nói được người dùng là ai và họ được gì (D)
- [ ] Có ước lượng impact, ghi rõ đâu là giả định (D)
- [ ] `REPORT.md` khớp với hệ thống đã làm (D)
