# VinSort: Phân công team 4 người

> **Đề tài:** Cánh tay robot phân loại linh kiện bằng thị giác, điều khiển lai Tự động và Cử chỉ tay
> **Đơn vị Vin:** Vin Robotics × VinFast (khâu phân loại linh kiện trên dây chuyền lắp ráp)
> **Hackathon:** Vin Perception Hackathon, Track 4 Capstone

---

## 0. Ràng buộc từ đề bài (đọc trước khi làm)

| Ràng buộc | Ảnh hưởng tới team |
|---|---|
| **Chỉ có 120 phút build** | Cắt bớt phạm vi: **không làm YOLO**, benchmark chỉ chạy khoảng 20 lần, teleop chỉ làm bản tối giản |
| **Mốc kiểm tra ở 1:15** | Phải có **một vòng chạy từ đầu đến cuối** (camera → phát hiện → gắp → thả), dù còn thô |
| **Dùng ít nhất 2 trụ cột** | Team dùng 5 trụ cột: **Hình học, Biểu diễn, Bất định, Thời gian, Bằng chứng** (xem mục 6) |
| **Một số đo, một ca lỗi** | Số đo: **sai số định vị (mm)** và **tỉ lệ gắp thành công**. Ca lỗi: **hai khối cùng màu nằm sát nhau** |
| **Giám khảo đưa đầu vào tại chỗ** | Giám khảo **dùng chuột kéo vật** tới vị trí bất kỳ trong PyBullet (Ctrl + kéo chuột), hoặc **dùng tay mình** để chạy teleop |
| **Dữ liệu an toàn** | Chỉ dùng tay của thành viên team, không quay hay lưu ảnh người khác |
| **Video dự phòng** | Bắt buộc quay trước 2:30 |

**Nguyên tắc vàng:** chạy được thô từ đầu đến cuối trước, đẹp sau. Mỗi người code trong **file riêng của mình** để không đụng nhau.

---

## 1. Tổng quan phân vai

| Vai | Người | File phụ trách | Trụ cột chính | Phần nói khi demo |
|---|---|---|---|---|
| **A: Sim & Điều khiển** (trưởng nhóm, người ghép code) | `________` | `scene.py`, `controller.py`, `main.py` | Biểu diễn (máy trạng thái) | Demo trực tiếp |
| **B: Nhận thức (Perception)** | `________` | `perception.py` | Hình học, Biểu diễn | Pipeline thị giác và số đo sai số |
| **C: Teleop & Tương tác** | `________` | `teleop.py` | Thời gian, Bất định | Demo teleop, cho giám khảo thử |
| **D: Bằng chứng & Thuyết trình** | `________` | `benchmark.py`, `slides`, video | Bằng chứng | Mở đầu (bài toán) và phần số đo, ca lỗi, impact |

---

## 2. Hợp đồng giao tiếp giữa các module (chốt ở phút 0:00–0:15)

Cả team **thống nhất chữ ký hàm này trước khi code**. Ai làm xong trước thì dùng bản giả (stub) của người khác để tự test.

```python
# common.py  (A tạo ngay phút đầu tiên, cả team dùng chung)
from dataclasses import dataclass

COLORS = ["red", "green", "blue"]          # 3 loại linh kiện, mỗi màu một thùng
TABLE_Z = 0.625                            # độ cao mặt bàn (A chỉnh lại theo scene thật)
WORKSPACE = dict(x=(0.35, 0.75), y=(-0.25, 0.25))  # vùng đặt vật / vùng teleop an toàn

@dataclass
class Detection:
    color: str          # "red" | "green" | "blue"
    uv: tuple           # tâm theo pixel (u, v)
    xyz: tuple          # tọa độ thế giới (m)
    yaw: float          # góc xoay của vật (rad) để xoay tay gắp
    area: int           # diện tích theo pixel (dùng để lọc nhiễu và phát hiện ca lỗi)
```

| Hàm | Người viết | Người dùng |
|---|---|---|
| `build_scene(seed, n_objects) -> dict` trả về `{robot, objects:[{id, color}], bins:{color: xyz}}` | A | B, C, D |
| `capture() -> (rgb, depth, seg)` | B | A, D |
| `detect(rgb, depth) -> list[Detection]` | B | A, D |
| `pick_and_place(robot, xyz, yaw, bin_xyz) -> bool` | A | main, D |
| `move_ee(robot, xyz, grip: bool)` (di chuyển tự do, dùng cho teleop) | A | C |
| `HandTracker().read() -> (xyz or None, grip: bool, frame, latency_ms)` | C | main |
| `run_benchmark(n_trials, noise=False) -> csv` | D | slide |

**Quy ước:** tọa độ tính bằng **mét**, góc bằng **radian**, hệ tọa độ thế giới của PyBullet (x hướng ra xa robot, z hướng lên).

---

## 3. Việc cụ thể của từng người

### 🅰️ Người A: Sim & Điều khiển (trưởng nhóm)

**Mục tiêu:** robot gắp được vật tại `(x, y, z)` bất kỳ và thả vào thùng.

| Thời gian | Việc | Xong khi |
|---|---|---|
| 0:15–0:25 | Tạo `common.py`, khung repo, `requirements.txt` | Cả team pull về được |
| 0:25–0:45 | `scene.py`: plane, bàn, Panda (`franka_panda/panda.urdf`, `useFixedBase=True`), 6 khối 3 màu đặt ngẫu nhiên theo `seed`, 3 thùng | Mở GUI thấy đầy đủ, vật nằm yên |
| 0:45–1:05 | `controller.py`: `move_ee` dùng `calculateInverseKinematics` (link `11`), máy trạng thái APPROACH → DESCEND → GRASP → LIFT → MOVE_BIN → RELEASE → HOME | Gắp được 1 vật khi **lấy tọa độ thật** từ engine |
| 1:05–1:15 | **Ghép với `detect()` của B** trong `main.py` | ✅ **Đạt mốc 1:15: vòng chạy đầu-cuối** |
| 1:15–1:45 | Ghép teleop của C, phím `A`/`T`/`R`/`Q`, HUD hiển thị chế độ và số vật đã phân loại | Chuyển chế độ mượt |
| 1:45–2:15 | Sửa lỗi, làm demo ổn định. **Đóng băng code lúc 2:15** | Chạy 3 lần liên tiếp không lỗi |

**Mẹo quan trọng:**
- **Gắp bằng constraint** (giả lập giác hút), đừng gắp bằng ma sát vì rất hay trượt:
  ```python
  # GRASP: khi tay đã chạm vật
  cid = p.createConstraint(robot, 11, obj_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
  # RELEASE
  p.removeConstraint(cid)
  ```
  Tìm `obj_id` gần tay nhất bằng `p.getClosestPoints(robot, obj, 0.03, 11)`. **Không** lấy ID từ bộ phát hiện, như vậy perception vẫn phải làm thật.
- Hướng tay gắp chĩa xuống: `p.getQuaternionFromEuler([np.pi, 0, yaw])`.
- Mỗi bước di chuyển chạy khoảng 120–240 lần `stepSimulation()`. Khi demo thêm `time.sleep(1/240)` để chạy với tốc độ thật.

**Câu hỏi cần chuẩn bị trả lời:** *"Vì sao dùng constraint thay vì gắp bằng ma sát?"* (Mô phỏng tiếp xúc trong PyBullet không ổn định, còn giác hút là loại tay gắp phổ biến cho chi tiết phẳng trong nhà máy.) *"IK giải thế nào, khi nào nó thất bại?"* (Khi điểm nằm ngoài tầm với hoặc gần điểm kỳ dị.)

---

### 🅱️ Người B: Nhận thức (Perception)

**Mục tiêu:** từ ảnh camera ảo, trả về danh sách `Detection` với **tọa độ 3D chính xác**.

| Thời gian | Việc | Xong khi |
|---|---|---|
| 0:15–0:30 | Viết tạm một scene tối giản (bàn và vài khối) để test riêng. `capture()`: camera đặt trên cao nhìn thẳng xuống bằng `computeViewMatrix` và `computeProjectionMatrixFOV` | `cv2.imshow` thấy bàn và các khối |
| 0:30–0:50 | `detect()`: chuyển sang HSV, `inRange` cho từng màu (đỏ cần **2 khoảng hue**), `morphologyEx`, `findContours`, lọc theo diện tích, `moments` để lấy tâm, `minAreaRect` để lấy góc | Vẽ khung và nhãn đúng lên mọi khối |
| 0:50–1:05 | `pixel_to_world()`: nghịch đảo `(Proj @ View)` cùng giá trị depth (công thức bên dưới) | Sai số so với vị trí thật **< 5 mm** |
| 1:05–1:15 | Giao `perception.py` cho A ghép | ✅ Mốc 1:15 |
| 1:15–1:45 | **Lớp kiểm tra độ tin cậy:** đánh dấu phát hiện "nghi ngờ" khi diện tích bất thường (lớn gấp khoảng 1.7 lần bình thường, nghĩa là có thể là 2 vật dính nhau) hoặc độ sâu không hợp lý (không nằm trên mặt bàn). Robot **bỏ qua** những phát hiện nghi ngờ thay vì gắp sai | Ca lỗi được phát hiện và cảnh báo trên HUD |
| 1:45–2:15 | Hỗ trợ D đo sai số định vị, viết 3 dòng giải thích pipeline cho slide | Có số liệu sai số |

**Công thức chuyển pixel sang tọa độ thế giới:**
```python
def pixel_to_world(u, v, d, view, proj, W, H):
    ndc = np.array([(2*u - W)/W, (H - 2*v)/H, 2*d - 1, 1.0])
    P = np.array(proj).reshape(4, 4, order="F")
    V = np.array(view).reshape(4, 4, order="F")
    pw = np.linalg.inv(P @ V) @ ndc
    return pw[:3] / pw[3]
```

**Lưu ý:** **không dùng ảnh `seg` để phát hiện vật**. Ảnh này chỉ dùng làm dữ liệu chuẩn (ground truth) cho D chấm điểm.

**Câu hỏi cần chuẩn bị trả lời:** *"Vì sao dùng HSV thay vì RGB?"* (Hue tách biệt với độ sáng, nên bền hơn khi ánh sáng thay đổi.) *"Sai số định vị đến từ đâu?"* (Độ phân giải pixel, việc làm tròn tâm, và giá trị depth ở mép vật.) *"Camera thật thì khác gì?"* (Cần hiệu chuẩn để có ma trận nội tại K và hệ số méo, cùng một phép biến đổi tay–mắt.)

---

### 🅲 Người C: Teleop & Tương tác

**Mục tiêu:** bàn tay trước webcam thật điều khiển cánh tay robot ảo, đồng thời là **đầu vào để giám khảo tự thử tại chỗ**.

| Thời gian | Việc | Xong khi |
|---|---|---|
| 0:15–0:35 | `HandTracker`: mở webcam, lật ảnh như gương, chạy MediaPipe Hands, vẽ các mốc trên bàn tay | Thấy khung xương bàn tay realtime |
| 0:35–0:55 | Ánh xạ mốc số 9 `(x, y)` vào `WORKSPACE` của robot. Kích thước bàn tay (khoảng cách mốc 0 đến 9) quyết định z. Giới hạn tọa độ trong vùng an toàn | In ra được `xyz` hợp lý |
| 0:55–1:15 | **Phát hiện gắp:** tỉ lệ `khoảng cách(4, 8) / kích thước bàn tay`. Đóng khi dưới `0.25`, mở khi trên `0.40` (**hai ngưỡng** để không nhấp nháy). **Làm mượt bằng EMA:** `pos = 0.7*pos + 0.3*new` | Chụm tay thì in "GRIP" ổn định |
| 1:15–1:45 | Cùng A ghép với `move_ee`. **Đo độ trễ** từ lúc chụp ảnh tới lúc ra lệnh robot (ms) và FPS, hiển thị lên HUD | Điều khiển robot gắp được 1 vật bằng tay |
| 1:45–2:15 | **Xử lý mất tay:** khi không thấy tay hoặc độ tin cậy thấp thì robot **giữ nguyên vị trí** (an toàn khi mất tín hiệu). Tập trước kịch bản cho giám khảo thử | Rút tay ra khỏi khung hình thì robot đứng yên |

**Phương án dự phòng:** nếu MediaPipe chạy chậm hoặc lỗi trên máy demo, điều khiển bằng bàn phím (W/A/S/D để di chuyển, Space để gắp). Phải luôn có cách cho giám khảo tương tác.

**Câu hỏi cần chuẩn bị trả lời:** *"Vì sao cần EMA, đánh đổi là gì?"* (EMA giảm rung nhưng làm tăng độ trễ. Hệ số 0.3 là mức cân bằng đã thử.) *"Độ trễ bao nhiêu thì chấp nhận được?"* *"Mất tay thì robot làm gì?"* (Giữ nguyên vị trí, theo nguyên tắc fail-safe.)

---

### 🅳 Người D: Bằng chứng & Thuyết trình

**Mục tiêu:** có **số đo kiểm chứng được**, **một ca lỗi có phân tích nguyên nhân**, **ước lượng tác động**, slide và video dự phòng.

| Thời gian | Việc | Xong khi |
|---|---|---|
| 0:15–0:40 | Khung slide 6 trang (mục 5). Viết phần bài toán: ai dùng, vấn đề gì. Viết **bảng ước lượng impact** dạng công thức có giả định | Slide 1–2 xong |
| 0:40–1:15 | `benchmark.py` chạy ở chế độ `p.DIRECT` (dùng tọa độ thật trước, ghép `detect` sau). Mỗi lần chạy: sinh scene theo seed, phát hiện vật, so với ground truth, ghi CSV | Chạy được 1 lần, ra CSV |
| 1:15–1:45 | Chạy **khoảng 20 seed**, tính: tỉ lệ phát hiện, **sai số định vị trung bình và lớn nhất (mm)**, **tỉ lệ gắp đúng thùng**, thời gian mỗi chu kỳ. Thêm chế độ `noise=True` (nhiễu Gauss lên ảnh, độ sáng ±30%) để so sánh | Có bảng số và 1 biểu đồ |
| 1:45–2:15 | **Dựng ca lỗi:** seed có 2 khối cùng màu nằm sát nhau. Chụp màn hình bị gộp contour, giải thích nguyên nhân và cách khắc phục. Hoàn thiện slide | Slide xong |
| 2:15–2:30 | **Quay video dự phòng** khoảng 2 phút (auto, teleop, ca lỗi). Chốt số đo. Cả team tập nói một lượt | Có file video trên máy demo và trên USB |

**Bảng ước lượng impact** (điền số thật sau benchmark. Các giả định phải **ghi rõ là giả định** trên slide):
```
Năng suất robot       = 3600 / thời_gian_chu_kỳ (giây)          → [đo từ benchmark] chi tiết/giờ
Số chi tiết phân loại đúng = năng suất × tỉ_lệ_thành_công          → [đo]
Giờ công giải phóng   = số_ca × số_giờ_mỗi_ca × số_trạm       → [GIẢ ĐỊNH, ghi rõ]
Rủi ro khi triển khai : sim-to-real gap, vật bóng hoặc trong suốt, vật chồng lên nhau
Hướng mở rộng         : YOLO train trên dữ liệu tổng hợp từ sim → camera thật hiệu chuẩn → robot thật
```
> ⚠️ **Không bịa số liệu về VinFast.** Thứ gì chưa đo được thì ghi là "giả định" và nói rõ khi thuyết trình. Giám khảo đánh giá cao sự trung thực (tiêu chí Bằng chứng).

**Câu hỏi cần chuẩn bị trả lời:** *"Đo bằng cách nào, có kiểm chứng lại được không?"* (Cố định seed, đối chiếu với ground truth của engine, ai chạy lại cũng ra cùng kết quả.) *"Kết quả này chưa chứng minh được điều gì?"* (Chưa chứng minh trên camera thật, ánh sáng thật hay vật thật.)

---

## 4. Lịch trình tổng hợp (mốc tính từ lúc phát đề)

| Mốc | A: Sim/Điều khiển | B: Perception | C: Teleop | D: Bằng chứng |
|---|---|---|---|---|
| **0:00–0:15** | Chốt hợp đồng hàm (mục 2), chia repo | Cùng chốt | Cùng chốt | Cùng chốt, mở file slide |
| **0:15–0:45** | `common.py`, `scene.py` | `capture()`, `detect()` | Webcam + MediaPipe | Slide bài toán + impact |
| **0:45–1:15** | `controller.py`, ghép E2E | `pixel_to_world` < 5mm | Ánh xạ tay + gắp | Khung `benchmark.py` |
| 🔴 **1:15** | **MỐC KIỂM TRA: vòng chạy đầu-cuối phải chạy được** | | | |
| **1:15–1:45** | Ghép teleop, HUD, phím | Lớp kiểm tra độ tin cậy | Đo độ trễ, ghép `move_ee` | Chạy 20 seed, có noise |
| **1:45–2:15** | Ổn định demo | Hỗ trợ đo | Fail-safe, phương án bàn phím | Ca lỗi, hoàn thiện slide |
| 🔴 **2:15** | **ĐÓNG BĂNG CODE** | | | |
| **2:15–2:30** | Chạy thử trên máy demo | Kiểm tra webcam ở phòng demo | Tập cho giám khảo thử | Quay video, tập nói |

**Nếu đến 1:15 vẫn chưa ghép được:** A cho robot dùng tọa độ thật để demo phần gắp, B demo phát hiện riêng, rồi ghép tiếp ngay sau đó. **Không ai được bắt đầu tính năng mới** khi vòng đầu-cuối chưa chạy.

---

## 5. Kịch bản thuyết trình 10 phút

| Phút | Nội dung | Người nói | Slide hoặc màn hình |
|---|---|---|---|
| 0:00–1:00 | **Bài toán:** công nhân phân loại linh kiện thủ công dễ sai và mệt, robot truyền thống "mù", cần vật nằm đúng chỗ. **Người dùng:** kỹ sư dây chuyền VinFast và người vận hành robot | D | Slide 1–2 |
| 1:00–3:00 | **Demo AUTO:** robot nhìn, gắp và phân loại. **Giám khảo dùng chuột kéo một vật** sang chỗ khác, bấm `R`, robot vẫn tìm thấy và gắp | A (B giải thích khung camera) | Màn hình demo |
| 3:00–5:00 | **Demo TELEOP:** người vận hành điều khiển bằng tay. **Mời giám khảo thử.** Rút tay ra thì robot dừng an toàn | C | Màn hình demo |
| 5:00–7:00 | **Bằng chứng:** sai số X mm, gắp thành công Y/20, kết quả khi có nhiễu. **Ca lỗi:** 2 khối dính nhau bị gộp thành 1, hệ thống phát hiện và bỏ qua. Trụ cột đã dùng. Impact và giới hạn | D (B trả lời phần sai số) | Slide 3–6 |
| 7:00–10:00 | **Hỏi đáp:** mỗi người trả lời phần của mình | Cả team | |

**Sáu slide:**
1. Bài toán và người dùng (VinFast / Vin Robotics)
2. Kiến trúc hệ thống: sơ đồ pipeline, ghi rõ trụ cột nằm ở khối nào
3. Số đo: bảng và 1 biểu đồ
4. Ca lỗi: ảnh chụp, nguyên nhân, cách khắc phục
5. Impact: công thức có giả định, hướng mở rộng, rủi ro
6. Giới hạn và hướng sim-to-real

---

## 6. Bản đồ trụ cột (để trả lời câu "Kiến thức nằm ở đâu?", tiêu chí 25 điểm)

| Trụ cột | Nằm ở đâu trong VinSort | Người phụ trách | Vì sao chọn cách này |
|---|---|---|---|
| **Hình học** | Chuyển pixel + depth sang 3D bằng nghịch đảo ma trận chiếu và ma trận view. IK cho cánh tay | B, A | Robot cần tọa độ thế giới, không cần tọa độ pixel |
| **Biểu diễn** | Thế giới được mã hóa thành `Detection(color, xyz, yaw)`. Robot hoạt động theo máy trạng thái 7 trạng thái. Bàn tay biểu diễn bằng 21 mốc | B, A, C | Biểu diễn gọn giúp tách rời các khối nhận thức, lập kế hoạch và điều khiển |
| **Bất định** | Lọc theo diện tích và độ sâu để đánh dấu phát hiện nghi ngờ. Hai ngưỡng cho thao tác gắp. Robot đứng yên khi mất tay | B, C | Thà bỏ qua còn hơn gắp sai |
| **Thời gian** | Đo độ trễ từ ảnh tới lệnh điều khiển, FPS teleop. Đánh đổi giữa độ mượt EMA và độ trễ | C | Teleop chỉ dùng được khi độ trễ thấp |
| **Bằng chứng** | Benchmark theo seed cố định, đối chiếu ground truth, so sánh có và không có nhiễu, ca lỗi | D | Kiểm chứng lại được, không dựa trên cảm giác |

---

## 7. Quy tắc làm việc nhóm

- **Repo:** mỗi người code trong file riêng. Chỉ A sửa `main.py` và `common.py`. Ai cần đổi chữ ký hàm thì **báo cả team trước**.
- **Cài sẵn môi trường trước buổi** (nếu BTC cho phép): `pip install pybullet opencv-python mediapipe numpy matplotlib`, rồi test webcam trên **đúng máy sẽ dùng để demo**.
- **Đồng bộ nhanh 2 phút** ở các mốc 0:45, 1:15, 1:45: mỗi người nói xong gì, đang kẹt ở đâu.
- **Kẹt quá 10 phút thì báo ngay**, đừng ngồi một mình.
- **Lưu bản chạy được:** mỗi khi một tính năng chạy, commit hoặc copy thư mục sang `backup_<giờ>/`.

---

## 8. Checklist trước khi lên demo (theo đề bài)

- [ ] Demo chạy trên đúng máy sẽ dùng, đã thử webcam ở phòng demo (A, C)
- [ ] Có video dự phòng trên máy và trên USB (D)
- [ ] Có số đo, cả team nói được nó đo như thế nào (D, B)
- [ ] Có ca lỗi kèm nguyên nhân (D, B)
- [ ] Chỉ ra được ít nhất 2 trụ cột và vị trí của chúng trong hệ thống (cả team, mục 6)
- [ ] Không dùng dữ liệu thật của người ngoài team (C)
- [ ] Mỗi thành viên trả lời được vì sao team chọn cách làm này (cả team)
- [ ] Nói được người dùng là ai và họ được gì (D)
- [ ] Có ước lượng impact, ghi rõ đâu là giả định (D)
