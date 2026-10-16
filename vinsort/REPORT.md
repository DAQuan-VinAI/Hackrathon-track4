# VinSort — Báo cáo

**Đề tài:** Cánh tay robot phân loại linh kiện bằng thị giác, điều khiển lai Tự động + Cử chỉ tay
**Đơn vị:** Vin Robotics × VinFast (khâu phân loại linh kiện trên dây chuyền lắp ráp)
**Hackathon:** Vin Perception Hackathon, Track 4 Capstone

## 1. Bài toán và người dùng

Phân loại linh kiện thủ công chậm, dễ nhầm. Robot truyền thống "mù", cần vật nằm đúng chỗ.
Người dùng: kỹ thuật viên trạm phân loại, người vận hành robot.

**Câu chuyện sản phẩm:** robot tự phân loại khi chắc chắn. Khi không chắc (hai linh kiện dính nhau),
robot không đoán bừa mà dừng, chuyển cho người vận hành điều khiển bằng cử chỉ tay, không cần chạm vào máy.

## 2. Kiến trúc (đã làm, toàn bộ trong mô phỏng)

| Khối | File | Việc |
|---|---|---|
| Thế giới | `scene.py` | PyBullet: Franka Panda gốc (0,0,0), 6 khối 4 cm, 3 màu trên mặt đất (`TABLE_Z=0`), 3 thùng ngoài khung hình camera |
| Nhận thức | `perception.py` | Camera ảo nhìn thẳng xuống (cao 1 m, FOV 30°, 640×480, `ER_TINY_RENDERER`) → HSV → mask → contour → tâm → nghịch đảo P·V + depth → xyz. Kiểm tra tin cậy: diện tích, tỉ lệ cạnh, độ cao → `suspicious` |
| Điều khiển | `controller.py` | IK tại link 11, đường thẳng Cartesian từng đoạn 2 cm, gắp bằng constraint (giả lập giác hút) giữ nguyên pose tương đối. FSM: HOME → APPROACH → DESCEND → GRASP → LIFT → MOVE_BIN → RELEASE → HOME |
| Người vận hành | `teleop.py` | Webcam → MediaPipe Hands (1 tay) → mốc 9 = xy, cỡ bàn tay = z, chụm ngón 4–8 hai ngưỡng = gắp, EMA 0.3, giới hạn 1.5 cm/frame, mất tay = giữ nguyên. Dự phòng bàn phím |
| Điều phối | `main.py` | AUTO ↔ TELEOP + HUD OpenCV. Phím đọc từ cả HUD và cửa sổ PyBullet |
| Bằng chứng | `benchmark.py` | `p.DIRECT`, seed cố định, so với ground truth của engine |

Không dùng YOLO, ArUco hay webcam để nhìn vật: vật được nhìn bằng camera ảo trong mô phỏng.
Webcam thật chỉ dùng cho bàn tay người vận hành.

## 3. Số đo (sim, seed 0–19 bình thường + 100–104 ca lỗi)

Chạy lại: `python benchmark.py` → `results/summary.txt`, CSV, `results/xy_error.png`.

| Số đo | Sạch | Có nhiễu |
|---|---|---|
| Recall phát hiện | 120/120 = 100% | 120/120 = 100% |
| False positive | 0 | 0 |
| Sai số XY (mm), trung bình / trung vị / lớn nhất | 2.55 / 2.63 / 4.82 | 2.55 / 2.63 / 4.82 |
| Sai số z (mm), mặt trên so với `CUBE` | −0.01 | −0.01 |
| Cờ suspicious đúng trên seed ca lỗi | 5/5 | 5/5 |
| Cờ suspicious báo nhầm trên seed thường | 0 | 0 |
| Tỉ lệ gắp đúng thùng (seed thường) | 120/120 = 100% | 120/120 = 100% |
| Gắp hụt | 0 | 0 |
| Ca lỗi: vật robot tự xếp đúng / vật chuyển cho người | 20/30, 10 vật (5 cặp) | — |
| Thời gian chu kỳ trong sim | 4.44 s/vật | — |

**Đọc số cho trung thực:**

- **Nhiễu không ảnh hưởng gì, và đó là giới hạn của sim, không phải điểm mạnh.** Nhiễu có được áp dụng
  (độ sáng ×U[0.7, 1.3] + Gauss σ=8, pixel lệch trung bình 40/255), nhưng mask HSV không đổi 1 pixel nào:
  khối trong sim có màu bão hòa tuyệt đối (S=255, V 153–239), cách xa ngưỡng. Kết luận đúng: HSV bền với
  nhiễu độ sáng **khi màu bão hòa**; chưa nói được gì cho linh kiện thật (kim loại xám, bóng, bẩn).
- Sai số XY 2.55 mm đến từ độ phân giải (~1.1 mm/pixel tính từ hình học) và phối cảnh làm lộ mặt bên ở rìa khung.
- Tỉ lệ gắp 100% đo **định vị + điều khiển**, không đo độ bám: gắp bằng constraint, không mô phỏng ma sát / lực hút.
- Thời gian chu kỳ phụ thuộc tham số team tự đặt (2 cm/đoạn, 4 bước/đoạn, 240 Hz). **Không phải tốc độ robot thật.**
- Ca lỗi seed 100–104: 2 vật/seed bị đánh dấu `area` (diện tích ≈ 1.8 × `A_REF`=1510 px) → robot không gắp →
  chuyển TELEOP. 20/30 là đúng thiết kế: 10 vật còn lại là phần người vận hành xử lý.

## 4. Ca lỗi: hai khối cùng màu nằm sát nhau

Ảnh: `debug/fail_rgb.png`, `debug/fail_mask.png`, `debug/fail_contour.png` (tạo bằng `python perception.py`).

- **Nguyên nhân:** mask HSV theo màu không tách được hai vùng cùng màu chạm nhau; `findContours` trả về một contour gộp,
  tâm rơi vào khe giữa hai khối. Gắp tại đó: hoặc trượt, hoặc kéo nhầm một khối.
- **Hệ thống phản ứng:** diện tích 2736 px > 1.6 × 1510 → `suspicious`, `reason="area"` → robot bỏ qua, xếp hết vật chắc chắn
  → HUD đỏ "CẦN NGƯỜI VẬN HÀNH: area" → TELEOP. Người vận hành tách một khối ra bằng tay, bấm `a`, robot quét lại và xếp nốt.
  Kiểm chứng tự động: `test_full_hybrid_story` trong `test_vinsort.py`.
- **Dự phòng thêm:** vật nào gắp hụt một lần bị đánh dấu `grasp_fail`, không thử lại mãi.
- **Khắc phục lâu dài:** distance transform + watershed để tách vùng dính, hoặc dùng cạnh trong ảnh depth.

## 5. Trụ cột

| Trụ cột | Ở đâu | Vì sao chọn |
|---|---|---|
| **Hình học** | Pixel + depth → 3D bằng nghịch đảo P·V; IK tại link 11 | Homography chỉ đúng trên một mặt phẳng; sim có sẵn depth nên nghịch đảo mô hình chiếu là đúng bản chất |
| **Bất định** | Cờ `suspicious` (diện tích, tỉ lệ cạnh, độ cao, gắp hụt) → chuyển cho người; hai ngưỡng gắp; mất tay đứng yên | Gắp sai đi tiếp xuống dây chuyền, tốn hơn chờ người vài giây |
| **Bằng chứng** | Benchmark theo seed, so ground truth, có/không nhiễu, ca lỗi dựng có chủ đích, đo cả báo nhầm | Ai chạy lại `python benchmark.py` cũng ra cùng số |
| Biểu diễn | `Detection(color, xyz, suspicious, reason)`, FSM 7 trạng thái, bàn tay 21 mốc | Tách nhận thức, quyết định, điều khiển |
| Thời gian | Độ trễ ảnh → lệnh trên HUD, EMA, sim 240 Hz với webcam ~30 FPS (8 bước/frame) | EMA một tham số thay Kalman: đủ, dễ giải thích đánh đổi trễ/rung |

## 6. Impact (ước lượng)

```text
Tỉ lệ phân loại đúng   = 100% trên 120 vật sim (ĐO, sim, màu bão hòa)
Năng suất robot        = [GIẢ ĐỊNH — điền chu kỳ gắp-thả robot công nghiệp, ghi nguồn]
Số chi tiết đúng / giờ = năng suất × tỉ lệ đúng                 (ƯỚC LƯỢNG)
Giờ công giải phóng    = số ca × giờ/ca × số trạm                (GIẢ ĐỊNH)
```

Không có số liệu VinFast nào trong báo cáo này. Mọi dòng không ghi "ĐO" là giả định.

## 7. Giới hạn và đường sim-to-real

Chưa chứng minh: camera thật, ánh sáng nhà xưởng, linh kiện thật (kim loại, bóng, trong suốt, chồng nhau), độ bám tay gắp thật,
người và robot chung vùng làm việc. Teleop chỉ mô phỏng: chưa có kênh dừng khẩn cấp phần cứng.

Đường đi tiếp: YOLO train trên dữ liệu tổng hợp từ sim → camera thật hiệu chuẩn (ma trận K, méo, hand-eye) → robot thật.

## 8. Dữ liệu an toàn

Webcam chỉ dùng tay thành viên team, `max_num_hands=1`. Không ghi ảnh webcam ra đĩa. Ảnh debug chỉ lấy từ camera ảo.
