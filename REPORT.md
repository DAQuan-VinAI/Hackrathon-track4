# VinSort: Cánh tay robot phân loại linh kiện bằng thị giác, điều khiển lai tự động và cử chỉ tay

**Vin Perception Hackathon — AI20k · Track 4 · Capstone**

## Nhóm: Clauder tập xài Codex

| Thành viên | MSHV | Vai trò |
|---|---|---|
| Đoàn Anh Quân | 2A202602803 | _[điền vai trò]_ |
| Hoàng Đức Dũng | 2A202602798 | _[điền vai trò]_ |
| Trần Thị Lan | 2A202602621 | _[điền vai trò]_ |
| Đặng Thái Anh | 2A202602740 | _[điền vai trò]_ |

> Các ô `[...]` là chỗ cần điền sau khi build và đo. Không điền số chưa đo được.

---

## 1. Bài toán và người dùng

- **Đơn vị Vin:** Vin Robotics (ứng dụng cho dây chuyền lắp ráp linh kiện, ví dụ kho linh kiện VinFast).
- **Người dùng:** công nhân / kỹ thuật viên tại trạm phân loại linh kiện nhỏ (ốc vít, đai ốc, vòng đệm, đầu nối...).
- **Nỗi đau:** phân loại thủ công chậm, dễ nhầm khi linh kiện giống nhau; robot tự động thuần thì "đứng hình" khi gặp vật lạ hoặc ca khó, và người vận hành không có cách can thiệp nhanh mà không dừng máy.
- **Giải pháp:** VinSort dùng camera nhận dạng linh kiện, định vị trên mặt bàn và điều khiển cánh tay gắp – thả vào đúng khay. Khi hệ thống không chắc chắn, người vận hành điều khiển trực tiếp bằng cử chỉ tay, không cần chạm vào máy.

## 2. Kiến trúc hệ thống

```
Camera (webcam) ──► Detection (YOLO pretrained / fine-tune) ──► box + class + confidence
        │                                                            │
        └──► ArUco markers ──► Homography (pixel → mặt bàn, mm) ─────┤
                                                                     ▼
Camera (webcam) ──► MediaPipe Hands ──► gesture ──► Mode arbiter (AUTO / MANUAL)
                                                                     │
                                                                     ▼
                                       Inverse kinematics ──► Cánh tay (sim / thật)
```

- **Đầu vào:** _[webcam laptop / camera trên cao]_
- **Cánh tay:** _[PyBullet sim / cánh tay thật — chốt một]_
- **Giao diện demo:** _[cửa sổ OpenCV / Streamlit]_

### Chế độ điều khiển lai

| Chế độ | Khi nào | Hành vi |
|---|---|---|
| AUTO | Mặc định, confidence ≥ ngưỡng `τ` | Chọn vật, tính tọa độ mặt bàn, gắp – thả vào khay theo class |
| MANUAL | Người vận hành giơ cử chỉ "dừng", hoặc confidence < `τ` | Cánh tay đi theo vị trí bàn tay; cử chỉ nắm / mở để gắp / thả |
| Chuyển về AUTO | Cử chỉ "tiếp tục" giữ ổn định `N` frame | Tiếp tục hàng đợi |

Bộ cử chỉ: _[liệt kê, ví dụ: bàn tay mở = dừng, nắm tay = gắp, chữ V = tiếp tục]_.

## 3. Trụ cột Track 4 đã dùng

| Trụ cột | Nằm ở đâu trong hệ thống | Vì sao chọn cách này |
|---|---|---|
| **Hình học** | ArUco + homography đổi pixel sang tọa độ mặt bàn (mm), rồi sang hệ tọa độ gốc cánh tay | Vật nằm trên một mặt phẳng nên homography đủ, không cần ước lượng độ sâu; rẻ và kiểm chứng được bằng sai số mm |
| **Bất định** | Ngưỡng confidence `τ` quyết định AUTO hay chuyển sang người; làm mượt landmark bàn tay (EMA / Kalman) | Robot không nên gắp khi không chắc; con người xử lý ca khó thay vì hệ thống đoán bừa |
| **Biểu diễn** | Linh kiện = box + tâm (x, y) trên mặt bàn; bàn tay = 21 landmark → trạng thái cử chỉ | Biểu diễn tối giản đủ cho gắp – thả và phân loại |
| **Thời gian** | Debounce cử chỉ `N` frame; đo độ trễ từ frame đến lệnh cánh tay | Tránh chuyển chế độ do một frame nhiễu |
| _[Học / Dữ liệu nếu có]_ | _[ví dụ: fine-tune YOLO trên ảnh tự chụp]_ | |

## 4. Bằng chứng

### 4.1 Số đo

| Số đo | Cách đo | Kết quả |
|---|---|---|
| Sai số định vị (mm) | Đặt vật tại `[n]` điểm đã biết tọa độ trên mặt bàn, so với tọa độ tính từ homography | _[mean ± std]_ |
| Tỷ lệ phân loại đúng | `[n]` lượt gắp – thả, đếm số vật vào đúng khay | _[x / n]_ |
| Độ chính xác cử chỉ | `[n]` lần mỗi cử chỉ, đếm nhận đúng; đếm số lần chuyển chế độ ngoài ý muốn (false trigger) | _[...]_ |
| Độ trễ | Thời gian từ frame camera đến khi gửi lệnh cánh tay | _[ms, FPS]_ |

### 4.2 Ca lỗi

- **Hiện tượng:** _[mô tả, ví dụ: hai linh kiện chạm nhau bị gộp thành một box; vật bóng phản sáng bị nhận sai class]_
- **Nguyên nhân:** _[phân tích]_
- **Hệ thống phản ứng thế nào:** _[confidence thấp → chuyển MANUAL?]_
- **Cách khắc phục nếu có thêm thời gian:** _[...]_

### 4.3 Giới hạn

- Chỉ kiểm thử trên `[n]` loại linh kiện, ánh sáng phòng lab, mặt bàn phẳng.
- Homography chỉ đúng cho vật nằm phẳng trên mặt bàn; vật cao hoặc chồng nhau sẽ lệch.
- _[Sim: chưa kiểm chứng trên cánh tay thật.]_
- Số đo trên tập nhỏ, chưa đủ để kết luận cho dây chuyền thật.

## 5. Độ hữu ích và Impact (ước lượng)

Công thức ước lượng, số liệu đầu vào là giả định cần kiểm chứng:

```
Thời gian tiết kiệm/ngày = (số linh kiện/ngày) × (thời gian phân loại tay − thời gian VinSort)
Lỗi giảm/ngày           = (số linh kiện/ngày) × (tỷ lệ lỗi tay − tỷ lệ lỗi VinSort)
```

| Tham số | Giá trị giả định | Nguồn |
|---|---|---|
| Số linh kiện / trạm / ngày | _[...]_ | giả định |
| Thời gian phân loại tay / linh kiện | _[...]_ | giả định / tự đo |
| Thời gian VinSort / linh kiện | _[...]_ | đo trong demo |

- **Đường mở rộng:** thêm class linh kiện bằng fine-tune; camera cố định trên băng chuyền; nhiều cánh tay dùng chung một bộ hiệu chuẩn.
- **Rủi ro khi triển khai:** an toàn khi người và cánh tay cùng vùng làm việc; ánh sáng nhà xưởng; cử chỉ bị kích hoạt nhầm khi công nhân thao tác gần camera.

## 6. Dữ liệu an toàn

Toàn bộ dữ liệu (ảnh linh kiện, cử chỉ tay) do thành viên nhóm tự tạo. Không dùng dữ liệu thật của bệnh nhân hay sinh viên.

## 7. Kế hoạch trình bày (10 phút)

| Thời lượng | Nội dung | Người trình bày |
|---|---|---|
| 1 phút | Bài toán Vin Robotics và người dùng | _[...]_ |
| 4 phút | Demo trực tiếp: AUTO phân loại → cử chỉ chuyển MANUAL → quay lại AUTO | _[...]_ |
| 2 phút | Số đo, ca lỗi, trụ cột, impact | _[...]_ |
| 3 phút | Hỏi đáp | Cả nhóm |

Video dự phòng: _[link]_

## 8. Chạy lại

```bash
# [điền lệnh cài đặt và chạy]
```
