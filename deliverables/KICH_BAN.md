# Lời dẫn và lịch trình 10 phút

Phụ trách D: **Trần Thị Lan**. Thời gian nói và demo tổng cộng 7 phút, hỏi đáp 3 phút.

| Mốc | Người | Nội dung / màn hình |
|---|---|---|
| 0:00–0:30 | Lan | Slide 1. Bài toán, kỹ thuật viên trạm phân loại, bối cảnh Vin Robotics × VinFast |
| 0:30–1:00 | Lan | Slide 2. Camera ảo, nhận thức, độ tin cậy, điều khiển và vai trò người |
| 1:00–2:45 | A + B | Demo AUTO, kéo một vật, quét lại, chỉ tọa độ trong HUD |
| 2:45–5:00 | C | Seed 100, cảnh nhập nhằng, TELEOP, mất tay giữ vật, phím a về AUTO |
| 5:00–5:40 | Lan | Slide 3. Đọc bảng số và biểu đồ, giải thích cách đo |
| 5:40–6:10 | Lan | Slide 4. RGB, mask, contour gộp, phản ứng và cách khắc phục |
| 6:10–6:35 | Lan | Slide 5. Impact là kịch bản giả định |
| 6:35–7:00 | Lan | Slide 6. Giới hạn, bước tiếp theo |
| 7:00–10:00 | Cả nhóm | Hỏi đáp theo phần phụ trách |

## Mở đầu, slide 1

“Người dùng nhóm hướng tới là kỹ thuật viên trạm phân loại linh kiện. Khi vật thay đổi vị trí hoặc
nằm sát nhau, hệ thống cần nhìn lại và biết lúc nào nên gọi người. VinSort minh họa cơ chế đó bằng
cánh tay robot trong PyBullet. Bối cảnh ứng dụng đề xuất là Vin Robotics và VinFast.”

## Kiến trúc, slide 2

“Camera ảo cung cấp ảnh màu và độ sâu. Nhóm tách màu bằng HSV, tìm contour và chuyển tâm pixel
thành tọa độ 3D bằng mô hình camera. Robot dùng IK để gắp vật rõ. Khi contour bất thường, hệ thống
chuyển TELEOP. Người vận hành tách vật rồi bấm a để robot nhìn lại. Webcam thật chỉ nhìn bàn tay.”

Chuyển lời cho A: “Anh Quân sẽ demo AUTO, Dũng giải thích phần nhận thức, sau đó C demo can thiệp.”
Tên A/B/C cần đối chiếu với phân công nhóm khi tập nói, không gán thay vai trò các thành viên.

## Bằng chứng, slide 3

Đọc số trực tiếp từ slide và REPORT.md của lần chạy. Nói đủ mẫu số và đơn vị:
“Đây là 20 scene thường, tổng cộng 120 vật. Recall ghép cùng màu, gần nhất theo XY trong 20 mm.
Sai số XY chỉ tính trên cặp đã ghép. Đúng thùng tính trên mọi vật sau lượt phân loại.
Nhiễu chỉ đổi RGB, depth giữ nguyên. Màu sim bão hòa nên kết quả chưa chứng minh được ánh sáng thật.”

## Ca lỗi, slide 4

“Hai vật cùng màu sát nhau tạo một mask và contour lớn. Diện tích vượt ngưỡng làm hệ thống gắn cờ
area. Robot xếp vật rõ trước rồi chuyển người. Hai vật chờ can thiệp là phần chưa tự động giải quyết.
Kiểm thử phục hồi và video dùng tay theo kịch bản. Nhóm chưa coi đó là kiểm thử webcam người thật.”

## Impact, slide 5

“Tỉ lệ đúng là số đo mô phỏng. Chu kỳ sáu giây cho robot dự kiến là giả định của nhóm. Vì vậy 600 vật
mỗi giờ chỉ là năng suất lý thuyết, chưa trừ dừng máy và can thiệp. Chưa có baseline người thật để tính
ROI hoặc giờ công thực sự tiết kiệm. Đây không phải số liệu vận hành của VinFast.”

## Giới hạn, slide 6

“Hiện nhóm kiểm chứng pipeline và cơ chế bàn giao trong sim. Camera thật, vật kim loại và độ bám
tay gắp đều cần kiểm thử tiếp. Bước tiếp theo là thu dữ liệu, hiệu chuẩn camera, đo can thiệp và thử
tay gắp thật với cơ chế an toàn phần cứng.”

## Nếu demo trực tiếp hỏng

Mở `VinSort_demo.mp4`, đọc: “Đây là video dự phòng dựng từ lần chạy PyBullet. Video dùng teleop theo
kịch bản, không thu webcam. Video thể hiện AUTO, vật đổi vị trí, ca lỗi, giữ vật khi mất tay và phục hồi.”
Video có chữ trên hình, không có tiếng. Lan thuyết minh theo từng đoạn. Không nói là giám khảo thật
đã kéo vật hoặc có người đang điều khiển webcam trong đoạn video này.

## Hỏi đáp dự kiến

- **Sao không dùng YOLO?** Ba màu trong sim đủ cho HSV, nhóm ưu tiên hình học, bất định và cơ chế phục hồi.
- **100% có ý nghĩa gì?** Kết quả trên mẫu mô phỏng nhỏ, gắp constraint và màu bão hòa. Chưa khẳng định với vật thật.
- **Ca lỗi có thất bại không?** AUTO chỉ xếp vật rõ, hai vật còn lại cần người. Báo riêng kết quả AUTO và phục hồi.
- **Có tiết kiệm 8 giờ công không?** Tám giờ chỉ là giờ vận hành trong kịch bản. Cần baseline và thời gian giám sát để tính tiết kiệm.
- **Có chạy lại được không?** Dùng `run_evidence.ps1`, CSV và hash nguồn trong manifest. Kết quả có thể khác chút theo engine/nền tảng.

## Checklist trước khi lên sân khấu

- Mở trước PPTX, video và báo cáo. Kiểm tra tiếng Việt trên máy trình chiếu.
- Chạy GUI/webcam trên máy demo, kiểm tra phím a/t/r và camera. Lần bàn giao này mới kiểm chứng headless.
- Sao chép `VinSort_D_ban_giao.zip` vào USB, giải nén và thử mở video trực tiếp từ USB.
- Tập đúng 7 phút trước hỏi đáp. Số đo phải đọc từ slide hiện tại.
