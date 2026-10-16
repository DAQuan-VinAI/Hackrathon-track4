# Bàn giao D: Bằng chứng & Thuyết trình

**Trần Thị Lan** · VinSort · Track 4 · 16/10/2026

## Mở để trình bày

1. `VinSort_6_slides.pptx`: 6 trang, bảng và biểu đồ có thể chỉnh sửa, lời dẫn trong Notes.
2. `VinSort_demo.mp4`: video dự phòng 2 phút, phụ đề đóng sẵn, không có tiếng. Dựng từ PyBullet,
   đầu vào tay theo kịch bản. Video không phải bản ghi thao tác webcam người thật.
3. `KICH_BAN.md`: lời dẫn, lịch trình 7 phút nội dung/demo và 3 phút hỏi đáp.
4. `../REPORT.md`: báo cáo, định nghĩa chỉ số, kết quả và giả định impact.

## Kiểm tra bằng chứng

- `results/detect.csv`, `detect_noise.csv`: 20 seed thường và 5 seed lỗi cho mỗi chế độ ảnh.
- `results/pick.csv`: 20 seed thường và 5 seed lỗi, ảnh sạch.
- `results/pick_noise.csv`: 20 seed thường, ảnh nhiễu.
- `results/xy_pairs*.csv`: sai số từng cặp ghép, có thể tính lại trung vị và biểu đồ.
- `results/xy_error.png`: biểu đồ phân bố sai số XY sạch và nhiễu.
- `bang_so_lieu.csv`: bảng tổng hợp UTF-8 có BOM, mở bằng Excel.
- `metrics.json`: số đầu vào để dựng slide, tính từ CSV.
- `results/manifest.json`, `environment.txt`: phiên bản môi trường và hash nguồn benchmark.
- `test_log.txt`: kết quả các bài kiểm thử có sẵn.
- `debug/`: camera ảo seed 0 và ba ảnh phân tích lỗi seed 100.
- `video_events.json`: mốc thời gian, chế độ và số vật đúng thùng trong lần dựng video.

Chỉ có số đo mô phỏng. Chưa đo độ chính xác cử chỉ với người, độ trễ webcam, tốc độ hoặc độ bám robot thật.
Tình huống impact 6 giây/vật là giả định minh họa của nhóm, chưa có nguồn vận hành VinFast.

## Chép sang USB

Chép `VinSort_D_ban_giao.zip` ra USB, giải nén rồi mở thử PPTX và MP4 ngay trên USB.
Lần bàn giao này chưa thấy USB kết nối nên chưa xác nhận sao chép vật lý.
Trong gói ZIP, báo cáo và các đường dẫn nguồn giữ nguyên cấu trúc thư mục.

## Chạy lại

Từ thư mục dự án trên Windows có WSL Ubuntu:

```powershell
powershell -ExecutionPolicy Bypass -File presentation_tools/run_evidence.ps1 -Video
```

Lệnh chạy headless và không cần webcam. Để chạy demo GUI/webcam cần môi trường đồ họa và các
thư viện theo `vinsort/requirements.txt`. Máy Windows hiện chưa có C++ Build Tools để build PyBullet.
Các script dựng slide sử dụng runtime đi kèm Codex, ghi tại `presentation_tools/build_slides.mjs`.
