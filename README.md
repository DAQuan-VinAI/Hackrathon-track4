## Track 4 Project Capstone — Vin Perception Hackathon
Trang đề bài hackathon cuối khóa Track 4 (Computer Vision & Robotics): 2 giờ build, 10 phút demo mỗi team, đề tài gắn với hệ sinh thái Vin.

- `index.html`: trang đề bài (một file, không cần build). Mở trực tiếp bằng trình duyệt.

Nội dung gồm hướng đề tài gợi ý, bốn yêu cầu của mỗi demo, bảy trụ cột Track 4, công cụ gợi ý, thang điểm, lịch trình, checklist trước demo và FAQ. Ngày giờ, địa điểm, giám khảo, giải thưởng sẽ được ban tổ chức cập nhật.

## Bàn giao phần D: Bằng chứng & Thuyết trình

Phụ trách: **Trần Thị Lan**. Bản mô phỏng hoàn chỉnh dùng để đo là [`vinsort/`](vinsort/),
khác với prototype và stub ở thư mục gốc.

- [Báo cáo số đo chạy lại](REPORT.md)
- [Slide 6 trang](deliverables/VinSort_6_slides.pptx)
- [Video dự phòng khoảng 2 phút](deliverables/VinSort_demo.mp4), đầu vào tay theo kịch bản, có chữ trên hình
- [CSV thô, biểu đồ và môi trường chạy](deliverables/results/)
- [Bảng số liệu tổng hợp](deliverables/bang_so_lieu.csv)
- [Lời dẫn và lịch trình thuyết trình](deliverables/KICH_BAN.md)
- [Gói để chép sang USB](deliverables/VinSort_D_ban_giao.zip)

Chạy lại trên Windows có WSL Ubuntu:

```powershell
powershell -ExecutionPolicy Bypass -File presentation_tools/run_evidence.ps1
# Thêm -Video để dựng lại video từ engine.
```

`benchmark.py` ở gốc chuyển tới benchmark trong `vinsort/`. Dùng Python 3.11 trên Linux để
cài wheel PyBullet 3.2.7. Máy bàn giao thiếu C++ Build Tools nên chưa kiểm thử GUI/webcam native Windows.
Video và bài kiểm thử headless không thay thế kiểm thử người dùng thật.
