# VinSort

Cánh tay robot phân loại linh kiện bằng thị giác, điều khiển lai Tự động + Cử chỉ tay.
Vin Robotics × VinFast. Mô phỏng PyBullet (Franka Panda), camera ảo nhìn thẳng xuống, MediaPipe Hands cho người vận hành.

## Cài

```bash
python -m venv .venv
.venv\Scripts\activate          # Mac/Linux: source .venv/bin/activate
pip install -r requirements.txt
```

## Chạy

Trên máy Windows demo: chạy `start_vinsort.bat` thay vì `python main.py` để PyBullet ưu tiên RTX 3060.
Kiểm tra console: `Renderer = NVIDIA ...`. Launcher Windows x64 dùng Python 3.11; venv mới cần chạy
`build_gpu_launcher.bat` (yêu cầu Visual Studio 2022 C++ tools + Python 3.11 development files).
Không có GPU launcher, file khởi động dùng Python bình thường; MediaPipe vẫn chạy CPU.

| Lệnh | Việc |
|---|---|
| `python main.py` | Demo: PyBullet GUI + HUD OpenCV + webcam |
| `python main.py --seed 100` | Ca lỗi: hai khối cùng màu dính nhau, robot chuyển cho người |
| `python main.py --keyboard` | Teleop bằng bàn phím (không webcam) |
| `python test_vinsort.py` | Tự kiểm headless (~2 phút) |
| `python benchmark.py` | Benchmark đầy đủ → `results/` |
| `python benchmark.py --quick` | 10 seed, bỏ pick có nhiễu |
| `python teleop.py` | Thử webcam + MediaPipe riêng |
| `python perception.py` | Lưu `debug/cap.png`, đo lại `A_REF` |

## Phím (HUD hoặc cửa sổ PyBullet đều nhận)

`a` AUTO · `t` TELEOP · `r` quét lại · `n` thêm vật · `q` thoát.
Teleop bàn phím: `i/k` xa/gần, `j/l` trái/phải, `u/o` lên/xuống, `space` gắp/thả.
Teleop nay với tới thùng ở `y=-0.50`: sau khi gắp, nâng tay lên ~0.30 m trước khi di chuyển sang thùng;
hệ thống tự giữ cao khi vừa cầm vật vừa đi ngang. Đến thùng thì hạ, rồi thả.

**Multi-View bật sẵn:** một cửa sổ hiển thị đồng thời 4 góc: bên / tay gắp / trước / trên.
Cả 4 góc cập nhật cùng lúc gần realtime (~20 FPS trên RTX 3060; 128×96 mỗi ô, cửa sổ phóng to nên ảnh mềm).
Bấm `v` để tắt/bật cả cửa sổ.
`1/2` xoay · `3/4` nghiêng · `5/6` zoom · `0` về góc mặc định ở cửa sổ PyBullet.
Tắt hẳn multi-view nếu máy vẫn lag: `set VINSORT_MV=0` trước khi chạy.

Giám khảo kéo vật: **chuột trái kéo thẳng** trong cửa sổ PyBullet (Ctrl + kéo là xoay camera).

## Cử chỉ

Mốc 9 (giữa lòng bàn tay) → xy · cỡ bàn tay (mốc 0–9) → z, tay gần camera thì hạ ·
chụm ngón cái–trỏ → gắp (hai ngưỡng 0.25 / 0.40) · mất tay → robot giữ nguyên, không thả vật.
