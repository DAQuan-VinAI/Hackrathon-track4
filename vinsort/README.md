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

Giám khảo kéo vật: **chuột trái kéo thẳng** trong cửa sổ PyBullet (Ctrl + kéo là xoay camera).

## Cử chỉ

Mốc 9 (giữa lòng bàn tay) → xy · cỡ bàn tay (mốc 0–9) → z, tay gần camera thì hạ ·
chụm ngón cái–trỏ → gắp (hai ngưỡng 0.25 / 0.40) · mất tay → robot giữ nguyên, không thả vật.
