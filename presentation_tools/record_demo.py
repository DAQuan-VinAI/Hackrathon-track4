"""Record actual PyBullet frames with scripted teleop, without webcam or GUI.

The overlay discloses scripted input throughout. No claimed human interaction.
"""
import json
import random
import sys
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vinsort"))
import cv2
import numpy as np
import pybullet as p
import imageio_ffmpeg
from PIL import Image, ImageDraw, ImageFont
import controller
import perception
from common import CUBE
from main import Orchestrator
from scene import random_free_xy

OUT = ROOT / "deliverables"
FPS = 12


class ScriptedTracker:
    def __init__(self, script=None):
        self.script = script or [(None, False)]
        self.i = 0

    def read(self, key=-1):
        import time
        self.t0 = time.perf_counter()
        xyz, grip = self.script[min(self.i, len(self.script) - 1)]
        self.i += 1
        return None if xyz is None else np.array(xyz), grip, None, 0.0


def main():
    OUT.mkdir(exist_ok=True)
    fonts = Path("C:/Windows/Fonts") if sys.platform == "win32" else Path("/mnt/c/Windows/Fonts")
    font = ImageFont.truetype(str(fonts / "arial.ttf"), 23)
    titlefont = ImageFont.truetype(str(fonts / "arialbd.ttf"), 30)
    smallfont = ImageFont.truetype(str(fonts / "arial.ttf"), 19)
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    proc = subprocess.Popen([ff, "-y", "-loglevel", "error", "-f", "rawvideo", "-vcodec", "rawvideo",
                             "-pix_fmt", "rgb24", "-s", "1280x720", "-r", str(FPS), "-i", "-",
                             "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "22",
                             "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(OUT / "VinSort_demo.mp4")], stdin=subprocess.PIPE)
    frames = 0
    events = []
    app = None
    caption = ""
    chapter = ""
    last = None

    def event(label):
        events.append(dict(video_s=round(frames / FPS, 2), event=label,
                           mode=app.mode if app else None,
                           correct=app.in_correct_bin() if app else None))
        print(events[-1], flush=True)

    def render():
        nonlocal last
        view = p.computeViewMatrixFromYawPitchRoll((0.35, -0.05, 0.05), 1.4, -90, -44, 0, 2)
        proj = p.computeProjectionMatrixFOV(50, 840/520, 0.05, 4)
        rgba = p.getCameraImage(840, 520, view, proj, renderer=p.ER_TINY_RENDERER)[2]
        world = np.asarray(rgba, dtype=np.uint8).reshape(520, 840, 4)[:, :, :3]
        img = Image.new("RGB", (1280, 720), "#101b2e")
        img.paste(Image.fromarray(world), (24, 110))
        rgb, depth, _ = perception.capture()
        dets = perception.detect(rgb, depth)
        camera = cv2.cvtColor(perception.draw(rgb, dets), cv2.COLOR_BGR2RGB)
        img.paste(Image.fromarray(camera).resize((360, 270)), (892, 110))
        d = ImageDraw.Draw(img)
        d.text((24, 18), chapter, font=titlefont, fill="white")
        d.text((24, 62), "MÔ PHỎNG PYBULLET · TELEOP THEO KỊCH BẢN · KHÔNG THU WEBCAM", font=smallfont, fill="#8de1d5")
        d.text((895, 400), "Camera ảo RGB + depth", font=font, fill="white")
        d.text((895, 450), f"Chế độ: {app.mode}", font=font, fill="#ffd186")
        d.text((895, 490), f"Đúng thùng: {app.in_correct_bin()}/6", font=font, fill="white")
        d.text((895, 530), "Ảnh lấy từ engine thật", font=smallfont, fill="#bac7d8")
        d.text((895, 562), "trong lần chạy mô phỏng", font=smallfont, fill="#bac7d8")
        for i, line in enumerate(caption.split("\n")):
            d.text((24, 643 + 31*i), line, font=font, fill="white")
        last = np.asarray(img).tobytes()
        return last

    def write(frame=None):
        nonlocal frames
        proc.stdin.write(frame or render())
        frames += 1

    def pause(seconds):
        frame = render()
        for _ in range(round(seconds * FPS)):
            write(frame)

    original = controller._step
    count = 0

    def step(realtime):
        nonlocal count
        original(False)
        count += 1
        if count % 20 == 0:
            write()

    try:
        chapter = "VinSort: tự động phân loại và chuyển cho người"
        app = Orchestrator(0, ScriptedTracker(), realtime=False, show=False)
        caption = "Bản dự phòng 2 phút. Robot phân loại 6 khối theo màu.\nCác thao tác di chuyển vật và tay trong video đều theo kịch bản."
        event("normal_start")
        pause(8)
        (OUT / "debug").mkdir(exist_ok=True)
        render()
        rgb, _, _ = perception.capture()
        cv2.imwrite(str(OUT / "debug" / "overview.png"), cv2.cvtColor(np.asarray(Image.frombytes('RGB',(1280,720),last)), cv2.COLOR_RGB2BGR))
        controller._step = step
        chapter = "01 / AUTO: camera nhìn lại sau mỗi lần gắp"
        caption = "HSV xác định màu, depth đổi pixel sang tọa độ 3D.\nIK điều khiển tay gắp vào thùng tương ứng."
        for _ in range(16):
            app.tick()
            if app.mode != "AUTO":
                break
        assert app.mode == "DONE" and app.in_correct_bin() == 6
        event("normal_6_of_6")
        pause(4)
        # Replay judge moving one part. This is a disclosed simulated perturbation.
        chapter = "02 / Vật đổi chỗ: quét lại trước khi gắp"
        oid = app.ids[0]
        xy = random_free_xy(random.Random(42), [i for i in app.ids if i != oid])
        p.resetBasePositionAndOrientation(oid, (*xy, CUBE / 2), (0, 0, 0, 1))
        p.resetBaseVelocity(oid, (0, 0, 0), (0, 0, 0))
        caption = "Kịch bản di chuyển một vật, mô phỏng tình huống giám khảo kéo vật.\nNhấn r để robot quét lại và phân loại tiếp."
        event("scripted_part_move")
        pause(5)
        app.tick(ord("r"))
        for _ in range(5):
            if app.mode != "AUTO":
                break
            app.tick()
        assert app.in_correct_bin() == 6
        event("rescan_recovered")
        pause(3)
        app = Orchestrator(100, ScriptedTracker(), realtime=False, show=False)
        chapter = "03 / Ca lỗi: hai khối cùng màu sát nhau"
        caption = "Mask HSV gộp thành một contour lớn. Cờ area báo nhập nhằng.\nRobot xếp vật rõ trước, rồi chuyển TELEOP."
        event("failure_seed_100")
        pause(7)
        for _ in range(16):
            app.tick()
            if app.mode != "AUTO":
                break
        assert app.mode == "TELEOP" and app.in_correct_bin() == 4
        event("handoff_4_of_6")
        pause(5)
        pair = [oid for oid in app.ids if not app.in_bin(oid)]
        x, y, _ = p.getBasePositionAndOrientation(pair[0])[0]
        fx, fy = random_free_xy(random.Random(0), [q for q in app.ids if q != pair[0]], min_dist=0.10)
        hover, low = (x, y, 0.20), (x, y, CUBE + 0.01)
        dst_hi, dst_lo = (fx, fy, 0.20), (fx, fy, CUBE + 0.015)
        script = [(hover, False)]*70 + [(low, False)]*40 + [(low, True)]*3 + [(hover, True)]*30
        script += [(None, True)]*20 + [(dst_hi, True)]*50 + [(dst_lo, True)]*30
        script += [(dst_lo, False)]*3 + [(dst_hi, False)]*30
        app.tracker = ScriptedTracker(script)
        chapter = "04 / TELEOP: tách cặp vật rồi trả về AUTO"
        caption = "Đầu vào tay theo kịch bản kiểm thử, không phải người thao tác webcam.\nTách một vật khỏi vùng gộp để robot nhìn lại."
        for i in range(len(script)):
            app.tick()
            if i == 143:
                assert app.tstate["cid"] is not None
                caption = "Mô phỏng mất tay: robot giữ vị trí và tiếp tục giữ vật.\nSau khi có lại đầu vào, robot đưa vật đến vùng trống."
                event("scripted_hand_lost_holding")
                pause(4)
            if i == 163:
                caption = "Tay theo kịch bản xuất hiện lại, tiếp tục tách vật.\nThả vật rồi nhấn a để trở lại AUTO."
        assert app.tstate["cid"] is None
        event("pair_separated")
        pause(3)
        chapter = "05 / AUTO: phục hồi và hoàn tất"
        caption = "Robot nhận lại hai contour riêng và xếp nốt.\nKhả năng phục hồi này kiểm chứng trong mô phỏng."
        app.tick(ord("a"))
        for _ in range(8):
            if app.mode != "AUTO":
                break
            app.tick()
        assert app.mode == "DONE" and app.in_correct_bin() == 6
        event("hybrid_recovered_6_of_6")
        chapter = "VinSort: kết quả và giới hạn"
        caption = "Hoàn tất 6/6 sau can thiệp theo kịch bản. Chưa kiểm chứng robot thật.\nSố đo nhiều seed và giả định impact nằm trong slide và REPORT.md."
        pause(max(6, 120 - frames / FPS))
        event("end")
    finally:
        controller._step = original
        proc.stdin.close()
        code = proc.wait()
        if p.isConnected():
            p.disconnect()
        (OUT / "video_events.json").write_text(json.dumps(dict(fps=FPS, frames=frames,
            duration_s=frames/FPS, scripted_input=True, events=events), ensure_ascii=False, indent=2), encoding="utf-8")
        if code:
            raise RuntimeError(f"ffmpeg failed: {code}")
    print(f"Video: {frames/FPS:.2f} seconds", flush=True)


if __name__ == "__main__":
    main()
