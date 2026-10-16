"""Audit the evidence and package the finished deliverables for USB."""
import csv
import hashlib
import json
from pathlib import Path
import zipfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "deliverables"


def main():
    import cv2
    for name, count in [('detect.csv', 25), ('detect_noise.csv', 25), ('pick.csv', 25), ('pick_noise.csv', 20)]:
        with (OUT / 'results' / name).open(newline='') as f:
            assert len(list(csv.DictReader(f))) == count, name
    manifest = json.loads((OUT / 'results' / 'manifest.json').read_text())
    for name, expected in manifest['sources'].items():
        assert hashlib.sha256((ROOT / 'vinsort' / name).read_bytes()).hexdigest() == expected, name
    metrics = json.loads((OUT / 'metrics.json').read_text())
    for mode in ['clean', 'noise']:
        m = metrics[mode]
        assert len(m['errors']) == m['matched']
        assert m['correct'] <= m['parts'] == 120
        assert m['mean'] <= m['max']
    with zipfile.ZipFile(OUT / 'VinSort_6_slides.pptx') as z:
        slides = [n for n in z.namelist() if n.startswith('ppt/slides/slide') and n.endswith('.xml')]
        assert len(slides) == 6
        ns = {'a': 'http://schemas.openxmlformats.org/drawingml/2006/main'}
        table_count = sum(len(ET.fromstring(z.read(s)).findall('.//a:tbl', ns)) for s in slides)
        assert table_count == 2
        assert any('/charts/chart' in n and n.endswith('.xml') for n in z.namelist())
        assert len([n for n in z.namelist() if n.startswith('ppt/notesSlides/notesSlide') and n.endswith('.xml')]) == 6
    events = json.loads((OUT / 'video_events.json').read_text())
    assert any(e['event'] == 'hybrid_recovered_6_of_6' and e['correct'] == 6 for e in events['events'])
    cap = cv2.VideoCapture(str(OUT / 'VinSort_demo.mp4'))
    assert cap.isOpened()
    duration = cap.get(cv2.CAP_PROP_FRAME_COUNT) / cap.get(cv2.CAP_PROP_FPS)
    assert 115 <= duration <= 135, duration
    previews = ROOT / '.build' / 'video_previews'
    previews.mkdir(exist_ok=True)
    for label in ['normal_start', 'handoff_4_of_6', 'scripted_hand_lost_holding', 'hybrid_recovered_6_of_6']:
        event = next(e for e in events['events'] if e['event'] == label)
        cap.set(cv2.CAP_PROP_POS_MSEC, (event['video_s'] + 0.5) * 1000)
        ok, frame = cap.read()
        assert ok, label
        cv2.imwrite(str(previews / (label + '.jpg')), frame)
    cap.release()
    files = [p for p in OUT.rglob('*') if p.is_file() and p.suffix != '.zip' and p.name != 'SHA256SUMS.txt']
    (OUT / 'SHA256SUMS.txt').write_text('\n'.join(
        hashlib.sha256(p.read_bytes()).hexdigest() + '  ' + p.relative_to(OUT).as_posix()
        for p in sorted(files)) + '\n', encoding='utf-8')
    source_files = [ROOT / n for n in ['README.md', 'REPORT.md', 'PHAN_CONG.md', 'benchmark.py']]
    source_files += list((ROOT / 'vinsort').glob('*.py'))
    source_files += list((ROOT / 'vinsort').glob('*.md'))
    source_files += [ROOT / 'vinsort' / 'requirements.txt']
    source_files += [p for p in (ROOT / 'presentation_tools').iterdir() if p.is_file()]
    with zipfile.ZipFile(OUT / 'VinSort_D_ban_giao.zip', 'w', zipfile.ZIP_DEFLATED) as z:
        for file in source_files + files + [OUT / 'SHA256SUMS.txt']:
            z.write(file, file.relative_to(ROOT))
    print(f'Audit passed: 6 slides, editable table/chart, raw CSV, {duration:.2f}s video, recovery 6/6.')
    print(OUT / 'VinSort_D_ban_giao.zip')


if __name__ == '__main__':
    main()
