param([switch]$Video)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$linuxRoot = (wsl -d Ubuntu -- wslpath -a $taskRoot).Trim()
$linuxUserHome = (wsl -d Ubuntu -- printenv HOME).Trim()
$linuxPython = "$linuxUserHome/.venvs/vinsort-evidence/bin/python"
if ($LASTEXITCODE -ne 0) { throw 'Can WSL Ubuntu de chay PyBullet 3.2.7.' }
wsl -d Ubuntu -- bash -lc 'test -x ~/.local/bin/uv || (curl -LsSf https://astral.sh/uv/install.sh | sh); ~/.local/bin/uv venv --allow-existing --python 3.11 ~/.venvs/vinsort-evidence; ~/.local/bin/uv pip install --python ~/.venvs/vinsort-evidence/bin/python pybullet==3.2.7 numpy==1.26.4 opencv-python-headless==4.11.0.86 matplotlib==3.11.2 pillow==12.3.0 imageio-ffmpeg==0.6.0'
if ($LASTEXITCODE -ne 0) { throw 'Khong cai duoc moi truong mo phong.' }
wsl -d Ubuntu -- $linuxPython "$linuxRoot/benchmark.py" --out "$linuxRoot/deliverables/results"
if ($LASTEXITCODE -ne 0) { throw 'Benchmark failed.' }
wsl -d Ubuntu -- $linuxPython "$linuxRoot/vinsort/test_vinsort.py"
if ($LASTEXITCODE -ne 0) { throw 'Self-check failed.' }
wsl -d Ubuntu -- $linuxPython "$linuxRoot/presentation_tools/prepare_evidence.py"
if ($LASTEXITCODE -ne 0) { throw 'Evidence preparation failed.' }
if ($Video) {
    wsl -d Ubuntu -- $linuxPython "$linuxRoot/presentation_tools/record_demo.py"
    if ($LASTEXITCODE -ne 0) { throw 'Video generation failed.' }
}
