/* NVIDIA Optimus must see this exported symbol in the Python process BEFORE OpenGL starts.
   Build with MSVC; keep the exe beside .venv\Scripts\python.exe for venv discovery. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
__declspec(dllexport) unsigned long NvOptimusEnablement = 0x00000001;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
int wmain(int argc, wchar_t **argv) { return Py_Main(argc, argv); }
