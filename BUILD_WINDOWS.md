# Windows (MSVC) Build Guide

This project can be built with CMake on Windows using MSVC. The steps below
use a clean out-of-source build directory (`build_clean`) and force fetching
fmt/spdlog to avoid conflicts with system or Anaconda-provided versions.

## Prerequisites

- Visual Studio 2022 Build Tools (MSVC toolchain + CMake)
- Git

Tip: run these commands from a "x64 Native Tools Command Prompt for VS 2022"
or a PowerShell that has MSVC tools on PATH.

## Configure

```powershell
cmake -S . -B build_clean -DLIBOCULUS_FETCH_DEPS=ON -DLIBOCULUS_FORCE_FETCH_DEPS=ON
```

## Build

```powershell
cmake --build build_clean --config Debug --target occlient
```

## Outputs

- Executable: `build_clean\bin\Debug\occlient.exe`
- Library: `build_clean\lib\Debug\oculus.lib` (static on MSVC)

## Run (example)

```powershell
.\build_clean\bin\Debug\occlient.exe -i "F:\Projects\liboculus\test\data\three_pings_8bit.raw" -v -o "F:\Projects\liboculus\test\data\out.raw"
```

## Troubleshooting

- If CMake links against the wrong fmt/spdlog (e.g., Anaconda), use a fresh
  build directory and keep `LIBOCULUS_FORCE_FETCH_DEPS=ON`.
- If `build_clean` cannot be deleted, close Visual Studio or any process that
  is holding files under `build_clean\.vs`.
