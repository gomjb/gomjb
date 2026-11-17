# Windows 빌드 가이드 (x64 / ARM64)

hullcad 데스크톱 앱(`hullcad.exe`)을 Windows x64 PC와 ARM64 PC(Snapdragon X 등)용으로 빌드하는 방법입니다.

## 1. 준비물

| 항목 | 내용 |
|---|---|
| Visual Studio 2022 | 워크로드 **"C++를 사용한 데스크톱 개발"**. ARM64용을 만들려면 개별 구성 요소에서 **"MSVC v143 - VS 2022 C++ ARM64/ARM64EC 빌드 도구"**도 체크하세요. CMake는 VS에 포함되어 있습니다. |
| Qt 6.8 이상 | [Qt 온라인 설치 프로그램](https://www.qt.io/download-qt-installer-oss)으로 설치합니다. 아래 표의 키트를 고르세요. 앱이 쓰는 모듈(Widgets, Svg, Concurrent)은 기본 구성에 들어 있습니다. |

| 빌드 대상 | 빌드하는 PC | 설치할 Qt 키트 | 예시 경로 |
|---|---|---|---|
| x64 | x64 PC (또는 ARM64 PC) | MSVC 2022 64-bit | `C:\Qt\6.8.3\msvc2022_64` |
| ARM64 | x64 PC (크로스 빌드) | MSVC 2022 64-bit **과** MSVC 2022 ARM64 (cross-compiled) | `C:\Qt\6.8.3\msvc2022_arm64` |
| ARM64 | ARM64 PC (네이티브 빌드) | MSVC 2022 ARM64 (Qt **6.11 이상**에서 제공) | `C:\Qt\6.11.3\msvc2022_arm64` |

x64 PC에서 ARM64용을 크로스 빌드할 때는 x64 키트도 함께 설치해야 합니다. moc 같은 Qt 도구를 x64 키트에서 가져오기 때문입니다.

Qt 계정 없이 명령줄로 설치하려면 [aqtinstall](https://github.com/miurahr/aqtinstall)을 씁니다(키트 하나 약 1 GB, 1분 안팎).

```powershell
python -m pip install aqtinstall
python -m aqt install-qt windows desktop 6.8.3 win64_msvc2022_64 --outputdir C:\Qt
python -m aqt install-qt windows desktop 6.8.3 win64_msvc2022_arm64_cross_compiled --outputdir C:\Qt   # ARM64 크로스 빌드용
[Environment]::SetEnvironmentVariable('QT_DIR_X64', 'C:\Qt\6.8.3\msvc2022_64', 'User')                   # 프리셋이 읽는 경로 (새 터미널부터 적용)
```

## 2. 한 번에 빌드하기 (권장)

PowerShell을 열고 `hullcad` 폴더에서 실행합니다.

```powershell
# 처음 한 번: 스크립트 실행 허용 (현재 사용자만)
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned

# x64
.\scripts\build-windows.ps1 -Arch x64 -QtX64 C:\Qt\6.8.3\msvc2022_64

# x64 + ARM64 (x64 PC에서 크로스 빌드)
.\scripts\build-windows.ps1 -Arch all -QtX64 C:\Qt\6.8.3\msvc2022_64 -QtArm64 C:\Qt\6.8.3\msvc2022_arm64

# ARM64 PC에서 네이티브 빌드
.\scripts\build-windows.ps1 -Arch arm64 -QtArm64 C:\Qt\6.11.3\msvc2022_arm64
```

스크립트는 다음 순서로 진행합니다.
1. CMake 구성
2. Release 빌드
3. 테스트 (x64, 그리고 ARM64 PC에서 빌드한 ARM64만. x64 PC는 ARM64 실행 파일을 실행할 수 없음)
4. 설치: `windeployqt`로 Qt DLL과 플러그인을 함께 복사
5. zip 압축

결과물:
- `dist\windows-x64\bin\hullcad.exe` 와 `dist\hullcad-windows-x64.zip`
- `dist\windows-arm64-cross\bin\hullcad.exe` 와 `dist\hullcad-windows-arm64.zip` (x64 PC에서 크로스 빌드한 경우)
- `dist\windows-arm64\bin\hullcad.exe` 와 `dist\hullcad-windows-arm64.zip` (ARM64 PC에서 네이티브 빌드한 경우)

zip을 풀면 Qt가 설치되지 않은 PC에서도 바로 실행됩니다. 단, Visual C++ 재배포 패키지가 필요할 수 있습니다.

## 3. 직접 빌드하기 (CMake 프리셋)

```powershell
$env:QT_DIR_X64 = "C:\Qt\6.8.3\msvc2022_64"
cmake --preset windows-x64
cmake --build --preset windows-x64
ctest --preset windows-x64
cmake --install build\windows-x64 --config Release     # dist\windows-x64 에 배포본 생성
```

| 프리셋 | 용도 | 필요한 환경 변수 |
|---|---|---|
| `windows-x64` | x64 | `QT_DIR_X64` |
| `windows-x64-cuda` | x64 + CUDA 백엔드 (4장) | `QT_DIR_X64` |
| `windows-arm64` | ARM64 PC에서 네이티브 빌드 | `QT_DIR_ARM64` |
| `windows-arm64-cross` | x64 PC에서 ARM64 크로스 빌드 | `QT_DIR_ARM64`, `QT_DIR_X64` |

Visual Studio에서 `hullcad` 폴더를 **"폴더 열기"**로 열어도 됩니다. 프리셋이 구성 목록에 나타납니다.
이때 환경 변수는 VS를 실행하기 전에 시스템에 설정해 두세요.

크로스 빌드한 ARM64 설치에서 Qt DLL이 복사되지 않았다면, x64 키트의 `windeployqt`로 직접 배포합니다.
(빌드 스크립트는 이 단계를 자동으로 처리합니다.)

```powershell
C:\Qt\6.8.3\msvc2022_64\bin\windeployqt.exe --release --qtpaths C:\Qt\6.8.3\msvc2022_arm64\bin\qtpaths.bat dist\windows-arm64-cross\bin\hullcad.exe
```

## 4. CUDA 백엔드 (선택, NVIDIA GPU)

정수압 적분 배치를 GPU에서 실행하는 백엔드입니다. 지금은 테스트(CPU와 결과 비교)와 `hullcad_demo` 벤치마크에서만
쓰입니다. 앱과 명령줄 도구의 실제 계산은 CPU 백엔드로 합니다. CPU 백엔드는 스레드 수와 관계없이 결과가 비트 단위로 같고,
GeForce에서는 GPU가 32스레드 CPU보다 약 1.15배 빠른 정도이기 때문입니다.

1. Visual Studio 2022를 먼저 설치한 뒤 CUDA Toolkit 12.8 이상을 설치합니다. 그래야 VS용 CUDA 빌드 파일도 함께
   설치됩니다. (약 3 GB, 관리자 권한 확인 창이 뜹니다.)

   ```powershell
   winget install --id Nvidia.CUDA -e
   ```

2. **새 터미널**을 열고 빌드합니다. 설치 프로그램이 등록한 `CUDA_PATH_V13_5` 같은 환경 변수는 새 터미널부터
   보입니다. 설치 전에 열어 둔 터미널에서는 구성 단계에서 `The CUDA Toolkit directory '' does not exist` 오류가 납니다.

   ```powershell
   cmake --preset windows-x64-cuda
   cmake --build --preset windows-x64-cuda
   ctest --preset windows-x64-cuda                        # GPU가 있으면 CPU와 GPU 결과 비교 검사 포함
   build\windows-x64-cuda\Release\hullcad_demo.exe        # 벤치마크 마지막 줄에 cuda 행과 CPU 대비 오차
   ```

기본 GPU 세대는 sm_75·86·89·120(RTX 20·30·40·50 시리즈)입니다. 바꾸려면 `-DCMAKE_CUDA_ARCHITECTURES=native`
(이 PC의 GPU만)처럼 지정합니다. CUDA 런타임은 정적으로 링크하므로, 실행 파일을 다른 PC에 가져가도
CUDA Toolkit 없이 NVIDIA 드라이버만 있으면 됩니다. 드라이버가 없으면 CPU 백엔드로 계산합니다.

## 5. 실행 파일

| 파일 | 내용 |
|---|---|
| `hullcad.exe` | 데스크톱 앱: 선형, 유체정역학·복원성, 블록 분할(야드 프로파일 입력 화면) |
| `hullcad_demo.exe` | 명령줄 도구: 같은 계산을 배치로 실행하고 CSV·SVG를 출력, what-if 스윕 |
| `yards\*.ini` | 야드 프로파일 예시 |

`hullcad.exe --yard yards\mid_yard.ini`처럼 시작할 때 야드 프로파일을 지정할 수 있습니다.

## 6. 검증 범위

| 항목 | 검증 방법 | 결과 |
|---|---|---|
| Windows x64 네이티브 빌드 | Windows 11 x64 PC, VS 2022 (MSVC 19.44), CMake 4.2, Qt 6.8.3, `cmake --preset windows-x64` | 오류·경고 0, 테스트 1881개 통과, 데모 실행 |
| Qt 앱 (Windows x64) | 위 빌드의 `hullcad.exe` 실행, 모든 탭 화면 확인 | 확인 |
| 배포본 (Windows x64) | `cmake --install`(windeployqt)로 만든 `dist\windows-x64`를 Qt가 PATH에 없는 상태에서 실행 | 확인 |
| CUDA 백엔드 (Windows x64) | CUDA 13.5, GeForce RTX 5060 Ti(sm_120), `windows-x64-cuda` 프리셋. 테스트는 CUDA 폴더가 PATH에 없는 상태에서 실행 | 오류·경고 0, 테스트 2001개 통과(GPU 비교 120개 포함), CPU 대비 체적 상대오차 1.3e-15 |
| 코어 라이브러리, 테스트, CLI (Windows ARM64) | Linux에서 clang(mingw)로 크로스 빌드 | 빌드만 확인 (ARM64는 실행하지 못함) |
