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
| `windows-arm64` | ARM64 PC에서 네이티브 빌드 | `QT_DIR_ARM64` |
| `windows-arm64-cross` | x64 PC에서 ARM64 크로스 빌드 | `QT_DIR_ARM64`, `QT_DIR_X64` |

Visual Studio에서 `hullcad` 폴더를 **"폴더 열기"**로 열어도 됩니다. 프리셋이 구성 목록에 나타납니다.
이때 환경 변수는 VS를 실행하기 전에 시스템에 설정해 두세요.

크로스 빌드한 ARM64 설치에서 Qt DLL이 복사되지 않았다면, x64 키트의 `windeployqt`로 직접 배포합니다.
(빌드 스크립트는 이 단계를 자동으로 처리합니다.)

```powershell
C:\Qt\6.8.3\msvc2022_64\bin\windeployqt.exe --release --qtpaths C:\Qt\6.8.3\msvc2022_arm64\bin\qtpaths.bat dist\windows-arm64-cross\bin\hullcad.exe
```

## 4. 실행 파일

| 파일 | 내용 |
|---|---|
| `hullcad.exe` | 데스크톱 앱: 선형, 유체정역학·복원성, 블록 분할(야드 프로파일 입력 화면) |
| `hullcad_demo.exe` | 명령줄 도구: 같은 계산을 배치로 실행하고 CSV·SVG를 출력, what-if 스윕 |
| `yards\*.ini` | 야드 프로파일 예시 |

`hullcad.exe --yard yards\mid_yard.ini`처럼 시작할 때 야드 프로파일을 지정할 수 있습니다.

## 5. 검증 범위

| 항목 | 검증 방법 | 결과 |
|---|---|---|
| 코어 라이브러리, 테스트, CLI (Windows x64) | clang(mingw)로 크로스 빌드하고 Wine에서 실행 | 테스트 851개 통과 |
| 코어 라이브러리, 테스트, CLI (Windows ARM64) | 같은 방식으로 크로스 빌드 | 빌드만 확인 (ARM64는 실행하지 못함) |
| Qt 앱 | Linux에서 Qt 6.8.3으로 빌드하고 실행, 세 탭 화면 확인 | 확인 |
| MSVC 빌드 (VS 2022) | 이 개발 환경에는 MSVC가 없어 아직 실행하지 못함 | 미검증 |

MSVC로 처음 빌드할 때 오류나 경고가 나오면 그 출력을 그대로 보내 주세요.
