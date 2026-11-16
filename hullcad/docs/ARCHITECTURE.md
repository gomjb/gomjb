# hullcad 모듈 구조

hullcad는 기능별 모듈로 나뉘어 있습니다. 모듈마다 독립된 라이브러리(`hullcad_<모듈>`)와 헤더 폴더를 가지며,
**의존 방향은 아래에서 위로 한 방향만** 허용됩니다. 각 모듈은 자기가 링크한 하위 모듈의 헤더만 볼 수 있어서
방향을 거스르는 `#include`는 컴파일 단계에서 "파일을 찾을 수 없음" 오류로 막힙니다.

```
                         ┌──────────────────────────────┐
                         │ app (Qt 앱) · apps (CLI)      │  화면, 명령줄
                         └──────────────┬───────────────┘
                         ┌──────────────┴───────────────┐
                         │ project  프로젝트 설정        │  야드 프로파일 통합, INI 읽기·쓰기
                         └──────────────┬───────────────┘
              ┌─────────────────────────┴──────────────┐
   ┌──────────┴──────────┐                              │
   │ production 생산관리  │ 유틸리티 원단위·사용량,       │
   │                     │ 일별 부하, 피크, 공급능력 검토 │
   └──────────┬──────────┘                              │
   ┌──────────┴──────────┐                              │
   │ planning 중일정관리  │ 작업일 달력, 탑재 순서,        │
   │                     │ 공정 역산 일정                │
   └──────────┬──────────┘                              │
   ┌──────────┴────────────────┐                        │
   │ production_design 생산설계 │ 존·심, 중량·형상 모델,   │
   │                           │ 야드 검사, 블록 분할,    │
   │                           │ 물량 산출, 공정 단계     │
   └───────┬───────────┬───────┘                        │
   ┌───────┴───────┐ ┌─┴──────────────────┐   ┌─────────┴──────────┐
   │ structure     │ │ yard 조선소 설비     │   │ naval 선박계산      │
   │ 구조설계       │ │ 설비 한계, 공급능력  │   │ 유체정역학, 복원성   │
   └───────┬───────┘ └────────────────────┘   │ CPU/CUDA 백엔드     │
   ┌───────┴──────────────────────────────┐   └─────────┬──────────┘
   │ cad 선형 CAD  (fairing 선형 순정이 위에)  ├─────────────┘
   │ B-spline, 파라메트릭 선형, 메쉬, 선도    │
   └───────┬──────────────────────────────┘
   ┌───────┴──────────────────────────────┐
   │ core 공통 기반: 벡터, 스레드풀, 날짜·달력 │
   └──────────────────────────────────────┘
```

## 모듈별 책임

| 모듈 | 업무 영역 | 주요 헤더 | 하는 일 | 의존 |
|---|---|---|---|---|
| `core` | 공통 | `core/vec3`, `core/parallel`, `core/calendar` | 벡터 연산(CPU·CUDA 공용), 결정적 스레드풀, 날짜·작업일 달력 | — |
| `cad` | 선형 설계 | `cad/bspline`, `cad/hull_param`, `cad/mesh`, `cad/lines_plan` | B-spline 곡선·곡면, 파라메트릭 선형, 닫힌 메쉬, 단면 절단, 선도 | core |
| `fairing` | 선형 순정 | `fairing/fairing` | 허용 오차 안 곡률 변화 에너지 최소화, 직선 구간 검출, G1/G2 접속, 곡률 콤·변곡점·에너지 | cad, core |
| `naval` | 기본설계 계산 | `naval/hydro_kernel`, `naval/compute_backend`, `naval/hydrostatics` | 배수량, 부심, 수선면, 정수압표, GZ 곡선. CPU 멀티스레드와 CUDA 백엔드 | core, cad |
| `structure` | 구조 설계 | `structure/structure` | 프레임 테이블, 참조 기반 부재 배치(격벽, 웹, 이중선체, 거더, 스트링거), 단면 부재·용접 접합부 | cad |
| `yard` | 조선소 설비 | `yard/facility` | 공정별 설비 한계(절단, 판넬라인, 곡블록, 중조립, 대조립, 반전, 도장, 트랜스포터, 탑재), 유틸리티 공급 능력. 데이터만 있음 | — |
| `production_design` | 생산 설계 | `production_design/blocks`, `…/design_settings`, `…/process` | 존·심, 슬라이스 중량·형상 모델, 블록별 설비 검사, 자동 분할(DP), 물량 산출, 공정 단계 정의 | structure, yard, core |
| `planning` | 중일정 관리 | `planning/schedule` | 탑재 순서, 탑재일에서 역산한 블록별 공정 일정, 탱크 시험 기간 | production_design, core |
| `production` | 생산 관리 | `production/utilities`, `production/daily_load` | 유틸리티 원단위 × 물량, 탱크 용적, 일별 부하, 피크, 공급 능력 검토 | planning, production_design, yard |
| `project` | 프로젝트 설정 | `project/yard_profile` | 모든 모듈 설정을 묶은 `YardProfile`, 옵션 등록부, INI 읽기·쓰기 | 위 전부 |

## 설정은 모듈이 소유합니다

각 모듈은 자기 설정 타입만 압니다. `project`의 `YardProfile`이 이 타입들을 모두 상속해 하나로 묶습니다.
그래서 `YardProfile`은 어느 모듈 함수에나 그대로 넘길 수 있습니다.

| 설정 타입 | 소유 모듈 | INI 섹션 |
|---|---|---|
| `FacilityProfile`, `SupplyCapacity` | yard | `[yard] [cutting] [panel_line] [curved_shop] [medium_assembly] [assembly] [turnover] [painting] [transporter] [erection] [capacity]` |
| `ProductionDesignSettings` (= FacilityProfile + 규칙) | production_design | `[weight] [division] [takeoff]` |
| `PlanningSettings` | planning | `[schedule]` |
| `UtilitySettings` | production | `[util_cutting] [util_welding] [util_heating] [util_painting] [util_handling] [util_testing]` |

```cpp
YardProfile y = load_yard_profile("yards/mid_yard.ini");
Division d = auto_divide(wm, p, st, y);                         // 생산설계: ProductionDesignSettings
UtilityReport u = estimate_utilities(d, tanks, y);              // 생산관리: UtilitySettings
ProductionSchedule s = schedule_blocks(d, y);                   // 중일정: PlanningSettings
DailyUtilityProfile load = daily_utilities(s, u, y.capacity);   // 생산관리: 일정 + 공급능력
```

## 폴더 구조

```
hullcad/
├─ modules/
│  ├─ CMakeLists.txt              모듈 라이브러리와 의존 관계 정의
│  ├─ core/        include/hullcad/core/…        src/…
│  ├─ cad/         include/hullcad/cad/…         src/…
│  ├─ naval/       include/hullcad/naval/…       src/… (backend_cuda.cu 포함)
│  ├─ structure/   include/hullcad/structure/…   src/…
│  ├─ yard/        include/hullcad/yard/…        (헤더만)
│  ├─ production_design/ …
│  ├─ planning/    …
│  ├─ production/  …
│  └─ project/     …
├─ app/      Qt 데스크톱 앱 (pipeline.* 는 Qt 비의존 계산 흐름)
├─ apps/     명령줄 도구 hullcad_demo
├─ tests/    단위 테스트
├─ yards/    야드 프로파일 예시
└─ docs/     문서와 화면
```

## 앞으로 붙일 모듈 (예정)

같은 규칙으로 새 기능도 독립 모듈로 추가합니다.

| 예정 모듈 | 업무 | 위치 (의존) |
|---|---|---|
| `cad` 확장 (OCCT 8.0) | B-rep 솔리드, STEP/IGES, 판재·보강재 실형상 | cad 안, 또는 cad 위 `cad_occt` |
| `fairing` 확장 | 곡선망(수선·버톡 교차점 공유) 순정, 곡면 곡률·제브라 검사, 다중 패치 | fairing 안 |
| `stability` | 자유 트림 GZ, 탱크·적하 조건, 손상 복원성(MARPOL), GPU 배치 | naval, structure 위 |
| `outfitting` | 의장품, 선행의장 물량 | structure 위 |
| `cost` | 공수, 원가 | production 위 |
| `leveling` | 일정 여유 안에서 부하 평준화 | planning, production 위 |
| `mes_bridge` | MES·ERP 연계 (실적 수집, 계획 내보내기) | project 위 |

## 이전 버전에서 바뀐 점

- 단일 라이브러리 `hullcad_core`가 모듈별 라이브러리 9개와 묶음 라이브러리 `hullcad`로 나뉘었습니다.
- 헤더 경로에 모듈 이름이 붙었습니다. 예: `hullcad/blocks.hpp` → `hullcad/production_design/blocks.hpp`
- 일정 API가 분리되었습니다. `schedule_utilities()` → `schedule_blocks()`(중일정) + `daily_utilities()`(생산관리)
- 용착량, SAW 사용, 도장 횟수 키가 소유 모듈 섹션으로 옮겨졌습니다. 예: `takeoff.coats` → `util_painting.coats`.
  옛 키도 계속 읽힙니다.
