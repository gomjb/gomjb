# Physics Designer (`gomjb.physics_designer`)

Isaac Sim에서 물리 모델을 만들 때 필요한 반복 작업(객체 이동·정렬, 스냅, 조립 기준점,
조인트 생성, 물리 속성 부여, 설정 검증)을 하나의 창에서 처리하는 Python 기반 디자이너
extension입니다. 모든 작업은 **Ctrl+Z로 되돌릴 수 있고**, 같은 기능을 스크립트에서도 그대로 쓸 수 있습니다.

> A Python designer extension for NVIDIA Isaac Sim that makes physics modelling easy:
> move/align, snapping, anchors, joint creation, physics properties, mechanism templates and
> setup validation — every operation undoable and scriptable.

---

## 1. 설치 / 실행

**Extension Manager로 추가**

1. Isaac Sim 실행 → `Window > Extensions`
2. 오른쪽 위 ☰(또는 ⚙) → *Extension Search Paths* 에 `<이 저장소>/exts` 추가
3. 목록에서 **Physics Designer** 를 찾아 *Enable* (원하면 *Autoload* 체크)
4. 메뉴 `Tools > Physics Designer` 로 창 열기/닫기

**명령줄로 실행**

```bat
isaac-sim.bat --ext-folder <repo>\exts --enable gomjb.physics_designer
```

검증 환경: **Isaac Sim 6.1.0 (Kit 110, Windows, RTX 5060 Ti)** — PhysX(기본 `isaac-sim.bat`)와
Newton(`isaac-sim.newton.bat`) 양쪽에서 5개 템플릿을 시뮬레이션해 확인했습니다.
물리 스키마는 USD 표준 `UsdPhysics`만 사용합니다. 외부 패키지는 정확해석기(2.9)의
[Exudyn](https://github.com/jgerstmayr/EXUDYN) 하나이며, 확장을 켤 때 Kit의 `[python.pipapi]`가
`exudyn==1.13.6`을 설치합니다(`--no-deps`: 유일한 의존성 numpy는 Kit에 포함). 인터넷이 없는 PC에서는
Isaac Sim의 `python.bat -m pip install exudyn==1.13.6` 으로 미리 설치하세요.

---

## 2. 창 구성과 사용법

창 맨 위에는 현재 선택이 **선택 순서대로** 표시됩니다 (`0: Base [rigid]  1: Arm`).
여러 기능이 이 순서를 사용합니다 — *첫 번째 = 기준/body0*, *마지막 = 타깃*.

창의 글자는 영어입니다. Isaac Sim 기본 UI 폰트에 한글 글리프가 없어 한글이 `?`로 표시되기 때문입니다.

### 2.0 Modeler (모델러)
RecurDyn·Adams의 바디 관례를 따라 **물리 준비가 끝난 강체**를 만듭니다. 바디 하나는 기준 좌표계, 형상,
재질, 질량 특성으로 이루어집니다. Isaac의 `Create > Shape`와 달리 바디 좌표계에는 스케일이 없어서 조인트와
앵커가 틀어지지 않습니다.

| 형상 | 치수 |
|---|---|
| Box | size x / y / z |
| Cylinder, Capsule, Cone | radius, height (Z축) |
| Sphere | radius |
| **Link** | pin distance, width, thickness — 양 끝이 둥근 막대, 양 끝 핀에 앵커 `PinA` / `PinB` 자동 생성(앵커 Z축 = 핀 축) |

* **재질**: steel·stainless·aluminum·cast_iron·brass·abs_plastic·wood·rubber 밀도 프리셋, 또는 밀도·질량 직접 입력
  (입력은 SI: kg/m³, kg — 스테이지 단위(m/cm)로 자동 환산)
* **질량 특성**: 질량·무게중심·주관성모멘트를 **해석식으로 계산해 명시적으로 기록**합니다(수치 적분으로 1% 이내 검증).
  충돌체가 없는 바디도 올바른 관성을 가집니다.
* **바디 종류**: dynamic / kinematic / static(충돌체만) / visual(물리 없음)
* **배치**: 지정 위치·자세, 선택 객체 위에 올리기, 선택한 앵커에 붙이기(Link는 `PinA`를 앵커에 맞춤)
* **Load from selected / Apply to selected**: 치수·재질·질량을 나중에 바꾸면 형상·질량·핀 앵커가 함께 갱신됩니다.
  (핀 위치가 바뀌면 붙어 있던 조인트는 *Realign selected joints* 로 다시 맞추세요.)

### 2.1 Move & Rotate (객체 이동)
| 기능 | 설명 |
|---|---|
| X±/Y±/Z± | *Move step* 만큼 이동. *Space* = world / local(객체 자신의 축) |
| RX±/RY±/RZ± | *Rotate step* 만큼 회전. Pivot = 원점(self) / 자기 bbox 중심 / 선택 전체 중심(group) |
| World position | 첫 선택 객체의 월드 위치 읽기 / 선택 전체에 적용 |
| Align | 축(X/Y/Z)·기준(min/center/max)으로 bbox 정렬. 기준 객체 = first / last / group |
| Distribute | 중심 간격 균등 분배, 또는 *gap* ≥ 0 이면 bbox 사이를 gap으로 붙여서 나열 |
| Drop to ground / onto objects | bbox 바닥을 지면 높이에, 또는 아래에 겹치는 충돌체 위에 올림 |
| Copy pose | 첫 객체의 위치·자세를 나머지에 복사 (스케일은 유지) |

부모가 회전/스케일된 계층에서도 **월드 기준으로 정확히** 계산되며, 기존 `float3`/`quatf`
xformOp 정밀도는 유지됩니다. 자식과 부모를 함께 선택하면 부모만 이동합니다(이중 이동 방지).

### 2.2 Snap (객체 스냅)
* **Pivot to grid / BBox corner to grid / Rotation to step** : 위치·bbox 모서리·자세(XYZ 오일러)를 격자/각도에 맞춤
* **Live grid snap / Live angle snap** : 뷰포트 기즈모로 끄는 동안 실시간 스냅 (실험적 기능)
* **Snap to target** : 마지막 선택 객체(타깃)의 원점 또는 bbox 중심으로 나머지를 이동
* **Snap to nearest point** : 이동 객체의 원점(또는 bbox 27점 중 가장 가까운 점)을 타깃의
  bbox 27점(모서리·모서리 중점·면 중심·중심) / 메쉬 정점 / 원점 중 가장 가까운 곳에 붙임
* **Mate faces / Stack** : 타깃의 면(+Z면이면 "위에 쌓기")에 이동 객체의 반대 면을 맞붙이고 중앙 정렬, *gap* 지원

### 2.3 Anchors (조립 기준점)
부품의 구멍·핀·장착면 같은 **이름 있는 좌표계**를 만들어 조립과 조인트의 기준으로 씁니다.

1. 부품 선택 → Location(원점 / bbox 중심 / ±X·±Y·±Z 면 / 직접 입력) → *Create anchor*
   (면 앵커는 Z축이 면 법선 방향). 뷰포트에는 guide용 주황색 구로 표시되며 충돌체가 되지 않습니다.
2. **움직일 앵커 → 타깃 앵커** 순서로 선택 → *Connect anchors*
   앵커의 소유자(가장 가까운 rigid body 조상, 없으면 부모)가 이동해 두 앵커가 일치합니다.
   *Flip* 은 두 Z축이 마주보게(커넥터 결합) 하고, *Then create joint* 로 그 자리에 바로 조인트 생성.

### 2.4 Joints (조인트 생성)
* 선택: **body0 → body1** (하나만 선택하면 월드에 고정된 조인트)
* Type: fixed / revolute / prismatic / spherical / distance / D6
* Location: body1 원점, body0 원점, 중점, bbox 중심, 선택한 앵커, 직접 입력
* World axis: ±X/±Y/±Z, 앵커 좌표계, body0의 로컬 축, 임의 벡터
  *Axis token* 은 조인트 프레임에서 그 방향에 대응시킬 축(USD `physics:axis`)
* Limits(각도는 deg), Drive(force/acceleration, stiffness, damping, target, max force), Break force,
  body 간 충돌 여부, articulation 제외, 조인트 prim 위치(`<root>/Joints` / body0 / body1 아래)
* *Add RigidBody to bodies* : rigid body가 없는 body에 RigidBodyAPI + 충돌체 자동 추가

조인트 프레임은 **하나의 월드 프레임에서 body0/body1의 `localPos/localRot`를 동시에 계산**하므로
(스케일·부모 회전 포함) 시뮬레이션 시작 시 물체가 튀는 현상이 없습니다.
조인트 생성 후 body를 옮겼다면 조인트를 선택하고 *Realign selected joints* 를 누르세요.

**Newton 엔진을 쓸 때 (`isaac-sim.newton.bat`)**
* body끼리 연결된 조인트는 반드시 articulation에 속해야 합니다 → 기구의 루트 prim에 *Articulation root* 적용.
* articulation 안에서는 **body0 = 부모(루트 쪽), body1 = 자식** 순서로 선택해 만드세요. 반대면 Newton이
  `Reversed joints are not supported` 로 초기화를 거부합니다.
* 루프를 닫는 조인트는 *Exclude from artic.* 로 빼고, 회전(revolute) 조인트로 닫으세요
  (Newton의 MuJoCo 솔버는 prismatic 루프 닫힘을 지원하지 않습니다).

### 2.5 Physics properties (물리 속성)
Rigid / Kinematic / Collider only (근사: none·convexHull·convexDecomposition·meshSimplification·
boundingCube·boundingSphere), 질량·밀도, 물리 머티리얼(정/동마찰, 반발계수 — `physics` purpose로
바인딩해 렌더 머티리얼과 분리), Articulation root, 두 객체 간 충돌 필터, Physics scene, Ground plane,
물리 속성 일괄 제거.

*Physics scene* 버튼으로 만든 씬은 **CPU 물리(CPU dynamics + MBP broadphase)** 로 설정됩니다.
기구 설계용 씬은 작지만 제약이 많아 CPU 솔버가 정확하고, Isaac Sim 6.1에서 GPU 솔버는 폐루프 기구(시저)가
다른 articulation과 함께 있을 때 발산(NaN)하는 것을 확인했기 때문입니다.

### 2.6 Mechanism templates (메커니즘 템플릿)
파라미터를 입력해 바로 시뮬레이션 가능한 기구를 생성합니다 (단위: stage unit, 기본 m / deg).

| 템플릿 | 구성 |
|---|---|
| Pendulum chain | 월드에 고정된 지지 링크 + 캡슐 링크 N개 revolute 연쇄, 초기 각도, 감쇠 |
| Hydraulic cylinder | 배럴 + 로드 + 구동 prismatic(스트로크 제한, PD 드라이브), 장착 앵커 `MountBase`/`MountRod` |
| Hinged door | 고정 포스트 + 도어 패널, 개방각 제한 revolute + 댐핑 |
| Linear slider | 고정 레일 + 캐리지, 행정 제한 prismatic |
| Scissor lift | N단 시저 암(좌/우), 중앙 핀·단간 핀, 하부 고정핀 + **구동 슬라이더(액추에이터)**, 상부 고정핀 + 자유 슬라이더 |

모든 템플릿은 Isaac Sim 로봇 에셋과 같은 관례로 **루트에 ArticulationRootAPI가 있는 고정 베이스
articulation** 이며, 조인트는 body0 = 부모, body1 = 자식입니다.

시저 리프트는 폐루프 기구입니다. 일반(maximal) 조인트로 만들면 기구 배율(3단·20°에서 약 8배) 때문에
반사 질량이 극단적으로 커져 PhysX 반복 솔버가 버티지 못하고 무너집니다(실측: 핀이 4 cm까지 벌어짐).
그래서 트리는 articulation으로 만들고 **루프를 닫는 회전 핀만 `excludeFromArticulation`** 으로 뺐습니다.
* 하부 `Actuator_*` prismatic 드라이브의 target을 음수로 줄수록 리프트가 올라갑니다.
  `mechanisms.scissor_slider_offset_for_height()` 가 목표 핀 높이에 해당하는 target을 계산합니다.
* 액추에이터 강성/감쇠는 하중과 기구 배율에서 자동 계산됩니다. `sag_tolerance`(기본 2 mm)는
  하중에 의한 플랫폼 정적 처짐 한도이고, 감쇠는 임계 감쇠입니다.

Isaac Sim 6.1 실측 (3단, 팔 1.2 m, 20°, 목표 핀 높이 2.2 m):

| 엔진 | 플랫폼 상승 (목표 +0.969 m) | 최대 핀 오차 |
|---|---|---|
| PhysX (CPU) | +0.977 m | 0.94 mm |
| Newton | +0.964 m | 0.94 mm |

### 2.7 Validate (물리 설정 검사)
*Validate stage* 로 다음을 검사하고, *Auto-fix* 로 고칠 수 있는 항목은 자동 수정합니다.

* 조인트 양쪽 프레임 불일치(→ realign), 존재하지 않는 body 대상, body 없는 조인트, 잘못된 limit
* 중첩된 rigid body, 충돌체/질량 없는 rigid body(→ 충돌체 추가), 비균일 스케일
* 동적 body의 triangle-mesh 충돌체(→ convexHull), PhysicsScene 없음(→ 생성)
* articulation 밖에서 두 body를 잇는 조인트 (Newton이 거부)
* articulation 안에서 월드에 직접 붙은 non-fixed 조인트 (PhysX는 월드-루트 사이에 fixed만 허용 → 그 DOF와 힘이
  articulation에서 빠짐. 월드에 고정한 베이스 링크를 두고 그 위에 조인트를 거세요)
* GPU 물리 + 폐루프 articulation 조합(→ CPU 물리로 전환)

### 2.8 Analysis (결과 기록·그래프)
기구를 선택하고 *Use selection* → *Start recording* → Play 하면 **매 물리 스텝마다** articulation의 상태를
기록합니다(Isaac Sim 6 텐서 API = 실제 시뮬레이션 값, SI 단위로 환산).

| 채널 | 내용 | 단위 |
|---|---|---|
| `pos:<DOF>` | 조인트 위치 | deg / m |
| `force:<DOF>` | DOF 축 방향 힘(드라이브·액추에이터 힘) | N·m / N |
| `link_force:<link>:x/y/z`, `link_torque:...` | 링크로 들어오는 조인트의 6축 반력 (자식 조인트 좌표계) | N / N·m |

* *Plot*: DOF 힘 / DOF 위치 / 조인트 반력 크기, *Name filter* 로 채널 선택(최대 8계열, 색은 채널에 고정)
* 요약(최소·최대·최종값)과 그래프(Isaac `XYPlot`), *Export CSV*(Excel·pandas에서 바로 열림)
* 측정되지 않는 것: 루프를 닫는 조인트(articulation 제외) 반력 — PhysX 텐서 API가 제공하지 않음.
  Newton은 힘을 제공하지 않아 위치만 기록됩니다.

**정확도 검증 (Isaac Sim 6.1, PhysX CPU)** — 정답이 알려진 문제와 비교:

| 케이스 | 측정 | 기준 | 오차 |
|---|---|---|---|
| 수직 프리즘 10 kg 지지력 | 98.100 N | m·g = 98.100 N | 0.00 % |
| 수평 링크 2 kg × 0.5 m 지지 토크 | 9.8100 N·m | 9.8100 N·m | 0.00 % |
| 시저 1단·한쪽 액추에이터 힘 | 443.61 N | 가상일 해석해 443.63 N | −0.01 % |
| 시저 3단·양쪽, 좌우 합 (높이 1.5–2.5 m, 올림·내림) | 1589–3354 N | 해석해 | **±0.4 % 이내** |

해석해 `mechanisms.scissor_actuator_force()` 는 실제 USD 모델의 위치에너지 수치미분과 0.1 % 이내로 일치합니다.

> **양쪽(좌·우) 시저의 좌우 분담은 강체 모델로 정해지지 않습니다.** 두 쪽이 같은 플랫폼을 받치는 정역학적
> 부정정 구조라서, 합은 정확하지만(위 표) 좌우 비율은 솔버가 중복 구속을 처리하는 방식에 따라 달라집니다
> (실측 1358.8 N 대 1298.1 N). 상용 MBD 툴도 같은 성질이 있으며(2.10), 강성으로 분담을 구하려면
> 정확해석기의 *bushings* 모드를 쓰세요(2.9).

### 2.9 Exact solver (정확해석기, Exudyn)
Analysis 섹션 아래쪽. PhysX 시뮬레이션(근사, 0.1–0.5 % 오차)과 별도로, 같은 USD 모델을 **구속 방정식
기반 다물체 동역학**으로 풉니다. 절대좌표와 라그랑주 승수를 쓰는 방식입니다(RecurDyn은 상대좌표 순환
정식화이고, 폐루프에서 라그랑주 승수를 씁니다 — 2.10).

* 바디: RigidBodyAPI + MassAPI(질량·질량중심·주관성)를 그대로 사용 — 모델러가 만든 값.
  kinematic 바디와 바디가 아닌 조인트 대상은 ground.
* 조인트: fixed / revolute / prismatic / spherical / D6(잠긴 축)이 **정확한 구속**이 되고, 루프를 닫는
  조인트(`excludeFromArticulation`)도 포함됩니다. 라그랑주 승수가 곧 조인트 반력입니다.
* 드라이브(stiffness > 0)는 목표값으로 **위치 지정 구동**(RecurDyn의 Motion)이 되고, 그 승수가 정확한
  액추에이터 힘입니다. damping만 있는 드라이브와 조인트 리밋은 해석에 들어가지 않습니다(요약에 표시).
* *Solve static*: 현재 드라이브 목표에서의 정적 평형. *Sweep drive*: 드라이브를 *From*→*To* 로 움직이며 각
  점에서 정적 평형(x축 = 드라이브 값). *Sweep drives* 를 비우면 모든 드라이브를 함께 움직입니다(양쪽 리프트).
* 결과 채널은 기록과 같고(`pos:`, `force:`) 조인트별 6축 반력이 `joint_force:<joint>:x/y/z`,
  `joint_torque:...`(조인트 좌표계, body1에 작용하는 힘 = PhysX 부호)로 추가됩니다. 같은 그래프·CSV를 씁니다.
* *Redundant* — 중복 구속 처리(2.10 참고):
  * `release`(기본): 종속 방정식을 수치적으로 찾아(랭크 판별) 루프 조인트부터 하나씩 해제하되 기구의 자유도는
    유지합니다. Adams·MotionSolve처럼 자동으로 해제하고, 해제 내역을 알려주지 않는 Adams·Simscape와 달리
    **해제한 방정식 목록을 요약에 표시**합니다. 합력은 정확하고, 중복 경로 사이 분담은 여러 해 중 하나입니다.
  * `bushings`: 구동되지 않는 조인트를 강성 부싱(기본 1e8 N/m, 1e7 N·m/rad, 자유축 강성 0)으로 바꿉니다.
    Adams·Altair가 비대칭 반력의 해법으로 권장하는 방식으로, 분담이 강성에 따라 물리적으로 정해집니다.

**정확도 (오프라인 + Isaac Sim 6.1 Kit, 해석해와 비교)**

| 케이스 | 정확해석기 | 기준 | 오차 |
|---|---|---|---|
| 수직 프리즘 10 kg 지지력 | 98.100 N | m·g | < 1e-6 |
| 회전 링크 2 kg × 0.5 m, 0°/50°, 조인트 축 X/Y/Z, 베이스 회전 3종 | | m·g·L/2·cos θ | < 1e-6 N·m |
| 시저 1·2·3단 한쪽, 35/45/60° 액추에이터 힘 | 예: 288.647 N | 가상일 288.647 N | < 1e-6 |
| 시저 3단 양쪽 좌우 합 (`release`) | 1982.21 N | 1982.21 N | < 1e-6 |
| 같은 모델 좌/우 분담 | `release` 217.6 / 1764.6 N, `bushings` 991.3 / 991.3 N | 대칭 구조 | bushings 합 +0.016 % (부싱 변형) |
| 같은 모델, 시저 높이 1.23 → 2.2 m (20° → 37.7°) 스윕 11점 (Isaac Sim GUI) | `release` 합 4204.63 → 1982.21 N | 해석해 | `release` 0.0000 %, `bushings` 최대 +0.09 % |
| 진자 주기(동적, generalized-α) | | 2π√(I/mgd) | < 0.2 % |

PhysX와 달리 루프를 닫는 조인트의 반력도 나오고, 결과가 시간 간격과 무관합니다.

### 2.10 과구속(중복 구속) 처리 — 다물체 동역학 프로그램 비교
공식 문서, 논문 원문, 소스 코드로 **확인된 내용만** 정리했습니다.

| 프로그램 | 확인된 내용 | 근거 |
|---|---|---|
| Adams 계열 | 중복 구속을 자동으로 검출해 제거합니다. 그래서 대칭 구조에서도 반력이 비대칭으로 나올 수 있고, 해법으로 부싱을 권장합니다 | [Solid Solutions](https://www.solidsolutions.co.uk/resolving-redundant-constraints-and-asymmetrical-results-in-motion-simulation/) |
| Adams, Simscape | 어떤 구속을 제거했는지 사용자에게 알려주지 않습니다 | [arXiv 2512.19289](https://arxiv.org/pdf/2512.19289) |
| SOLIDWORKS Motion (Adams 계열 솔버) | 적분기 GSTIFF / WSTIFF / SI2 | [SOLIDWORKS 도움말](https://help.solidworks.com/2018/english/solidworks/motionstudies/c_integration_methods_analysis.htm) |
| Altair Inspire / MotionSolve | 중복 방정식을 제거하고, 제거된 자유도에는 반력이 없습니다. 유연 조인트를 권장합니다 | [Altair 도움말](https://help.altair.com/inspire/en_us/topics/inspire/motion/redundant_constraints_r.htm) |
| Abaqus | 일관된 과구속은 가능하면 자동으로 제거하고 경고를 내며, 일관되지 않은 과구속은 오류로 처리합니다. 과구속은 부정확한 해나 수렴 실패를 일으킬 수 있고, 해가 나와도 반력이 부정확할 수 있습니다 | [Abaqus Overconstraint checks](https://abaqus-docs.mit.edu/2017/English/SIMACAECSTRefMap/simacst-c-overconstraintchecks.htm) |
| RecurDyn | 상대좌표 순환 정식화를 쓰고, 폐루프는 컷 조인트와 라그랑주 승수로 닫습니다 | [Bae·Haug 계열 논문](https://www.bohrium.com/paper-details/a-recursive-formulation-for-flexible-multibody-dynamics-part-ii-closed-loop-systems/812414973014704128-3776), [Solver Theoretical Manual](https://www.scribd.com/doc/78016413/Recurdyn-Solver-Theoretical-Manual) |
| Simcenter 3D Motion | 가변 스텝 BDF 적분을 씁니다(실시간 옵션은 고정 스텝 BDF) | [Siemens 지원 문서](https://support.sw.siemens.com/en-US/okba/KB000112713_EN_US/About-Simcenter-3D-Motion-Model-Export/index.html) |
| Exudyn | 기본은 잉여(절대) 좌표 정식화이며, 최소좌표 `KinematicTree`도 지원합니다 | [Exudyn 문서](https://exudyn.readthedocs.io/) |
| Exudyn | 자유도와 중복 구속은 `ComputeSystemDegreeOfFreedom`(numpy `matrix_rank` / SVD)으로 판별합니다. 특이 자코비안을 허용하는 `ignoreSingularJacobian`은 Eigen FullPivLU를 쓰며, 문서가 잘못된 결과가 나올 수 있다고 경고합니다 | Exudyn 1.13.6 소스 |
| 아주대 박태원 연구실 자체 해석기 | 평면 강체는 데카르트 좌표 (x, y, θ), 대변형 보는 ANCF로 표현합니다. 라그랑주 승수를 쓴 가속도 수준 증강 행렬식(식 14)을 4차 Runge-Kutta로 적분하고, 조인트 반력은 승수 λ에서 구합니다. 슬라이딩 조인트는 non-generalized 좌표로 정식화합니다 | [Seo·Jung·Park·Chai, JSME Int. J. C 48(2), 2005](https://www.jstage.jst.go.jp/article/jsmec/48/2/48_2_224/_article) |
| 아주대 박태원 연구실 | 여유구속이 있는 위성 태양전지판은 강체 모델로는 반력을 신뢰할 수 없어 패널을 **유연체(CMS)**로 모델링했습니다. 강체 비교 모델에서는 회전조인트를 하나만 두어 여유구속을 없앴습니다(해석은 DADS) | [서종휘·박태원 외 2003](https://www.koreascience.kr/article/CFKO200333239349542.pdf) |

이론 배경: 중복 구속이 있으면 강체 모델의 반력은 **유일하지 않습니다**. 어떤 반력 성분이 유일하게
정해지는지는 [Wojtyra 2005](https://link.springer.com/article/10.1007/s11044-005-5967-0)가,
정해지지 않는 성분을 고르는 실용 기준(가중 최소노름)은
[García de Jalón & Gutiérrez-López 2013](https://link.springer.com/article/10.1007/s11044-013-9358-7)이
다룹니다.

이 디자이너에 반영한 것:
* `release` — Adams·MotionSolve·Abaqus처럼 중복 구속을 자동으로 해제합니다. 해제 내역을 알려주지 않는
  Adams·Simscape와 달리, 해제한 방정식 목록을 요약에 표시합니다. 해제된 방정식에는 반력이 없습니다(Altair와 같음).
* `bushings` — Adams·Altair가 권장하는 대로 강성 부싱으로 분담을 구합니다. 실제 분담이 중요하면 실제 부품
  강성을 넣으세요. 아주대 연구처럼 부품 자체를 유연체로 만드는 것이 더 정확하지만, 이 디자이너는 아직 강체만 다룹니다.

---

## 3. 스크립트에서 사용하기

`core` 패키지는 `pxr`만 사용하므로 Script Editor, standalone Python, 오프라인 `usd-core` 어디서든 동작합니다
(`core.mbd` 만 `exudyn` 추가 필요).

```python
from gomjb.physics_designer.core.mbd import MbdModel

model = MbdModel(stage, "/World/ScissorLift", redundancy="release")   # 또는 "bushings"
print(model.report())                     # 자유도, 드라이브, 해제한 중복 방정식
rec = model.static_sweep(["Actuator_L", "Actuator_R"], [0.0, -0.1, -0.2])   # m (회전은 deg)
rec.to_csv("lift_static.csv")
```

```python
import omni.usd
from gomjb.physics_designer.core import joints, mechanisms, physics, snapping, transform
from gomjb.physics_designer.commands import designer_edit, run_edit

stage = omni.usd.get_context().get_stage()
base = stage.GetPrimAtPath("/World/Base")
arm = stage.GetPrimAtPath("/World/Arm")

# 하나의 Undo 단계로 묶기
with designer_edit(["/World/Base", "/World/Arm"], "Assemble arm"):
    physics.apply_rigid_body([base, arm])
    snapping.mate_faces(arm, base, "+Z")                       # base 위에 올리기
    spec = joints.JointSpec("revolute", axis="X", lower=-45, upper=45,
                            drive=joints.DriveSpec(stiffness=1e4, damping=1e2))
    joints.create_joint_at(stage, spec, base, arm, location="body1_bbox", world_axis="Y")

# 템플릿
run_edit(lambda s: mechanisms.build_scissor_lift(s, stages=4, arm_length=1.0))
```

**시뮬레이션 중 제어와 상태 읽기는 텐서 API로** 하세요. 재생 중에는 물리 상태가 USD가 아닌
Fabric/텐서에 있으므로, USD에서 위치를 읽으면 초기값만 나옵니다. Newton은 실행 중 USD 속성 변경도 읽지 않습니다.

```python
from isaacsim.core.experimental.prims import Articulation, RigidPrim

lift = Articulation("/World/ScissorLift")
dofs = [i for i, n in enumerate(lift.dof_names) if n.startswith("Actuator")]
target = mechanisms.scissor_slider_offset_for_height(1.2, 20, 3, height=2.2)
lift.set_dof_position_targets([[target] * len(dofs)], dof_indices=dofs)
z = RigidPrim("/World/ScissorLift/Platform").get_world_poses()[0].numpy()[0, 2]
```

예제:
* `examples/offline_build_scene.py` — Isaac Sim 없이(`pip install usd-core`) 데모 씬 `.usda` 생성
* `examples/isaac_standalone_scissor_lift.py` — `python.bat` 로 실행, 시저 리프트를 목표 높이까지 올리며 높이 출력

---

## 4. 구조

```
exts/gomjb.physics_designer/
├── config/extension.toml
├── gomjb/physics_designer/
│   ├── extension.py        # IExt: 커맨드 등록, Tools 메뉴, 창
│   ├── commands.py         # PhysicsDesignerEditCommand, run_edit(), designer_edit()
│   ├── recording.py        # Analysis: 물리 스텝마다 articulation 상태·힘 기록 (Isaac Sim 런타임)
│   ├── core/               # Kit 비의존 (pxr만 사용)
│   │   ├── math_utils.py   # 양자화, 회전/프레임, 분해/합성
│   │   ├── transform.py    # 월드 기준 이동/회전/정렬/분배/드롭
│   │   ├── snapping.py     # 격자/각도/타깃/최근접점/면맞춤/앵커
│   │   ├── joints.py       # 조인트 생성, 로컬 프레임 계산, 재정렬
│   │   ├── physics.py      # rigid body, collider, mass, material, scene
│   │   ├── modeling.py     # 모델러: 형상 바디, 해석적 질량 특성, Link 핀 앵커
│   │   ├── mechanisms.py   # 파라메트릭 템플릿 + 시저 해석해
│   │   ├── results.py      # 시계열 기록, CSV
│   │   ├── mbd.py          # 정확해석기: USD → Exudyn 구속 모델, 정적·스윕·동적, 중복 구속 처리
│   │   ├── validation.py   # 검사 + 자동 수정
│   │   └── snapshot.py     # 경로 단위 Undo 스냅샷
│   ├── ui/                 # omni.ui 창(+ modeler_section, analysis_section), 위젯, 라이브 스냅
│   └── tests/              # Kit 테스트 러너용 테스트
├── tests_offline/          # pytest + usd-core (Kit 없이 실행)
└── examples/
```

**Undo 방식**: 각 작업 전에 수정할 prim 하위 트리만 익명 레이어에 복사해 두고, 작업 후 새로 생긴
prim 경로를 비교해 기록합니다. Undo 시 새 prim은 삭제하고 수정된 prim은 원래 spec으로 덮어씁니다
(형제 순서 유지). 무거운 메쉬가 있는 씬에서도 레이어 전체를 복사하지 않습니다.

---

## 5. 테스트

```bash
pip install usd-core pytest
cd exts/gomjb.physics_designer
python -m pytest tests_offline -q
```

`tests_offline/kit_stubs.py` 는 omni.ui / omni.usd / omni.kit.commands 의 최소 대역으로, 실제 창의
버튼 콜백과 Undo 커맨드를 usd-core 스테이지에서 실행해 봅니다.

Kit 내부 테스트(`gomjb/physics_designer/tests`: 실제 omni.ui 창, 조인트·템플릿 생성, Undo/Redo, 오류 롤백)는
Isaac Sim 자체 확장 테스트와 같은 방식으로 실행합니다:

```bat
set ISAAC=E:\isaac-sim-standalone-6.1.0-windows-x86_64
"%ISAAC%\kit\kit.exe" --empty --enable omni.kit.test --/exts/omni.kit.test/runTestsAndQuit=true ^
  --/exts/omni.kit.test/testExts/0=gomjb.physics_designer --/exts/omni.kit.test/testExtArgs/0=--no-window ^
  --ext-folder <repo>\exts --ext-folder "%ISAAC%\exts" --ext-folder "%ISAAC%\extscache" ^
  --ext-folder "%ISAAC%\extsDeprecated" --ext-folder "%ISAAC%\extsInternal" --ext-folder "%ISAAC%\apps" --no-window
```

## 6. 알려진 제한

* Live snap은 USD 변경 알림으로 기즈모 드래그 결과를 다음 프레임에 양자화하는 방식이라, 드래그 중
  약간의 떨림이 있을 수 있습니다. Kit 뷰포트 자체 스냅 기능과 함께 쓰는 것은 권장하지 않습니다.
* 템플릿은 Z-up 기준으로 작성되며 Y-up 스테이지에서는 루트를 회전시켜 배치합니다.
* 드라이브 stiffness 단위는 PhysX 규약을 따릅니다(각도 드라이브는 deg 기준).
* PhysX GPU 물리에서 폐루프 articulation(시저)과 다른 articulation을 한 씬에 두면 발산합니다(Isaac Sim 6.1 실측).
  디자이너가 만드는 씬은 CPU 물리이고, 검증기가 GPU 씬에서 이를 경고·수정합니다.
