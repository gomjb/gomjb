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

```bash
./isaac-sim.sh --ext-folder <repo>/exts --enable gomjb.physics_designer
```

지원 버전: Isaac Sim 4.x / 5.x (Kit 105+). 외부 의존성은 없고, 물리 스키마는 USD에 포함된
`UsdPhysics`만 사용하므로 PhysX가 그대로 해석합니다.

---

## 2. 창 구성과 사용법

창 맨 위에는 현재 선택이 **선택 순서대로** 표시됩니다 (`0: Base [rigid]  1: Arm`).
여러 기능이 이 순서를 사용합니다 — *첫 번째 = 기준/body0*, *마지막 = 타깃*.

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
* **Pivot → grid / BBox corner → grid / Rotation → step** : 위치·bbox 모서리·자세(XYZ 오일러)를 격자/각도에 맞춤
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

### 2.5 Physics properties (물리 속성)
Rigid / Kinematic / Collider only (근사: none·convexHull·convexDecomposition·meshSimplification·
boundingCube·boundingSphere), 질량·밀도, 물리 머티리얼(정/동마찰, 반발계수 — `physics` purpose로
바인딩해 렌더 머티리얼과 분리), Articulation root, 두 객체 간 충돌 필터, Physics scene, Ground plane,
물리 속성 일괄 제거.

### 2.6 Mechanism templates (메커니즘 템플릿)
파라미터를 입력해 바로 시뮬레이션 가능한 기구를 생성합니다 (단위: stage unit, 기본 m / deg).

| 템플릿 | 구성 |
|---|---|
| Pendulum chain | 캡슐 링크 N개 + revolute 연쇄, 초기 각도, 감쇠, articulation 옵션 |
| Hydraulic cylinder | 배럴 + 로드 + 구동 prismatic(스트로크 제한, PD 드라이브), 장착 앵커 `MountBase`/`MountRod` |
| Hinged door | 고정 포스트 + 도어 패널, 개방각 제한 revolute + 댐핑 |
| Linear slider | 고정 레일 + 캐리지, 행정 제한 prismatic |
| Scissor lift | N단 시저 암(좌/우), 중앙 핀·단간 핀, 하부 고정핀 + **구동 슬라이더(액추에이터)**, 상부 고정핀 + 자유 슬라이더 |

시저 리프트는 폐루프 기구이므로 articulation이 아닌 일반(maximal coordinate) 조인트로 만들어집니다.
하부 `Actuator_*` prismatic 드라이브의 target을 음수로 줄수록 리프트가 올라갑니다.
`mechanisms.scissor_slider_offset_for_height()` 로 목표 높이에 해당하는 target을 계산할 수 있습니다.

### 2.7 Validate (물리 설정 검사)
*Validate stage* 로 다음을 검사하고, *Auto-fix* 로 고칠 수 있는 항목은 자동 수정합니다.

* 조인트 양쪽 프레임 불일치(→ realign), 존재하지 않는 body 대상, body 없는 조인트, 잘못된 limit
* 중첩된 rigid body, 충돌체/질량 없는 rigid body(→ 충돌체 추가), 비균일 스케일
* 동적 body의 triangle-mesh 충돌체(→ convexHull), PhysicsScene 없음(→ 생성)

---

## 3. 스크립트에서 사용하기

`core` 패키지는 `pxr`만 사용하므로 Script Editor, standalone Python, 오프라인 `usd-core` 어디서든 동작합니다.

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

예제:
* `examples/offline_build_scene.py` — Isaac Sim 없이(`pip install usd-core`) 데모 씬 `.usda` 생성
* `examples/isaac_standalone_scissor_lift.py` — `./python.sh` 로 실행, 시저 리프트를 드라이브로 상승시키며 높이 출력

---

## 4. 구조

```
exts/gomjb.physics_designer/
├── config/extension.toml
├── gomjb/physics_designer/
│   ├── extension.py        # IExt: 커맨드 등록, Tools 메뉴, 창
│   ├── commands.py         # PhysicsDesignerEditCommand, run_edit(), designer_edit()
│   ├── core/               # Kit 비의존 (pxr만 사용)
│   │   ├── math_utils.py   # 양자화, 회전/프레임, 분해/합성
│   │   ├── transform.py    # 월드 기준 이동/회전/정렬/분배/드롭
│   │   ├── snapping.py     # 격자/각도/타깃/최근접점/면맞춤/앵커
│   │   ├── joints.py       # 조인트 생성, 로컬 프레임 계산, 재정렬
│   │   ├── physics.py      # rigid body, collider, mass, material, scene
│   │   ├── mechanisms.py   # 파라메트릭 템플릿
│   │   ├── validation.py   # 검사 + 자동 수정
│   │   └── snapshot.py     # 경로 단위 Undo 스냅샷
│   ├── ui/                 # omni.ui 창, 위젯, 라이브 스냅
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
버튼 콜백과 Undo 커맨드를 usd-core 스테이지에서 실행해 봅니다. Kit 내부 테스트는
`gomjb/physics_designer/tests` 에 있으며 Isaac Sim의 테스트 러너로 실행합니다.

## 6. 알려진 제한

* Live snap은 USD 변경 알림으로 기즈모 드래그 결과를 다음 프레임에 양자화하는 방식이라, 드래그 중
  약간의 떨림이 있을 수 있습니다. Kit 뷰포트 자체 스냅 기능과 함께 쓰는 것은 권장하지 않습니다.
* 템플릿은 Z-up 기준으로 작성되며 Y-up 스테이지에서는 루트를 회전시켜 배치합니다.
* 드라이브 stiffness 단위는 PhysX 규약을 따릅니다(각도 드라이브는 deg 기준).
