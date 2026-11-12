# Kinematics (`gomjb.kinematics`)

Physics Designer(`gomjb.physics_designer`)로 만든 Isaac Sim USD 모델의 **기구학 툴**입니다.

* **정확해석기**(Exudyn): 모든 조인트를 정확한 구속으로 풀어 정적 평형, 드라이브 스윕, 동적 해석을 하고,
  중복 구속을 처리합니다.
* **기구학 모드**: 시뮬레이션 중 기구를 PhysX 조인트로 풀지(`physics`), 정확해석기의 위치해석으로 움직일지
  고릅니다. 위치해석은 운동표(`table`) 또는 매 스텝 Co-Sim(`cosim`)입니다.

모델은 USD 하나입니다. 이 확장은 Physics Designer가 만든 body·조인트·모션을 그대로 읽고, 결과는 같은
그래프·CSV 뷰로 보여 줍니다.

> A kinematics tool for NVIDIA Isaac Sim on models made with the Physics Designer: an exact
> constraint-based multibody solver (Exudyn) and kinematic mechanisms driven by a motion table or a
> co-simulation with it.

---

## 1. 설치 / 실행

Physics Designer처럼 Isaac Sim의 `extsUser`에 **디렉터리 정션**으로 연결합니다. 이 확장은 Physics Designer에
의존하므로 둘 다 연결합니다.

```bat
set ISAAC=E:\isaac-sim-standalone-6.1.0-windows-x86_64
mklink /J "%ISAAC%\extsUser\gomjb.physics_designer" "<repo>\exts\gomjb.physics_designer"
mklink /J "%ISAAC%\extsUser\gomjb.kinematics" "<repo>\exts\gomjb.kinematics"
```

`Window > Extensions` → **Kinematics** → *Enable*(Physics Designer도 함께 켜집니다), 메뉴 `Tools > Kinematics`로
창을 엽니다. 명령줄에서는 `isaac-sim.bat --enable gomjb.kinematics`.

외부 패키지는 [Exudyn](https://github.com/jgerstmayr/EXUDYN) 하나이며, 확장을 켤 때 Kit의 `[python.pipapi]`가
`exudyn==1.13.6`을 설치합니다(`--no-deps`: 유일한 의존성 numpy는 Kit에 포함). 인터넷이 없는 PC에서는
Isaac Sim의 `python.bat -m pip install exudyn==1.13.6` 으로 미리 설치하세요.

---

## 2. 창 구성과 사용법

*Unit system*(Physics Designer 창과 같은 설정), *Mechanism*(기구 루트, *Use selection*), *Kinematics*,
*Exact solver*, *Results*(Physics Designer의 Analysis와 같은 그래프·요약·CSV), 아래 상태줄로 이루어집니다.

### 2.1 Exact solver (정확해석기, Exudyn)
*Exact solver* 섹션. PhysX 시뮬레이션(근사, 0.1–0.5 % 오차)과 별도로, 같은 USD 모델을 **구속 방정식
기반 다물체 동역학**으로 풉니다. 절대좌표와 라그랑주 승수를 쓰는 방식입니다(RecurDyn은 상대좌표 순환
정식화이고, 폐루프에서 라그랑주 승수를 씁니다 — 2.2).

* 바디: RigidBodyAPI + MassAPI(질량·질량중심·주관성)를 그대로 사용 — 모델러가 만든 값.
  kinematic 바디와 바디가 아닌 조인트 대상은 ground.
* 조인트: fixed / revolute / prismatic / spherical / D6(잠긴 축)이 **정확한 구속**이 되고, 루프를 닫는
  조인트(`excludeFromArticulation`)도 포함됩니다. 라그랑주 승수가 곧 조인트 반력입니다. 거리 조인트,
  PhysX 기어·랙앤피니언, mimic 조인트는 지원하지 않으며, 조용히 빠지지 않고 오류로 알립니다.
* 월드(ground)에 붙은 조인트는 body 쪽 프레임에 고정합니다. 조인트를 만든 뒤 옮기거나 복제한 기구도
  지금 서 있는 자리에서 풉니다(월드 쪽 프레임은 월드 좌표라 복제 시 옛 위치가 남습니다).
* 기구 밖의 동역학 body(예: 플랫폼에 고정한 짐)와 연결된 조인트는 해석에서 빼고 요약에 표시합니다.
  그 body를 ground로 보면 기구가 월드에 고정되기 때문입니다.
* 드라이브(stiffness > 0)는 목표값으로 **위치 지정 구동**(RecurDyn의 Motion)이 되고, 그 승수가 정확한
  액추에이터 힘입니다. damping만 있는 드라이브와 조인트 리밋은 해석에 들어가지 않습니다(요약에 표시).
* *Solve static*: 현재 드라이브 목표에서의 정적 평형. *Sweep drive*: 드라이브를 *From*→*To* 로 움직이며 각
  점에서 정적 평형(x축 = 드라이브 값). *Sweep drives* 를 비우면 모든 드라이브를 함께 움직입니다(양쪽 리프트).
* *Simulate*: *End time* 까지 동적 해석(generalized-α). 모션이 있는 조인트는 모션을 따르고, 나머지 드라이브는 목표를
  유지합니다(실측: 0 → 60° STEP 후 각도 오차 < 1e-6°, 정지 토크 = m·g·L/2·cos θ ± 0.1 %).
* 결과 채널은 기록과 같고(`pos:`, `force:`) 조인트별 6축 반력이 `joint_force:<joint>:x/y/z`,
  `joint_torque:...`(조인트 좌표계, body1에 작용하는 힘 = PhysX 부호)로 추가됩니다. 같은 그래프·CSV를 씁니다.
* *Redundant* — 중복 구속 처리(2.2 참고):
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

### 2.2 과구속(중복 구속) 처리 — 다물체 동역학 프로그램 비교
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

### 2.3 Kinematics (기구학 모드: 물리 / 운동표 / Co-Sim)
*Kinematics* 섹션에서 기구를 시뮬레이션할 때 어떻게 움직일지 고르고 *Apply kinematics*를
누릅니다(Undo 가능). 모드는 기구 루트에 저장되므로 모델과 함께 다니고, 복제된 기구에도 그대로 따라갑니다.

| 모드 | 기구를 움직이는 쪽 | 실행 중 계산 | 쓰는 경우 |
|---|---|---|---|
| `physics` (기본) | PhysX가 조인트와 드라이브를 풉니다(지금까지의 방식) | GPU/CPU 물리 | 하중이 기구의 운동을 바꾸는 경우 |
| `table` (운동표) | 정확해석기(Exudyn)가 Play 때 구동 행정을 따라 위치해석을 한 번 하고, 실행 중에는 보간만 합니다 | 표 보간(GPU 커널 1회) | 정해진 운동, 대량 병렬 |
| `cosim` (Co-Sim) | 정확해석기가 매 물리 스텝마다 위치해석을 합니다 | CPU 해석 1회/스텝 | 독립 구동이 여러 개인 경우, 운동표 범위를 벗어나는 경우 |

Process Simulate처럼 장치의 기구학과 물리를 분리하는 방식입니다. `table`·`cosim`에서는 기구의 body가
kinematic body가 되어 정확해석기의 해(루프가 정확히 닫힌 위치)를 따라가고, PhysX는 그 기구에 닿는
물체(상자, 부품)만 풉니다. 루프 벌어짐과 처짐이 0이 되고, 남는 오차는 운동표의 보간 오차뿐입니다.
모든 조인트(종속 조인트 포함)가 그 해를 따라 맞춰집니다.

* 모델은 USD 하나입니다. 정확해석기가 USD의 body·조인트를 그대로 읽고(2.1), 모드를 바꾸면
  body를 kinematic으로, 기구 안쪽 조인트를 비활성으로, articulation root를 제거로 바꿉니다. 바꾼 것마다
  표시를 남기므로 `physics`로 되돌리면 모델이 원래대로 복원되고, 정확해석기는 kinematic 모드에서도
  설계된 그대로의 기구를 봅니다. 기구 밖의 동역학 body와 연결된 조인트는 그대로 둡니다
  (예: 플랫폼에 고정한 짐은 물리로 따라옵니다).
* 구동값은 조인트 모션(Physics Designer 2.4, Include Motion)에서 옵니다. 모션이 없는 드라이브는 목표값을 유지합니다.
  모든 자유도에 드라이브가 있어야 합니다(구동값이 자세를 결정해야 하므로).
* **운동표**는 행정 하나를 다룹니다. 모션이 있는 구동 조인트들은 같은 모션이어야 하고(예: 리프트 양쪽
  액추에이터), 범위는 그 조인트들의 리밋입니다(범위 밖은 끝값으로 고정). 처음에 17점을 고르게 푼 뒤,
  보간 오차가 *Table tolerance*(기본 0.1 mm, 회전 오차는 기구 반경을 곱해 길이로 환산)를 넘는 구간을
  반으로 나눕니다. 오차는 구간 중점의 실제 해와 비교해 측정합니다. 시저가 평평해지는 쪽처럼 기구학이 급하게
  변하는 곳에만 점이 촘촘해집니다(등간격 61점은 37 mm가 틀렸습니다).
* 운동표는 *Apply kinematics* 때 미리 만들어지고(상태줄에 점 수·오차·시간 표시), Play 때는 **캐시**에서
  읽습니다. 캐시는 Kit 캐시 폴더 `${omni_cache}/gomjb.kinematics/motion_tables`(이 설치에서는
  `kit\cache\...`, 파일 하나 약 0.4 MB)에 있고, 기구학 fingerprint(아래)와 스테이지 단위로 구분합니다. 모델을
  고치면 새로 만들고, GUI와 배치 실행(python.bat)이 같은 캐시를 씁니다.
* 같은 기구의 복제본(복제 환경, 복사본)은 자동으로 묶여 운동표 하나와 커널 실행 하나를 같이 씁니다.
  Isaac Sim의 cloner(기본 `copy_from_source=False`)처럼 원본을 상속해 만든 복제본은 같은 USD spec으로
  이루어졌다는 것(prim stack)만으로 묶고, 속성은 읽지 않습니다. 그 밖의 복사본은 fingerprint로 비교합니다.
  fingerprint는 루트 기준 body 자세와 조인트 속성(루트 밖에 있는 그 기구의 조인트 포함)입니다. 복사한 뒤
  고친 기구는 따로 계산합니다.
* 실행: CPU 물리는 kinematic target을, GPU 물리는 자세와 질량중심 속도(다음 자세까지의 차분)를 매 스텝
  넣습니다(PhysX 110.3 GPU 텐서 API에는 kinematic target이 구현되어 있지 않음). 어느 쪽이든 PhysX가
  body의 속도를 알고 있어 위에 놓인 물체가 접촉과 마찰로 실려 갑니다.
* kinematic 기구는 articulation이 아니므로 Physics Designer의 *Start recording*은 쓰지 않습니다. 기구의 힘·반력은 정확해석기
  (2.1)로 구합니다. 상태 메시지와 로그에 운동표의 점 수, 측정 오차, 생성 시간이 나옵니다.

**검증**

| 케이스 | 결과 | 기준 |
|---|---|---|
| 3단 양면 시저, 운동표(0.1 mm), 행정 전 구간 35점 | 플랫폼 상승 오차 ≤ 0.1 mm, 팔 회전 ≤ 0.01°, 플랫폼 수평 | 해석식 n·L·(sin a − sin a0) |
| 같은 모델, 운동표 크기 | 419점, 3.8 s, 측정 오차 0.081 mm / 0.001° | |
| Co-Sim, 매 스텝 | 오차 < 1e-7 m | 해석식 |
| Isaac Sim, 2단 리프트 + 상자, CPU 물리 (`table`, `cosim`) | 플랫폼 0.69 m 상승 ±1 mm, 상자가 함께 올라감(±10 mm) | Kit 테스트 |
| Isaac Sim, 3단 리프트 + 상자 16개 복제, GPU 물리 | 플랫폼 상승 1.195 m 오차: 운동표 0.007 mm, Co-Sim 0.000 mm, 상자 16/16 | 해석식 |
| 같은 모델 1024 / 4096개 복제, 운동표, GPU(RTX 5060 Ti, 60 Hz) | 352 / 221 steps/s, 상자 1024/1024, 4096/4096, 오차 0.007 mm | |
| 같은 모델 1024개, Co-Sim, GPU | 146 steps/s (CPU 해석 1회/스텝을 모든 복제본이 공유) | |
| 비교: `physics` 모드, rigid 루프 1024 / 4096개, 상자 없음 | 165 / 79 steps/s, 처짐 4.5 mm, 루프 벌어짐 0.4 mm | |

대량 병렬로 돌릴 때는 Fabric 갱신을 꺼야 합니다(`--/physics/fabricUpdateTransformations=false`,
`fabricUpdateVelocities`, `fabricUpdateForceSensors`, `fabricUpdateJointStates` 모두 false — Isaac의
`isaacsim.exp.full.newton.kit`과 같은 값). 켜 두면 매 스텝 모든 body 변환을 CPU로 Fabric에 복사해 1024개에서
22배 느려집니다(GPU 사용률 5%). 화면이 필요하면 `--/physics/fabricUseGPUInterop=true`.
Play 직후 첫 스텝(4096개 복제): 운동표가 캐시에 있으면 **7.7 s**입니다. 이 중 디자이너 코드는 1.3 s
(텐서 뷰 생성 0.6 s, 복제본 묶기 0.12 s)이고, 나머지는 PhysX가 4096개 env를 올리는 시간입니다. 처음 만드는
실행은 27.9 s입니다. 캐시와 복제본 묶기를 넣기 전에는 52 s, 그 전에는 307 s였습니다.

---

## 3. 스크립트에서 사용하기

`core` 패키지는 `pxr`, numpy, `gomjb.physics_designer.core`를 쓰고, 해석에는 `exudyn`이 필요합니다.
Script Editor, standalone Python, 오프라인 `usd-core` 어디서든 동작합니다.

```python
from gomjb.kinematics.core.mbd import MbdModel

model = MbdModel(stage, "/World/ScissorLift", redundancy="release")   # 또는 "bushings"
print(model.report())                     # 자유도, 드라이브, 해제한 중복 방정식
rec = model.static_sweep(["Actuator_L", "Actuator_R"], [0.0, -0.1, -0.2])   # m (회전은 deg)
rec.to_csv("lift_static.csv")
```

kinematic 기구(2.3) — 복제 환경을 만든 뒤 Play만 하면 이 확장의 런타임이 복제본을 묶어 구동합니다:

```python
from gomjb.kinematics.core import kinematic_mode, kinematics
from gomjb.physics_designer.core import motion

root = stage.GetPrimAtPath("/World/envs/env_0/ScissorLift")
for side in "LR":
    motion.set_motion(stage.GetPrimAtPath(f"{root.GetPath()}/Joints/Actuator_{side}"),
                      motion.Motion("displacement", "STEP(TIME, 0, 0, 1, -0.3)"))
kinematic_mode.set_mode(root, "table", tolerance=1e-4)      # 또는 "cosim", "physics"(복원)
# ... GridCloner로 복제 → Play

table = kinematics.build_table(kinematics.kinematic_model(stage, str(root.GetPath())))
print(len(table.stroke), table.error)                        # 점 수, 측정 보간 오차 (m, deg)
```

---

## 4. 구조

```
exts/gomjb.kinematics/
├── config/extension.toml   # gomjb.physics_designer에 의존, [python.pipapi] exudyn
├── gomjb/kinematics/
│   ├── extension.py        # IExt: Physics Designer 모션 런타임에 참여, Tools 메뉴, 창
│   ├── runtime.py          # Play 중 kinematic 기구: 복제본 묶기, 묶음마다 driver 하나
│   ├── driver.py           # 모든 복제본의 자세를 Warp 커널 하나로, 텐서 API로 입력 (CPU 타깃 / GPU 자세+속도)
│   ├── cache.py            # 운동표 캐시 폴더 (${omni_cache}/gomjb.kinematics/motion_tables)
│   ├── core/               # Kit 비의존 (pxr, numpy, exudyn)
│   │   ├── mbd.py          # 정확해석기: USD → Exudyn 구속 모델, 정적·스윕·동적, 중복 구속 처리
│   │   ├── kinematic_mode.py # 기구 모드(physics / table / cosim) 저장·전환·복원, 복제본 묶기
│   │   └── kinematics.py   # 위치해석: 적응형 운동표(캐시), Co-Sim
│   ├── ui/window.py        # "Kinematics" 창 (결과 뷰는 Physics Designer의 ui.result_view)
│   └── tests/              # Kit 테스트 러너용 테스트
└── tests_offline/          # pytest + usd-core + exudyn (Physics Designer의 Kit 대역 kit_stubs 사용)
```

Physics Designer와의 경계: Physics Designer의 모션 런타임(`motion_runtime`)이 Play 중 스텝을 돌리고, 이 확장은
거기에 참여자로 등록합니다(`skips` / `start` / `step` / `stop`). Physics Designer는 이 확장을 알지 못합니다.

---

## 5. 테스트

```bash
pip install usd-core pytest exudyn==1.13.6
cd exts/gomjb.kinematics
python -m pytest tests_offline -q
```

Kit 내부 테스트는 두 확장을 한 번에 돌립니다:

```bat
set ISAAC=E:\isaac-sim-standalone-6.1.0-windows-x86_64
"%ISAAC%\kit\kit.exe" --empty --enable omni.kit.test --/exts/omni.kit.test/runTestsAndQuit=true ^
  --/exts/omni.kit.test/testExts/0=gomjb.physics_designer --/exts/omni.kit.test/testExts/1=gomjb.kinematics ^
  --/exts/omni.kit.test/testExtArgs/0=--no-window ^
  --ext-folder "%ISAAC%\extsUser" --ext-folder "%ISAAC%\exts" --ext-folder "%ISAAC%\extscache" ^
  --ext-folder "%ISAAC%\extsDeprecated" --ext-folder "%ISAAC%\extsInternal" --ext-folder "%ISAAC%\apps" --no-window
```

## 6. 알려진 제한

* PhysX 110.3의 GPU 텐서 API에는 kinematic target이 없어, GPU에서는 자세와 속도를 직접 넣습니다(2.3).
* Isaac Sim 6.1의 Newton 통합은 복제된 env를 모두 **MuJoCo world 하나**에 넣습니다(`world_count = 1`).
  kinematic 모드는 Newton에서도 동작하지만(16개 복제: 플랫폼 오차 0.007 mm, 상자 16/16), 비용이 자유도의
  거의 세제곱으로 늘어 4개 184 steps/s, 16개 3–8.5 steps/s, 256개는 10분 안에 끝나지 않았습니다. 대량 병렬은
  PhysX GPU로 하세요(Isaac Lab은 env마다 world를 만듭니다).
* 운동표는 행정 하나만 다룹니다. 독립 구동이 여러 개면 Co-Sim을 쓰세요.
