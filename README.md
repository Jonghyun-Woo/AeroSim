# AeroSim

Randy Beard & Tim McLain, *Small Unmanned Aircraft: Theory and Practice*
(UAVBook) 기준의 6자유도(6-DOF) Aerosonde 고정익 모델과 successive-loop-closure
오토파일럿을 파이썬으로 구현한 시뮬레이션 환경입니다.

## 폴더 구조

```
AeroSim/
  environment.yml       # conda 환경 정의
  cfg/                  # 모델별 설정 (yaml)
    aerosonde.yaml       # Aerosonde 기체 제원 / 공력·추진 계수
    autopilot.yaml       # 오토파일럿 게인
  models/                # 6DOF 모델 (공유 베이스 클래스 + 기체별 구현)
    base_vehicle.py        # RigidBody6DOF: 쿼터니언 기반 6DOF 커널(향후 멀티콥터 등도 공유)
    aerosonde.py            # Aerosonde 공력/추진 모델 + trim solver
  ctrls/                 # 제어기 (PID 이외의 제어기도 이 폴더에 추가)
    pid.py                  # PID / rate-feedback PD 겸용 블록
    autopilot.py            # Ch.6 successive-loop-closure 오토파일럿
  sim/                   # 설정 로더, 로거 등 시뮬레이션 유틸리티
  run_aerosonde_sim.py   # 실행 스크립트 (trim -> 시뮬레이션 -> plot)
```

## 환경 설정 (conda)

이 프로젝트는 conda로 가상환경을 관리합니다. [Miniconda](https://docs.conda.io/en/latest/miniconda.html)
또는 Anaconda가 설치되어 있어야 합니다.

```bash
# 1. 저장소 루트에서 conda 환경 생성 (environment.yml 사용)
conda env create -f environment.yml

# 2. 환경 활성화
conda activate aerosim
```

환경을 이미 만든 뒤 `environment.yml`이 업데이트된 경우에는 아래로 동기화합니다.

```bash
conda env update -f environment.yml --prune
```

환경을 삭제하려면:

```bash
conda deactivate
conda env remove -n aerosim
```

## 실행

저장소 루트에서 (환경이 활성화된 상태로) 실행합니다.

```bash
conda activate aerosim
python run_aerosonde_sim.py
```

실행하면 다음 순서로 동작합니다.

1. `cfg/aerosonde.yaml`, `cfg/autopilot.yaml` 설정 로드
2. `Aerosonde.compute_trim()`으로 지정한 대기속도(Va)·경로각(gamma)에 대한 trim 계산
   (콘솔에 trim 잔차 및 trim 조종면/스로틀 값 출력)
3. trim 상태에서 시작하여 course/altitude/airspeed 커맨드를 순차로 변경하며 시뮬레이션
4. 3D 궤적, 위치, 대기자료(Va/alpha/beta), 자세/course, 조종면 시계열 plot 표시

## 참고

- `cfg/aerosonde.yaml`의 기체 제원·공력 계수는 UAVBook Appendix E에 공개된 Aerosonde
  값을 기반으로 하였습니다. 책의 부록과 대조하여 확인하는 것을 권장합니다.
- `cfg/autopilot.yaml`의 게인은 책의 전달함수 기반 게인 설계식으로 유도한 값이 아니라
  임의로 정한 시작값입니다. `run_aerosonde_sim.py`로 얻은 plot을 보면서 직접 튜닝해야
  합니다.
- MultiCopter, Dryden 바람 모델, 외란관측기(DOB) 등 기존 MATLAB 코드에 있던 기능은
  이번 파이썬 이식에는 포함되지 않았습니다. `models/base_vehicle.py`의
  `RigidBody6DOF`는 이런 다른 기체 모델도 공유할 수 있도록 설계되어 있어, 추후 필요할
  때 `models/` 아래에 새 서브클래스로 추가하면 됩니다.
