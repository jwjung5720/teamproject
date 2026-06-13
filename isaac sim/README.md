## 작업 개요

| 항목 | 내용 |
|------|------|
| 시뮬레이터 | NVIDIA Isaac Sim 5.1 (Windows) |
| 로봇 | TurtleBot3 Burger |
| 작업 내용 | Isaac Sim에서 TurtleBot3 소환 및 전진 이동 동작 확인 |

---

##수행한 작업

1. **Isaac Sim 5.1 환경 설치 및 세팅**
   - Windows에서 Isaac Sim 5.1 standalone 설치
   - TurtleBot3 USD 씬(`test_1.usd`) 구성

2. **TurtleBot3 소환 및 기본 이동 확인**
   - Isaac Sim Script Editor에서 Python 스크립트로 TurtleBot3 불러오기
   - 전진 이동 명령 입력 및 동작 확인

3. **ROS 2 연결 시도**
   - WSL2 ROS 2 Jazzy 환경과 Isaac Sim 브릿지 연결 실험

---

## 사용 환경

- OS: Windows 11
- GPU: NVIDIA RTX 4070 Ti
- Isaac Sim: 5.1 standalone
- ROS 2: Jazzy (WSL2 Ubuntu)
