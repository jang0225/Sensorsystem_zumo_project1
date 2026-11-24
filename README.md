# Aerospace Sensor Systems - Zumo Project 1

Arduino Uno와 Zumo Shield를 사용하여 지정된 코스를 자율 주행하도록 구현한 항공우주 센서시스템 텀프로젝트 1입니다.

## 주요 기능

- 6채널 IR 센서를 이용한 Section 1 경계선 주행
- 교차 지점 감지 후 Section 2 중앙선 진입
- 초음파 센서를 이용한 차단기 감지 및 정지/재출발
- 체크포인트 감지, 정지 및 유턴
- 중앙선을 따라 출구까지 복귀
- 상태 기계 기반 비차단 주행 제어
- EEPROM 기반 시험 로그 저장
- 실제 Uno와 시뮬레이터 빌드 환경 분리

## 저장소 구성

```text
firmware/
  include/             Zumo 하드웨어 및 주행 제어 코드
  src/main.cpp         상태 기계와 전체 주행 로직
  platformio.ini       PlatformIO 빌드 설정
docs/
  report_figures/      보고서에 사용한 실험 사진과 그림
  *.pdf, *.docx        최종 보고서
  *.md                 계획, 보고서 원문 및 개발 기록
media/
  result.mp4           실제 주행 결과 영상
```

## 빌드

PlatformIO가 설치된 환경에서 다음과 같이 빌드합니다.

```powershell
cd firmware
pio run -e uno
```

시뮬레이터용 펌웨어는 별도 환경으로 빌드합니다.

```powershell
pio run -e sim
```

실제 보드 업로드:

```powershell
pio run -e uno -t upload
```

## 문서

- `docs/2022-10949_최장혁_텀프로젝트1 보고서.pdf`: 최종 보고서
- `docs/TERM_PROJECT_1_DEVELOPMENT_LOG.md`: 실제 시험과 시뮬레이션을 포함한 개발 기록
- `docs/TERM_PROJECT_1_PLAN.md`: 초기 구현 및 시험 계획

## 작성자

2022-10949 최장혁

