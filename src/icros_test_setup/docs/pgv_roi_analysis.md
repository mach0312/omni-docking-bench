# PGV Sensor ROI Analysis — Heading Angle Constraint

## Sensor Specifications

| 항목 | 값 | 비고 |
|---|---|---|
| 모델 | PGV100-F200A-R4 | Pepperl+Fuchs |
| 읽기 영역 (FOV) | **120mm x 80mm** | 읽기 거리 100mm 기준 |
| 코드 밴드 | PGV-CA25 (Width:1) | Data Matrix 25mm x 25mm |
| 밴드 폭 | **25mm** | 단일 컬럼 |
| 읽기 거리 | 100mm | 센서 ~ 바닥 |
| 위치 분해능 | 0.1mm | 패킷 단위 |
| 발행 주기 | 20Hz | 실제 센서 기준 |

## 로봇 장착 조건

| 항목 | 값 |
|---|---|
| 센서 오프셋 (base_link → sensor) | x=0.0m, **y=0.51m** |
| 장착 방향 | 로봇 좌측 (Y+) |

## ROI 판단 기준

### 원리
PGV 센서의 카메라가 바닥의 Data Matrix 코드를 인식하려면, **코드 전체(25mm)**가 **읽기 영역(120mm)** 안에 완전히 포함되어야 한다.

### 횡이동 계산
로봇이 heading θ만큼 회전하면, 센서는 heading=0° 위치에서 횡방향으로 이동한다:

```
횡이동 = offset × sin(θ) = 510mm × sin(θ)
```

코드가 FOV 안에 완전 포함되기 위한 최대 허용 횡이동:

```
max_shift = (FOV_width - band_width) / 2
         = (120mm - 25mm) / 2
         = 47.5mm
```

### 최대 허용 heading

```
θ_max = arcsin(max_shift / offset)
      = arcsin(47.5 / 510)
      = ±5.3°
```

### Heading별 상세

| heading | 횡이동 | FOV 여유 | 판정 |
|---|---|---|---|
| ±1° | 8.9mm | +38.6mm | OK |
| ±2° | 17.8mm | +29.7mm | OK |
| ±3° | 26.7mm | +20.8mm | OK |
| ±4° | 35.6mm | +11.9mm | OK |
| ±5° | 44.4mm | +3.1mm | EDGE |
| ±5.3° | 47.1mm | +0.4mm | 한계 |
| ±6° | 53.3mm | -5.8mm | OUT |
| ±8° | 71.0mm | -23.5mm | OUT |
| ±10° | 88.6mm | -41.1mm | OUT |

## 실험 설계 반영

### 실기 유효 heading 범위
**0°, ±2°, ±3°, ±4°, ±5°** (총 9개)

- ±5°는 FOV 여유 3.1mm으로 경계 조건 — 센서 노이즈/장착 오차에 민감
- ±4° 이하가 안정적 동작 보장 구간

### 시뮬레이션 전용 (dummy_pgv)
±8°, ±10° 등 — dummy_pgv는 밴드 폭 미시뮬레이션 (무한 FOV)이므로 테스트는 가능하나 실기 재현 불가. 분석 시 별도 표기 필요.

## References
- [Pepperl+Fuchs PGV100-F200A-R4-V19 Product Page](https://www.pepperl-fuchs.com/en-us/products-gp25581/60355)
- [PGV100-F200A-B17-V1D Datasheet (PDF)](https://files.pepperl-fuchs.com/webcat/navi/productInfo/pds/285693-100000_eng.pdf)
