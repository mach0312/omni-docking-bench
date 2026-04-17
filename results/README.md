# results/

이 디렉토리는 `experiment_runner_node` 가 런타임에 출력하는 실험 결과의
**호스트 공유 마운트 포인트** 다. Docker / docker-compose 실행 시
`../results` (= 이 디렉토리) 가 컨테이너의 `icros_test_setup/results/`
로 바인드되며, 생성된 CSV · plot 이 여기에 누적된다.

## 저장소 정책

- 본 디렉토리 **자체**는 git 에 추적되지만, 그 **하위의 실제 실험 출력은
  `.gitignore`** 로 제외된다.
- **샘플 데이터** (`_sample` 접미사) 만은 예외적으로 git 에 커밋되어 있어,
  사용자가 전체 실험을 돌리기 전에 플로팅 파이프라인이 정상 동작하는지를
  확인할 수 있다.
- 샘플 데이터는 `src/icros_test_setup/results/` 아래에도 동일한 파일이
  포함되어 있다. 실제 실험 결과의 해석 방법은 루트
  [`docs/EXPERIMENT_ANALYSIS.md`](../docs/EXPERIMENT_ANALYSIS.md) 를 참고.

## Sample 데이터

| 디렉토리 | 내용 |
|----------|------|
| `src/icros_test_setup/results/kinematic_sample/` | Kinematic 컨트롤러 144회 시행의 요약(`summary.csv`) / 시계열(`timeseries/*.csv`) / 지표(`metrics.csv`) / 플롯(`plots/`) |
| `src/icros_test_setup/results/MPPI_sample/` | MPPI 컨트롤러 144회 시행의 요약 / 시계열 / 지표 / 플롯 |
| `src/icros_test_setup/results/comparison_sample/` | 상기 두 컨트롤러 비교 결과 (`plot_compare.py` 출력 — `comparison.csv`, `*.png`, `analysis.md`) |

샘플 데이터로 파이프라인을 확인하려면:

```bash
# analyzer 단독 재실행
python3 -m icros_test_setup.experiment_analyzer \
  src/icros_test_setup/results/kinematic_sample

# 컨트롤러 비교 시각화
python3 src/icros_test_setup/scripts/plot_compare.py \
  src/icros_test_setup/results/kinematic_sample \
  src/icros_test_setup/results/MPPI_sample
```

## 출력 디렉토리 포맷

```
<controller>_<YYYYMMDD_HHMMSS>/
├── summary.csv
├── timeseries/
│   └── trial_NNN.csv
├── metrics.csv
├── report.txt
└── plots/
    ├── convergence.png
    ├── direction_bars.png
    ├── polar.png
    ├── heading_bars.png              # (heading ≥ 2 시)
    ├── heatmap_settling.png          # (heading ≥ 2 시)
    └── steering_vs_settling.png
```

컬럼 정의 · 지표 정의 · 비교 플롯 설명은 모두
[`docs/EXPERIMENT_ANALYSIS.md`](../docs/EXPERIMENT_ANALYSIS.md) 에 있다.
