#!/bin/bash
# 시뮬레이션 실행 → 종료 후 자동으로 그래프 생성

WS=/home/amg/tita_0309_ws/tita_ws
DATA_DIR=$WS/src/tita_bal_ign/data

source $WS/install/setup.bash

echo "=== 시뮬레이션 시작 ==="
ros2 launch tita_bal_ign tita_pos_2_gazebo.launch.py

echo ""
echo "=== 시뮬레이션 종료 → 그래프 생성 중 ==="

# 가장 최근 result.csv 확인
LATEST=$(ls -t $DATA_DIR/*_result.csv 2>/dev/null | head -1)
if [ -z "$LATEST" ]; then
    echo "[경고] *_result.csv 파일이 없습니다."
    exit 1
fi

echo "CSV: $LATEST"
cd $DATA_DIR
export TITA_CSV=$LATEST
/home/amg/.local/bin/jupyter nbconvert --to notebook --execute plot.ipynb \
    --output /tmp/plot_executed.ipynb \
    --ExecutePreprocessor.timeout=120 2>&1 | grep -v "^$"

echo ""
echo "=== 완료 ==="
echo "생성된 이미지:"
ls $DATA_DIR/$(basename $LATEST _result.csv)_result_*.png 2>/dev/null
