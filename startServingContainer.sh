docker run --gpus all -p 8501:8501 \
--mount type=bind,\
source=/mnt/hdd2/Sketch2Circuit/DataProcessing/CompleteModel/Exported,\
target=/models/CompleteModel \
-e MODEL_NAME=CompleteModel -t tensorflow/serving:latest-gpu &