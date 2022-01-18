docker run --gpus all -p 8501:8501 \
--mount type=bind,\
source=/mnt/hdd2/Sketch2Circuit/FlowServer/test/serving/tensorflow_serving/servables/tensorflow/testdata/saved_model_half_plus_two_cpu,\
target=/models/half_plus_two \
-e MODEL_NAME=half_plus_two -t tensorflow/serving:latest-gpu &