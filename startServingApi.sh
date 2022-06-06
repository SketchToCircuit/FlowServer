#!/bin/bash
eval "$(conda shell.bash hook)"
conda activate neural
uvicorn --reload FlowServer.asgi:app --port 3002