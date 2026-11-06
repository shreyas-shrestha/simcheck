# Agent sandbox image. Build: docker build -t simcheck-agent:latest .
FROM python:3.12-slim
RUN pip install --no-cache-dir gtsam==4.3.0 numpy==2.5.4 scipy==1.18.1
WORKDIR /tmp
