# Agent sandbox image. Build from the repo root: docker build -t simcheck-agent:latest .
# Rebuild whenever simcheck/agent_worker.py changes.
FROM python:3.12-slim
RUN pip install --no-cache-dir gtsam==4.3.0 numpy==2.5.4 scipy==1.18.1
# The protocol server lives in the image, owned by root and read-only to the agent user,
# so agent code cannot replace it. Only solution.py (and a public/ copy) is mounted at run time.
COPY simcheck/agent_worker.py /opt/simcheck/agent_worker.py
RUN chmod 0444 /opt/simcheck/agent_worker.py
USER 65534:65534
WORKDIR /tmp
