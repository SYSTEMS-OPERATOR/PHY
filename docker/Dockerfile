# simple cpu image
FROM python:3.11-slim
WORKDIR /app
COPY . /app
RUN pip install --no-cache-dir ray[default]==2.9.3 stable-baselines3 gymnasium
CMD ["python", "bin/run_curriculum.py"]
