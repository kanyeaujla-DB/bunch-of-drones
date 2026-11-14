# Runtime image for Drona's Airlink host. Ubuntu 22.04 = the ROS 2 Humble base.
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive PYTHONUNBUFFERED=1
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip python3-venv libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /app
COPY pyproject.toml README.md ./
COPY src ./src
COPY sim ./sim
RUN pip3 install --no-cache-dir .
COPY config ./config
ENV PLUTO_SWARM_ROOT=/app
# docker run --network host <image> --profile airlink --mission hover_test
ENTRYPOINT ["pluto-run"]
CMD ["--profile", "sim", "--mission", "hover_test"]
