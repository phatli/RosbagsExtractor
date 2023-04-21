FROM ros:noetic

RUN apt-get update && apt-get install -y \
    python3-rosbag \
    python3-autopep8 \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir pandas tqdm

CMD ["/bin/bash"]
