FROM ros:noetic

# Dependencies for glvnd and X11.
RUN apt-get update \
    && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    libxrandr2 \
    libglu1 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    python3-rosbag \
    python3-autopep8 \
    python3-pip \
    git \
    opencv-python \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir pandas tqdm rosnumpy

CMD ["/bin/bash"]
