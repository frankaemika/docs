FROM ubuntu:18.04
RUN apt-get update && apt-get install -y \
    git \
    npm \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*
