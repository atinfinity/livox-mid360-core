# Ubuntu 24.04 build/test environment mirroring CI. Used for local verification on non-Linux hosts.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential g++-13 g++-14 clang-19 libclang-rt-19-dev clang-format-19 clang-tidy-19 \
      cmake ninja-build catch2 git python3 python3-pip ca-certificates \
    && pip3 install --break-system-packages --no-cache-dir ruff==0.16.9 \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
