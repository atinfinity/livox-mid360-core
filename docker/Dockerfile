# Ubuntu 24.04 build/test environment mirroring CI. Used for local verification on non-Linux hosts.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential g++-13 g++-14 clang-19 libclang-rt-19-dev cmake ninja-build catch2 git python3 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /src
