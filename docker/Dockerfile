# Ubuntu 24.04 build/test environment mirroring CI. Used for local verification on non-Linux hosts.
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential g++-13 g++-14 clang-19 libclang-rt-19-dev clang-format-19 clang-tidy-19 \
      cmake ninja-build catch2 gcovr cppcheck git python3 ca-certificates \
    && rm -rf /var/lib/apt/lists/*
# The Python lint tools, installed with uv as in lint.yml (#191); the tests need only python3.
COPY --from=ghcr.io/astral-sh/uv:0.12.21 /uv /uvx /usr/local/bin/
ENV UV_TOOL_DIR=/opt/uv-tools UV_TOOL_BIN_DIR=/usr/local/bin
RUN uv tool install ruff==0.16.9 \
    && for tool in ament_cppcheck ament_lint_cmake; do \
         uv tool install "${tool} @ git+https://github.com/ament/ament_lint.git@jazzy#subdirectory=${tool}"; \
       done \
    && uv cache clean
WORKDIR /src
