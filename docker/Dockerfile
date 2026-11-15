# CAD toolchain for the holder project. Network is needed only while building
# this image; every run goes through ./dock with --network none.
FROM python:3.12-slim-bookworm AS base
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    DO_NOT_TRACK=1
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      librsvg2-bin libgl1 libglib2.0-0 libxrender1 libx11-6 libxext6 libegl1 libxcursor1 libxft2 libxinerama1 \
 && rm -rf /var/lib/apt/lists/*
RUN pip install --no-cache-dir uv==0.13.8

FROM base AS final
ENV UV_PROJECT_ENVIRONMENT=/opt/venv \
    UV_LINK_MODE=copy \
    UV_NO_CACHE=1
WORKDIR /build
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --no-install-project --group dev \
 && chmod -R a+rX /opt/venv
ENV PATH=/opt/venv/bin:$PATH \
    PYTHONPATH=/work/src \
    PYTHONDONTWRITEBYTECODE=1 \
    UV_OFFLINE=1 \
    HOME=/tmp
WORKDIR /work
