# Dockerfile.base
FROM python:3.9-slim-buster

# Install test dependencies
RUN pip install pytest pytest-html can-utils pyserial numpy matplotlib

# Install hardware access tools
RUN apt-get update && apt-get install -y \
    can-utils \
    usbutils \
    i2c-tools

WORKDIR /app
