FROM python:3.9

# Install test dependencies
RUN pip install pytest pytest-html can-utils pyserial numpy matplotlib

# Install hardware access tools
RUN apt-get update && apt-get install -y \
    can-utils \
    usbutils \
    i2c-tools

WORKDIR /app

# Dockerfile.test-framework
FROM hil-test-base:latest

# Copy test framework code
COPY ./test_framework /app/test_framework
COPY ./config /app/config

# Entry point that runs tests
ENTRYPOINT ["python", "-m", "pytest", "/app/test_framework"]
