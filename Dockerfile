FROM python:3.9

# Install system dependencies for CAN bus and other hardware interfaces
RUN apt-get update && apt-get install -y \
    can-utils \
    iproute2 \
    python3-dev \
    gcc \
    make \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first (for better caching)
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the entire application code
COPY . .

# Install the project in development mode
RUN pip install -e .

# Create directory for test reports
RUN mkdir -p /app/test_reports

# Set environment variables
ENV PYTHONPATH=/app
ENV PYTHONUNBUFFERED=1

# Command to run tests (can be overridden)
CMD ["python", "-m", "pytest", "tests/", "--html=/app/test_reports/report.html"]
