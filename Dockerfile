FROM python:3.11-slim

WORKDIR /app

# mono-complete is required for pythonnet/pmclib to interact with .NET DLLs on Linux
# libpython3-dev helps with header/linking issues in slim images
RUN apt-get update && apt-get install -y \
    mono-complete \
    libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# pmclib (Planar Motor) is not on PyPI: it is installed from the wheel in dependencies/
COPY dependencies/ ./dependencies/
RUN pip install --no-cache-dir dependencies/pmclib-117.9.1-py3-none-any.whl

COPY planar/ ./planar/
COPY entrypoint.sh .

# Convert line endings and make entrypoint script executable
RUN sed -i 's/\r$//' entrypoint.sh && chmod +x entrypoint.sh

# Output flushed at once; the OPC UA endpoint (docker-compose.yml sets it from .env)
ENV PYTHONUNBUFFERED=1 \
    OPCUA_ENDPOINT=opc.tcp://0.0.0.0:4840/

EXPOSE 4840

# Healthy while the OPC UA server takes connections
HEALTHCHECK --interval=30s --timeout=5s --start-period=60s --retries=3 \
    CMD python -c "import os, socket, urllib.parse as u; p = u.urlparse(os.environ['OPCUA_ENDPOINT']).port or 4840; socket.create_connection(('127.0.0.1', p), 3).close()"

ENTRYPOINT ["./entrypoint.sh"]
