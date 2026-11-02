FROM python:3.11-slim

WORKDIR /app

# mono: pmclib runs Planar Motor's .NET library through pythonnet; libpython3-dev for pythonnet
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

# Output flushed at once; the OPC UA endpoint (docker-compose.yml sets it from .env)
ENV PYTHONUNBUFFERED=1 \
    OPCUA_ENDPOINT=opc.tcp://0.0.0.0:4840/

EXPOSE 4840

# Healthy while the OPC UA server takes connections
HEALTHCHECK --interval=30s --timeout=5s --start-period=60s --retries=3 \
    CMD python -c "import os, socket, urllib.parse as u; p = u.urlparse(os.environ['OPCUA_ENDPOINT']).port or 4840; socket.create_connection(('127.0.0.1', p), 3).close()"

# The table layout is read from the PMC at the table's Reset; arguments are passed on (see --help)
ENTRYPOINT ["python", "-m", "planar"]
