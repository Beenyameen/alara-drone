FROM python:3.11

WORKDIR /app

RUN pip install --no-cache-dir pyzmq rvl

COPY broker.py .

CMD ["python", "-u", "broker.py"]
