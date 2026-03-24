FROM python:3.12
WORKDIR /app
ENV PYTHONUNBUFFERED=1
COPY . /app
RUN pip install --no-cache-dir -r requirements.txt
CMD ["python", "-u", "networking.py"]
