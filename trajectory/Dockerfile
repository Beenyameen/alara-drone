FROM trajectory:latest

WORKDIR /app

COPY . /app

RUN cmake . && make

CMD ["./trajectory"]
