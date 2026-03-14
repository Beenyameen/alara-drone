FROM enord-reconstruction:latest
WORKDIR /app
COPY . /app
RUN cmake . && make
CMD ["./reconstruction"]
