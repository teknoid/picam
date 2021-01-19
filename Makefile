CFLAGS = -Wall

LIBS = -lpthread

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

all: mcp flamingoread flamingosend crypt mcp3204-test

mcp: mcp.o utils.o xmas.o mcp3204.o webcam.o
	$(CC) $(CFLAGS) $(LIBS) -o mcp mcp.o utils.o xmas.o mcp3204.o webcam.o

flamingoread: flamingoread.o flamingocrypt.o utils.o 
	$(CC) $(CFLAGS) $(LIBS) -lwiringPi -lrt -o flamingoread flamingoread.o flamingocrypt.o utils.o 

flamingosend: flamingosend.o flamingocrypt.o utils.o 
	$(CC) $(CFLAGS) $(LIBS) -lwiringPi -lrt -o flamingosend flamingosend.o flamingocrypt.o utils.o 

mcp3204-test: mcp3204-test.o mcp3204.o utils.o 
	$(CC) $(CFLAGS) $(LIBS) -o mcp3204-test mcp3204-test.o mcp3204.o utils.o 

crypt: crypt.o utils.o flamingocrypt.o utils.o
	$(CC) $(CFLAGS) $(LIBS) -o crypt crypt.o flamingocrypt.o utils.o

.c.o:
	$(CC) -c $(CFLAGS) $< 

.PHONY: clean install install-service

clean:
	rm -f *.o mcp flamingoread flamingosend mcp3204-test crypt

install:
	@echo "[Installing and starting mcp]"
	systemctl stop mcp
	install -m 0755 mcp /usr/local/bin
	install -m 0755 flamingoread /usr/local/bin
	install -m 0755 flamingosend /usr/local/bin
	install -m 0755 mcp3204-test /usr/local/bin
	systemctl start mcp

install-service:
	@echo "[Installing systemd service unit]"
	mkdir -p /usr/local/lib/systemd/system/
	install -m 0644 misc/mcp.service /usr/local/lib/systemd/system/
	systemctl daemon-reload
	systemctl enable mcp
