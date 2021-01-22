CFLAGS = -Wall

LIBS = -lpthread -lwiringPi -lrt

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

all: mcp flamingoread flamingosend flamingo-test mcp3204-test

mcp: mcp.o utils.o xmas.o mcp3204.o webcam.o flamingo.o
	$(CC) $(CFLAGS) $(LIBS) -o mcp mcp.o utils.o xmas.o mcp3204.o webcam.o flamingo.o

flamingoread: flamingo.o flamingoread.o utils.o 
	$(CC) $(CFLAGS) $(LIBS) -o flamingoread flamingoread.o flamingo.o utils.o 

flamingosend: flamingo.o flamingosend.o utils.o 
	$(CC) $(CFLAGS) $(LIBS) -o flamingosend flamingosend.o flamingo.o utils.o 

flamingo-test: flamingo-test.o flamingo.o utils.o
	$(CC) $(CFLAGS) $(LIBS) -o flamingo-test flamingo-test.o flamingo.o utils.o

mcp3204-test: mcp3204-test.o mcp3204.o utils.o 
	$(CC) $(CFLAGS) $(LIBS) -o mcp3204-test mcp3204-test.o mcp3204.o utils.o 

.c.o:
	$(CC) -c $(CFLAGS) $< 

.PHONY: clean install install-service

clean:
	rm -f *.o *-test mcp flamingoread flamingosend

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
