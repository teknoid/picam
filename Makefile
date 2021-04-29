CFLAGS = -Wall -Wno-unused-function
LFLAGS = -Wall

LIBS = -lpthread -lrt -lbcm2835 -lwiringPi

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

all: clean mcp flamingo mcp3204 rfsniffer

mcp: mcp.o utils.o xmas.o mcp3204.o webcam.o flamingo.o rfsniffer.o rfcodec.o frozen.o
	$(CC) $(CFLAGS) -o mcp mcp.o utils.o xmas.o mcp3204.o webcam.o flamingo.o rfsniffer.o rfcodec.o frozen.o $(LIBS)

flamingo: flamingo.o utils.o 
	$(CC) $(CFLAGS) -DFLAMINGO_MAIN -c flamingo.c
	$(CC) $(LFLAGS) -o flamingo flamingo.o utils.o $(LIBS)

mcp3204: mcp3204.o utils.o
	$(CC) $(CFLAGS) -DMCP3204_MAIN -c mcp3204.c
	$(CC) $(LFLAGS) -o mcp3204 mcp3204.o utils.o $(LIBS)

rfsniffer: rfsniffer.o flamingo.o utils.o frozen.o
	$(CC) $(CFLAGS) -DRFSNIFFER_MAIN -c flamingo.c rfsniffer.c rfcodec.c
	$(CC) $(LFLAGS) -o rfsniffer flamingo.o rfsniffer.o rfcodec.o utils.o frozen.o $(LIBS)

#rfsniffer-wiringpi: rfsniffer-wiringpi.o flamingo.o utils.o
#	$(CC) $(CFLAGS) -lpthread -lrt -lwiringPi -o rfsniffer-wiringpi rfsniffer-wiringpi.o flamingo.o utils.o

.c.o:
	$(CC) -c $(CFLAGS) $<

.PHONY: clean install install-service install-webcam

clean:
	rm -f *.o mcp mcp3204 flamingo rfsniffer

install:
	@echo "[Installing and starting mcp]"
	systemctl stop mcp
	install -m 0755 mcp /usr/local/bin
	install -m 0755 flamingo /usr/local/bin
	install -m 0755 mcp3204 /usr/local/bin
	install -m 0755 rfsniffer /usr/local/bin
	systemctl start mcp

install-service:
	@echo "[Installing systemd service unit]"
	mkdir -p /usr/local/lib/systemd/system/
	install -m 0644 misc/mcp.service /usr/local/lib/systemd/system/
	systemctl daemon-reload
	systemctl enable mcp

install-webcam:
	@echo "[Installing webcam module]"
	mkdir -p /home/www/webcam
	cp -arv webcam/* /home/www/webcam
	chown -R www-data.www-data /home/www/webcam
