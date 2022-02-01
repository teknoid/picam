CFLAGS = -Wall

LIBS = -lpthread -lrt

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

all: clean mcp mcp3204 flamingo rfsniffer gpio-bcm2835

mcp: mcp.o utils.o gpio-bcm2835.o lumi.o xmas.o mcp3204.o webcam.o flamingo.o rfsniffer.o rfcodec.o rfcodec-nexus.o rfcodec-flamingo.o frozen.o
	$(CC) $(CFLAGS) -o mcp mcp.o utils.o gpio-bcm2835.o lumi.o xmas.o mcp3204.o webcam.o flamingo.o rfsniffer.o rfcodec.o rfcodec-nexus.o rfcodec-flamingo.o frozen.o $(LIBS)

mcp3204: mcp3204.o utils.o
	$(CC) $(CFLAGS) -DMCP3204_MAIN -c mcp3204.c
	$(CC) $(CFLAGS) -o mcp3204 mcp3204.o utils.o $(LIBS)

flamingo: flamingo.o rfsniffer.o rfcodec.o rfcodec-nexus.o rfcodec-flamingo.o utils.o frozen.o gpio-bcm2835.o
	$(CC) $(CFLAGS) -DFLAMINGO_MAIN -c flamingo.c
	$(CC) $(CFLAGS) -o flamingo flamingo.o rfsniffer.o rfcodec.o rfcodec-nexus.o rfcodec-flamingo.o utils.o frozen.o gpio-bcm2835.o $(LIBS)

rfsniffer: rfsniffer.o rfcodec.o rfcodec-nexus.o rfcodec-flamingo.o utils.o frozen.o gpio-bcm2835.o
	$(CC) $(CFLAGS) -DRFSNIFFER_MAIN -c rfsniffer.c
	$(CC) $(CFLAGS) -o rfsniffer rfsniffer.o rfcodec.o rfcodec-nexus.o rfcodec-flamingo.o utils.o frozen.o gpio-bcm2835.o $(LIBS)

gpio-bcm2835: gpio-bcm2835.o
	$(CC) $(CFLAGS) -DGPIO_MAIN -c gpio-bcm2835.c -Wno-unused-function 
	$(CC) $(CFLAGS) -o gpio-bcm2835 gpio-bcm2835.o

flamingo-old: flamingo-old.c utils.o
	$(CC) $(CFLAGS) -o flamingo-old flamingo-old.c utils.o $(LIBS) -lwiringPi

.c.o:
	$(CC) -c $(CFLAGS) $<

.PHONY: clean install install-service install-webcam

clean:
	rm -f *.o mcp mcp3204 flamingo rfsniffer gpio-bcm2835

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
