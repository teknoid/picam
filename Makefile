CFLAGS = -Wall

LIBS = -lpthread -lrt

SRCS = $(wildcard *.c)
OBJS = $(SRCS:.c=.o)

COBJS-COMMON	= utils.o
COBJS-RFSNIFFER	= rfsniffer.o rfsniffer-stream.o rfsniffer-realtime.o rfcodec.o rfcodec-nexus.o rfcodec-flamingo.o 

all: clean mcp sensors flamingo rfsniffer gpio-bcm2835

mcp: mcp.o gpio-bcm2835.o sensors.o xmas.o webcam.o flamingo.o frozen.o smbus.o $(COBJS-COMMON) $(COBJS-RFSNIFFER)
	$(CC) $(CFLAGS) -o mcp mcp.o gpio-bcm2835.o sensors.o xmas.o webcam.o flamingo.o frozen.o smbus.o $(COBJS-COMMON) $(COBJS-RFSNIFFER) $(LIBS)

sensors: sensors.o smbus.o $(COBJS-COMMON)
	$(CC) $(CFLAGS) -DSENSORS_MAIN -c sensors.c smbus.c
	$(CC) $(CFLAGS) -o sensors sensors.o smbus.o $(COBJS-COMMON) $(LIBS)

flamingo: flamingo.o frozen.o gpio-bcm2835.o $(COBJS-COMMON) $(COBJS-RFSNIFFER)
	$(CC) $(CFLAGS) -DFLAMINGO_MAIN -c flamingo.c
	$(CC) $(CFLAGS) -o flamingo flamingo.o frozen.o gpio-bcm2835.o $(COBJS-COMMON) $(COBJS-RFSNIFFER) $(LIBS)

rfsniffer: frozen.o gpio-bcm2835.o $(COBJS-COMMON) $(COBJS-RFSNIFFER)
	$(CC) $(CFLAGS) -DRFSNIFFER_MAIN -c rfsniffer.c
	$(CC) $(CFLAGS) -o rfsniffer frozen.o gpio-bcm2835.o $(COBJS-COMMON) $(COBJS-RFSNIFFER) $(LIBS)

gpio-bcm2835: gpio-bcm2835.o
	$(CC) $(CFLAGS) -DGPIO_MAIN -c gpio-bcm2835.c -Wno-unused-function 
	$(CC) $(CFLAGS) -o gpio-bcm2835 gpio-bcm2835.o

flamingo-old: flamingo-old.c utils.o
	$(CC) $(CFLAGS) -o flamingo-old flamingo-old.c utils.o $(LIBS) -lwiringPi

.c.o:
	$(CC) -c $(CFLAGS) $<

.PHONY: clean install install-service install-webcam

clean:
	rm -f *.o mcp sensors flamingo rfsniffer gpio-bcm2835

install:
	@echo "[Installing and starting mcp]"
	systemctl stop mcp
	install -m 0755 mcp /usr/local/bin
	install -m 0755 flamingo /usr/local/bin
	install -m 0755 sensors /usr/local/bin
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
