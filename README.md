# picam

control wireless wall plugs and webcam by environment luminosity

# Features

* written in plain C using eclipse CDT
* running on many ARM Boards (tested on Raspberry 2, Odroid C2)
* controls pluggable wall sockets of type Flamingo
* configurable SUNRISE/SUNDOWN events depending on environment luminousity
* read luminousity from illumination resistor connected to a MCP3204 A/D converter via SPI interface

# News

Jan 2021 implemented Flamingo 28bit Rolling Code enryption & decryption

Sep 2019 git project + github upload

...

??? 2016 project start  

# Components

* picam: background daemon
* flamingoread: utility to record code sequences from the flamingo remote control
* flamingosend: utility to send code sequences defined in flamingo.h
* mcp3204-test: print current luminousity (raw D/A converter value of illumination resistor)


# Usage

Compile

```bash
make clean
make
```
Install a systemd service unit:

```bash
make install-service
```
Install and run mcp as systemd service:

```bash
make install
```