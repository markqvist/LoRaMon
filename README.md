# LoRa packet sniffer for RNode hardware

## Intro

This utility allows you to sniff LoRa networks with an [RNode](https://unsigned.io/projects/rnode/), and dump captured packets to the console or files.

```
usage:
```

## Dependencies

The config utility requires Python 2.7 and pyserial. To install:

```
sudo apt install python python-pip
sudo pip install pyserial
```

## Installation

Just clone or download this repository, place wherever you'd like and run loramon (remember to set executable permissions):

```
git clone https://github.com/markqvist/LoRaMon.git
cd LoRaMon
chmod a+x loramon
./loramon --help
```

## Examples

### Dump to console

Listens on a specified frequency and displays captured packets in the console.

```
./loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C
```

### Dump to console and disk

Like above, but also writes all captured packets individually to a specified directory.

```
./loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C -W capturedir
```