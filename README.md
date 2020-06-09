# LoRa packet sniffer for RNode hardware

## Intro

This utility allows you to sniff LoRa networks with an [RNode](https://unsigned.io/projects/rnode/), and dump captured packets to the console or files.

```sh
usage: loramon [-h] [-C] [-W directory] [--freq Hz] [--bw Hz] [--txp dBm]
               [--sf factor] [--cr rate]
               [port]

LoRa packet sniffer for RNode hardware.

positional arguments:
  port           Serial port where RNode is attached

optional arguments:
  -h, --help     show this help message and exit
  -C, --console  Print captured packets to the console
  -W directory   Write captured packets to a directory
  --freq Hz      Frequency in Hz
  --bw Hz        Bandwidth in Hze
  --txp dBm      TX power in dBm
  --sf factor    Spreading factor
  --cr rate      Coding rate
```

## Installation

If you already have Python3 and pip installed, you can easily install LoRaMon through pip:

```sh
pip3 install loramon
```

On some operating systems, programs installed by pip cannot be run simply by typing their name. If you get a __command not found__ error, you will have to add the pip install directory to your PATH variable. The best way to do this is to edit the ".profile" file in your home directory and add the following lines at the bottom of the file:

```sh
# Include locally installed programs in path
if [ -d "$HOME/.local/bin" ] ; then
    PATH="$HOME/.local/bin:$PATH"
fi
```

If you want to install directly from this repository, first install the dependencies:

```sh
sudo apt install python3 python3-pip
sudo pip3 install pyserial
```

And then clone the repository and make LoRaMon executable:

```sh
git clone https://github.com/markqvist/LoRaMon.git
cd LoRaMon
chmod a+x loramon
./loramon --help
```

## Usage Examples

### Dump to console

Listens on a specified frequency and displays captured packets in the console.

```sh
loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C
```

### Dump to console and disk

Like above, but also writes all captured packets individually to a specified directory.

```sh
loramon /dev/ttyUSB0 --freq 868000000 --bw 125000 --sf 7 --cr 5 -C -W capturedir
```