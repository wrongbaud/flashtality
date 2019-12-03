# Flashtality
Utility to dump parallel flash chips using MCP20317 ICs and the ESP32 microcontroller.

See blog post [here](https://wrongbaud.github.io/MK-Teardown/) for an overview of what this project is for

This project borrows heavily from a handful of projects in the espressif examples directory.

## Usage
This project uses the espressif ESP32 libraries, as such you can configure the project as follows:

```
idf.py menuconfig
```

Use this to configure the WiFi AP that you want to connect to, the data is output over wifi to a client, whose script can be found in ```scripts```

To build the project do the following:

```
idf.py build
```

In order to flash the project to your ESP32 run:

```
idf.py -p /path/to/serial flash
```

## Dumping Flash

With the project flashed, you can run the following python script in order to connect to your ESP32 and dump the target flash chip:

```
python scripts/dump.py -i 192.168.1.186 -p 3333 -a 0 -s 400000 -o wifi-flash-dump.bin
```

The arguments are as follows:

* ```-i``` IP address of ESP32 to connect to (you can get this info from the serial logs)
* ```-p``` Port of ESP32 to connect to, this can be configured via ```idf.py menuconfig```
* ```-a``` Start address to begin extraction from
* ```-s``` Size of flash dump
* ```-o``` Output file, resulting file will contain data starting at the specified address
