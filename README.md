# STLINK-V3-BRIDGE_libusb

STLINK-V3-BRIDGE API which does not uses the ST's libSTLinkUSBDriver but uses libusb.

Built for validation purposes, but it might be useful in the case if you want to use the STLINK-V3-BRIDGE API on a non-x86 platform (like Raspberry PI).

Original version can be downloaded from here:

https://www.st.com/en/development-tools/stlink-v3-bridge.html

# Building
This project uses the [meson build system](https://mesonbuild.com), however there are some wrapper `make` commands for convenience.
* `make` or `make all` to build the library and example.
* `make install` to install the library.
* `make clean` to delete the `build` and `inst_root` directories.
* `make install-local` to perform an install in `./inst_root`. Once can run `tree inst_root` afterwards to see what running `make install` will produce.

One can run the example program after building by doing `./build/example/stlinkv3-lib-example-app`

# Contributing
We use [pre-commit](https://pre-commit.com/) to manage pre-commit Git hooks. Install them via:
```
pip install pre-commit
pre-commit install
```
To run on all files now: `pre-commit run --all-files`

# Feedback

If you end up building upon this library let us know, we could list it here (for e.g. I2C EEPROM programming utility, etc.).
Let's make the STLinkV3 a more versatile tool for hackers!

## Achtung !! Warning!! Pozor!!
This is not a traditional open source project, it licensed under ST's Ultimate Liberty License see:

https://www.st.com/SLA0044
