nrf51-pure-gcc-setup
====================

A simple and cross-platform GCC setup for nRF51 development. I've developed
it on Linux, but should also work fine on OS X and on Windows. Beware that
neither of those are regularly used by me, though. Feel free to submit a
pull request if you find something that doesn't work.

This was started (long) before the official SDK got GCC support, but I've kept
maintaining it, since I don't really like the official Makefiles...

Usage
-----
Make a pure-gcc subfolder in your project folder (i.e. alongside the
arm/gcc/iar folders from the SDK) and copy an example Makefile from the
examples/ folder into it. Make sure that the SDK_PATH and TEMPLATE_PATH points
towards your nRF51 SDK installation and the template/ subfolder of your clone
of this repository respectively.

Project options
---------------
All source files, if they are in the application folder (../ from the
Makefile) or somewhere in the SDK should be possible to include directly,
without a path. If you want to add some other path to the search paths, add it
to LIBRARY_PATHS (header files) or SOURCE_PATHS.

Make sure that the USE_SOFTDEVICE variable is set to whichever softdevice is
currently programmed on your board, lowercase. The value is directly used in
paths, so if it's uppercase, you'll most likely end up with files not being
found, at least on OS X/Linux. Currently S110, S120, S210 and S310 should be
supported, although only S110 have been well-tested.

If you use the boards.h file, make sure that the BOARD define is set
to the board you're currently using.

If you have multiple J-Links connected to your system, you should
set the SEGGER_SERIAL variable to the serial number of your J-Link, so that
the programming Just Works (tm). It seems Segger on Linux isn't capable of
giving you a selection of J-Links, as on Windows.

If you want to use the GDB functionality with multiple J-Links, you should
make sure that all projects have a unique GDB port number defined in their
project Makefile.

You can override most of the defines from the other Makefiles by setting them
in your project file, since most of them use ?=.

Targets
-------
Most of the targets provided should be self explanatory, but some may use some
extra explanation:

### erase-all:
Does an erase all of a chip.

### recover:
Provides equal functionality to that of nrfjprog / nRFgo Studio on Windows.

### flash-softdevice SOFTDEVICE=$(PATH_TO_SOFTDEVICE_WITHOUT_SPACES)
Can be used to flash a softdevice to a chip. The path to the softdevice hex
needs to be without spaces, due to Make limitations.

### startdebug:
Starts a J-Link GDB Server in a separate terminal window, and then GDB
also in a separate window. If you change the code, you can then make directly
from gdb, and do load to run the new code.


