README.txt
^^^^^^^^^^

This directory contains miscellaneous files needed to install
pascal runtime logic into the NuttX source tree.  After
installation, the NuttX source tree contain the following files

pcode
|-- Makefile
|-- include
|   `-- Common header files
|-- libboff
|   `-- Pascal object format (POFF) library
`--insn
    |-- include
    |   `-- model-specific header files
    `-- prun
        `-- model-specific source files

This directory contains:

INSTALL.sh -- The script that performs the operation.  Usage:

    ./INSTALL.sh [-16|-32] <NuttX-path>

Makefile -- The NuttX makefile for the runtime logic

keywords.h -- A version that adjusts build context for the NuttX
  environment.


