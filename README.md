# IBM3705_R3
The main updates with Release 3 of the IBM 3705 SIMH emulator:

1 - Multiple SDLC lines support

Below is a detailed overview of updates of this release:

Central Control Unit (CCU/CPU)

- Updates to support multiple SDLC lines.

Channel Adapter

- None

Scanner

- Each line has now it own ICW
- ABAR contains the line addr of the line that initiated an interrupt.
- For the moment BSC support has been suspended. (Use R2 instead)
- Each line has its own Tx/Rx buffer
- #define MAXLINES n sets the number of SDLC lines during compilation.

SDLC LIC

- Minor

BSC LIC

- Suppended (use R2 if required)

IBM 3274

- None

IBM 3271

- None

IBM 3705 Front-Panel

- None

To do

- DLSw support
- Full duplex support for SDLC lines

EF & HJS (C)2023
