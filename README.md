# ps2-to-quad-mouse

An adapter to connect a PS/2 mouse to a Macintosh Plus/512k/128k's quadrature mouse port.


## Technical Details

### Connections

```
                            .--------.
                    Supply -|01 \/ 08|- Ground
Mouse X Interrupt <--- RA5 -|02    07|- RA0 <--> PS/2 Data
Mouse Y Interrupt <--- RA4 -|03    06|- RA1 <--> PS/2 Clock
 Mouse Quadrature <--- RA3 -|04    05|- RA2 ---> Mouse Button
                            '--------'
```

Mouse quadrature signal must be pulled down with a 1 MΩ resistor and inverted using a 2N7000 MOSFET.  PS/2 data and clock must be pulled up with a resistor of between 1 and 10 kΩ.


### Configuration

Configuration mode can be entered by holding down the left and right mouse buttons for approximately five seconds without moving the mouse.

Once entered, the mouse speed can be increased by clicking the right mouse button and decreased by clicking the left mouse button.  There are seven speed settings:

| Setting     | Multiplier* |
| ----------- | ----------- |
| 1 (fastest) | 4x          |
| 2           | 2x          |
| 3           | 1.3x        |
| 4 (default) | 1x          |
| 5           | 0.8x        |
| 6           | 0.66666667x |
| 7 (slowest) | 0.57142857x |

\* Versus normal quadrature mouse speed

Configuration mode is exited by waiting for approximately five seconds without moving the mouse or clicking any button.


### Compatibility

This firmware is compatible with all Macintosh computers that use a quadrature mouse (Plus, 512k, and 128k).  It should also in theory be compatible with the Lisa and the Apple II, but this has not been tested.


### Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB. Note that you must use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.
