# MKHLib
### This is the MKHL catapult/intake control library and ez template extension

## Intake and Catapult Control

MKHLib has a powerful ```CatapultIntakeController``` class that handles control of the intake and catapult. The two are merged together to safeguard against discs being intaken under a recovering catapult.

The ```CatapultIntakeController``` has functions for shooting with limit switch stopping, and waiting for different states in autonomous.

The gear ratio for intake to roller is given in the constructor, so that speeds and positions can be given relative to roller and intake. The roller can be controller by PID or by time.

## Ez Template extension - better drive code.

MKHLib implements 2 new custom driver control schemes:

### Normalized Arcade



### Spervature Arcade
