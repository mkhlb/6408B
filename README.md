# MKHLib
### This is the MKHL catapult/intake control library and ez template extension

## Intake and Catapult Control

MKHLib has a powerful ```CatapultIntakeController``` class that handles control of the intake and catapult. The two are merged together to safeguard against discs being intaken under a recovering catapult.

The ```CatapultIntakeController``` has functions for shooting with limit switch stopping, and waiting for different states in autonomous.

The gear ratio for intake to roller is given in the constructor, so that speeds and positions can be given relative to roller and intake. The roller can be controller by PID or by time.

## Ez Template extension - better drive code.

MKHLib implements 2 new custom driver control schemes:

### Normalized Arcade

#### The Problem

Given $R$ and $L$ as motor powers (ranging between -1 and 1), and $Y$ and $X$ (also ranging between -1 and 1) as stick values, the classic arcade swerve drive formula is as follows:

$L = Y + X$ and $R = Y - X$

The above equations elegantly combine the forward and backward motion of the $Y$ axis with a differential turning of the $X$

So, given $Y = .40$ and $X = .25$, $L = .65$ and $R = .15$

Note how the ratio ${L \over R} = {13 \over 3}$.

Now suppose we double $Y$ and $X$, giving $.80$ and $.50$ respectfully. Now $L = 1.30$ and $R = .3$.

This makes sense, if we double the input stick values, the output motor power should double too.
### Spervature Arcade
