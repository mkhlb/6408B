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

So, given $Y = .20$ and $X = .10$: $L = .30$ and $R = .10$

Note how the ratio ${L \over R} = {3 \over 1}$.

The ratio $L \over R$ describes the slope the robot will curve, while the size of this curve can vary the ratio $L \over R$ tells how much the robot will turn for how much it goes forward.

This ratio will stay the same if the stick values are both multiplied by the same number, as if they're both bigger by the same amount only the magnitude speed should change, not the slope of the curve

For instance suppose we double $Y$ and $X$, giving $.40$ and $.20$ respectfully. Now $L = .60$ and $R = .20$.

Note how both motor powers double, and cancel out to the same ratio $3 \over 1$.

So the ratio should grow and be consistent as long as relative magnitudes of inputs are the same, but this math breaks down at higher speeds.

Let's double the previously doubled stick values once more. 

$Y = .80$ and $X = .40$.

$L = 1.2$ and $R = .40$.

${L \over R} = {3 \over 1}$.

This is fine and dandy. However, how can 120% power be applied to the left motor? It can't. In the classical arcade skid drive $L$ is rounded down to $1.0$. With the rounded $L$, the ratio $L \over R$ is now $5 \over 2$.

This is a less aggressive turn (as ${5 \over 2} < {3 \over 1}$), and so <b>at higher speeds it's harder for the robot to turn.</b>


### Spervature Arcade
