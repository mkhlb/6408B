# MKHLib
### This is the MKHL catapult/intake control library and ez template extension

## Intake and Catapult Control

MKHLib has a powerful ```CatapultIntakeController``` class that handles control of the intake and catapult. The two are merged together to safeguard against discs being intaken under a recovering catapult.

The ```CatapultIntakeController``` has functions for shooting with limit switch stopping, and waiting for different states in autonomous.

The gear ratio for intake to roller is given in the constructor, so that speeds and positions can be passed relative to roller and intake. The roller can be controller by PID or by time.

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

This ratio will stay the same if the stick values are both multiplied by the same number, as multiplying every term in both the bottom and top of a fraction by the same number is like multiplying the fraction by $1$.

For instance suppose we double $Y$ and $X$, giving $.40$ and $.20$ respectfully. Now $L = .60$ and $R = .20$.

Note how both motor powers double, and cancel out to the same ratio $3 \over 1$.

So the ratio should grow consistently as long as relative magnitudes of inputs are the same, but this math breaks down at higher speeds.

Let's double the previously doubled stick values once more. 

$Y = .80$ and $X = .40$.

$L = 1.2$ and $R = .40$.

${L \over R} = {3 \over 1}$.

This is fine and dandy. However, how can 120% power be applied to the left motor? It can't. In the classical arcade skid drive $L$ is rounded down to $1.0$. With the rounded $L$, the ratio $L \over R$ is now $5 \over 2$.

This is a less aggressive turn (as ${5 \over 2} < {3 \over 1}$), and so <b>at higher speeds it's harder for the robot to turn.</b>

#### The Solution

Instead of rounding when $L$ or $R$ are higher than $1.0$, normalized Arcade brings them both down proportionally with division.

So $L = 1.2 / 1.2 = 1.0$, $R = .4 / 1.2 = {1 \over 3}$ and most importantly the ratio is preserved with ${L \over R} = {3 \over 1}$.

The normalized Arcade retains the classic arcade feel while still performing well on turns at high speeds.

#### The Implementation

The MKHLib normalized arcade is built on top of EZ-Template. This means you get your input curving and active brake, just with an alternate control method. Simply call the new ```Drive.arcade_normalized_standard(ez::e_type stick_type)``` and ```Drive.arcade_normalized_reversed(ez::e_type stick_type)``` functions just as you would use any other Ez Template drive function in `opcontrol()`.

### Curvatherp Arcade

(Curvature throttle interp)

#### Curvature

The ratio $L/R$ gives curvature. The idea with curvature drive is to have the $X$ stick control curvature, with larger values of $X$ having ${L \over R} \to \infty$, and smaller values having ${L \over R} \to 1$ ($L = R$ and there is no curvature).

The formulae

$L = Y + |Y| \times X$

$R = Y - |Y| \times X$

With the above equations the ratio $L \over R$ is independent to the $Y$.

${L \over R} = {{Y + |Y|X} \over {Y - |Y|X}}$

$Y \geq 0$:

${L \over R} {{Y + YX} \over {Y - YX}}$

${L \over R} {{Y(1 + X)} \over {Y(1 - X)}}$

${L \over R} {{1 + X} \over {1 - X}}$

$Y \leq 0$:

${L \over R} {{Y - YX} \over {Y + YX}}$

${L \over R} {{Y(1 - X)} \over {Y(1 + X)}}$

${L \over R} {{1 - X} \over {1 + X}}$

In both versions, the simplified ratio $L \over R$ is independent of the $Y$ variable.

This means super high performance and intuitive control on turns at high speeds.

#### The Problem

What about point turning? 

Generally when $Y = 0$ (there is no forward/backwards movement), we want to engage in a point turn on the spot. With the previous curvature function a $Y = 0$ would always give an $L$ and $R$ of $0$. This is obviously undesirable.

Most implementations fix this by swapping to tank controls sharply (by way of a deadzone) under very low speeds (of the orders of $Y < 2$ or even $Y < .5$). Having tried this, it feels clunky and imperfect. Additionally to the point turn issue, the curvature drive doesn't work well under low $Y$ values in general, as it spits out swing movements too slow to really be useful. 

So curvature drive performs well at high speeds, but bad at low ones, and a hard deadzone switch feels unnatural. 

#### The Solution

Normal tank drive feels great at low speeds, but bad at high ones.

The curvatherp algorithm smoothly linearly interpolates between tank drive for the slow speeds, and curvature for the high speeds. This means the point turns and tight low speed performance of tank drive, and the intuitive high speed turning of curvature, without the clunky discrete switch.

The interpolation is done simply with linear weighting:
,
Let $I$ be the amount of curvature drive you wish to use. $0 \leq I \leq 1$ where $I = 0$ means that only the tank function ( $T_L$ and $T_R$ ) will be used, $I = 1$ means that only curvature ( $C_R$ and $C_L$ ) will be used, and $I = 0.5$ means that half curvature and half tank will be used ( $P_L$ and $P_R$ are the respective averages of the two sides of $T$ s and two sides of $C$ s )

Power $P = CI + T(1-I)$

Boilerplate linear weighting. The issue is finding $I$. You could simply make $I = Y$, but that offers little control to tune the lerp to your preferences. Instead it's advantageous to have an interpolation that is some linear function of $Y$ (clamping it between $0$ and $1$), as it allows larger ranges of $Y$ values to be eintirely tank drive or entirely curvature drive.

The user inputs 2 values

$I_s$: the $|Y|$ value to start the interpolation at ( $0 \leq I_s < 1$ ).

$I_e$: the $|Y|$ value to end the interpolation at ( $0 < I_e \leq 1$ ).

( $I_e > I_s$ )

$I$ is defined as:

$I = m|Y| + b$ ( $I$ is a linear function of $Y$)

Substituting:

$1 = mI_e + b$ (At $|Y| = I_e$ $I = 1$, its max value exactly)

$0 = mI_s + b$ (At $|Y| = I_s$ $I = 0$, its min value exactly)

2 unknowns, 2 equations. First solve for $m$:

$b = -mI_s$

$1 = mI_e - mI_s$

$m = {1 \over {I_e - I_s}}$

Now solve for $b$:

$0 = mI_s + b$

$0 = {I_s \over {I_e - I_s}} + b$

$b = -{I_s \over {I_e - I_s}}$

Plug back into original defenition of $I$ and simplify:

$I = {|Y| \over {I_e - I_s}} - {I_s \over {I_e - I_s}}$

$\therefore I = {{|Y| - I_s} \over {I_e - I_s}}$

If the maths proof isn't enough, it can be explained intuitively:

First start with $I = |Y|$

$I_s$ is an $x$-intercept. It gives the value that $I = 0$. To make $I = |Y|$'s $x$-intercept (which is normally 0) equal $I_s$, simply transform it to the right by $I_s$. to do that you subtract it from $|Y|$, giving the equation $I = |Y| - I_s$.

Now we must make the equation rise to $1.0$ by $I_e$. This is done by applying a slope, multiplying $|Y| - I_s$ by some number $m$. A slope $m$ is defined as $m = {rise \over run}$. We know the rise is $1.0$, and the run can be given as the distance from the start of the iterpolation to the end: $I_e - I_s$. Apply slope $1 \over {I_e - I_s}$ to get $I = {{|Y| - I_s} \over {I_e - I_s}}$, which intercepts $x$ at $I_s$ thanks to a tranformation of $I_s$ and intercepts $I_e$ thanks to a slope if run $I_e - I_s$.

#### The Implementation

Again the curvatherp is built on top of Ez Template. Unlike normalized arcade, `Drive::arcade_curvatherp_standard(e_type stick_type, double interpolator_start, double interpolator_end)` and `Drive::arcade_reversed_standard(e_type stick_type, double interpolator_start, double interpolator_end)` need 2 special parameters for the start and end of the interpolation. These parameters are between 0 and 127, as the max value of the controller stick is 127 in practice.
