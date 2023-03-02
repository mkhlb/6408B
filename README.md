# MKHLib
### This is the MKHL catapult/intake control library and ez template extension

## Intake and Catapult Control

MKHLib has a powerful ```CatapultIntakeController``` class that handles control of the intake and catapult. The two are merged together to safeguard against discs being intaken under a recovering catapult.

The ```CatapultIntakeController``` has functions for shooting with limit switch stopping, and waiting for different states in autonomous.

The gear ratio for intake to roller is given in the constructor, so that speeds and positions can be passed relative to roller and intake. The roller can be controller by PID or by time.

## Ez Template Extension - Better Drive Code.

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

${L \over R} = {{Y + YX} \over {Y - YX}}$

${L \over R} = {{Y(1 + X)} \over {Y(1 - X)}}$

${L \over R} = {{1 + X} \over {1 - X}}$

$Y \leq 0$:

${L \over R} = {{Y - YX} \over {Y + YX}}$

${L \over R} = {{Y(1 - X)} \over {Y(1 + X)}}$

${L \over R} = {{1 - X} \over {1 + X}}$

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

Now we must make the equation rise to $1.0$ by $I_e$. This is done by applying a slope, multiplying $|Y| - I_s$ by some number $m$. A slope $m$ is defined as $m = {rise \over run}$. We know the rise is $1.0$, and the run can be given as the distance from the start of the iterpolation to the end: $I_e - I_s$. Apply slope $1 \over {I_e - I_s}$ to get $I = {{|Y| - I_s} \over {I_e - I_s}}$, which intercepts $x$ at $I_s$ thanks to a tranformation of $I_s$ and intercepts $I_e$ thanks to a slope of run $I_e - I_s$.

#### The Implementation

Again the curvatherp is built on top of Ez Template. Unlike normalized arcade, `Drive::arcade_curvatherp_standard(e_type stick_type, double interpolator_start, double interpolator_end)` and `Drive::arcade_reversed_standard(e_type stick_type, double interpolator_start, double interpolator_end)` need 2 special parameters for the start and end of the interpolation. These parameters are between 0 and 127, as the max value of the controller stick is 127 in practice.

## Ez Template Extension - Odometry

### Odometry

#### The Theory

[This is very basic.](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf)

#### The Implementation

Odometry is implemented directly into the Ez Template `chassis`. Simply pass a width into the constructor and odometry is handled using the same IMU and encoders for PID.

### Planned Motion

#### Turning To A Point

MKHLib implements a simple function to turn to a point. It simply acts as a proxy to the normal Ez Template turn PID and sets the target value based on the angle to the point. That means that if the robot is pushed after the turn to point has been called, it wont adjust and still turn to that point; the PID only accesses the odometry position at the beginning of the motion, hence the term motion planning. MKHLib isn't running a PID on any of the odometry data, it's simply using it to inform the normal Ez Template PIDs.

#### Dot Products

The dot product is the multiplication of two vectors that yields a scalar.

There are two main ways to calculate the dot product of two vectors:

In cartesian coordinates: 

$\vec{v} \cdot \vec{v} = {\vec{v}_1}_x{\vec{v}_1}_x + {\vec{v}_1}_y{\vec{v}_1}_y$

This is a simple multiplication and addition of both $x$ components and $y$ components.

Yielding an identical result, in polar coordinates: 

$\vec{v}_1 \cdot \vec{v}_2 = |\vec{v}_1||\vec{v}_2|cos(\theta)$

This involves two operations: first you project one vector onto the other, then you multiply their magnitudes.
The cosin between two angles is the projection, it gives the adjacent component of a vector to an angle. This is illustrated in this image from [mathisfun.com](mathisfun.com): 

![image](https://www.mathsisfun.com/algebra/images/dot-product-a-cos.svg)

#### Driving To A Point

<s>MKHLib doesn't mess with complicated driving algorithms.</s> To simply drive to a point using motion planning MKHLib does two things: turns to the point, then drive as close to it as it can in a straight line. So how does MKHLib figure out what distance it needs to drive to get as close as possible?

Consider two vectors: the unit vector representing the direction the robot is facing $\hat{\omega}$, and the vector from the robot to the target point $\vec{T}$. $\hat{\omega}$ and $\vec{T}$ are seperated by an angle $\theta$. Recall that a dot product is a projection and then a multiplication, and that multiplication of $x \cdot 1 = x$, and $|\hat{v}| = 1$, so a dot product of a vector is just a projection, because the multiplication by $1$ does nothing. So $\hat{\omega} \cdot \vec{T}$ is just a projection of $\vec{T}$ onto $\hat{\omega}$. 

$\hat{\omega} \cdot \vec{T}$ gives the distance to drive in direction $\hat{\omega}$ to get as close as possible to $\vec{T}$. If you don't intuitively understand this, allow a mathematical illustration:

The distance to drive $D$ times the orientation vector $\hat{\omega}$ ( $\vec{D}$ ) forms a right angle triangle with $\vec{T}$ (by definition of being as close as possible).

### Iterative Motion - Points

While originally there was no intention to do so, iterative motion has been added to MKHLib. The addition of this reduced the time of the skills autonomous by 10 seconds.

#### Point Turning

There is iterative point turning, to facilitate more accurate auto aim.

#### Point Driving

Point driving is simple to do, but hard to make stable. In theory its just combining two of the previous functions talked about and running them iteratively. By driving straight to a point, and constantly turning to face it as accurately as possible, you can achieve motion to that point.

However, iteratively turning to a point becomes increasingly unstable as the translational error $e$ decreases. With a small overshoot the angular error ${\omega}_e$ can completely reverse, and the robot can get stuck doing loops around a target point.

So the trick is to split the motion up into phases, and simply turn off the angular component of movement at a certain distance. It works as follows:

PHASE 0, error > 12: iteratively update straight drive target and point turn target using math described above. <br>
PHASE 1, error > 9: iteratively update straight drive target. <br>
PHASE 2: stop updating all targets, let PIDs run their course.

Note: all drive PIDs run on encoders, and the distance target is converted into an encoder target. Because of this the user just has to tune one PID for all driving, odom based or encoder based.

#### Achieving A Target Orientation

The classical drive functions give a good fast path to reach a target position, but they don't control for orientation. The stupid way to do that is simply to turn after the drive is done. The issue is that this doesn't look smooth, and in some cases isn't fast. The alternative is driving in a curve, and in some cases this curve is faster, because when driving in curves you spend less time accelerating. One way to generate these curves on the fly with nothing more than creative iteration of the point driving that has already been tuned is the boomerang controller.

Given a target point $\vec{T}$ and a target orientation ${\omega}_T$, the boomerang controller actually never drives to $\vec{T}$. Instead it drives to a virtual point (known as the "carrot point" $\vec{T}_c$) colinear with the target orientation vector $\hat{\omega}_T$ when relative to $\vec{T}$.

The distance of the carrot point from $\vec{T}$ is a simple function of the distance of the robot from $\vec{T}$, $T_e$. This function is controlled by a tuned variable $kQ$, essentially an aggression constant, with values of $kQ$ closer to 1 meaning more aggressive curves and values of 0 having no curve.

The final formula for the carrot point is: $\vec{T}_c = \vec{T} + \hat{\omega}_T * T_e * kQ$

With this you can get semi-accurate control of orientation in drive motions. The boomerang controller isn't perfectly accurate, and should not be used for applications where orientation needs to be precise, such as aiming a shot. For that a point turn should be done after the motion.

### Iterative Motion - Paths

Much in the vain of the boomerang controller, path following is done through virtual points.

Pure pursuit is simply done by picking the farthest point on a path up to a certain distance and following it.

#### Classical Pure Pursuit



#### Point Drive Pure Pursuit



## On Implementation And Usage of Motion

### One Motion Profile To Rule Them All

I love PIDs. They are simple and they work. The idea with MKHLib originally was actually just to be a wrapper for Ez Template that calls the Ez Template functions iteratively based on odometry. Eventually it became more feasable to integrate it into Ez Template but the idea of iterative calls of straight line PIDs to drive in 2D space has been kept around since then. The reason for this is that tuning in 1D space is way easier than tuning in 2D space, and having your 2D functions just be 1D functions with some sauce means you never need to do that tricky 2D tuning. To tune all of my driving all I had to do was tune two straight drive PIDs, a turn PID, and a heading PID.

### Don't Run Away From the Abstract

In reality all points that you drive to, all paths that you follow, are abstract. The values you input are not real field inches. They're close to positions on the field, but they have variance. This is acceptable. You don't need a perfectly accurate coordinate system that works just like reality. You just need it to be consistent. It's importatn to recall that everything is virtual when writing libraries. Some programmers use a tool to <a href="https://lemlib.github.io/Path-Gen/">draw paths on an image of a field</a>. I don't. A path drawer is too idealized, it exists in a utopian reality where your robot's real driving path is accurate to some value you can enter on your browser. Rather than banging your head against this with endless tuning it's better to let the chaos into your life and accept it and the numerical tuning that comes with it.
