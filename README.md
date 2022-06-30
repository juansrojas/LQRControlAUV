# LQR Control of an Autonomous Underwater Vehicle

This code performs a basic LQR control simulation in MATLAB using a dynamics model derived for an autonomous 
underwater vehicle (AUV). The controller simultaneously controls the AUV's movement in six degrees-of-freedom underwater. 
The primary script is `lqr_matlab_sim.m`. A brief overview of the model used in the controller is provided below.

### State-Space Model
A state-space model is a representation of the physics that govern the robot’s movement. The state-space model used 
in the LQR controller is defined as follows:

$$ \dot{x} = Ax + Bu$$

The goal of the LQR control system is to reach a target state,  $x_{target}$, by calculating what the most optimal thrust 
output,  $u$, is based on the robot’s current state, $x$. Here, the $A$ and $B$ matrices are used to model the physics of 
the robot, and $\dot{x}$ is the derivative of $x$.

The robot’s overall state, $x$, is comprised of a 12 element vector:

$x$: The x-coordinate of the position of the robot 

$y$: The y-coordinate of the position of the robot 

$z$: The z-coordinate of the position of the robot

$\phi$: The robot’s roll (i.e. the rotation about the x-axis)

$\theta$: The robot’s pitch (i.e. the rotation about the y-axis)

$\psi$: The robot’s yaw (i.e. the rotation about the z-axis)

$u$:The x-component of the robot’s velocity

$v$: The y-component of the robot’s velocity

$w$: The z-component of the robot’s velocity

$p$: The robot’s angular velocity in the roll direction

$q$: The robot’s angular velocity in the pitch direction

$r$: The robot’s angular velocity in the yaw direction

NOTE: the $x$ symbol is used to represent both the x-coordinate of the robot's position and the robot’s overall state. 
Unless otherwise specified, the $x$ symbol refers to the robot’s overall state.

### Derivation of LQR Model
A robot-like vectorial model for marine craft was used as the basis for the dynamics model (i.e. the $A$ and 
$B$ matrices). The model consists of various matrices that describe different aspects of the robot’s physical behaviour. 
Each matrix is a function of at least one of the 12 states. There are three resources that were used to derive the robot's LQR model:

*Handbook of Marine Craft Hydrodynamics and Motion Control* by Thor I. Fossen (2011)

*Computer-Aided Control System Design* by Cheng Siong Chin (2013)

*Feedback Systems, an Introduction to Scientists and Engineers* by Astrom & Murray (2009)

Below is a high-level overview of the derivation of the robot’s LQR model, which combines elements from all three 
resources mentioned above:

#### Non-Linear Dynamics Model
The non-linear dynamics model for an underwater vehicle is defined as follows:

$$M\dot{v} + C(v)v + D(v)v + G(\eta) = \tau$$

Where,

$M$ is the mass/inertia matrix

$C$ is the coriolis matrix

$D$ is the damping matrix

$G$ is the gravity/buoyancy matrix

$\tau$ is the external forces/moments matrix

$v$ is the velocity vector

$\eta$ is the pose vector

$\dot{v}$ is the derivative of $v$ with respect to time

These matrices are comprised of robot-specific parameters such as mass and drag coefficients.

#### State-Space Representation of Non-Linear Dynamics Model
Let  $x = [\eta\ \ v]^T$ 
be a 12-element state vector where $\eta$ is the 6-element pose vector and $v$ is the 
6-element velocity vector (both corresponding to the 6 degrees-of-freedom).

Let J($\eta$) be the transformation matrix of the robot’s pose, from BODY to NED coordinates.

The non-linear dynamics model can be represented in state-space form as follows:

$$ F = Ax + Bu = f(x,t) + g(u,t)$$

$$f(x,t) = 
\begin{bmatrix}
0_{6x6} & J(\eta)\\
0_{6x6} & -M^{-1}(C - D)
\end{bmatrix}
\
\begin{bmatrix}
\eta\\
v
\end{bmatrix}
 + 
\begin{bmatrix}
0_{6x1}\\
-M^{-1}G
\end{bmatrix}
$$

$$g(u,t) = 
\begin{bmatrix}
0_{6x1}\\
-M^{-1}\tau
\end{bmatrix}
$$

The $\tau$ term corresponds to the thruster dynamics which are modeled as follows:

$$\tau = Tu$$

$$u = f_{t} \overline{u}$$

$$\overline{u} = \delta|\delta|$$

Where $T$ is the thrust allocation matrix (where the robot’s thruster configuration is represented), $f_{t}$ is the 
conversion factor from thrust-effort (%) to thrust-force (N), and $\delta$ is the raw thrust value.

#### Linearization
LQR can only be used for linear systems, hence the non-linear state-space model must be linearized:

Let $F$ be the non-linear state-space model, $x$ be the state vector, and $u$ be the control output vector.

Conducting **Jacobian Linearization** for the non-linear dynamics model yields the linear $A$ and $B$ matrices:

$$ A = \frac{\partial F}{\partial x}$$

$$ B = \frac{\partial F}{\partial u}\Bigr|_{u = [1\ 1\ 1\ 1\ 1\ 1]}$$

Notice how although the $B$ matrix is constant, the $A$ matrix is a function of the robot's state. This makes it possible 
to perform **real-time linearization** about the current state. This means that the $A$ matrix is re-generated 
in real-time based on what the current state is.

We now have the linearized state-space model:

$$ \dot{x} = Ax + Bu$$

#### Cost Matrices
Having a linearized state-space model, the final requirement to run LQR is to populate the $Q$ and $R$ cost matrices. 
These matrices are diagonal matrices with each element corresponding to either a state cost ($Q$ matrix), or a thrust 
cost ($R$ matrix). The $Q$ matrix is an $nxn$ matrix, where $n$ is the number of elements in the state. The $R$ matrix is an 
$mxm$ matrix where $m$ is the number of thrusters. The value for these elements were determined experimentally through 
trial and error. The $Q$ and $R$ matrices are essentially how the LQR control system is tuned/calibrated.

#### LQR Loop
Having all four matrices ($A$, $B$, $Q$, and $R$), the LQR loop can be run to generate the control thrust output as follows:

Apply lqr function:

$$K = lqr(A,B,Q,R)$$

Generate raw control output in thrust-force (N):

$$ \overline{du} _{lqr} = -K(x -  x _{target} )$$

Convert thrust output to thrust-effort (%):

$$du_{lqr} = f_{t}^{-1}\ \overline{du}_{lqr}$$

where  $f_{t}^{-1}$ is the inverse of the conversion factor from thrust-effort to thrust-force.

#### Feedforward Force
In the linearization of the non-linear model, the Gravity matrix term was eliminated 
from the final linear model because it was not a function of either the state or the control output. As such, in order 
to represent the gravity and buoyancy forces, a feedforward force was added to the control output. This force 
counteracts the gravity and buoyancy forces acting on the robot. The feedforward force is implemented as follows:

$$du_{feedforward} = f_{t}^{-1}\ [T * G]$$ 

where $T$ and $G$ are the thrust allocation and gravity matrices, respectively.

The gravity and buoyancy forces are calculated through the $G$ matrix. The forces are then allocated to the appropriate 
thrusters using the thrust allocation matrix. Finally, the thrust-force is converted to thrust-effort.

Having calculated the feedforward thrust, the final thrust output is a combination of the two thrust components:

$$du = du_{lqr} - du_{feedforward}$$

This final $du$ value is what is sent to the thruster controllers which convert the value from thrust-effort to RPM.

#### Parameter Estimation
With the matrix math out of the way, the final step to having a complete LQR model is populating the $M$, $C$, $D$, and $G$ 
matrices with the relevant physical robot parameters. These estimated parameters are used to populate the `robot_config.m` file.
Finding accurate values for these parameters typically requires the use of expensive experimental setups, however some 
simplifying assumptions can be made to get rough estimates of these values.
