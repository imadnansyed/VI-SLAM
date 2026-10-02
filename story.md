Yes — below is the same content in plain Notepad-friendly format. I kept the maths and explanation, but removed Markdown formatting, bold, code fences, and fancy boxes.

IMU PREINTEGRATION — COMPLETE STORY + MATH

1. THE STORY STARTS

We have an IMU between two camera/keyframes.

Camera keyframe:

i ------------------------------> j

Between i and j, the IMU gives many measurements:

i
|
| gyro + accel
v
t1
|
| gyro + accel
v
t2
|
| gyro + accel
v
...
|
v
j

Instead of sending every IMU measurement to the optimizer, we summarize all of them into:

ΔR_ij
Δv_ij
Δp_ij

And we also keep:

bias Jacobians
covariance Σ

So the optimizer receives one compact IMU constraint between i and j.

2. FIRST: WE START WITH BIAS ESTIMATES

The IMU measurements are not perfect.

Gyroscope:

ω̃ = ω + b_g + η_g

Accelerometer:

ã = a + b_a + η_a

where:

ω̃ = measured angular velocity
ã = measured acceleration/specific force
b_g = gyroscope bias
b_a = accelerometer bias
η_g = gyroscope noise
η_a = accelerometer noise

So before integration we remove our current bias estimate:

ω = ω̃ - b_g

a = ã - b_a

3. WE INITIALIZE THE PREINTEGRATED MEASUREMENT

At the beginning of an interval i → j:

ΔR_i = I

Δv_i = 0

Δp_i = 0

Δt = 0

Why?

Because we have not integrated anything yet.

Rotation:

ΔR = I

means:

"I have accumulated zero rotation."

Velocity:

Δv = 0

means:

"I have accumulated zero relative velocity."

Position:

Δp = 0

means:

"I have accumulated zero relative position."

4. WE ALSO INITIALIZE THE BIAS JACOBIANS

We want to know:

"If my bias estimate is slightly wrong,
how much will ΔR, Δv and Δp change?"

So we maintain:

J_R_bg = ∂ΔR/∂b_g

J_v_bg = ∂Δv/∂b_g

J_v_ba = ∂Δv/∂b_a

J_p_bg = ∂Δp/∂b_g

J_p_ba = ∂Δp/∂b_a

Initially:

J_R_bg = 0

J_v_bg = 0

J_v_ba = 0

J_p_bg = 0

J_p_ba = 0

Because no IMU measurement has been integrated yet.

5. WE ALSO INITIALIZE COVARIANCE

Our preintegrated measurement is uncertain.

For example:

ΔR is uncertain
Δv is uncertain
Δp is uncertain

So we maintain a 9 x 9 covariance:

Σ ∈ R^(9x9)

The error state is:

δx =
[ δθ
δv
δp ]

where each component is 3-dimensional.

Therefore:

δx ∈ R^9

and:

Σ = E[δx δx^T]

Initially:

Σ = 0

meaning:

"We have not accumulated any IMU uncertainty yet."

6. NOW AN IMU MEASUREMENT ARRIVES

Suppose we receive:

gyro = ω̃_k

accel = ã_k

dt = Δt

First we remove the estimated biases:

ω_k = ω̃_k - b_g

a_k = ã_k - b_a

Now these are the measurements we actually integrate.

7. WE SAVE THE CURRENT STATE

Before processing this measurement, we save:

ΔR_k
J_R_bg,k
J_v_bg,k
J_v_ba,k
J_p_bg,k
J_p_ba,k

Why?

Because the current IMU sample moves us from:

k → k+1

The equations for the new state depend on the OLD state.

So conceptually:

OLD STATE
|
| IMU measurement
v
NEW STATE

8. ROTATION COMES FIRST

The gyroscope tells us:

angular velocity = ω_k

During:

Δt

the small rotation is:

φ_k = ω_k Δt

This is a rotation vector.

We convert it to a rotation matrix:

ΔR_inc = Exp([φ_k]x)

where:

[φ_k]x = skew-symmetric matrix

[φ]x =
[  0    -φ_z    φ_y
φ_z   0     -φ_x
-φ_y   φ_x     0  ]

Then the accumulated rotation becomes:

# ΔR_(k+1)

ΔR_k ΔR_inc

or:

# ΔR_(k+1)

ΔR_k Exp([ω_k Δt]x)

9. WHY DO WE USE Exp?

Because:

ω_k Δt

is a small rotation vector.

But we need an actual valid rotation matrix.

So:

rotation vector
|
| Exp
v
rotation matrix

The result satisfies:

R^T R = I

and:

det(R) = 1

Therefore it remains a valid SO(3) rotation.

10. VELOCITY UPDATE

The accelerometer gives specific force.

After bias correction:

a_k = ã_k - b_a

But this acceleration is expressed in the current IMU frame.

Our accumulated rotation:

ΔR_k

converts it into the starting preintegration frame.

Therefore:

ΔR_k a_k

is the acceleration expressed in the starting frame.

Over Δt:

# Δv_(k+1)

Δv_k
+
ΔR_k a_k Δt

So:

# Δv_(k+1)

Δv_k + ΔR_k a_k Δt

11. POSITION UPDATE

Position is obtained by integrating velocity.

Using constant acceleration during this small interval:

# Δp_(k+1)

Δp_k
+
Δv_k Δt
+
1/2 ΔR_k a_k Δt^2

So:

# Δp_(k+1)

Δp_k
+
Δv_k Δt
+
1/2 ΔR_k a_k Δt^2

12. IMPORTANT: WHY IS GRAVITY NOT HERE?

Because Δv and Δp are defined as gravity-free relative measurements.

We are storing:

IMU relative motion

not:

complete world motion.

Later, when connecting two actual states:

# v_j

v_i
+
g Δt
+
R_i Δv_ij

and:

# p_j

p_i
+
v_i Δt
+
1/2 g Δt^2
+
R_i Δp_ij

So gravity is handled when we connect the preintegrated measurement to the actual states.

13. NOW WE HANDLE BIAS SENSITIVITY

Suppose our gyroscope bias estimate is slightly wrong.

Then:

b_g
↓
corrected ω changes
↓
ΔR changes
↓
Δv changes
↓
Δp changes

Similarly:

b_a
↓
corrected a changes
↓
Δv changes
↓
Δp changes

That is why we maintain bias Jacobians.

14. GYRO BIAS → ROTATION

We maintain:

# J_R_bg

∂ΔR / ∂b_g

For one integration step:

φ_k = ω_k Δt

and:

ω_k = ω̃_k - b_g

Therefore:

∂ω_k / ∂b_g = -I

The right Jacobian of SO(3) tells us how a small change in the rotation vector changes the resulting local rotation.

Call it:

J_r(φ_k)

Therefore the update is:

# J_R_bg,(k+1)

## ΔR_inc^T J_R_bg,k

J_r(φ_k) Δt

15. WHAT DOES J_r MEAN?

J_r is the SO(3) right Jacobian.

It describes the local sensitivity of the exponential map.

For:

φ = ω Δt

we have:

# J_r(φ)

## I

(1-cosθ)/θ^2 [φ]x
+
(θ-sinθ)/θ^3 [φ]x^2

where:

θ = ||φ||

For very small rotations:

J_r(φ) ≈ I

So for intuition:

small rotation
↓
J_r ≈ I

16. GYRO BIAS → VELOCITY

Velocity depends on rotation:

# Δv_(k+1)

Δv_k
+
ΔR_k a_k Δt

If gyroscope bias changes:

b_g
↓
ΔR changes
↓
ΔR a changes
↓
Δv changes

Therefore:

# J_v_bg

∂Δv / ∂b_g

and the update is:

# J_v_bg,(k+1)

## J_v_bg,k

ΔR_k [a_k]x J_R_bg,k Δt

17. ACCELEROMETER BIAS → VELOCITY

Now:

a_k = ã_k - b_a

Therefore:

∂a_k / ∂b_a = -I

Velocity:

# Δv_(k+1)

Δv_k
+
ΔR_k a_k Δt

Therefore:

# J_v_ba,(k+1)

## J_v_ba,k

ΔR_k Δt

18. GYRO BIAS → POSITION

Position depends on velocity and rotation:

# Δp_(k+1)

Δp_k
+
Δv_k Δt
+
1/2 ΔR_k a_k Δt^2

Therefore:

# J_p_bg,(k+1)

J_p_bg,k
+
J_v_bg,k Δt
-----------

1/2 ΔR_k [a_k]x J_R_bg,k Δt^2

19. ACCELEROMETER BIAS → POSITION

Similarly:

# J_p_ba,(k+1)

J_p_ba,k
+
J_v_ba,k Δt
-----------

1/2 ΔR_k Δt^2

20. NOW COMES UNCERTAINTY PROPAGATION

We have propagated:

ΔR
Δv
Δp

and their bias sensitivities.

But we also need to know:

"How uncertain are these values?"

This is where covariance propagation starts.

We define the error state:

δx =
[ δθ
δv
δp ]

The error dynamics are approximated as:

# δx_(k+1)

F_k δx_k
+
G_k n_k

where:

F_k = state-transition Jacobian

G_k = noise Jacobian

n_k = IMU measurement noise

21. THE NOISE VECTOR

Our IMU has two noise sources:

gyro noise:
η_g

accelerometer noise:
η_a

Combine them:

n_k =
[ η_g
η_a ]

Therefore:

n_k ∈ R^6

22. Q: IMU NOISE COVARIANCE

We describe the uncertainty of the IMU noise using:

# Q_k

E[n_k n_k^T]

Assuming gyro and accelerometer noise are independent:

Q_k =
[ Σ_g   0
0   Σ_a ]

where:

Σ_g = gyro noise covariance

Σ_a = accelerometer noise covariance

So Q is:

6 x 6

23. F: STATE-TRANSITION JACOBIAN

F tells us:

"If my current state has a small error,
how does that error move into the next state?"

We have:

# δx_(k+1)

F_k δx_k
+
...

For our state:

δx =
[ δθ
δv
δp ]

F is:

F =
[ F_RR   F_Rv   F_Rp
F_vR   F_vv   F_vp
F_pR   F_pv   F_pp ]

24. ROTATION PART OF F

Rotation error propagates according to:

# A_k

Exp(-[φ_k]x)

So:

F_RR = A_k

Therefore:

# F_RR

Exp(-[ω_k Δt]x)

For small Δt:

F_RR
≈
I - [ω_k]x Δt

25. VELOCITY PART OF F

Velocity is:

# Δv_(k+1)

Δv_k
+
ΔR_k a_k Δt

A small rotation error changes the acceleration direction.

That gives:

# δv_(k+1)

## δv_k

ΔR_k [a_k]x δθ_k Δt
+
...

Therefore:

# F_vR

-ΔR_k [a_k]x Δt

and:

F_vv = I

26. POSITION PART OF F

Position:

# Δp_(k+1)

Δp_k
+
Δv_k Δt
+
1/2 ΔR_k a_k Δt^2

Therefore:

# F_pR

-1/2 ΔR_k [a_k]x Δt^2

# F_pv

I Δt

# F_pp

I

27. COMPLETE F MATRIX

Therefore:

F_k =
[ A_k                         0              0
-ΔR_k[a_k]x Δt               I              0
-1/2 ΔR_k[a_k]x Δt^2         I Δt           I ]

28. G: HOW IMU NOISE ENTERS THE STATE

F tells us how OLD STATE ERROR propagates.

G tells us how NEW IMU NOISE enters the state.

We have:

# δx_(k+1)

F_k δx_k
+
G_k n_k

For:

n_k =
[ η_g
η_a ]

G becomes:

G =
[ G_Rg   G_Ra
G_vg   G_va
G_pg   G_pa ]

29. GYRO NOISE → ROTATION

Gyroscope noise changes:

ω

which changes:

φ = ω Δt

Therefore:

# G_Rg

J_r(φ_k) Δt

There is no direct gyro-noise contribution to velocity or position during this first-order step.

So:

# G_Rg

J_r(φ_k) Δt

30. ACCELEROMETER NOISE → VELOCITY

Acceleration noise enters:

a

and velocity is:

# Δv_(k+1)

Δv_k
+
ΔR_k a_k Δt

Therefore:

# G_va

ΔR_k Δt

31. ACCELEROMETER NOISE → POSITION

Position receives acceleration noise through:

1/2 ΔR_k a_k Δt^2

Therefore:

# G_pa

1/2 ΔR_k Δt^2

32. COMPLETE G MATRIX

Therefore:

G_k =
[ J_r(φ_k)Δt       0
0         ΔR_kΔt
0       1/2ΔR_kΔt^2 ]

33. NOW WE PROPAGATE COVARIANCE

We already have:

# δx_(k+1)

F_k δx_k
+
G_k n_k

The covariance propagation equation is:

# Σ_(k+1)

F_k Σ_k F_k^T
+
G_k Q_k G_k^T

This is one of the most important equations in the entire preintegration implementation.

34. WHAT DOES THE FIRST TERM MEAN?

The first term is:

F_k Σ_k F_k^T

It means:

"Take the uncertainty we already had
and move it through the new motion."

Story:

old uncertainty
↓
F
↓
new uncertainty

35. WHAT DOES THE SECOND TERM MEAN?

The second term is:

G_k Q_k G_k^T

It means:

"Add the uncertainty introduced
by this new IMU measurement."

Story:

new IMU noise
↓
Q
↓
G
↓
added uncertainty

36. SO THE COMPLETE COVARIANCE STORY IS

Previous uncertainty:

Σ_k

*

new IMU measurement uncertainty:

Q_k

↓

propagate both through the system:

F_k
G_k

↓

new uncertainty:

Σ_(k+1)

Mathematically:

# Σ_(k+1)

F_k Σ_k F_k^T
+
G_k Q_k G_k^T

37. THEN WE UPDATE THE ACTUAL PREINTEGRATED STATE

After calculating all the Jacobians and covariance using the OLD state, we finally update:

Δp

Δv

ΔR

The position update:

# Δp_(k+1)

Δp_k
+
Δv_k Δt
+
1/2 ΔR_k a_k Δt^2

The velocity update:

# Δv_(k+1)

Δv_k
+
ΔR_k a_k Δt

The rotation update:

# ΔR_(k+1)

ΔR_k Exp([ω_k Δt]x)

38. FINALLY UPDATE TIME

We add:

# Δt_(k+1)

Δt_k + dt

So after one IMU sample:

Δt += dt

39. NOW THE FIRST IMU SAMPLE IS FINISHED

We started with:

ΔR = I
Δv = 0
Δp = 0
Σ = 0

Then one IMU measurement arrived:

gyro
accel
dt

We performed:

bias correction
↓
rotation increment
↓
F
↓
G
↓
covariance propagation
↓
bias Jacobians
↓
position
↓
velocity
↓
rotation
↓
time

40. THEN THE SECOND IMU SAMPLE ARRIVES

Now the important thing is:

We DO NOT start from zero again.

We already have:

ΔR_1
Δv_1
Δp_1
Σ_1

and the second measurement:

gyro_2
accel_2
dt_2

continues from there.

So:

IMU 1
↓
ΔR_1, Δv_1, Δp_1, Σ_1
↓
IMU 2
↓
ΔR_2, Δv_2, Δp_2, Σ_2
↓
IMU 3
↓
...
↓
Keyframe j

41. AFTER ALL IMU MEASUREMENTS

Eventually we have:

ΔR_ij

Δv_ij

Δp_ij

Σ_ij

and:

J_R_bg

J_v_bg

J_v_ba

J_p_bg

J_p_ba

42. THIS IS THE FINAL PREINTEGRATED MEASUREMENT

Conceptually:

IMU measurements
from i → j
|
v
Preintegrator
|
+-- ΔR
|
+-- Δv
|
+-- Δp
|
+-- Σ
|
+-- Bias Jacobians
|
v
IMU constraint
between keyframe i and j

43. THEN THE OPTIMIZER USES IT

The actual states are:

R_i, v_i, p_i

and:

R_j, v_j, p_j

The preintegrated rotation gives:

R_j
≈
R_i ΔR_ij

Velocity:

v_j
≈
v_i
+
g Δt
+
R_i Δv_ij

Position:

p_j
≈
p_i
+
v_i Δt
+
1/2 g Δt^2
+
R_i Δp_ij

44. THE COMPLETE MATHEMATICAL PICTURE

Raw IMU:

ω̃_k
ã_k

↓

Bias correction:

ω_k = ω̃_k - b_g

a_k = ã_k - b_a

↓

Preintegrated state:

# ΔR_(k+1)

ΔR_k Exp([ω_kΔt]x)

# Δv_(k+1)

Δv_k + ΔR_k a_k Δt

# Δp_(k+1)

Δp_k
+
Δv_kΔt
+
1/2ΔR_k a_kΔt^2

↓

Bias sensitivity:

J_R_bg
J_v_bg
J_v_ba
J_p_bg
J_p_ba

↓

State uncertainty:

# δx_(k+1)

F_kδx_k + G_kn_k

↓

Noise covariance:

Q_k =
[ Σ_g  0
0    Σ_a ]

↓

Covariance:

# Σ_(k+1)

F_kΣ_kF_k^T
+
G_kQ_kG_k^T

↓

Final result:

ΔR_ij
Δv_ij
Δp_ij
Σ_ij
bias Jacobians

45. THE ONE-LINE STORY

IMU measurements
→ remove bias
→ integrate rotation, velocity and position
→ track how bias affects them
→ track how noise affects them using F, G and Q
→ accumulate covariance Σ
→ produce one relative IMU measurement between two keyframes.
