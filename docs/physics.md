# Physics Model

## Units

All internal quantities use SI units.

| Quantity | Unit |
|----------|------|
| Length | m |
| Time | s |
| Mass | kg |
| Velocity | m/s |
| Acceleration | m/s^2 |
| Rolling resistance coefficient mu_r | dimensionless |

Regulation dimensions are converted from inches in `sim_core.utils.constants`.
For example, the 9 ft table playing surface is approximately 2.54 m by 1.27 m,
and a regulation ball radius is 0.028575 m.

## Phase 1: Center-of-mass rolling resistance

This phase models a single ball after the cue strike, with no spin state,
collisions, cushion rebounds, or pockets. The ball is treated as a point mass
for translational motion while retaining radius and mass for later phases.

Rolling resistance is modeled as a force opposite the direction of travel:

\[
\mathbf{F}_r = -\mu_r N \hat{\mathbf{v}}
\]

On a level table, the normal force is:

\[
N = mg
\]

Newton's second law gives:

\[
m\mathbf{a} = -\mu_r mg \hat{\mathbf{v}}
\]

After dividing by mass:

\[
\mathbf{a} = -\mu_r g \hat{\mathbf{v}}
\]

The deceleration magnitude is therefore:

\[
a_r = \mu_r g
\]

Mass cancels because a heavier ball presses proportionally harder into the
cloth. This is why `Ball.mass` is stored for future collision phases but does
not affect Phase 1 rolling deceleration.

## Closed-form timestep update

While the ball is moving in a straight line and no collision occurs, direction
is constant during a timestep. Let:

\[
v_0 = \|\mathbf{v}_0\|,\quad \hat{\mathbf{u}} = \frac{\mathbf{v}_0}{v_0},
\quad a_r = \mu_r g
\]

The time to stop is:

\[
t_{\text{stop}} = \frac{v_0}{a_r}
\]

The stopping distance is:

\[
d_{\text{stop}} = \frac{v_0^2}{2a_r}
\]

For a timestep \(\Delta t < t_{\text{stop}}\):

\[
\mathbf{x}_{1} = \mathbf{x}_0 + \hat{\mathbf{u}}
\left(v_0\Delta t - \frac{1}{2}a_r\Delta t^2\right)
\]

\[
\mathbf{v}_{1} = \hat{\mathbf{u}}\left(v_0 - a_r\Delta t\right)
\]

If \(\Delta t \ge t_{\text{stop}}\), the ball moves exactly
`d_stop` along \(\hat{\mathbf{u}}\) and velocity is set to zero. This avoids
overshooting past the physical stopping point and avoids timestep-stability
issues from explicit Euler integration.

## Stopping threshold

`velocity_stop_threshold` clamps tiny speeds to zero. It is a numerical
cleanup threshold, not part of the physical model.

## Phase 2: Ball-ball collisions (frictionless, normal impulse)

Equal-mass spheres collide along the line of centers. The **contact normal** \(\mathbf{n}\)
points from ball A toward ball B:

\[
\mathbf{n} = \frac{\mathbf{x}_B - \mathbf{x}_A}{\|\mathbf{x}_B - \mathbf{x}_A\|}
\]

Relative normal velocity:

\[
v_{\text{rel},n} = (\mathbf{v}_B - \mathbf{v}_A) \cdot \mathbf{n}
\]

If \(v_{\text{rel},n} \ge 0\), the balls are separating and no impulse is applied.

Otherwise, with coefficient of restitution \(e \in [0, 1]\):

\[
J = -\frac{(1 + e)\, v_{\text{rel},n}}{\frac{1}{m_A} + \frac{1}{m_B}}
\]

Velocity updates (no tangential friction in Phase 2):

\[
\mathbf{v}_A \leftarrow \mathbf{v}_A - \frac{J}{m_A}\mathbf{n},
\quad
\mathbf{v}_B \leftarrow \mathbf{v}_B + \frac{J}{m_B}\mathbf{n}
\]

For equal mass and \(e = 1\), a head-on collision swaps the velocity components along \(\mathbf{n}\).

**Positional correction:** if centers overlap, each ball is shifted by half the penetration depth along \(\mathbf{n}\) to remove overlap before the next sub-step.

**Assumptions:** point-mass translational dynamics; no spin transfer; no ball-ball friction. Tangential effects are deferred to Phase 5.

Each simulation step integrates rolling resistance first, then resolves contacts (up to `collision_iterations` passes).

## Not yet modeled

- Cushion and rail reflections
- Pockets and table boundary constraints
- Spin, angular momentum, rolling/sliding transitions
- Cue strike impulse model

## Validation

The tests compare implementation results against the derived equations:

- Uniform motion when `mu_r = 0`
- Constant speed loss \(v_1 = v_0 - \mu_r g \Delta t\)
- Stopping time \(t_{\text{stop}} = v_0 / (\mu_r g)\)
- Stopping distance \(d_{\text{stop}} = v_0^2 / (2\mu_r g)\)
- Exact clamping at the physical stopping point when a timestep runs past rest
