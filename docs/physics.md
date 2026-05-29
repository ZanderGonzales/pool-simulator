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

## Phase 2: Ball-ball collisions (normal impulse)

Phase 2 introduced normal impulse only. Phase 5 added tangential impulse with
default \(\mu_{bb} = 0.15\). The derivation below is the normal component;
tangential terms are in the Phase 5 section.

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

## Phase 3: Rail collisions

Rails are modeled as finite `CushionSegment` line segments on `TableConfig`.
The default table uses four rectangular cushions around the playable surface.
Each cushion has a unit normal pointing into the playable area.

For each active ball and cushion, the detector projects the ball center onto
the finite segment and clamps the projection to the segment endpoints. If
\(\mathbf{c}\) is the closest point and \(\mathbf{n}\) is the inward cushion
normal, the signed center distance is:

\[
d = (\mathbf{x} - \mathbf{c}) \cdot \mathbf{n}
\]

The ball penetrates when:

\[
r - d > 0
\]

where \(r\) is the ball radius. This closest-point test handles the finite
length of each rail segment without assuming an infinite wall. Exact segment
endpoints are excluded for now because pocket/corner geometry is deferred.

**Positional correction / anti-tunneling:** Phase 3 uses post-step positional
correction rather than sub-stepping. Whenever penetration is positive, the ball
is translated by \((r - d)\mathbf{n}\), independent of velocity. This matches
the Phase 2 overlap correction pattern and prevents a high-speed ball that
crosses a rail in one timestep from remaining outside the playable area.

Velocity reflection is applied only when the ball moves into the cushion:

\[
v_n = \mathbf{v} \cdot \mathbf{n} < 0
\]

The normal component is then reflected with coefficient of restitution \(e\):

\[
\mathbf{v}'_n = -e\mathbf{v}_n
\]

The tangential component is preserved by default. `cushion_tangential_damping`
optionally scales tangential velocity on cushion hits:

\[
\mathbf{v}'_t = (1 - d_t)\mathbf{v}_t
\]

where \(d_t \in [0, 1]\). Spin and physically detailed cushion friction remain
out of scope until the spin phase.

Each simulation step integrates rolling resistance, resolves ball-ball
contacts, resolves rail contacts, and then runs a final ball-ball cleanup pass.
The final pass is a stability guard for dense scenes: rail correction can push
balls back into nearby balls, so the last pass removes any new pairwise
penetration before time advances.

## Phase 4: Full rack simulation

Phase 4 adds layout helpers for a regulation-style 15-ball triangle and a
break-shot factory. The table uses coordinates `[0, width] x [0, height]` with
cushions on the boundary edges.

Default placement on the long axis:

- Foot spot (rack apex): `(0.25 * width, 0.5 * height)`
- Head spot (cue ball target line): `(0.75 * width, 0.5 * height)`

The triangle uses five rows (1 + 2 + 3 + 4 + 5 balls). Row spacing along the
table is `sqrt(3) * radius`; balls within a row are spaced `2 * radius`.
Rows extend from the apex toward the foot rail (-x). The cue ball starts on the
head side with velocity toward the apex.

**Assumptions:** no pockets, no cue english on the break, ball-ball contacts use
default \(\mu_{bb}\) from Phase 5,
and O(n^2) all-pairs collision detection (acceptable for 16 balls).

Diagnostics exposed for tests:

- total kinetic energy
- maximum pairwise overlap
- moving-ball count

The Phase 4 break-shot tests treat total kinetic energy as a regression signal:
with rolling resistance and restitution at or below 1, the simulation should not
gain translational kinetic energy from one timestep to the next beyond a small
floating-point tolerance. This is not a full physical validation of a real
break because pockets and a full cue impact model are still absent.

## Phase 5: Spin and rolling/sliding cloth friction

Phase 5 adds scalar spin about the vertical axis \(\omega_z\) (rad/s) to each
`Ball`. Solid-sphere inertia:

\[
I = \frac{2}{5} m r^2
\]

**Slip velocity** (lumped 2D cloth model, motion-aligned):

\[
\mathbf{v}_\text{slip} = \mathbf{v} - \omega_z r \hat{\mathbf{v}}
\]

When \(\|\mathbf{v}\|\approx 0\), a fixed reference axis avoids singularities.
Positive \(\omega_z\) is **top spin** and negative \(\omega_z\) is **draw**
along the current velocity direction on a break toward \(-x\). Side english and
a full contact-patch solve are not modeled.

**Cloth friction:**

- **Sliding:** reduce \(\|\mathbf{v}_\text{slip}\|\) by up to \(\mu_s g\Delta t\)
  along \(-\hat{\mathbf{v}}_\text{slip}\), with coupled \(\omega\) adjustment.
- **Rolling:** when slip is below `sliding_speed_threshold` or rim speed exceeds
  center speed, use Phase 1 rolling resistance on translation plus optional spin
  decay \(d\omega/dt = -k_\omega \omega\).

This is a **lumped cloth model**, not a full rigid-body contact solve. Energy is
not fully conserved once sliding friction and inelastic tangential impulses act.

**Ball-ball tangential impulse** (after normal impulse \(J_n\)):

- Tangential unit vector \(\mathbf{t} = \mathbf{n}^\perp\)
- Relative tangential velocity at contact includes spin:
  \(v_{\text{rel},t} = (\mathbf{v}_B - \mathbf{v}_A)\cdot\mathbf{t} + r(\omega_A + \omega_B)\)
- Impulse capped by Coulomb friction \(|J_t| \le \mu_{bb} |J_n|\)
- Default \(\mu_{bb} = 0.15\) (`DEFAULT_BALL_BALL_FRICTION`). Set
  `ball_ball_friction=0` only to reproduce frictionless normal-only contacts.

**Cue english:** `ShotParams(speed, omega)` sets initial cue velocity and
\(\omega_z\) in `create_break_setup` (no full cue–ball impact model).

**Diagnostics / snapshot:** `rotational_kinetic_energy`, `total_energy`, and
`omega` per ball in `snapshot()`.

**Explicitly not modeled:** 3D spin vector, Magnus force, pockets, cue impact
beyond initial conditions, cushion throw beyond existing tangential damping.

## Not yet modeled

- Pockets and table boundary constraints
- Cue strike impulse model (dynamic cue–ball contact)
- Visualization and inverse shot optimization (later phases)

## Validation

The tests compare implementation results against the derived equations:

- Uniform motion when `mu_r = 0`
- Constant speed loss \(v_1 = v_0 - \mu_r g \Delta t\)
- Stopping time \(t_{\text{stop}} = v_0 / (\mu_r g)\)
- Stopping distance \(d_{\text{stop}} = v_0^2 / (2\mu_r g)\)
- Exact clamping at the physical stopping point when a timestep runs past rest
- Specular reflection off a straight cushion with \(e = 1\)
- Rail overlap separation even when the ball is not moving into the cushion
- Non-overlapping 15-ball triangle rack geometry
- Break-shot energy does not increase step-to-step beyond numerical tolerance
- Break-shot overlap correction prevents persistent ball intersections
- Diagnostic helpers report kinetic energy, overlap, and moving-ball count
- Slip velocity near zero when \(|\mathbf{v}| \approx |\omega| r\) (pure rolling)
- Spin decay under cloth friction; sliding balls lose speed and spin
- Tangential ball-ball impulse changes \(\omega\) and respects Coulomb cap
