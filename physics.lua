-- physics.lua
-- simplephys.physics
-- Minimal 2D physics for a single rigid body -- either a box (OBB, the
-- default) or an arbitrary CONVEX POLYGON (a sprite DOP hull: the tight hull
-- of the sprite's opaque pixels) -- colliding against axis-aligned rectangles
-- (floating windows, screen edges). Obstacles may move: windows carry a
-- per-second velocity and transfer it to the body on impact.
--
-- Detection uses the Separating Axis Theorem over the polygon's OWN edge
-- normals (for a box that degenerates to the 4 OBB axes); response is an
-- impulse-style reflection with restitution, tangential friction, and contact
-- friction feeding angular velocity (it rolls when sliding, spins off wall
-- hits).
--
-- No awesome dependencies: can be exercised with plain lua.

local function sign(x)
    return x > 0 and 1 or -1
end

-- Normalize an angle into (-π, π]: the upper bound (π) is inclusive, so a raw
-- π folds to π rather than to -π.
local function wrap_angle(a)
    local r = a % (2 * math.pi)
    if r > math.pi then r = r - 2 * math.pi end
    return r
end

-- math.atan2(y, x) is the two-argument form on LuaJIT / Lua 5.1-5.2 (and on
-- Lua 5.3+ it is also available). Lua 5.3/5.4 renamed it to math.atan(y, x),
-- so support both: prefer math.atan2, otherwise fall back to a manual impl.
--
-- The probe for a two-argument math.atan must compare RESULTS, not merely
-- check that the call succeeds: a one-argument math.atan (Lua 5.1 / LuaJIT)
-- silently ignores the extra argument, so `pcall(math.atan, 1, 1)` returns
-- atan(1) = pi/4 -- exactly the correct two-argument answer for (1, 1) -- and
-- the detection would wrongly conclude that math.atan(y, x) works. atan(0, -1)
-- separates the two unambiguously: the two-argument form is pi, the
-- one-argument form is atan(0) = 0.
local function atan2_fallback(y, x)
    if x > 0 then
        return math.atan(y / x)
    elseif x < 0 then
        return math.atan(y / x) + math.pi * (y >= 0 and 1 or -1)
    elseif y > 0 then
        return math.pi / 2
    elseif y < 0 then
        return -math.pi / 2
    end
    return 0
end

local atan2
if math.atan2 then
    atan2 = math.atan2
else
    local ok, v = pcall(math.atan, 0, -1)
    if ok and type(v) == "number" and math.abs(v - math.pi) < 1e-9 then
        atan2 = math.atan
    else
        atan2 = atan2_fallback
    end
end

-- Some helpers are exported on the `physics` table (physics.X) as a small
-- public API for callers that drive a body directly. They are kept as locals
-- so step()/resolve() can call them directly, then aliased onto the `physics`
-- table for external use.
local physics = {}
-- Shared portability/math helpers, exported so the Awesome-facing layers
-- (box.lua) use exactly one implementation instead of duplicating the
-- version probe.
physics.atan2 = atan2
physics.wrap_angle = wrap_angle
physics.options = {
    gravity = 980,      -- px/s^2 (Earth-like for typical DPI/font sizes)
    restitution = 0.62, -- bounciness on impact (0 = dead, 1 = superball)
    damping = 0.9985,   -- air drag applied to BOTH velocity axes every sub-step
    friction = 0.92,    -- tangential velocity damping on impact
    max_dt = 0.1,       -- clamp frames at 10fps minimum so restarts/lag
                        -- never freeze the box into slow motion
    substeps = 4,       -- integration sub-steps per frame
    rest_epsilon = 30,  -- impact speeds below this rest instead of bouncing

    -- Adaptive sub-stepping (tunneling control). A sub-step never sweeps more
    -- than substep_travel px of linear+angular motion; max_substeps caps the
    -- resulting budget so a pathological velocity cannot stall the main loop.
    -- The default cap resolves ~8px walls up to roughly 190k px/s at max_dt.
    substep_travel = 4,   -- px of sweep allowed per sub-step
    max_substeps = 2400,  -- hard ceiling on sub-steps per frame

    -- Rotation. angle is in radians, omega in rad/s.
    spin_damping = 0.995, -- air drag applied to spin every sub-step
    spin_impulse = 0.6,   -- contact friction -> spin coupling strength
    spin_rest = 0.03,     -- spin below this is snapped to 0
    max_spin = 30,        -- rad/s spin cap

    -- Friction model.
    --   "damping" (default): the historical model -- tangential velocity is
    --      multiplied by `friction` on impact and bled off by slide_atten
    --      while resting. Cheap, and the plushie tuning is built on it.
    --   "impulse": Coulomb friction. The tangential impulse is clamped to
    --      mu * normal impulse, which is the physically consistent form:
    --      friction scales with contact load, rolling-without-slip is
    --      approached instead of approximated, and it cannot compound across
    --      a varying sub-step count.
    friction_model = "damping",
    mu = 0.6,             -- Coulomb friction coefficient (impulse model)

    -- Resting contacts (supported from below). The body settles fully, and a
    -- critically-damped angular spring topples corner/edge perches toward
    -- the nearest axis-aligned angle, hinged on the deepest corner so the
    -- fall is part of the continuous simulation (it carries the landing
    -- momentum and eases out like a real body); once within snap_angle it
    -- snaps exactly flat.
    rest_damping = 0.98, -- angular damping per sub-step while supported
    tip_spring = 24,     -- topple acceleration per radian of tilt (1/s^2)
    sleep_speed = 6,     -- total speed below which a supported body may snap
    snap_angle = 0.02,   -- angle window (~1deg) for snapping to an axis
    snap_spin = 0.03,    -- angular velocity below which a resting body is "still"

    -- Velocity / position smoothing during settling. Instead of an
    -- instantaneous hard cutoff when impact velocity drops below
    -- rest_epsilon, vertical velocity bleeds off via exponential decay
    -- (v *= e^{-rest_atten * sdt}) so the free-fall-to-rest transition
    -- has no visible discontinuity. Baumgarte_beta corrects a fraction of
    -- the floor-penetration per sub-step rather than teleporting the body
    -- onto the surface in a single step.
    rest_atten = 30,      -- exponential decay rate for settling velocity (1/s)
    slide_atten = 3.0,    -- exponential decay rate for horizontal sliding friction while resting
    baumgarte_beta = 0.15, -- fractional positional correction per sub-step
    snap_ease = 0.4,      -- per-frame exponential easing toward the snap target
    max_step_angle = 0.1, -- cap sub-step rotation at ~5.7 deg (radians)

    -- General (non-settle) contact position correction. Pushing a body out
    -- by the FULL overlap the instant a contact is detected pops it when
    -- several contacts fight over the same body (a corner wedged between two
    -- windows is pushed twice in one pass). Instead, leave contact_slop px of
    -- allowed penetration and correct contact_beta of the excess per pass,
    -- letting the sub-steps converge. contact_beta = 1 with slop 0 restores
    -- the old hard behaviour.
    contact_slop = 0.5,   -- px of penetration left uncorrected
    contact_beta = 0.35,  -- fraction of the excess corrected per pass

    -- Resting contact manifold. A body resting on a flat edge really has two
    -- contact points; resolving only the deepest vertex makes the support
    -- alternate between them and buzz. Vertices within manifold_tol px of the
    -- deepest one join the manifold and share the friction/spin impulse, so
    -- opposing torques cancel and a flat pose rests still.
    manifold_tol = 1.5,   -- px depth window for a second contact point

    -- Ejection velocity used by physics.unclip when it frees a wedged body.
    unclip_push = 90,

    no_snap = false,     -- bodies that should keep their angle as-is when
                         -- resting (no topple spring, no axis-aligned snap) so
                         -- a body lies however it lands
}

-- Option validation. A merged option table is the input to every integrator
-- expression, so a nonsensical value (a negative damping factor, a zero
-- max_dt, a NaN leaked in from a user config) does not merely look wrong --
-- it makes the simulation explode or divide by zero. Clamp the merged copy
-- once, at merge time, instead of defending inside the hot loops. The
-- caller's own table is never modified: only the cached merge is sanitised.
local function clampn(v, lo, hi, fallback)
    if type(v) ~= "number" or v ~= v then return fallback end
    if lo and v < lo then return lo end
    if hi and v > hi then return hi end
    return v
end

local function clamp_options(o)
    local d = physics.options
    -- Unit-interval factors: a per-sub-step multiplier outside [0, 1] either
    -- adds energy every sub-step (>1) or flips the sign of the velocity (<0).
    o.restitution  = clampn(o.restitution, 0, 1, d.restitution)
    o.damping      = clampn(o.damping, 0, 1, d.damping)
    o.friction     = clampn(o.friction, 0, 1, d.friction)
    o.spin_damping = clampn(o.spin_damping, 0, 1, d.spin_damping)
    o.rest_damping = clampn(o.rest_damping, 0, 1, d.rest_damping)
    o.baumgarte_beta = clampn(o.baumgarte_beta, 0, 1, d.baumgarte_beta)
    o.contact_beta = clampn(o.contact_beta, 0, 1, d.contact_beta)
    o.snap_ease    = clampn(o.snap_ease, 0, 1, d.snap_ease)
    -- Non-negative rates and thresholds.
    o.gravity      = clampn(o.gravity, nil, nil, d.gravity)
    o.rest_atten   = clampn(o.rest_atten, 0, nil, d.rest_atten)
    o.slide_atten  = clampn(o.slide_atten, 0, nil, d.slide_atten)
    o.tip_spring   = clampn(o.tip_spring, 0, nil, d.tip_spring)
    o.spin_impulse = clampn(o.spin_impulse, 0, nil, d.spin_impulse)
    o.mu           = clampn(o.mu, 0, nil, d.mu)
    o.rest_epsilon = clampn(o.rest_epsilon, 0, nil, d.rest_epsilon)
    o.sleep_speed  = clampn(o.sleep_speed, 0, nil, d.sleep_speed)
    o.snap_angle   = clampn(o.snap_angle, 0, nil, d.snap_angle)
    o.snap_spin    = clampn(o.snap_spin, 0, nil, d.snap_spin)
    o.spin_rest    = clampn(o.spin_rest, 0, nil, d.spin_rest)
    o.contact_slop = clampn(o.contact_slop, 0, nil, d.contact_slop)
    o.unclip_push  = clampn(o.unclip_push, 0, nil, d.unclip_push)
    -- Strictly positive: these are divisors or step budgets.
    o.max_dt       = clampn(o.max_dt, 1e-4, 10, d.max_dt)
    o.max_spin     = clampn(o.max_spin, 1e-3, nil, d.max_spin)
    o.max_step_angle = clampn(o.max_step_angle, 1e-4, math.pi, d.max_step_angle)
    o.substep_travel = clampn(o.substep_travel, 0.25, nil, d.substep_travel)
    o.substeps     = math.floor(clampn(o.substeps, 1, 1000, d.substeps))
    o.max_substeps = math.floor(clampn(o.max_substeps, o.substeps, 100000,
                                       d.max_substeps))
    return o
end
physics.clamp_options = clamp_options

-- Merge a caller option table over the defaults. Each known option is read
-- from `opt` via the `[]` operator, which respects metatable __index, so a
-- config table that inherits tuning (e.g. plushie-specific rest_damping /
-- slide_atten) through `__index = module.config` actually sees them. The
-- result is cached on the caller's table (rawset), so per-frame stepping costs
-- no table churn; pass nil to fall back to the shared defaults.
--
-- The cache is only valid while the config is unchanged. Callers that mutate
-- a config after stepping has begun (box:set_sprite flips no_snap when the
-- body becomes a polygon hull) MUST call physics.invalidate_options, or the
-- stale merge would silently keep the old value forever. Mutating a shared
-- base table that other configs inherit from is covered by the generation
-- counter: physics.invalidate_options() with no argument invalidates every
-- cached merge in the process.
local options_generation = 0

function physics.invalidate_options(cfg)
    if cfg ~= nil then
        rawset(cfg, "__merged_options", nil)
    else
        options_generation = options_generation + 1
    end
end

local function merge_options(opt)
    if not opt then return physics.options end

    local merged = rawget(opt, "__merged_options")
    if not merged or merged.__gen ~= options_generation then
        merged = {}
        for k, default in pairs(physics.options) do
            local v = opt[k]
            if v ~= nil then
                merged[k] = v
            else
                merged[k] = default
            end
        end
        merged.__gen = options_generation
        clamp_options(merged)
        rawset(opt, "__merged_options", merged)
    end

    return merged
end
physics.merge_options = merge_options

-- The axis-aligned orientation (multiple of pi/2) nearest to `angle`. On
-- exact ties (diagonal, ~45+90k deg) it prefers the flatter pose (a multiple
-- of pi) so dropped boxes lie down rather than standing on an edge. Sprite
-- polygon bodies use no_snap so this is only consulted for rectangular OBBs
-- with axis-aligned symmetries.
local function stable_target(angle)
    local two = math.pi / 2
    local k = math.floor((angle + two / 2 - 1e-6) / two)
    local cand = k * two
    local nextk = (angle >= cand) and (k + 1) or (k - 1)
    local nextc = nextk * two
    -- On a near-tie (diagonal), prefer a flat pose (even k, multiple of pi)
    -- over an edge/vertical one.
    if math.abs(math.abs(angle - cand) - math.abs(angle - nextc)) < 1e-3 then
        return (k % 2 == 0) and cand or nextc
    end
    return cand
end
physics.stable_target = stable_target

-- Area centroid of a convex polygon given as a flat local-vertex list. For a
-- symmetric box this is the pivot (0, 0); for a sprite hull built around the
-- sprite's BOUNDING-BOX centre it generally is not, and that difference is
-- what decides which edges the body can rest on.
local function poly_centroid(verts, n)
    local area2, cxs, cys = 0, 0, 0
    for i = 1, n do
        local j = i % n + 1
        local xi, yi = verts[(i - 1) * 2 + 1], verts[(i - 1) * 2 + 2]
        local xj, yj = verts[(j - 1) * 2 + 1], verts[(j - 1) * 2 + 2]
        local cross = xi * yj - xj * yi
        area2 = area2 + cross
        cxs = cxs + (xi + xj) * cross
        cys = cys + (yi + yj) * cross
    end
    if math.abs(area2) < 1e-9 then
        -- Degenerate (zero-area) hull: fall back to the vertex average.
        local sx, sy = 0, 0
        for i = 1, n do
            sx = sx + verts[(i - 1) * 2 + 1]
            sy = sy + verts[(i - 1) * 2 + 2]
        end
        if n > 0 then return sx / n, sy / n end
        return 0, 0
    end
    return cxs / (3 * area2), cys / (3 * area2)
end
physics.poly_centroid = poly_centroid

-- Compute the set of angles at which an edge of the convex polygon
-- lies flat on a horizontal floor (outward normal pointing down) AND
-- the centre of mass sits above that edge -- the body's gravity-stable
-- rest orientations. For a box this gives the four axis-aligned angles
-- {0, ±π/2, π}; for an arbitrary convex hull it gives the exact resting
-- pose the body would assume under gravity.
-- Each entry is a single angle in [-π, π].
--
-- The centre of mass is the hull's area CENTROID, not the pivot. Sprite hulls
-- are built around the sprite's bounding-box centre, which for an asymmetric
-- figure (a plushie with a heavy head, say) is nowhere near the centroid, so
-- assuming the pivot was the CoM admitted resting poses the body would
-- actually topple out of -- and rejected the ones it really falls into.
-- Stability requires the CoM to project INSIDE the supporting edge, which is a
-- two-sided test; the old single comparison only checked the first vertex, so
-- an edge the CoM overhung on the far side still counted as stable.
local function compute_stable_angles(verts, n, com_x, com_y)
    if not com_x then
        com_x, com_y = poly_centroid(verts, n)
    end
    local angles = {}
    for i = 1, n do
        local j = i % n + 1
        local xi, yi = verts[(i - 1) * 2 + 1], verts[(i - 1) * 2 + 2]
        local xj, yj = verts[(j - 1) * 2 + 1], verts[(j - 1) * 2 + 2]
        local dx, dy = xj - xi, yj - yi
        local d2 = dx * dx + dy * dy
        if d2 > 1e-18 then
            -- Outward edge normal: one of ±(dy, -dx), whichever points AWAY
            -- from the centre of mass (winding-order independent, so a hull
            -- from either chain direction works).
            local nx, ny = dy, -dx
            if (com_x - xi) * nx + (com_y - yi) * ny > 0 then
                nx, ny = -nx, -ny
            end
            -- Rotating the body by `a` must bring that outward normal to
            -- (0, +1) -- straight down in screen coordinates (y grows
            -- downward), i.e. the edge is flat on the floor and the body is
            -- above it. atan2 of (0,1) is π/2, so a = π/2 - angle(n).
            local a = wrap_angle(math.pi / 2 - atan2(ny, nx))
            -- Stability: the CoM must project onto the edge SEGMENT, not just
            -- its infinite line. t is the projection parameter along the edge.
            local t = ((com_x - xi) * dx + (com_y - yi) * dy) / d2
            if t >= 0 and t <= 1 then
                angles[#angles + 1] = a
            end
        end
    end
    -- Deduplicate near-identical angles (degenerate / near-flat hulls can
    -- yield the same stable pose from adjacent edges).
    local unique = {}
    for i = 1, #angles do
        local a = angles[i]
        local dup = false
        for j = 1, #unique do
            if math.abs(wrap_angle(unique[j] - a)) < 1e-3 then
                dup = true
                break
            end
        end
        if not dup then unique[#unique + 1] = a end
    end
    return unique
end
physics.compute_stable_angles = compute_stable_angles

-- Exact moment of inertia (per unit mass) of a convex polygon about its
-- pivot (the body center, origin of the local vertex list). Uses the
-- standard polyhedral formula scaled by area, then applies the parallel
-- axis theorem to shift from the centroid to the pivot.
-- For a box the centroid coincides with the pivot, so this reduces to
-- (w^2 + h^2) / 12.
--
-- The centroid used for the shift comes from poly_centroid (Cx = Σ(xi+xj)·cross
-- / (3·area2)). The previous inline version divided by 6·area2, i.e. half the
-- real offset, so the parallel-axis term was a quarter of its true value and
-- every asymmetric hull was modelled as noticeably easier to spin about its
-- pivot than it should be.
local function poly_inertia(verts, n, com_x, com_y)
    local area2 = 0
    for i = 1, n do
        local j = i % n + 1
        local xi, yi = verts[(i - 1) * 2 + 1], verts[(i - 1) * 2 + 2]
        local xj, yj = verts[(j - 1) * 2 + 1], verts[(j - 1) * 2 + 2]
        area2 = area2 + xi * yj - xj * yi
    end
    local area = math.abs(area2) / 2
    if area < 1e-9 then
        return 0
    end
    if not com_x then
        com_x, com_y = poly_centroid(verts, n)
    end
    local I_o = 0
    for i = 1, n do
        local j = i % n + 1
        local xi, yi = verts[(i - 1) * 2 + 1], verts[(i - 1) * 2 + 2]
        local xj, yj = verts[(j - 1) * 2 + 1], verts[(j - 1) * 2 + 2]
        local cross = xi * yj - xj * yi
        I_o = I_o + cross * (xi * xi + yi * yi
                           + xi * xj + yi * yj
                           + xj * xj + yj * yj)
    end
    -- I_o is the second moment about the ORIGIN (the pivot), which is exactly
    -- what the solver wants -- no parallel-axis shift is needed for it. It is
    -- returned directly; the centroid is only used by the stable-angle test.
    return math.abs(I_o) / (12 * area)
end

-- cx, cy is the body CENTER (rotation happens about it). w/h are the full
-- AABB extents. Obstacles are plain {x,y,width,height} AABBs.
-- Bodies store their collision shape as `verts`: a flat list of local
-- coordinates relative to the center, {x1,y1,x2,y2,...}, convex, in any
-- winding order. Step() transforms them to world space each sub-step (into
-- the body's reuse arrays _wvx/_wvy) and SAT tests the OUTER shape's edge
-- normals -- a box OBB is just a 4-vertex polygon.
local function attach_verts(body, verts)
    local n = #verts / 2
    body.verts = verts
    body._n = n
    body._wvx = {}
    body._wvy = {}
    for i = 1, n do
        body._wvx[i] = 0
        body._wvy[i] = 0
    end
    body._rx = 0
    body._ry = 0
    -- Local AABB (w/h): torque arm scaling and the settle hinge use these
    -- as an approximation for polygon bodies.
    local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
    for i = 1, n do
        local lx, ly = verts[(i - 1) * 2 + 1], verts[(i - 1) * 2 + 2]
        if lx < minx then minx = lx end
        if lx > maxx then maxx = lx end
        if ly < miny then miny = ly end
        if ly > maxy then maxy = ly end
    end
    body.w = maxx - minx
    body.h = maxy - miny
    -- Half-extents (_rx/_ry) and AABB centre (_abx/_aby) of the LOCAL
    -- bbox. For a symmetric box these equal ±(w/2, h/2) and the centre
    -- is (0,0). For an asymmetric sprite hull the local AABB centre is
    -- offset from the pivot (0,0) by (_abx, _aby) -- this matters in
    -- sat() where the axis projections and broad phase must reference the
    -- actual AABB centre, not the body centre.
    body._rx = (maxx - minx) / 2
    body._ry = (maxy - miny) / 2
    body._abx = (minx + maxx) / 2
    body._aby = (miny + maxy) / 2
    -- Polygon centre of mass (area centroid) in LOCAL body coordinates. The
    -- pivot (0, 0) is the sprite's bounding-box centre, which for an
    -- asymmetric hull is not the centroid; the stable-angle test needs the
    -- real one. Exposed as com_lx/com_ly so callers (and future contact
    -- solving) can transform it to world space.
    local com_x, com_y = poly_centroid(verts, n)
    body.com_lx, body.com_ly = com_x, com_y
    -- Polygon moment of inertia about the pivot (per unit mass).
    body._inertia = poly_inertia(verts, n, com_x, com_y)
    -- Precompute gravity-stable edge-flat angles (the poses at which
    -- the body naturally rests on a horizontal floor). Used by the
    -- no_snap settling branch so non-axis-aligned polygons still tip
    -- to their true stable orientation.
    body._stable_angles = compute_stable_angles(verts, n, com_x, com_y)
end

function physics.new_box(cx, cy, w, h)
    local hw, hh = w / 2, h / 2
    local body = {
        cx = cx, cy = cy,
        vx = 0, vy = 0,
        angle = 0, omega = 0,
    }
    attach_verts(body, { -hw, -hh, hw, -hh, hw, hh, -hw, hh })
    return body
end

-- A convex polygon body from a flat local-vertex list (sprite DOP hulls).
function physics.new_poly(cx, cy, verts)
    local body = {
        cx = cx, cy = cy,
        vx = 0, vy = 0,
        angle = 0, omega = 0,
    }
    attach_verts(body, verts)
    return body
end

-- Clear the transient contact state of a body. `prev_settling` persists across
-- step() calls on purpose (it gates the angle integration of the next frame's
-- first sub-step, so a hinged settle is not re-dug), which makes it WRONG the
-- moment something outside the integrator moves the body: a respawn, a grab, a
-- throw or a shape swap all invalidate the contact the flag refers to, and a
-- stale `true` would silently skip angle integration on the first sub-step
-- after the change. Call this whenever the body is repositioned or reshaped
-- by hand.
function physics.wake(body)
    if body then
        body.prev_settling = false
    end
end

-- Transform the body's local verts to world space for the current angle,
-- tracking the world AABB (rx/ry half-extents and abx/aby centre) used by
-- the broad phase and the torque arm. Caller updates cx/cy/angle first;
-- results land in the body's reuse arrays (_wvx/_wvy) and _rx/_ry/_abx/_aby
-- -- no per-contact allocation.
local function world_verts(box, cos_a, sin_a)
    local n = box._n
    local wvx, wvy = box._wvx, box._wvy
    local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
    for i = 1, n do
        local lx = box.verts[(i - 1) * 2 + 1]
        local ly = box.verts[(i - 1) * 2 + 2]
        local wx = box.cx + lx * cos_a - ly * sin_a
        local wy = box.cy + lx * sin_a + ly * cos_a
        wvx[i], wvy[i] = wx, wy
        if wx < minx then minx = wx end
        if wx > maxx then maxx = wx end
        if wy < miny then miny = wy end
        if wy > maxy then maxy = wy end
    end
    box._rx = (maxx - minx) / 2
    box._ry = (maxy - miny) / 2
    -- World AABB centre -- for asymmetric hulls this differs from (cx, cy)
    box._abx = (minx + maxx) / 2
    box._aby = (miny + maxy) / 2
end
physics.world_verts = world_verts

-- Convex polygon vs AABB overlap via SAT. Tests the obstacle's two
-- axis-aligned axes plus every edge normal of the body's WORLD-space
-- polygon (the _wvx/_wvy arrays, transformed by the caller for the body's
-- current angle; a box OBB degenerates to its 4 corners, so for boxes the
-- fallback axes are exactly the old OBB axes). Returns the minimum
-- penetration depth and that (unit) axis, or nil when separated. A
-- conservative AABB broad-phase check up front skips the axis loop entirely
-- for obstacles the body cannot possibly reach.
local function sat(box, ob, wvx, wvy, n, rx, ry)
    local ob_cx = ob.x + ob.w / 2
    local ob_cy = ob.y + ob.h / 2
    -- Use the world AABB centre, not (box.cx, box.cy), so asymmetric
    -- polygon hulls whose centroid differs from their pivot are
    -- projected correctly onto the obstacle axes.
    local dcx = box._abx - ob_cx
    local dcy = box._aby - ob_cy
    if box._abx + rx < ob.x or box._abx - rx > ob.x + ob.w
    or box._aby + ry < ob.y or box._aby - ry > ob.y + ob.h then
        return nil
    end

    -- Axis (1, 0) and (0, 1): the obstacle's own edges.
    local o1 = ob.w / 2 + rx - math.abs(dcx)
    if o1 <= 0 then return nil end
    local o2 = ob.h / 2 + ry - math.abs(dcy)
    if o2 <= 0 then return nil end

    local best_o, best_ax, best_ay = math.min(o1, o2), 1, 0
    if o2 <= o1 then
        best_ax, best_ay = 0, 1
    end

    -- The body's edge normals. Project both shapes onto each and keep the
    -- shallowest separating/penetrating axis.
    for i = 1, n do
        local j = i % n + 1
        local dx = wvx[j] - wvx[i]
        local dy = wvy[j] - wvy[i]
        local len = math.sqrt(dx * dx + dy * dy)
        if len > 1e-9 then
            local ax, ay = dy / len, -dx / len
            local bmin, bmax = math.huge, -math.huge
            for k = 1, n do
                local d = wvx[k] * ax + wvy[k] * ay
                if d < bmin then bmin = d end
                if d > bmax then bmax = d end
            end
            local omin, omax = math.huge, -math.huge
            for k = 0, 3 do
                local vx = ob.x + (k % 2) * ob.w
                local vy = ob.y + math.floor(k / 2) * ob.h
                local d = vx * ax + vy * ay
                if d < omin then omin = d end
                if d > omax then omax = d end
            end
            local o = math.min(bmax, omax) - math.max(bmin, omin)
            if o <= 0 then return nil end
            if o < best_o then
                best_o, best_ax, best_ay = o, ax, ay
            end
        end
    end

    return best_o, best_ax, best_ay
end
physics.sat = sat

-- Resolve a single polygon/AABB contact: push the body out along the axis of
-- minimum penetration, reflect the normal velocity component, damp the
-- tangential one, and feed contact friction into the spin.
-- Obstacles may carry a per-second velocity (ob.vx, ob.vy -- moving
-- windows): the collision is evaluated in the obstacle's reference frame,
-- so a window sliding into a resting body shoves it along and a rising one
-- bumps it up. Obstacles without velocity fields treat as static.
-- Returns true when a collision was resolved. `st` is a per-frame state
-- table; st.rested is set when the body rests on a support from below.
-- The internal form expects the caller to have transformed the body's
-- verts into world space (_wvx/_wvy, _rx/_ry) for the current angle.
-- Exported (physics.resolve) as a self-contained single-body resolver.
-- Tangential response on impact, extracted from resolve() for clarity.
-- Two models: "impulse" (Coulomb friction derived from contact-point velocity
-- including spin, applied to linear + angular velocity) and "damping"
-- (historical tangential velocity damping). Mutates box.vx/vy and box.omega.
-- `obx/oby` is the obstacle's per-second velocity.
local function resolve_tangential(box, obx, oby, opt, cx0, cy0, nx, ny,
                                  rvx, rvy, vn, resting, nvn,
                                  tvx, tvy, wvx, wvy, n)
    if opt.friction_model == "impulse" then
        -- Coupled Coulomb friction (item 13). The tangential impulse is derived
        -- from the contact-point velocity (includes spin) and applied to both
        -- linear and angular velocity through the contact's effective mass.
        local jn = nvn - vn            -- final vn minus initial vn
        if jn < 0 then jn = 0 end
        -- Contact point: body vertex deepest along the contact normal.
        local cpx, cpy = cx0, cy0
        local cmind = math.huge
        for k = 1, n do
            local d = (wvx[k] - cx0) * nx + (wvy[k] - cy0) * ny
            if d < cmind then
                cmind = d
                cpx, cpy = wvx[k], wvy[k]
            end
        end
        local rx, ry = cpx - cx0, cpy - cy0
        -- Unit tangent (perpendicular to the normal).
        local tx, ty = -ny, nx
        local tlen = math.sqrt(tvx * tvx + tvy * tvy)
        if tlen > 1e-9 then
            local tnx, tny = tvx / tlen, tvy / tlen
            -- Contact-point velocity relative to the obstacle, including spin
            -- (ω × r = (-ω*ry, ω*rx) in y-down screen coords).
            local cvx = box.vx - obx - box.omega * ry
            local cvy = box.vy - oby + box.omega * rx
            local vt = cvx * tnx + cvy * tny
            -- Effective inverse mass along the tangent: 1 + (r × t)² / I.
            local rxt = rx * ty - ry * tx
            local I = box._inertia > 1e-9 and box._inertia or 1e-9
            local inv_m = 1 + (rxt * rxt) / I
            local jt = math.min(math.abs(vt) / inv_m, opt.mu * jn)
            box.vx = obx + (tvx - jt * tnx) + nvn * nx
            box.vy = oby + (tvy - jt * tny) + nvn * ny
            box.omega = box.omega - jt * rxt / I
        else
            box.vx = obx + tvx + nvn * nx
            box.vy = oby + tvy + nvn * ny
        end
    else
        local active_friction = (resting and ny < 0) and 1.0 or opt.friction
        -- Re-emit in the world frame: obstacle motion + resolved relative.
        box.vx = obx + tvx * active_friction + nvn * nx
        box.vy = oby + tvy * active_friction + nvn * ny
    end
end

-- Resting contact response, extracted from resolve() for clarity. Decays
-- relative velocity along the contact normal (rest_atten) and tangential
-- plane (slide_atten), relative to the moving support. Records the support
-- in `st` for the end-of-frame snap test. Mutates box.vx/vy and st.
local function resolve_resting(box, ob, obx, oby, opt, sdt, st, nx, ny)
    st.rested = true
    st.floor_top = ob.y     -- informational (horizontal supports)
    st.contact_nx, st.contact_ny = nx, ny
    -- Remember the support's velocity so resting tests (below and in the
    -- step-level snap) can be evaluated relative to the support (item 11).
    st.support_vx, st.support_vy = obx, oby
    -- Support-plane offset along the contact normal: the obstacle's extreme
    -- projection in the +n direction. Exact for slanted polygon-edge normals
    -- as well as axis-aligned faces.
    local pd = -math.huge
    for k = 0, 3 do
        local vx = ob.x + (k % 2) * ob.w
        local vy = ob.y + math.floor(k / 2) * ob.h
        local d = vx * nx + vy * ny
        if d > pd then pd = d end
    end
    st.plane_d = pd
    -- Decay the relative velocity about the CONTACT normal (not the world
    -- axes): the normal component rests, the tangential one slides.
    local tx, ty = -ny, nx
    local rrvx = box.vx - obx
    local rrvy = box.vy - oby
    local rvn = rrvx * nx + rrvy * ny
    local rvt = rrvx * tx + rrvy * ty
    rvn = rvn * math.exp(-opt.rest_atten * sdt)
    rvt = rvt * math.exp(-opt.slide_atten * sdt)
    box.vx = obx + rvn * nx + rvt * tx
    box.vy = oby + rvn * ny + rvt * ty
    st.settling = true
end

-- Contact-manifold spin impulse, extracted from resolve() for clarity.
-- Collects every vertex within manifold_tol of the deepest projection and
-- shares the spin impulse across them; on a flat rest the two corners' torques
-- cancel. Skipped when near-rest on a settle support (the spring owns omega
-- there). Mutates box.omega.
local function resolve_manifold_spin(box, obx, oby, opt, st, resting, ny,
                                     cx0, cy0, nx, ny, wvx, wvy, n)
    local min_d = math.huge
    for k = 1, n do
        local d = (wvx[k] - cx0) * nx + (wvy[k] - cy0) * ny
        if d < min_d then min_d = d end
    end

    -- A body settling near-rest on a floor support is exempt: the spring owns
    -- omega there, and the impulse would "see" the pinned topple rotation as
    -- sliding and brake it. Rolling bodies move above sleep_speed relative to
    -- the support, so they still receive the impulse.
    local rel_vx = box.vx - (st.support_vx or 0)
    local rel_vy = box.vy - (st.support_vy or 0)
    local near_rest_settle = resting and ny < 0
        and math.abs(rel_vx) + math.abs(rel_vy) < opt.sleep_speed

    if not near_rest_settle then
        -- Count manifold points first so the impulse can be averaged.
        local mcount = 0
        for k = 1, n do
            local d = (wvx[k] - cx0) * nx + (wvy[k] - cy0) * ny
            if d <= min_d + opt.manifold_tol then
                mcount = mcount + 1
            end
        end
        local wshare = mcount > 0 and (1 / mcount) or 1
        for k = 1, n do
            local d = (wvx[k] - cx0) * nx + (wvy[k] - cy0) * ny
            if d <= min_d + opt.manifold_tol then
                -- Torque arm from the body pivot (cx0, cy0), not the world
                -- AABB centre, so asymmetric sprite hulls get correct spin.
                local rxp = wvx[k] - cx0
                local ryp = wvy[k] - cy0
                -- Surface velocity at the contact point includes angular
                -- motion: v_contact = v + ω × r. In y-down screen coords,
                -- ω × r = (-ω*ry, ω*rx).
                local cvx = (box.vx - obx) - box.omega * ryp
                local cvy = (box.vy - oby) + box.omega * rxp
                local dot = cvx * nx + cvy * ny
                local ttvx = cvx - dot * nx
                local ttvy = cvy - dot * ny
                local inertia = rxp * rxp + ryp * ryp + box._inertia
                if inertia > 1e-9 then
                    box.omega = box.omega
                        - (rxp * ttvy - ryp * ttvx)
                          * opt.spin_impulse * wshare / inertia
                end
            end
        end
    end
end

local function resolve(box, ob, opt, sdt, st, wvx, wvy, rx, ry, n)
    local overlap, ax, ay = sat(box, ob, wvx, wvy, n, rx, ry)
    if not overlap then return false end

    local dcx = box._abx - (ob.x + ob.w / 2)
    local dcy = box._aby - (ob.y + ob.h / 2)
    local flip = dcx * ax + dcy * ay < 0 and -1 or 1
    -- Collision normal, pointing from the obstacle toward the body.
    local nx, ny = ax * flip, ay * flip

    -- Capture the pre-correction centre: the contact-depth and torque-arm
    -- math below must use the pose the world verts were built for, not the
    -- position after the push-out -- a stale arm would be offset by the
    -- penetration and exaggerate the spin impulses.
    local cx0, cy0 = box.cx, box.cy
    -- Positional correction with slop + Baumgarte instead of ejecting the
    -- full overlap in one shot. Pushing out the entire penetration the instant
    -- it is detected makes a body wedged against two obstacles pop (each
    -- contact shoves it the full depth in the same pass); leaving contact_slop
    -- px of allowed penetration and correcting only contact_beta of the excess
    -- lets the two passes / sub-steps converge smoothly. The velocity response
    -- below still uses the true normal, so this only softens the position, not
    -- the bounce.
    local corr = overlap - opt.contact_slop
    if corr < 0 then corr = 0 end
    corr = corr * opt.contact_beta
    box.cx = cx0 + nx * corr
    box.cy = cy0 + ny * corr

    -- Body velocity relative to the obstacle. With a static obstacle this
    -- collapses to the body's own velocity, so the numbers below are the
    -- historical ones; a moving obstacle transfers its motion through the
    -- same impulse math evaluated in its frame.
    local obx, oby = ob.vx or 0, ob.vy or 0
    local rvx, rvy = box.vx - obx, box.vy - oby
    local vn = rvx * nx + rvy * ny
    if vn < 0 then
        local tvx = rvx - vn * nx
        local tvy = rvy - vn * ny
        -- When impact speed is below rest_epsilon, the body is entering a
        -- resting state: NO bounce (nvn = 0). The normal velocity is bled
        -- off by the resting block below via exponential damping instead of
        -- an instant cutoff, so the free-fall-to-settle transition has no
        -- visible discontinuity. A restitution-scaled "resting bounce"
        -- would re-inject a small separating impulse every sub-step; on a
        -- polygon hull with slanted contact normals that impulse carries a
        -- horizontal component and slowly pumps the body sideways.
        local resting = -vn < opt.rest_epsilon
        local nvn = 0
        if not resting then
            nvn = -vn * opt.restitution
        end
        -- Report a genuine impact (bounce) so a caller (e.g. box.lua) can play
        -- a sound. Fires once per sub-step while the body is separating; the
        -- caller is responsible for debouncing. `box._on_impact` is set by the
        -- actor layer and is intentionally NOT part of the merged options.
        if not resting and nvn > 0 and box._on_impact then
            box._on_impact(nvn)
        end

        -- Tangential response. Two models (opt.friction_model):
        --   "impulse": Coulomb friction. The tangential impulse jt that would
        --      arrest sliding is clamped to mu * jn (jn is the normal impulse
        --      just applied). This is the physically consistent form -- it
        --      scales with contact load and cannot compound across a varying
        --      sub-step count -- but it changes the plushie feel, so it is
        --      opt-in.
        --   "damping" (default): the historical tangential velocity damping.
        --      On a floor the flat opt.friction is skipped in favour of the
        --      exponential slide decay in the resting block (applying both
        --      compounds per sub-step and evaporates horizontal momentum).
        resolve_tangential(box, obx, oby, opt, cx0, cy0, nx, ny,
            rvx, rvy, vn, resting, nvn, tvx, tvy, wvx, wvy, n)

        -- Resting on a support from below (screen y is down, so a floor's
        -- outward normal is ny < 0): settle. Horizontal velocity decays via
        -- exponential damping (frame-rate independent: e^{-slide_atten * sdt}
        -- per sub-step) instead of multiplying by a flat opt.friction that
        -- would aggressively zero vx over 4-to-240 sub-steps. Vertical
        -- velocity decays separately via rest_atten so the body eases onto
        -- the surface. The contact normal AND point are recorded so the
        -- settle block below can correct penetration along the true normal
        -- (correct for slanted polygon contacts, not just horizontal floors)
        -- and hinge the topple on the contact point.
        if resting and ny < 0 then
            resolve_resting(box, ob, obx, oby, opt, sdt, st, nx, ny)
        end

        -- Contact manifold: the deepest vertex of the polygon along the
        -- incoming normal is the primary contact, but a body resting on a
        -- flat edge really touches at TWO points. Resolving only the single
        -- deepest vertex makes the support alternate between the two corners
        -- and buzz. Collect every vertex within manifold_tol of the deepest
        -- projection and share the spin impulse across them: on a flat rest
        -- the two corners' torque arms are equal and opposite, so their
        -- contributions cancel and the pose stays still, while a single
        -- tilted contact still spins the body as before.
        resolve_manifold_spin(box, obx, oby, opt, st, resting, ny,
            cx0, cy0, nx, ny, wvx, wvy, n)
    end
    return true
end

-- Exported form: self-contained, computes the world verts from the body's
-- current state. The hot path inside step() passes everything in to avoid
-- the trig and table churn.
function physics.resolve(box, ob, opt, sdt, st)
    opt = merge_options(opt)
    world_verts(box, math.cos(box.angle), math.sin(box.angle))
    local hit = resolve(box, ob, opt, sdt, st,
                   box._wvx, box._wvy, box._rx, box._ry, box._n)
    -- Refresh the world caches if a collision moved the body, so callers
    -- reading these arrays see the post-resolution position.
    if hit then
        world_verts(box, math.cos(box.angle), math.sin(box.angle))
    end
    return hit
end

-- Positional correction after a settle hinge rotation. Rotating the body about
-- its contact point can press another vertex into the support; the general
-- contact correction only notices on the NEXT sub-step, which shows up as a
-- faint buzz for polygon hulls resting on imperfect contacts. Correct a
-- Baumgarte fraction of the residual penetration right away, measured along
-- the recorded CONTACT NORMAL rather than straight down -- a polygon edge can
-- meet an AABB along a slanted minimum-penetration axis, where a purely
-- vertical correction is both too small and pointed the wrong way.
-- Returns true when the body was moved (its world verts are then rebuilt).
local function settle_correct(box, opt, st, cos_a, sin_a)
    local nx, ny = st.contact_nx, st.contact_ny
    if not nx or not st.plane_d then return false end
    world_verts(box, cos_a, sin_a)
    local wvx, wvy = box._wvx, box._wvy
    local min_d = math.huge
    for i = 1, box._n do
        local d = wvx[i] * nx + wvy[i] * ny
        if d < min_d then min_d = d end
    end
    local penetration = st.plane_d - min_d
    if penetration > 0 then
        local corr = penetration * opt.baumgarte_beta
        box.cx = box.cx + nx * corr
        box.cy = box.cy + ny * corr
        world_verts(box, cos_a, sin_a)
        return true
    end
    return false
end

-- Integrate the body forward by dt seconds against the obstacles.
-- `opt` may be a partial table; it is merged over the defaults so callers
-- only need to set the options they care about. The merge result is cached
-- on the caller's table (the config table is rebuilt per-instance, never
-- per-frame), so per-frame stepping costs no table churn.
function physics.step(box, dt, obstacles, opt)
    opt = opt or physics.options
    if opt ~= physics.options then
        opt = merge_options(opt)
    end
    dt = math.min(dt, opt.max_dt)
    -- Adaptive sub-stepping: when a lag frame leaks a large dt the raw
    -- velocity can be thousands of px/s, and fixed 4 substeps would then
    -- move the body tens of thousands of px per sub-step -- straight
    -- through any wall thinner than that jump. Scale the sub-step count so
    -- a single sub-step never sweeps more than opt.substep_travel px. This
    -- makes wall crossing impossible up to the max_substeps ceiling: with the
    -- defaults (4px travel, 2400 steps, max_dt 0.1) a sub-step is at worst
    -- 41.7us, so tunneling an 8px wall would take ~190k px/s. Beyond that the
    -- off-screen recovery in box:step is the backstop.
    -- Adaptive sub-step count covers BOTH linear and angular sweep, so a fast
    -- spin cannot tunnel or visually sweep through a thin obstacle either.
    -- Linear and angular sweep distances per frame, in px: speed * dt for
    -- the linear term, |omega| * radius * dt for the arc the rim sweeps.
    -- Summing them directly is dimensionally correct; the old code
    -- multiplied the already-distance spin term by dt a second time,
    -- overstating the required sub-step count.
    local radius = math.max(box._rx, box._ry)
    local linear_dist = math.max(math.abs(box.vx), math.abs(box.vy)) * dt
    local spin_dist = math.abs(box.omega) * radius * dt
    local step_dist = linear_dist + spin_dist
    local substeps = opt.substeps + math.floor(step_dist / opt.substep_travel)
    if substeps > opt.max_substeps then substeps = opt.max_substeps end
    local sdt = dt / substeps
    local st = { rested = false, support_vx = 0, support_vy = 0 }
    local was_settling = box.prev_settling or false

    for _ = 1, substeps do
        st.settling = false
        -- Reset per-substep contact state: a transient touch in an early
        -- substep must not leave st.rested=true (and a stale support velocity)
        -- for the end-of-frame snap check. The final substep re-sets these when
        -- the body is genuinely at rest on a support.
        st.rested = false
        st.support_vx = 0
        st.support_vy = 0
        box.vy = box.vy + opt.gravity * sdt
        -- Linear (air) drag: exponential decay so it is sub-step-count
        -- independent (with adaptive sub-stepping the count varies 4..240).
        -- Applied to BOTH axes -- this is air drag on the body's velocity, and
        -- damping only vx made the option a horizontal-only brake that
        -- contradicted its own name (and let vertical motion coast
        -- undamped, so a bouncing body lost energy only through
        -- restitution). With the default 0.9985 the vertical effect is a
        -- 0.09/s decay: terminal velocity stays far above any real throw,
        -- so this corrects the model without changing the feel.
        local drag = math.exp(-(1 - opt.damping) * 60 * sdt)
        box.vx = box.vx * drag
        box.vy = box.vy * drag

        -- Velocity is never clamped: flings may move the box arbitrarily
        -- fast. Extreme speeds in a lag frame (dt near max_dt) could in
        -- principle tunnel an obstacle; the off-screen recovery in box:step
        -- is the backstop, and typical real speeds (a few thousand px/s)
        -- move the box only a few px per sub-step, well inside any wall.

        -- While settling, the settle block below drives the rotation about
        -- the hinge, so the angle is not advanced about the center here
        -- (that would dig the corner into the floor and make the resolve
        -- push sideways). The status must persist across step() calls, so
        -- the first sub-step of a frame never re-dig.
        if not was_settling then
            -- Cap per-sub-step rotation to prevent single-frame rotational
            -- jitter at high spin rates.
            local max_step_omega = opt.max_step_angle / sdt
            if math.abs(box.omega) > max_step_omega then
                box.omega = max_step_omega * sign(box.omega)
            end
            box.angle = box.angle + box.omega * sdt
        end
        -- Angular drag: exponential decay scaled by sdt so it is frame-rate
        -- independent. The original code used box.omega * opt.spin_damping as
        -- a flat per-sub-step multiplier -- with up to 240 sub-steps during
        -- fast motion, 0.995^240 ≈ 0.29, stripping most spin in a single
        -- frame and causing unnatural rotational acceleration/deceleration.
        box.omega = box.omega * math.exp(-(1 - opt.spin_damping) * 60 * sdt)
        -- Same for spin: while settling, the spring owns omega, so do not
        -- snap its small increments to zero here.
        if not was_settling and math.abs(box.omega) < opt.spin_rest then
            box.omega = 0
        elseif math.abs(box.omega) > opt.max_spin then
            box.omega = opt.max_spin * sign(box.omega)
        end

        box.cx = box.cx + box.vx * sdt
        box.cy = box.cy + box.vy * sdt

        -- The rotation of this sub-step is final: cos/sin (used by the
        -- world transforms) are computed once here, and the body's world
        -- verts + AABB are built for every contact of the sub-step.
        local cos_a, sin_a = math.cos(box.angle), math.sin(box.angle)
        world_verts(box, cos_a, sin_a)
        local wvx, wvy = box._wvx, box._wvy
        local rx, ry, n = box._rx, box._ry, box._n

        -- Two passes keep the body stable when pressed between neighbours.
        for pass = 1, 2 do
            local resolved = false
            for i = 1, #obstacles do
                if resolve(box, obstacles[i], opt, sdt, st, wvx, wvy, rx, ry, n) then
                    resolved = true
                    -- The body moved; rebuild its world-space caches so the
                    -- next obstacle (and next pass) tests the corrected pose.
                    world_verts(box, cos_a, sin_a)
                    rx, ry = box._rx, box._ry
                end
            end
            if not resolved then break end
        end

        -- Settling: while the box supports its weight on a surface from
        -- below, an angular spring topples it toward its resting pose.
        -- omega stays live physics state (the landing momentum carries
        -- into the fall, and the spring eases the box to a stop like a
        -- real topple). The axis-aligned branch rotates about the deepest
        -- corner of its current pose (freshly computed, so the resolve can
        -- never leave a stale hinge), pinning that corner to the support
        -- top so the contact never lifts, digs, or stutters. The no_snap
        -- branch (rolling sprites) never hinges: see below.
        if st.settling then
            -- Deepest vertex = the contact point (the hinge) with the floor,
            -- used by the axis-aligned topple branch below.
            local px, py, best = 0, 0, -math.huge
            for i = 1, n do
                local wy = wvy[i]
                if wy > best then
                    best = wy
                    px = wvx[i]
                    py = wy
                end
            end

            if opt.no_snap then
                -- Rolling sprites / arbitrary polygons: tip toward the body's
                -- own gravity-stable edge-flat angles instead of an
                -- axis-aligned pose.
                --
                -- While the body is still moving (rolling / sliding) the
                -- settle code must NOT hinge it around the contact point:
                -- the body has already been translated by vx/vy this
                -- sub-step, and hinging adds free translation on top of
                -- that -- a rolling body would effectively receive
                -- "translation from vx + translation from the rolling
                -- hinge" and appear to self-accelerate. So while moving,
                -- just damp the spin; only near rest does a weak spring
                -- ease the body toward its nearest stable pose.
                local speed = math.abs(box.vx) + math.abs(box.vy)

                if speed > opt.sleep_speed then
                    -- Active rolling/sliding: no hinge, no spring -- just
                    -- angular drag. The rotation is advanced here because
                    -- the settle block owns the angle while was_settling
                    -- (the integration above skips it).
                    if was_settling then
                        -- Cap the per-sub-step rotation here too: the
                        -- integrator above only caps it while was_settling is
                        -- false, so a rolling body hinging on contact would
                        -- otherwise rotate an uncapped amount (item 10).
                        local max_step_omega = opt.max_step_angle / sdt
                        if math.abs(box.omega) > max_step_omega then
                            box.omega = max_step_omega * sign(box.omega)
                        end
                        box.angle = box.angle + box.omega * sdt
                    end
                    box.omega = box.omega
                        * math.exp(-(1 - opt.rest_damping) * 60 * sdt)
                    -- Rolling advances the angle, which can dig a vertex in;
                    -- correct along the contact normal (no hinge translation,
                    -- so this cannot add the free rolling displacement the
                    -- hinge used to).
                    settle_correct(box, opt, st,
                                   math.cos(box.angle), math.sin(box.angle))
                else
                    -- Near rest: gently settle toward the nearest
                    -- gravity-stable edge-flat pose. A very weak spring --
                    -- tip_spring * 5 is far too strong for a body that
                    -- should roll (24 * 5 would act like a wheel motor) --
                    -- so cap the effective rate. The spring only runs
                    -- here, at speeds below sleep_speed, so it can never
                    -- act as a "wheel motor" on a rolling body (that was
                    -- the old code's problem: the 5x-strong spring + hinge
                    -- applied at ANY speed). The rotation is hinged about
                    -- the contact point: the body is (nearly) motionless,
                    -- so this is the settle topple (the pinned corner
                    -- carries the topple's momentum) -- NOT the free
                    -- translation of an actively rolling body. The
                    -- end-of-step snap below finishes the job once the
                    -- body is truly still.
                    local stable_angles = box._stable_angles or {}
                    local best_da = math.huge
                    for s = 1, #stable_angles do
                        local sang = stable_angles[s]
                        local sang_da = wrap_angle(sang - box.angle)
                        if math.abs(sang_da) < math.abs(best_da) then
                            best_da = sang_da
                        end
                    end

                    if #stable_angles > 0 then
                        local k_tip = math.min(opt.tip_spring, 2)
                        box.omega = box.omega
                            * math.exp(-2 * math.sqrt(k_tip) * sdt)
                            + best_da * k_tip * sdt
                    end
                    box.omega = box.omega
                        * math.exp(-(1 - opt.rest_damping) * 60 * sdt)
                    -- Hinge the rotation about the contact point (pinned
                    -- corner), so the topple is part of the continuous
                    -- simulation and carries the landing momentum. Only on
                    -- contact sub-steps AFTER the first: while was_settling
                    -- was false the integrator above already advanced the
                    -- angle by omega*sdt, so hinging again here would rotate
                    -- the body twice on its very first contact sub-step
                    -- (item 9).
                    if was_settling then
                        local dstep = box.omega * sdt
                        local cos_d, sin_d = math.cos(dstep), math.sin(dstep)
                        local ox = box.cx - px
                        local oy = box.cy - py
                        box.angle = box.angle + dstep
                        box.cx = px + cos_d * ox - sin_d * oy
                        box.cy = py + sin_d * ox + cos_d * oy
                        -- The hinge rotation may have pressed another hull
                        -- vertex into the support; correct that residual
                        -- penetration along the contact normal now instead of
                        -- buzzing on it until the next sub-step's resolve pass.
                        settle_correct(box, opt, st,
                                       math.cos(box.angle), math.sin(box.angle))
                    end
                end
            else
                local da = stable_target(box.angle) - box.angle
                if math.abs(da) > opt.snap_angle then
                    -- Topple toward the nearest axis-aligned angle, hinged on
                    -- the deepest corner so the fall is part of the continuous
                    -- simulation (it carries the landing momentum and eases out).
                    box.omega = box.omega
                        * math.exp(-(1 - opt.rest_damping) * 60 * sdt)
                        + da * opt.tip_spring * sdt
                    -- Hinge only on contact sub-steps after the first
                    -- (item 9): the integrator already advanced the angle by
                    -- omega*sdt this sub-step while was_settling was false, so
                    -- a second hinge here would double-rotate on first contact.
                    if was_settling then
                        local dstep = box.omega * sdt
                        local cos_d, sin_d = math.cos(dstep), math.sin(dstep)
                        local ox = box.cx - px
                        local oy = box.cy - py
                        box.angle = box.angle + dstep
                        box.cx = px + cos_d * ox - sin_d * oy
                        box.cy = py + sin_d * ox + cos_d * oy
                        -- Baumgarte positional stabilization along the contact
                        -- normal: correct a fraction of the residual penetration
                        -- so the pivot follows the corner smoothly. Using the
                        -- recorded normal (not a bare vertical delta) keeps this
                        -- correct if the support contact was slanted.
                        settle_correct(box, opt, st,
                                       math.cos(box.angle), math.sin(box.angle))
                    end
                else
                    -- Already nearly aligned: just damp the residual spin.
                    -- The position still gets its Baumgarte nudge, otherwise
                    -- the body would sit permanently contact_slop px inside
                    -- the surface (the general contact correction stops at the
                    -- slop by design); with it, a fully settled body rests
                    -- flush.
                    box.omega = box.omega
                        * math.exp(-(1 - opt.rest_damping) * 60 * sdt)
                    settle_correct(box, opt, st, cos_a, sin_a)
                end
            end
        end

        was_settling = st.settling

        -- Impact friction may have re-fed a sliver of spin; while resting
        -- the settle block (or the snap below) handles omega, so only
        -- put back the spin of a free box.
        if not st.settling and math.abs(box.omega) < opt.spin_rest then
            box.omega = 0
        end
    end

    -- Settling status outlives the frame: it gates the angle integration
    -- of the first sub-step of the next step() call.
    box.prev_settling = was_settling

    -- A body resting on a support that is nearly still and nearly axis
    -- aligned is eased to that orientation: the remaining fraction of a
    -- degree is interpolated over several frames instead of hard-snapping,
    -- so there is no twitch or drift at rest. Once the angle is within
    -- 1e-4 rad (~0.006 deg) of the target, it is pinned exactly and all
    -- residual velocities are zeroed.
    --
    -- For no_snap bodies the target is the nearest gravity-stable
    -- edge-flat angle (computed from the body's own hull) rather than
    -- the axis-aligned stable_target, so a polygon that landed at an
    -- arbitrary orientation tips to its true resting pose instead of
    -- freezing mid-tilt.
    -- Snap condition is evaluated relative to the support: a body truly at
    -- rest rides with a moving support (its world speed can be large) (item 11).
    local snap_rel_vx = box.vx - (st.support_vx or 0)
    local snap_rel_vy = box.vy - (st.support_vy or 0)
    if st.rested
    and math.abs(snap_rel_vx) + math.abs(snap_rel_vy) < opt.sleep_speed
    and math.abs(box.omega) < opt.snap_spin then
        local target
        if opt.no_snap and box._stable_angles then
            local best_da, best_ang = math.huge, box.angle
            local sa = box._stable_angles
            for s = 1, #sa do
                local sang = sa[s]
                local sang_da = wrap_angle(sang - box.angle)
                if math.abs(sang_da) < math.abs(best_da) then
                    best_da, best_ang = sang_da, sang
                end
            end
            target = best_ang
        else
            target = stable_target(box.angle)
        end
        local da = wrap_angle(target - box.angle)
        if math.abs(da) < opt.snap_angle then
            box.angle = box.angle + opt.snap_ease * da
            if math.abs(da) < 1e-4 then
                box.angle = target
                -- Pin the body to its support instead of zeroing world
                -- velocity: on a moving support this makes the body ride
                -- along rather than freeze in world space (item 12).
                box.vx = st.support_vx or 0
                box.vy = st.support_vy or 0
                box.omega = 0
            end
        end
    end
end

-- Eject `box` out of any obstacle it is currently intersecting, along the
-- shallowest separating axis reported by the SAT (the minimum-penetration
-- axis). This is a positional correction performed iteratively: each pass
-- finds the deepest remaining overlap across all obstacles, pushes the body
-- out by penetration + 1px along that escape normal, and repeats up to max_iter
-- times. It exists so a body that spawns clipped into a wall, floating window
-- or wibar (classic AwesomeWM restart: state is lost, a fresh body is placed
-- at the workarea centre which may overlap a running floating client) starts
-- the frame free to collide instead of fused to its own geometry. resolve()
-- alone cannot free a deeply-wedged spawn in one substep, and leaving the
-- body intersecting would make it re-stick as soon as resolve pushes it
-- along. A small separating velocity is set along the escape normal so the
-- body leaves the seam with momentum rather than settling right back onto it.
-- angle is left untouched (the box is pushed, not spun). Returns the number
-- of ejection iterations performed (0 => the body was already free) plus the
-- escape normal of the final push, so a caller can blend its own velocity
-- with the ejection instead of losing it.
--
-- `keep_velocity` suppresses the velocity overwrite: box:release_toss uses it
-- so a throw that happens to start inside geometry is preserved rather than
-- replaced by the ejection shove.
function physics.unclip(box, obstacles, opt, max_iter, keep_velocity)
    max_iter = max_iter or 8
    opt = merge_options(opt)
    local count = 0
    local cos_a, sin_a = math.cos(box.angle), math.sin(box.angle)
    world_verts(box, cos_a, sin_a)
    local wvx, wvy, n = box._wvx, box._wvy, box._n
    local rx, ry = box._rx, box._ry
    local esc_nx, esc_ny = 0, 0
    for _ = 1, max_iter do
        local best_o, best_nx, best_ny = nil, 0, 0
        for i = 1, #obstacles do
            local ob = obstacles[i]
            local dcx = box._abx - (ob.x + ob.w / 2)
            local dcy = box._aby - (ob.y + ob.h / 2)
            -- AABB broad-phase using the body's world AABB: if the AABBs
            -- are separated, the SAT cannot be.
            if math.abs(dcx) <= ob.w / 2 + rx
               and math.abs(dcy) <= ob.h / 2 + ry then
                local o, ax, ay = sat(box, ob, wvx, wvy, n, rx, ry)
                if o then
                    -- flip so the normal points from the obstacle toward the
                    -- body centre (i.e. outward / away from the obstacle).
                    local flip = dcx * ax + dcy * ay < 0 and -1 or 1
                    local nx, ny = ax * flip, ay * flip
                    if not best_o or o < best_o then
                        best_o, best_nx, best_ny = o, nx, ny
                    end
                end
            end
        end
        if not best_o then break end
        box.cx = box.cx + best_nx * (best_o + 1)
        box.cy = box.cy + best_ny * (best_o + 1)
        -- The body moved: rebuild its world verts for the next pass.
        world_verts(box, math.cos(box.angle), math.sin(box.angle))
        rx, ry = box._rx, box._ry
        esc_nx, esc_ny = best_nx, best_ny
        if not keep_velocity then
            -- Overwrite velocity with a modest separating shove; the body was
            -- wedged, so it should leave the wall rather than inherit whatever
            -- spawn velocity happened to push it deeper.
            local push = opt.unclip_push
            box.vx, box.vy = best_nx * push, best_ny * push
            box.omega = 0
        end
        count = count + 1
    end
    if count > 0 then
        -- The body was teleported: any settling state it had refers to a
        -- contact that no longer exists, so the next step must integrate the
        -- angle normally instead of assuming a live hinge.
        box.prev_settling = false
    end
    return count, esc_nx, esc_ny
end

return physics
