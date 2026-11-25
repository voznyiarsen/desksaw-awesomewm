-- pose.lua
-- desksaw.pose
-- The skeleton: bones, Godot Bone2D angle semantics, and the soupik two-bone
-- IK. Everything here is pure Lua and is a direct port of PET_RIG_SPEC.md §3
-- and §7 (which were themselves parsed mechanically out of
-- scenes/sawianBase.tscn and soup_two_bone_ik.gd).
--
-- Coordinate conventions (this is where ports usually go wrong):
--   * Skeleton space has y DOWN, the hip at y=-20 and the FEET at y=0.
--   * The pet's local origin (0,0) is the CONTROLLER BODY ORIGIN, which sits
--     on the ground plane at the pet's feet (spec §3.2: capsule pos y=-23.75,
--     height 47.5 -> bottom exactly y=0). So (0,0) is between the feet, NOT
--     the figure's centre.
--   * Godot 2D rotations are CLOCKWISE-positive on screen with y down, so a
--     bone's child offset (cx, cy) points at global angle
--     `bone_global_rot + atan2(cy, cx)`.
--   * `bone_angle` is authored ONLY on leaf bones (head/hand/Foot/Tail) and
--     WINS over the rest matrix (spec §3: for head the rest matrix gives
--     -1.689 deg but the authored bone_angle is -90.0). Non-leaf bones derive
--     theirs from the child's position.

local pose = {}

-- ---------------------------------------------------------------------------
-- 2x3 transforms, Godot Transform2D layout: {xx, xy, yx, yy, ox, oy}
--   x' = xx*x + yx*y + ox
--   y' = xy*x + yy*y + oy
-- ---------------------------------------------------------------------------

function pose.tr(x, y) return { 1, 0, 0, 1, x, y } end

function pose.rot(a)
    local c, s = math.cos(a), math.sin(a)
    return { c, s, -s, c, 0, 0 }
end

function pose.scl(sx, sy) return { sx, 0, 0, sy, 0, 0 } end

-- A * B : B applied first, then A.
function pose.mul(A, B)
    return {
        A[1] * B[1] + A[3] * B[2],
        A[2] * B[1] + A[4] * B[2],
        A[1] * B[3] + A[3] * B[4],
        A[2] * B[3] + A[4] * B[4],
        A[1] * B[5] + A[3] * B[6] + A[5],
        A[2] * B[5] + A[4] * B[6] + A[6],
    }
end

function pose.xform(M, x, y)
    return M[1] * x + M[3] * y + M[5], M[2] * x + M[4] * y + M[6]
end

-- atan2 over the full circle. LuaJIT's math.atan takes y/x, so a wrapper is
-- needed (and Lua 5.4 has no atan2 at all).
function pose.atan2(y, x)
    if x > 0 then
        return math.atan(y / x)
    elseif x < 0 and y >= 0 then
        return math.atan(y / x) + math.pi
    elseif x < 0 then
        return math.atan(y / x) - math.pi
    elseif y > 0 then
        return math.pi / 2
    elseif y < 0 then
        return -math.pi / 2
    end
    return 0
end

local atan2 = pose.atan2

local function wrap(a)
    a = a % (2 * math.pi)
    if a > math.pi then a = a - 2 * math.pi end
    return a
end

pose.wrap = wrap

-- ---------------------------------------------------------------------------
-- The 12 bones (spec §3, verbatim from the scene).
--
-- `rot` is the authored local rotation, which is 0 for every non-leaf bone and
-- 0 for most leaves too. Leaf bones encode their angle through the child
-- offset instead: a child at local (0, +d) already points "down" at +90 deg,
-- which is what Godot's bone_angle means. So no extra rotation is needed.
-- ---------------------------------------------------------------------------

pose.BONES = {
    hip   = { parent = nil,     pos = { -1, -20 },                     rot = 0 },
    back  = { parent = "hip",   pos = { 0, -6 },                       rot = 0 },
    head  = { parent = "back",  pos = { 0, -9 },                       rot = 0 },
    fArm  = { parent = "back",  pos = { 0.00039161742, -7.2501087 },  rot = 0 },
    fLArm = { parent = "fArm",  pos = { -8.245654e-05, 7 },           rot = 0 },
    handF = { parent = "fLArm", pos = { -7.3677336e-05, 6.2538557 },  rot = 0 },
    bArm  = { parent = "back",  pos = { 0.00039161742, -7.250109 },   rot = -0.06446554 },
    bLArm = { parent = "bArm",  pos = { -8.245654e-05, 7 },           rot = 0 },
    handB = { parent = "bLArm", pos = { -7.367725e-05, 6.2538486 },   rot = 0 },
    legF  = { parent = "hip",   pos = { 0.00023555769, -1.1441317e-05 }, rot = 0 },
    crusF = { parent = "legF",  pos = { 3.2530215, 7.501091 },         rot = -0.0005843137 },
    footF = { parent = "crusF", pos = { -6.5410585, 4.0360456 },       rot = 0.4408467 },
    legB  = { parent = "hip",   pos = { 0.00023555769, -1.1441317e-05 }, rot = 0 },
    crusB = { parent = "legB",  pos = { 3.2502916, 7.515524 },         rot = 0 },
    footB = { parent = "crusB", pos = { -6.538575, 4.048879 },         rot = 0.4408467 },
    tail  = { parent = "hip",   pos = { 0.00023555756, -1.1444092e-05 }, rot = 0 },
    -- IKNodes carries the animation's IK targets. Its authored local position
    -- (34.25, -22.25) and rotation (-0.2679) are in the HIP's space.
    ikNodes = { parent = "hip", pos = { 34.250618, -22.24968 },        rot = -0.26791823 },
}

-- Segment lengths used by the IK (spec §3.1): |child.position| for the two
-- bones of each chain.
function pose.seg(bone)
    local b = pose.BONES[bone]
    return math.sqrt(b.pos[1] * b.pos[1] + b.pos[2] * b.pos[2])
end

pose.SEG = {
    thighF = pose.seg("crusF"),   -- 8.176
    shinF  = pose.seg("footF"),   -- 7.686
    thighB = pose.seg("crusB"),   -- 8.188
    shinB  = pose.seg("footB"),   -- 7.691
    upperF = pose.seg("fLArm"),   -- 7.0
    foreF  = pose.seg("handF"),   -- 6.2539
    upperB = pose.seg("bLArm"),   -- 7.0
    foreB  = pose.seg("handB"),   -- 6.2538
    head   = pose.seg("head"),    -- 9.0
}

-- IK chains (spec §2): joint_one -> joint_two. The target is reached at the
-- END of joint_two, i.e. at the TIP bone's origin (footF / handF).
--   flip = the scene's flip_bend_direction (true on both legs, unset on arms)
pose.IK_CHAINS = {
    legF = { b1 = "legF", b2 = "crusF", tip = "footF", target = "ik/IKFLeg:position", flip = true },
    legB = { b1 = "legB", b2 = "crusB", tip = "footB", target = "ik/IKBLeg:position", flip = true },
    armF = { b1 = "fArm", b2 = "fLArm", tip = "handF", target = "ik/IKFArm:position", flip = false },
    armB = { b1 = "bArm", b2 = "bLArm", tip = "handB", target = "ik/IKBArm:position", flip = false },
}

-- soup_two_bone_ik.gd line 14:
--   bend_direction_coefficient = (int(!flip_bend_direction) * 2 - 1)
function pose.bend_of(chain)
    return chain.flip and -1 or 1
end

-- ---------------------------------------------------------------------------
-- Clip sampling
-- ---------------------------------------------------------------------------

function pose.sample_track(track, t)
    local times, vals = track[1], track[2]
    local n = #times
    if n == 0 then return nil end
    if n == 1 or t <= times[1] then return vals[1] end
    if t >= times[n] then return vals[n] end
    local lo, hi = 1, n
    while hi - lo > 1 do
        local mid = math.floor((lo + hi) / 2)
        if times[mid] <= t then lo = mid else hi = mid end
    end
    local a, b = times[lo], times[hi]
    local u = (b > a) and (t - a) / (b - a) or 0
    local va, vb = vals[lo], vals[hi]
    if type(va) == "table" then
        return { va[1] + (vb[1] - va[1]) * u, va[2] + (vb[2] - va[2]) * u }
    end
    return va + (vb - va) * u
end

-- ---------------------------------------------------------------------------
-- The two-bone solve (spec §7, ported verbatim).
--
-- Godot's formulation, with the y-down clockwise-positive convention:
--
--   scale_orient      = sign(det(joint_one.global_transform))    -> 1 here
--   first_bone_vector  = joint_two.position * joint_one.scale
--   second_bone_vector = Vector2(joint_two.get_length(), 0)
--                          .rotated(joint_two.get_bone_angle() * scale_orient)
--                        * |joint_two.global_scale|
--   target_vector      = target.global_position - joint_one.global_position
--
--   length_check(): |target| < |l1 - l2|  or  |target| < 0.001
--       upper_global = PI * (|v1| < |v2|) + target_vector.angle()
--   else:
--       upper_global = acos(clamp((l1^2 + d^2 - l2^2)/(2*l1*d), -1, 1))
--                        * bend_direction_coefficient * scale_orient
--                      + (target - joint_one.global_position).angle()
--                      - first_bone_vector.angle()
--
--   lower_global = (target - joint_two.global_position).angle()
--                  - second_bone_vector.angle()
--
-- Returns (r1, r2) as LOCAL rotations: r1 relative to the parent of joint_one,
-- r2 relative to joint_one (i.e. joint_two's local rotation).
-- ---------------------------------------------------------------------------

local function solve_two_bone(px, py, tx, ty, l1, l2, fb_ang, sb_ang, bend)
    local dx, dy = tx - px, ty - py
    local d = math.sqrt(dx * dx + dy * dy)
    if d < 1e-6 then d = 1e-6 end

    local target_angle = atan2(dy, dx)
    local upper_global

    -- length_check()
    if d < 0.001 or d < math.abs(l1 - l2) then
        upper_global = target_angle
        if l1 < l2 then upper_global = upper_global + math.pi end
    else
        local cos1 = (l1 * l1 + d * d - l2 * l2) / (2 * l1 * d)
        if cos1 < -1 then cos1 = -1 elseif cos1 > 1 then cos1 = 1 end
        upper_global = math.acos(cos1) * bend + target_angle - fb_ang
    end

    -- joint_two's world position (knee / elbow)
    local kx = px + l1 * math.cos(upper_global + fb_ang)
    local ky = py + l1 * math.sin(upper_global + fb_ang)
    local lower_global = atan2(ty - ky, tx - kx) - sb_ang

    return wrap(upper_global), wrap(lower_global - upper_global)
end

-- ---------------------------------------------------------------------------
-- Build the full pose.
--
-- opts:
--   anim, clip, phase    animation
--   hip_extra {x,y}      extra hip offset (skeleton units) -- crouch/offset
--   hip_rot              extra hip rotation (radians) -- lean
--   overrides            bone -> extra local rotation (LiveBody springs)
--   rag                  ragdoll limb state (bone -> {a = extra rot})
-- ---------------------------------------------------------------------------

function pose.build(opts)
    local anim = opts.anim
    local clip = opts.clip or "idleagain"
    local t = opts.phase or 0
    local clipdata = anim[clip]
    local tracks = {}
    if clipdata then
        local len = clipdata.length
        if len and len > 0 and clipdata.loop then t = t % len end
        for k, tr in pairs(clipdata.tracks) do
            tracks[k] = pose.sample_track(tr, t)
        end
    end

    local facing = opts.facing or 1

    local hip_pos = tracks["hip:position"] or { -1, -20 }
    local hip_rot = (tracks["hip:rotation"] or 0) + (opts.hip_rot or 0)
    if opts.hip_extra then
        hip_pos = { hip_pos[1] + opts.hip_extra[1], hip_pos[2] + opts.hip_extra[2] }
    end

    local out = {}

    -- Forward kinematics, parents first.
    local function put(name)
        local b = pose.BONES[name]
        if not b then return end
        local r
        if opts.rag and opts.rag[name] then
            local rs = opts.rag[name]
            local n = (opts.rag_neutral and opts.rag_neutral[name]) or 0
            -- rag_neutral already holds the correct rest rotation (authored
            -- b.rot for non-IK bones, IK-solved rotation for IK-chain bones).
            -- Do not add b.rot again or IK-chain bones (bArm/crusF/etc.) get
            -- double-rotated and limbs detach from the torso.
            r = n + (rs.a or 0)
        else
            local extra = (opts.overrides and opts.overrides[name]) or 0
            r = (name == "hip" and hip_rot or b.rot) + extra
        end
        local p = (name == "hip" and hip_pos or b.pos)
        local local_xf = pose.mul(pose.tr(p[1], p[2]), pose.rot(r))
        local M = b.parent and pose.mul(out[b.parent].M, local_xf) or local_xf
        out[name] = { M = M, rot = r }
    end

    local order = { "hip", "back", "head",
                    "fArm", "fLArm", "handF",
                    "bArm", "bLArm", "handB",
                    "legF", "crusF", "footF",
                    "legB", "crusB", "footB",
                    "tail", "ikNodes" }
    for _, n in ipairs(order) do put(n) end

    -- Two-bone IK (spec §7). Skipped while ragdolled: the limbs are limp then
    -- and the clip's targets are not driving them.
    if not opts.rag then
        for _, chain in pairs(pose.IK_CHAINS) do
            local tgt = tracks[chain.target]
            if tgt then
                -- The clip target is authored in the IKNodes frame.
                local tx, ty = pose.xform(out.ikNodes.M, tgt[1], tgt[2])
                -- Do NOT negate tx when facing left: the draw transform already
                -- mirrors the skeleton via scale.x = -scale, which places the
                -- limb at the mirrored target in screen space. Negating tx here
                -- would double-mirror it. Matches Godot where soup_two_bone_ik
                -- runs on the mirrored skelparent transform; only scale_orient
                -- (bend direction) flips, not the target position.

                local b1 = pose.BONES[chain.b1]
                local b2 = pose.BONES[chain.b2]
                local b3 = pose.BONES[chain.tip]

                local px, py = pose.xform(out[b1.parent].M, b1.pos[1], b1.pos[2])

                local l1, l2
                if chain.b1 == "legF" then l1, l2 = pose.SEG.thighF, pose.SEG.shinF
                elseif chain.b1 == "legB" then l1, l2 = pose.SEG.thighB, pose.SEG.shinB
                elseif chain.b1 == "fArm" then l1, l2 = pose.SEG.upperF, pose.SEG.foreF
                else l1, l2 = pose.SEG.upperB, pose.SEG.foreB end

                -- first_bone_vector.angle() = direction of joint_two's offset
                local fb_ang = atan2(b2.pos[2], b2.pos[1])
                -- second_bone_vector.angle() = joint_two's bone angle
                local sb_ang = atan2(b3.pos[2], b3.pos[1])

                local r1, r2 = solve_two_bone(px, py, tx, ty, l1, l2,
                                              fb_ang, sb_ang, pose.bend_of(chain))

                -- r1 is GLOBAL: make it local to joint_one's parent.
                local parent_rot = atan2(out[b1.parent].M[2], out[b1.parent].M[1])
                local local1 = wrap(r1 - parent_rot)

                out[chain.b1] = { M = pose.mul(out[b1.parent].M,
                                    pose.mul(pose.tr(b1.pos[1], b1.pos[2]),
                                             pose.rot(local1))), rot = local1 }
                out[chain.b2] = { M = pose.mul(out[chain.b1].M,
                                    pose.mul(pose.tr(b2.pos[1], b2.pos[2]),
                                             pose.rot(r2))), rot = r2 }
                out[chain.tip] = { M = pose.mul(out[chain.b2].M,
                                     pose.mul(pose.tr(b3.pos[1], b3.pos[2]),
                                              pose.rot(b3.rot))), rot = b3.rot }
            end
        end
    end

    return out, tracks
end

return pose
