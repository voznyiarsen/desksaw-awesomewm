-- skeleton.lua
-- desksaw.skeleton
-- Articulated rendering of the desksaw pet: the bone chain, the body-part
-- sprites, and Cairo drawing.
--
-- Coordinate systems (this is the part that is easy to get wrong):
--
--   SKELETON SPACE  y down. The pet's local origin (0,0) is the CONTROLLER
--                   BODY ORIGIN, which sits on the ground plane at the pet's
--                   FEET (spec §3.2: capsule pos y=-23.75, height 47.5 ->
--                   bottom exactly y=0; hip at y=-20; head at y=-35).
--                   So (0,0) is BETWEEN THE FEET, not the figure's centre.
--
--   SCREEN SPACE    y down (cairo). The physics body's centre is placed at
--                   (body.cx, body.cy) in a wibox whose centre is (w/2, h/2).
--
-- The placement matrix maps skeleton space to screen. Because the skeleton's
-- origin is at the feet and the wibox is centred on the BODY, the matrix has
-- to place (0,0) at the feet, i.e. at (w/2, h/2 + h_body/2) in wibox space.
-- That single fact is what makes the feet land on the floor instead of
-- hovering above it or sinking through it.

local lgi = require("lgi")
local cairo = lgi.cairo

local anim = require("desksaw.anim")
local rig = require("desksaw.pose")

local skeleton = {}

-- ---------------------------------------------------------------------------
-- Body-part table (spec §2 + §4): which bone carries each part, the
-- RemoteTransform2D attachment offset, the Sprite2D local offset/rotation,
-- and the draw order.
-- ---------------------------------------------------------------------------

local PARTS = {
    -- tail (behind everything)
    { tex = "experimentTail",     bone = "tail",  attach = { 0, 0 },                    spr = { 0.99999917, 0.99998856, 0 },       z = 1 },
    -- back arm
    { tex = "experimentUpArm",    bone = "bArm",  attach = { 0, 5.44 },                 spr = { 0, 0.995, 0 },                        z = 2 },
    { tex = "experimentDownArm",  bone = "bLArm", attach = { 0, 5.44 },                 spr = { 0.03, 2.544, 0 },                     z = 2 },
    { tex = "experimentHandF",    bone = "handB", attach = { 0, 5.44 },                 spr = { 0.345, 2.544, 0 },                    z = 2 },
    -- back leg
    { tex = "experimentThigh",    bone = "legB",  attach = { 1.9999529, 4.0000234 },    spr = { 0.018392444, -0.019227982, -0.19698195 }, z = 3 },
    { tex = "experimentCrus",     bone = "crusB", attach = { -5.0000234, 1.9999411 },   spr = { 0.47213244, -0.41910458, 1.1059602 }, z = 3 },
    { tex = "experimentFoot",     bone = "footB", attach = { 1.6724386, 1.5125306 },    spr = { -0.44444656, 1.7090778, -0.7258136 }, z = 3 },
    -- torso
    { tex = "experimentDownTorso", bone = "hip",  attach = { 1.0002584, -1.9999998 },   spr = { 0, 0, 0 },                            z = 4 },
    { tex = "experimentUpTorso",  bone = "back",  attach = { 0.50035715, -4.0000777 },  spr = { 0.5, 0, 0 },                          z = 5 },
    -- front leg
    { tex = "experimentThigh",    bone = "legF",  attach = { 1.9999529, 4.0000234 },    spr = { 0.018392444, -0.019227982, -0.19698195 }, z = 6 },
    { tex = "experimentCrus",     bone = "crusF", attach = { -5.0000234, 1.9999411 },   spr = { 0.47213244, -0.41910458, 1.1059602 }, z = 6 },
    { tex = "experimentFoot",     bone = "footF", attach = { 1.622695, 1.678925 },      spr = { -0.44444656, 1.7090778, -0.7258136 }, z = 6 },
    -- head + eye
    { tex = "experimentHeadBack", bone = "head",  attach = { 0.583, -7.075 },           spr = { 0.42069682, 2.078674, 0 },            z = 7 },
    { tex = "experimentEyeOpen",  bone = "head",  attach = { 0.583, -7.075 },           spr = { 0.42069682, 2.078674, 0 },            z = 7.5 },
    -- front arm
    { tex = "experimentUpArm",    bone = "fArm",  attach = { 0, 5.44 },                 spr = { 0, 0.995, 0 },                        z = 9 },
    { tex = "experimentDownArm",  bone = "fLArm", attach = { 0, 5.44 },                 spr = { 0.03, 2.544, 0 },                     z = 9 },
    { tex = "experimentHandF",    bone = "handF", attach = { 0, 5.44 },                 spr = { 0.345, 2.544, 0 },                    z = 9 },
}

skeleton.PARTS = PARTS
skeleton.texture_dir = "/home/tsuchinoko/desksaw/assets/Body/"
skeleton.eye_texture = "experimentEyeOpen"
skeleton.head_texture = "experimentHeadBack"

-- ---------------------------------------------------------------------------
-- Texture loading + tight opaque bounds.
-- ---------------------------------------------------------------------------

local surfaces = {}

-- Pre-computed tight opaque bounds (alpha > 8) for the shipped PNGs, in
-- surface pixel coordinates {x0, y0, x1, y1}. Reading pixels at load needs
-- LuaJIT's FFI, which the system AwesomeWM build does NOT have (Lua 5.4), so
-- the old fallback used the full padded canvas. That made the runtime physics
-- box 88x63 units instead of the offline 31x48 -- so the pet collided with
-- invisible padding and its measured feet did not match the art. These bounds
-- are generated from the exact PNGs in assets/Body, so every runtime uses the
-- same geometry (the file is a fixed game asset).
local KNOWN_BBOX = {
    ["experimentCrus"]        = { 2, 4, 6, 12 },
    ["experimentDownArm"]     = { 4, 7, 8, 20 },
    ["experimentDownTorso"]   = { 4, 5, 11, 15 },
    ["experimentEyeClosed"]   = { 25, 15, 30, 17 },
    ["experimentEyeGone"]     = { 25, 14, 30, 19 },
    ["experimentEyeGoneHealed"] = { 25, 14, 31, 18 },
    ["experimentEyeHalfClosed"] = { 25, 14, 30, 17 },
    ["experimentEyeHalfClosedBack"] = { 25, 14, 30, 17 },
    ["experimentEyeHappy"]    = { 24, 11, 31, 18 },
    ["experimentEyeLookBack"] = { 26, 14, 30, 17 },
    ["experimentEyeOpen"]     = { 26, 14, 30, 17 },
    ["experimentEyePanic"]    = { 25, 13, 32, 20 },
    ["experimentEyeSad"]      = { 25, 12, 31, 19 },
    ["experimentEyeSadBack"]  = { 25, 12, 31, 19 },
    ["experimentEyeScared"]   = { 24, 12, 30, 18 },
    ["experimentEyeScaredBack"] = { 24, 12, 30, 18 },
    ["experimentFoot"]        = { 3, 6, 9, 17 },
    ["experimentHandB"]       = { 2, 2, 7, 6 },
    ["experimentHandF"]       = { 3, 2, 8, 6 },
    ["experimentHead"]        = { 14, 8, 34, 20 },
    ["experimentHeadBack"]    = { 14, 8, 34, 20 },
    ["experimentHeadBackMouth"] = { 14, 8, 34, 21 },
    ["experimentHeadBackMouthMini"] = { 14, 8, 34, 20 },
    ["experimentHeadDisfigured1"] = { 14, 8, 34, 20 },
    ["experimentHeadDisfigured1Healed"] = { 14, 8, 34, 20 },
    ["experimentHeadDisfigured2"] = { 14, 8, 33, 20 },
    ["experimentHeadDisfigured2Healed"] = { 14, 8, 33, 20 },
    ["experimentHeadDisfigured3"] = { 14, 8, 34, 20 },
    ["experimentHeadDisfigured3Healed"] = { 14, 8, 34, 20 },
    ["experimentNosebleed"]   = { 12, 7, 14, 11 },
    ["experimentTail"]        = { 20, 10, 42, 30 },
    ["experimentThigh"]       = { 3, 5, 9, 16 },
    ["experimentUpArm"]       = { 2, 3, 6, 14 },
    ["experimentUpTorso"]     = { 5, 3, 12, 13 },
}

local function opaque_bbox(surf)
    -- Tight bounds of the VISIBLE art inside a PNG canvas. The source PNGs are
    -- padded (experimentHeadBack is a 54x28 canvas holding a 20x12 head), and
    -- using the full canvas inflates the bounds so the physics box is much
    -- taller than the pet -- which is exactly what makes it hover or clip.
    --
    -- Reading pixels needs LuaJIT's FFI, which AwesomeWM does not expose as a
    -- requireable module (it is a built-in there). Without it we fall back to
    -- the canvas: the bounds are then conservative, never too small.
    local okffi, ffi = pcall(require, "ffi")
    if not okffi then return nil end
    local W, H = surf:get_width(), surf:get_height()
    local ok, data = pcall(function() return surf:get_data() end)
    if not ok or not data then return nil end
    local p = ffi.cast("uint32_t*", data)
    local stride = surf:get_stride()
    local minx, miny, maxx, maxy = W, H, -1, -1
    for y = 0, H - 1 do
        for x = 0, W - 1 do
            local a = p[y * (stride / 4) + x] % 256
            if a > 8 then
                if x < minx then minx = x end
                if x > maxx then maxx = x end
                if y < miny then miny = y end
                if y > maxy then maxy = y end
            end
        end
    end
    if maxx < 0 then return nil end
    return { x0 = minx, y0 = miny, x1 = maxx + 1, y1 = maxy + 1 }
end

local function load_surface(name)
    local hit = surfaces[name]
    if hit then return hit end
    local path = skeleton.texture_dir .. name .. ".png"
    local ok, surf = pcall(cairo.ImageSurface.create_from_png, path)
    if not ok or not surf then
        print("[desksaw] could not load body part: " .. tostring(path))
        return nil
    end
    local s = { surface = surf, w = surf:get_width(), h = surf:get_height() }
    local kb = KNOWN_BBOX[name]
    if kb then
        s.bbox = { x0 = kb[1], y0 = kb[2], x1 = kb[3], y1 = kb[4] }
    else
        s.bbox = opaque_bbox(surf)
    end
    surfaces[name] = s
    return s
end

function skeleton.load_surface(name) return load_surface(name) end

function skeleton.clear_textures() surfaces = {} end

-- ---------------------------------------------------------------------------
-- Placement geometry
--
-- The skeleton's origin is at the FEET (spec §3.2). The physics body is a box
-- centred on (body.cx, body.cy) whose half-height is `body.h/2`, and the body
-- box is built so that its BOTTOM is the feet. Therefore the skeleton origin
-- must be drawn at:
--
--     x = body.cx
--     y = body.cy + body.h/2      <- the feet, i.e. the bottom of the box
--
-- This is the whole fix for the floor clipping: drawing at body.cy would put
-- the feet at the MIDDLE of the box, so half the pet would be underground.
-- ---------------------------------------------------------------------------

-- Skeleton-space bounding box of the idle pose (x0, y0, x1, y1). y0 is the
-- top of the head, y1 is the soles of the feet.
local bounds

local function compute_bounds()
    local G = rig.build({ anim = anim, clip = "idleagain", phase = 0 })
    local minx, miny, maxx, maxy = math.huge, math.huge, -math.huge, -math.huge
    for _, part in ipairs(PARTS) do
        local s = load_surface(part.tex)
        if s then
            local M = rig.mul(G[part.bone].M,
                        rig.mul(rig.tr(part.attach[1], part.attach[2]),
                          rig.mul(rig.tr(part.spr[1], part.spr[2]),
                                  rig.rot(part.spr[3]))))
            -- The sprite's local position places the texture CENTRE on the
            -- bone (Godot Sprite2D, centered = true). So surface pixel (x, y)
            -- lands at bone-local (x - s.w/2, y - s.h/2), and the art quad is
            -- (bb.x0 - s.w/2, bb.y0 - s.h/2)..(bb.x1 - s.w/2, bb.y1 - s.h/2).
            local bb = s.bbox
            local ax0, ay0, ax1, ay1 = 0, 0, s.w, s.h
            if bb then
                ax0, ay0, ax1, ay1 = bb.x0, bb.y0, bb.x1, bb.y1
            end
            local hw, hh = s.w / 2, s.h / 2
            local corners = {
                { ax0 - hw, ay0 - hh }, { ax1 - hw, ay0 - hh },
                { ax1 - hw, ay1 - hh }, { ax0 - hw, ay1 - hh },
            }
            for _, c in ipairs(corners) do
                local x, y = rig.xform(M, c[1], c[2])
                if x < minx then minx = x end
                if x > maxx then maxx = x end
                if y < miny then miny = y end
                if y > maxy then maxy = y end
            end
        end
    end
    return minx, miny, maxx, maxy
end

function skeleton.bounds()
    if not bounds then bounds = { compute_bounds() } end
    return bounds[1], bounds[2], bounds[3], bounds[4]
end

-- Clip length helper (seconds) used by offline validators/preview.
function skeleton.clip_length(clip)
    local anim = require("desksaw.anim")
    local c = anim[clip]
    if not c or not c.length then return 0 end
    return c.length
end

-- Width / height of the pet in skeleton units.
function skeleton.extents()
    local x0, y0, x1, y1 = skeleton.bounds()
    return x1 - x0, y1 - y0, x0, y0
end

-- The skeleton-space y of the soles (the ground plane in this frame).
function skeleton.foot_y()
    local _, _, _, y1 = skeleton.bounds()
    return y1
end

-- ---------------------------------------------------------------------------
-- Ragdoll limb springs (spec §8.5: 16 bodies, pin joints, angular limits).
--
-- A full rigid-body ragdoll is out of scope for a wibox; this is the cheap
-- approximation the spec sanctions: each limb gets a damped angular spring
-- that goes limp when the body is flung, and the physics box (still a single
-- body) tumbles instead. The limbs splay outward, which reads as a crumple.
-- ---------------------------------------------------------------------------

-- Neutral (zero-delta) ragdoll pose = the IK-solved rest LOCAL rotations at
-- phase 0, computed once at load. Read it via pose.build().rot (which is the
-- local r each bone was assigned, including IK overrides for legF/crusF/etc).
local RAG_BONES = { "legF", "crusF", "legB", "crusB",
                    "fArm", "fLArm", "bArm", "bLArm", "head", "tail" }
local RAG_NEUTRAL
do
    local G = rig.build({ anim = anim, clip = "idleagain", phase = 0 })
    RAG_NEUTRAL = {}
    for _, b in ipairs(RAG_BONES) do
        RAG_NEUTRAL[b] = G[b].rot or 0
    end
end
skeleton.RAG_NEUTRAL = RAG_NEUTRAL

function skeleton.new_rag_state()
    local rs = {}
    for _, b in ipairs(RAG_BONES) do rs[b] = { a = 0, w = 0 } end
    return rs
end

function skeleton.step_ragdoll(rs, dt, acc_x, acc_y, speed)
    dt = math.min(dt, 1 / 30)
    -- Clamp input acceleration so sudden collision velocity reversals do not
    -- inject explosive impulses into the joints.
    acc_x = math.max(-4000, math.min(4000, acc_x or 0))
    acc_y = math.max(-4000, math.min(4000, acc_y or 0))

    for _, b in ipairs(RAG_BONES) do
        local s = rs[b]
        -- Limp target: limbs hang down (positive local rotation = down here).
        local target
        if b == "legF" or b == "legB" then target = 0.30
        elseif b == "crusF" or b == "crusB" then target = -0.45
        elseif b == "fArm" or b == "bArm" then target = -0.40
        elseif b == "fLArm" or b == "bLArm" then target = -0.70
        elseif b == "head" then target = 0.35
        else target = 0.20 end

        local k, c = 26, 5.5
        local drive = (-k * (s.a - target) - c * s.w) * dt
        -- Body motion drags the limbs around.
        drive = drive + (-acc_x * 0.00035 * math.sin(s.a + 1.1)
                         - acc_y * 0.00035 * math.cos(s.a + 0.4)) * dt * 60
        s.w = s.w + drive
        s.a = s.a + s.w * dt

        -- Anatomically tighter, per-bone angle limits so parent + child
        -- rotations cannot compound into detached limbs.
        local lo, hi
        if b == "head" then
            lo, hi = -0.45, 0.55
        elseif b == "legF" or b == "legB"
            or b == "fArm" or b == "bArm"
            or b == "tail" then
            lo, hi = -0.65, 0.65
        else
            -- lower limbs: crus*, fLArm, bLArm
            lo, hi = -0.85, 0.45
        end

        -- Preserve damped bounce at joint limits rather than hard-zeroing.
        if s.a > hi then
            s.a = hi
            s.w = math.min(0, s.w * -0.2)
        elseif s.a < lo then
            s.a = lo
            s.w = math.max(0, s.w * -0.2)
        end
        if speed < 40 then
            s.w = s.w * (1 - math.min(1, dt * 6))
            s.a = s.a * (1 - math.min(1, dt * 5))
        end
    end
end

-- ---------------------------------------------------------------------------
-- Draw
--
--   o.x, o.y    screen position of the SKELETON ORIGIN (the feet), in wibox px
--   o.angle     body rotation (rad)
--   o.scale     pixels per skeleton unit
--   o.facing    1 or -1 (mirrors about the vertical axis)
--   o.clip      animation name
--   o.phase     clip time (s)
--   o.lean      extra hip rotation (rad)
--   o.crouch    hip drop (skeleton units)
--   o.rag       ragdoll limb state
--   o.eye       eye texture name
-- ---------------------------------------------------------------------------

function skeleton.draw(cr, o)
    local scale = o.scale or 1
    local facing = o.facing or 1

    -- Placement: skeleton space -> screen.
    --   T(x, y) * S(scale*facing, scale)
    -- The skeleton stays upright at all times; facing is a horizontal mirror
    -- (scale.x = -1 flips), matching the Godot original where skelparent.scale.x
    -- handles facing and RigidBodies.lock_rotation = true keeps the body upright.
    -- phys.angle is NOT applied here -- the pet never tumbles visually.
    local M = rig.mul(rig.tr(o.x, o.y),
              rig.scl(scale * facing, scale))

    local G = rig.build({
        anim      = anim,
        clip      = o.clip or "idleagain",
        phase     = o.phase or 0,
        hip_extra = (o.crouch and o.crouch ~= 0) and { 0, o.crouch } or nil,
        -- Godot multiplies lean by mirror (sign of skelparent.scale.x
        -- determinant) so the lean direction flips when the pet faces left.
        hip_rot   = (o.lean or 0) * (o.facing or 1),
        rag       = o.rag,
        rag_neutral = o.rag_neutral or RAG_NEUTRAL,
        facing    = o.facing or 1,
    })

    local draw_list = {}
    for _, p in ipairs(PARTS) do draw_list[#draw_list + 1] = p end
    table.sort(draw_list, function(a, b) return a.z < b.z end)

    local eye = o.eye or skeleton.eye_texture
    local head = o.head or skeleton.head_texture

    for _, part in ipairs(draw_list) do
        local tex = part.tex
        if part.tex == "experimentEyeOpen" then tex = eye end
        if part.tex == "experimentHeadBack" then tex = head end
        local s = load_surface(tex)
        if s then
            -- Bone -> part transform (RemoteTransform2D attach, then the
            -- Sprite2D's own local offset/rotation).
            local Mpart = rig.mul(G[part.bone].M,
                          rig.mul(rig.tr(part.attach[1], part.attach[2]),
                            rig.mul(rig.tr(part.spr[1], part.spr[2]),
                                    rig.rot(part.spr[3]))))
            local W = rig.mul(M, Mpart)

            -- Godot's Sprite2D draws its texture CENTRED on the node's
            -- position (centered = true by default). cairo's
            -- set_source_surface(surf, 0, 0) puts the texture's TOP-LEFT at
            -- the current origin, so the texture must be shifted by half its
            -- canvas -- in the PART's local frame, before the bone rotation
            -- and scale are applied. Baking it into the matrix (rather than
            -- using cr:translate after cr:transform, which would translate in
            -- the already-scaled space) is what keeps parts joined: without
            -- it every part is displaced by half its canvas in its own
            -- rotated direction, which is exactly the "scattered limbs" look.
            local hw, hh = s.w / 2, s.h / 2
            local Wc = {
                W[1], W[2], W[3], W[4],
                W[5] - (W[1] * hw + W[3] * hh),
                W[6] - (W[2] * hw + W[4] * hh),
            }

            -- NOTE: cairo.Matrix must be built from a named-field table.
            -- LGI's cairo.Matrix(6 positional args) constructor segfaults.
            cr:save()
            cr:transform(cairo.Matrix {
                -- pose.lua matrices use the Godot layout
                --   x' = M1*x + M3*y + M5
                --   y' = M2*x + M4*y + M6
                -- cairo uses
                --   x' = xx*x + xy*y + x0
                --   y' = yx*x + yy*y + y0
                -- so the off-diagonal terms are swapped relative to ours.
                -- Passing them through positionally silently transposes
                -- every bone rotation, which splays rotated parts away from
                -- their joints (the "fragmented" look).
                xx = Wc[1], xy = Wc[3], yx = Wc[2], yy = Wc[4],
                x0 = Wc[5], y0 = Wc[6],
            })
            cr:set_source_surface(s.surface, 0, 0)
            local pat = cr:get_source()
            if pat then
                pat:set_extend(cairo.Extend.NONE)
                pat:set_filter(cairo.Filter.NEAREST)
            end
            -- paint() fills the whole SOURCE CANVAS (the PNGs are padded --
            -- the head is a 54x28 canvas holding a 20x12 head, the tail an
            -- 88x40 canvas holding a 22x20 tuft). Clip to the opaque art rect
            -- so only the figure is drawn. (rectangle()+fill() paints nothing
            -- in this LGI/cairo build; clip() works.)
            local bb = s.bbox
            if bb then
                cr:rectangle(bb.x0, bb.y0, bb.x1 - bb.x0, bb.y1 - bb.y0)
                cr:clip()
            end
            cr:paint()
            cr:restore()
        end
    end
end

-- ---------------------------------------------------------------------------
-- Wibox shape mask.
--
-- AwesomeWM's wibox shape callback paints into an A1 (1-bit) surface, so
-- anything not painted is clipped away. Drawing only the CURRENT pose (the
-- old code did exactly that) makes the mask a snapshot: once the walk cycle
-- swings a leg outside the idle silhouette, that leg is cut off until the
-- facing happens to change and the mask is rebuilt. That is the walking
-- "fragmentation".
--
-- Build ONE mask that is the union of every clip at several phases (plus the
-- lean/crouch extremes) and reuse it. The shape then never changes while
-- walking, and the visible sprite is always inside it.
-- ---------------------------------------------------------------------------

local MASK_CLIPS = { "idleagain", "moveF", "moveB", "jump", "fall", "sit", "laydown" }
local MASK_PHASES = 4

-- w, h: mask size in wibox pixels (the shape callback's width/height).
-- poses: optional list of { lean = rad, crouch = units } variants.
function skeleton.build_mask(w, h, scale, facing, poses)
    local surf = cairo.ImageSurface(cairo.Format.ARGB32, w, h)
    local cr = cairo.Context(surf)
    cr:set_operator(cairo.Operator.CLEAR)
    cr:paint()
    cr:set_operator(cairo.Operator.OVER)

    local variants = poses or { {} }
    for _, clip in ipairs(MASK_CLIPS) do
        local len = skeleton.clip_length(clip)
        for i = 0, MASK_PHASES - 1 do
            local phase = (len and len > 0) and (len * i / MASK_PHASES) or 0
            for _, v in ipairs(variants) do
                pcall(skeleton.draw, cr, {
                    x = w / 2, y = h / 2,
                    scale = scale, facing = facing,
                    clip = clip, phase = phase,
                    lean = v.lean or 0,
                    crouch = v.crouch or 0,
                })
            end
        end
    end
    return surf
end

return skeleton
