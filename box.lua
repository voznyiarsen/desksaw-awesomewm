-- box.lua
-- desksaw.box
-- Renders the pet as a floating AwesomeWM wibox and drives it with the
-- physics stepper. Adds DeskSaw's character logic on top of simplephys:
--   * wander AI (idle/moving/jumping/falling states)
--   * ragdoll when moving too fast (goes limp, then recovers)
--   * lean from acceleration
--   * grab + pendulum throw
--   * paplay impact sounds on hard bounces

local gears = require("gears")
local wibox = require("wibox")
local awful = require("awful")
local GLib = require("lgi").GLib
local cairo = require("lgi").cairo

local physics = require("desksaw.physics")
local skeleton = require("desksaw.skeleton")

local box = {}
box.__index = box
box.now = function() return GLib.get_monotonic_time() / 1e6 end

-- Body states.
local STATE_IDLE = "idle"
local STATE_MOVING = "moving"
local STATE_JUMPING = "jumping"
local STATE_FALLING = "falling"
local STATE_RAGDOLL = "ragdoll"

local function sign(v)
    return v > 0 and 1 or (v < 0 and -1 or 0)
end

local function default_sound_path()
    -- Default next to this file; the module config in init.lua overrides it.
    return (debug.getinfo(1, "S").source:match("@(.*/)") or "")
        .. "assets/sounds/effects/thudswoosh.ogg"
end

function box:play_impact(speed)
    if not self.config.sound_enabled then return end
    if not speed or speed < (self.config.sound_min_speed or 100) then return end
    local now = box.now()
    if now < (self._sound_next or 0) then return end
    self._sound_next = now + (self.config.sound_cooldown or 0.15)
    local path = self.config.sound_path or default_sound_path()
    awful.spawn(string.format("paplay %q", path))
end

function box.new(screen, config)
    local self = setmetatable({}, box)
    self.config = config
    self.screen = screen

    -- The pet is drawn from the articulated skeleton (bones + IK + clips).
    --
    -- IMPORTANT: the skeleton's origin is at the pet's FEET (spec §3.2 --
    -- the capsule bottom is exactly y=0, hip at y=-20, head at y=-35). So the
    -- physics body is anchored at the feet, not at the figure's centre.
    --
    --   phys.cx, phys.cy  = the point BETWEEN THE FEET
    --   box extends UPWARD from there: y in [cy - h, cy]
    --
    -- Drawing at (cy) would put the feet in the middle of the box and half the
    -- pet underground -- that was the floor clipping.
    self.scale = config.sprite_scale or 4
    local bw, bh = skeleton.extents()
    self.w = bw * self.scale
    self.h = bh * self.scale
    self.hw = self.w / 2
    self.hh = self.h / 2
    self.size = math.max(self.w, self.h)

    -- Animation state.
    self.clip = "idleagain"
    self.phase = 0
    self.rag = skeleton.new_rag_state()
    self.rag_prev_vx = 0
    self.rag_prev_vy = 0

    -- The wibox must be big enough for the rest pose at any angle plus loose
    -- ragdoll limbs sticking out.
    local vis = math.ceil(self.size * math.sqrt(2) + 2 * self.size * 0.45)
                        + 2 * (config.border_width or 0)

    self.wibox = wibox {
        screen = screen,
        ontop = true,
        visible = true,
        x = 0, y = 0,
        width = vis,
        height = vis,
        border_width = config.border_width or 0,
        border_color = "#00000000",
        cursor = "arrow",
        type = "dock",
    }
    -- The drawable sets bg = beautiful.bg_normal by default; override with
    -- fully transparent so the area around the pet stays see-through.
    self.wibox:set_bg(gears.color.transparent)

    -- The wibox draw callbacks run inside AwesomeWM's C draw path. An error
    -- thrown there is NOT catchable by Lua and takes the whole WM down (this
    -- already happened once during development), so every callback is guarded:
    -- on failure we drop the shape/bgimage and leave a plain transparent
    -- wibox instead of crashing the session.
    local function draw_pet(cr, w, h)
        local ok, err = pcall(function()
            -- The wibox is drawn with its top-left at (screen x, y) and the
            -- body's FEET at (phys.cx, phys.cy). The wibox centre therefore
            -- sits at the FEET position, and the skeleton origin (also the
            -- feet) goes at the wibox's BOTTOM-CENTRE... except the wibox is
            -- a square of side `vis` centred on the feet, so the skeleton
            -- origin lands at (w/2, h/2) in wibox coordinates when the feet
            -- are at the wibox centre. That is what we pass.
            --
            -- The key point: (o.x, o.y) is the FEET, and the renderer maps the
            -- skeleton origin (0,0) there. It is NOT the figure's centre.
            skeleton.draw(cr, {
                x = w / 2, y = h / 2,
                angle = self.phys and self.phys.angle or 0,
                scale = self.scale,
                facing = self.facing or 1,
                clip = self.clip,
                phase = self.phase,
                lean = math.rad(self.lean or 0),
                crouch = (self.crouch or 0) / self.scale,
                rag = (self.state == STATE_RAGDOLL) and self.rag or nil,
                eye = self.eye_texture,
                head = self.head_texture,
            })
        end)
        if not ok then
            if not self._draw_warned then
                self._draw_warned = true
                print("[desksaw] draw error, disabling pet rendering: " .. tostring(err))
            end
            return false
        end
        return true
    end
    self._draw_pet = draw_pet

    local function shape_fn(cr, w, h)
        -- Paint the cached union silhouette. It covers every clip/phase, so
        -- the mask never clips a walking or jumping limb.
        local mask = self:_shape_mask(w, h)
        if mask then
            cr:set_source_surface(mask, 0, 0)
            cr:paint()
        end
        -- Ragdoll limbs are driven by springs driven by collisions, so they
        -- can reach past the clip union; add the live pose.
        if self.state == STATE_RAGDOLL then
            draw_pet(cr, w, h)
        end
    end
    self._shape_fn = shape_fn
    self.wibox:set_shape(shape_fn)

    -- Store the bgimage callback so it can be re-set to force a redraw when
    -- the face texture changes (blink, lookback, talking). Without this, the
    -- wibox only redraws when the pet moves, so a standing pet's eyes would
    -- never update. `set_bgimage` emits widget::redraw_needed which forces a
    -- repaint even when the geometry is unchanged.
    self._bgimage_func = function(context, cr, w, h, _, self_ref)
        -- Reset the clip to the full surface. The drawable clips to the dirty
        -- area before calling this callback, so cr:paint() would otherwise
        -- only cover the dirty region. CLEAR operator zeroes every pixel,
        -- making the whole wibox surface transparent before we draw the pet.
        cr:reset_clip()
        cr:set_operator(cairo.Operator.CLEAR)
        cr:paint()
        cr:set_operator(cairo.Operator.OVER)
        cr:save()
        self_ref._draw_pet(cr, w, h)
        cr:restore()
    end
    self.wibox:set_bgimage(self._bgimage_func, nil, self)
    -- No widget hierarchy: nothing should be drawn after the bgimage callback.
    -- Setting fg to transparent prevents any leftover widget drawing.
    self.wibox:set_widget(nil)
    self.wibox:set_bg("#00000000")
    self.wibox:set_fg("#00000000")

    -- Build both facing masks now, so the first direction flip does not hitch.
    self:warm_shape_masks(vis, vis)

    -- Physics body. `physics.new_box(cx, cy, w, h)` creates a box centred on
    -- (cx, cy). We want the box's BOTTOM at the feet, so place the centre one
    -- half-height above the feet position.
    local a = screen.workarea
    local feet_x = a.x + a.width / 2
    local feet_y = a.y + a.height * 0.6     -- drop in from mid-air
    self.phys = physics.new_box(feet_x, feet_y - self.hh, self.w, self.h)
    self.phys.vx = 0
    self.phys.vy = 0
    self.phys.angle = 0
    self.phys.omega = 0
    -- Matches the Godot original's RigidBodies.lock_rotation = true: the body
    -- must never tip onto a corner. A corner contact lifts the centre (the
    -- pet floats) and can wedge it in place (it stops walking).
    self.phys.lock_rotation = true
    -- Character state.
    self.state = STATE_IDLE
    self.dir = 0          -- -1 left, 0 none, 1 right
    self.facing = 1       -- 1 right, -1 left
    self.lean = 0         -- degrees
    self.crouch = 0       -- px
    self.visual_offset_x = 0
    self.visual_offset_y = 0

    -- Face system (ported from Scripts/impo/faceSys.gd).
    -- Emotion-driven eye textures, blinking, talking (mouth via head
    -- texture), and lookback (eye looks toward the cursor when it sits on
    -- the opposite side from the pet's facing).
    local EYES = {
        normal  = { open = "experimentEyeOpen",   closed = "experimentEyeClosed",   back = "experimentEyeLookBack" },
        sad     = { open = "experimentEyeSad",    closed = "experimentEyeClosed",   back = "experimentEyeSadBack" },
        sleep   = { open = "experimentEyeClosed", closed = "experimentEyeClosed",   back = "experimentEyeClosed" },
        tired   = { open = "experimentEyeHalfClosed", closed = "experimentEyeClosed", back = "experimentEyeHalfClosedBack" },
        scared  = { open = "experimentEyeScared", closed = "experimentEyeClosed",   back = "experimentEyeScaredBack" },
        panic   = { open = "experimentEyePanic",  closed = "experimentEyePanic",    back = "experimentEyePanic" },
        happy   = { open = "experimentEyeHappy",  closed = "experimentEyeHappy",    back = "experimentEyeHappy" },
    }
    local HEADS = {
        default = "experimentHeadBack",
        open    = "experimentHeadBackMouth",
        opensmall = "experimentHeadBackMouthMini",
    }
    self.face_emotion = "normal"
    self.face_talking = false
    self.face_blinking = false
    self.face_lookback = false
    self.face_blink_timer = 0          -- counts up to 2-4s between blinks
    self.face_blink_for = 0            -- 0.15s blink duration
    self.face_talk_timer = 0           -- cycles mouth while talking
    self.face_eye_timer = 0            -- 0.2s eye-direction updates
    self.face_eyes = EYES
    self.face_heads = HEADS
    self.eye_texture = EYES.normal.open
    self.head_texture = HEADS.default

    self.jump_timer = 0
    self.since_grounded = 0
    self.grounded = true
    self.ragdoll_timer = 0
    self.wander_timer = 0
    -- Wander pattern (DeskSaw behavior.gd wandering()): idle 4-8s, then walk
    -- 0.5-2s, then idle again. The pet spends most of its time standing still.
    self.wander_phase = "idle"
    self.wander_idle_for = 4 + math.random() * 4
    self.wander_walk_for = 0
    -- Wall-stall: when the pet tries to walk into a wall (dir set, grounded,
    -- but vx ≈ 0), switch to idle and remember to pick the opposite direction
    -- for the next wander.
    self.wall_stall_timer = 0
    self.next_wander_dir = nil
    self._last_vx = 0
    self._last_vy = 0
    self.last_tick = box.now()

    self.grabbing = false
    self.grab_side = nil
    self.grab_lx, self.grab_ly = 0, 0
    self.grab_mx, self.grab_my = 0, 0
    self.pend_w = 0
    self.pivot_samples = {}
    self._sound_next = 0
    self._last_angle = nil
    self.spawned = true

    -- Click to grab.
    self.wibox:buttons(gears.table.join(
        awful.button({}, 1, function() self:grab() end)
    ))

    return self
end

function box:workarea() return self.screen.workarea end

-- Cached wibox-shape masks, one per (size, facing). See skeleton.build_mask.
function box:_shape_mask(w, h, facing)
    self._shape_masks = self._shape_masks or {}
    facing = facing or self.facing or 1
    local key = w .. "x" .. h .. "x" .. facing
    local m = self._shape_masks[key]
    if not m then
        local lean = math.rad(self.config.lean_max or 25)
        local crouch = (self.config.land_crouch_max or 55) / self.scale
        m = skeleton.build_mask(w, h, self.scale, facing, {
            {}, { lean = lean }, { lean = -lean }, { crouch = crouch },
        })
        self._shape_masks[key] = m
    end
    return m
end

-- Build both facings up front. Building lazily on the first direction flip
-- would stall that frame (~0.2 s for the union mask).
function box:warm_shape_masks(w, h)
    self:_shape_mask(w, h, 1)
    self:_shape_mask(w, h, -1)
end

function box:respawn()
    if not self.phys then return end
    local a = self:workarea()
    self.phys.cx = a.x + a.width / 2
    self.phys.cy = a.y + a.height - self.hh   -- box bottom (feet) on the floor
    self.phys.vx, self.phys.vy = 0, 0
    self.phys.angle, self.phys.omega = 0, 0
    self.state = STATE_IDLE
    self.dir = 0
    self.lean = 0
    self.crouch = 0
    self:set_geometry()
end

function box:set_visible(visible)
    if self.wibox then self.wibox.visible = visible end
    if not visible and self.grabbing then
        self.grabbing = false
        self.grab_side = nil
        self.phys.vx, self.phys.vy, self.phys.omega = 0, 0, 0
    end
end

function box:destroy()
    if self.wibox then
        pcall(function() self.wibox:set_bgimage(nil) end)
        pcall(function() self.wibox:set_shape(nil) end)
        pcall(function() self.wibox:buttons({}) end)
        self.wibox.visible = false
        self.wibox = nil
    end
    self.spawned = false
end

-- ---------------------------------------------------------------------------
-- Obstacles: screen edges + visible floating windows.
-- ---------------------------------------------------------------------------

function box:obstacles()
    local a = self:workarea()
    local w = (self.config and self.config.wall_thickness) or 8
    local list = {}

    local function emit(x, y, ow, oh)
        list[#list + 1] = { x = x, y = y, w = ow, h = oh, vx = 0, vy = 0 }
    end

    emit(a.x - w, a.y, w, a.height)           -- left
    emit(a.x + a.width, a.y, w, a.height)      -- right
    emit(a.x, a.y - w, a.width, w)             -- ceiling
    emit(a.x, a.y + a.height, a.width, w)      -- floor

    local clients = client.get(self.screen, true)
    for _, c in ipairs(clients) do
        if c:isvisible() and not c.minimized and not c.hidden
           and c.floating and not c.fullscreen and not c.maximized then
            local g = c:geometry()
            if g then
                list[#list + 1] = {
                    x = g.x, y = g.y, w = g.width, h = g.height,
                    vx = 0, vy = 0,
                }
            end
        end
    end

    return list
end

-- ---------------------------------------------------------------------------
-- Ragdoll / character movement (DeskSaw port).
-- ---------------------------------------------------------------------------

function box:enter_ragdoll()
    if self.state == STATE_RAGDOLL then return end
    self.state = STATE_RAGDOLL
    self.dir = 0
    self.lean = 0
    self.crouch = 0
    self.visual_offset_x = 0
    self.visual_offset_y = 0
    -- Go limp: kill spin, let physics drag take over.
    self.phys.omega = 0
    -- Seed the ragdoll acceleration baseline so the first tick does not
    -- compute a massive false spike from 0 to the trigger speed.
    self.rag_prev_vx = self.phys.vx
    self.rag_prev_vy = self.phys.vy
    self._last_rag_a = nil
end

function box:recover_from_ragdoll()
    if self.state ~= STATE_RAGDOLL then return end
    self.state = STATE_IDLE
    self.dir = 0
    self.lean = 0
    self.crouch = 0
    self.visual_offset_x = 0
    self.visual_offset_y = 0
    self.phys.vx, self.phys.vy = 0, 0
    self.phys.angle, self.phys.omega = 0, 0
    -- Reset ragdoll state so stale angles/velocities do not carry over into
    -- the next ragdoll trigger.
    self.rag = skeleton.new_rag_state()
    self.rag_prev_vx, self.rag_prev_vy = 0, 0
    self.ragdoll_timer = 0
    -- Reset the shape-sync tracking so the mask snaps back to idleagain.
    self._last_rag_a = nil
    self._last_angle = nil
    self.wibox:set_shape(self._shape_fn)
    self:set_geometry()
end

-- ---------------------------------------------------------------------------
-- Debug / test helpers: force the pet into a specific state so each pose can
-- be inspected. Bound to a right-click menu from init.lua.
-- ---------------------------------------------------------------------------

--- Force the pet into a given state for testing.
-- @param state one of "idle", "moving", "jumping", "falling", "ragdoll"
-- @param dir   -1 left, 0 none, 1 right (only used for moving/idle-facing)
function box:set_debug_state(state, dir)
    if state == STATE_RAGDOLL then
        self:enter_ragdoll()
        return
    end
    if self.state == STATE_RAGDOLL then
        self:recover_from_ragdoll()
    end
    self.dir = dir or 0
    if dir and dir ~= 0 then
        self.facing = dir
    end
    self.state = state
    -- Reset animation phase so the clip starts fresh.
    self.phase = 0
    self.clip = state
    self.wander_timer = 0
    self.wander_phase = state == STATE_MOVING and "walk" or "idle"
end

--- Set the walking direction (and facing) for testing.
function box:set_dir(dir)
    self.dir = dir
    if dir ~= 0 then
        self.facing = dir
        self.state = STATE_MOVING
    else
        self.state = STATE_IDLE
    end
    self.phase = 0
end

-- Pick a wander direction + optionally jump.
function box:pick_wander()
    -- If the pet was blocked by a wall, pick the opposite direction so it
    -- walks away from the wall rather than back into it.
    local d = self.next_wander_dir
    if d then
        self.next_wander_dir = nil
    else
        d = math.random(2) == 1 and 1 or -1
    end
    self.dir = d
    self.facing = d
    self.wander_phase = "walk"
    self.wander_timer = 0
    self.wander_walk_for = 0.5 + math.random() * 1.5  -- 0.5-2s walk
end

-- ---------------------------------------------------------------------------
-- Physics step.
-- ---------------------------------------------------------------------------

function box:step(dt)
    dt = math.min(dt, 0.1)

    if self.grabbing then
        self:grab_step(dt)
        return
    end
    if not self.phys then return end

    local face_dirty = false

    -- Ragdoll: drive the limp limbs from the body's motion, then recover once
    -- the body has settled (or after a hard timeout).
    if self.state == STATE_RAGDOLL then
        self.ragdoll_timer = self.ragdoll_timer + dt
        local ax = (self.phys.vx - (self.rag_prev_vx or 0)) / math.max(dt, 1e-4)
        local ay = (self.phys.vy - (self.rag_prev_vy or 0)) / math.max(dt, 1e-4)
        local speed = math.sqrt(self.phys.vx ^ 2 + self.phys.vy ^ 2)
        skeleton.step_ragdoll(self.rag, dt, ax, ay, speed)
        self.rag_prev_vx, self.rag_prev_vy = self.phys.vx, self.phys.vy
        local spd = math.abs(self.phys.vx) + math.abs(self.phys.vy)
        -- Recovery on a window/corner: the solver zeroes vy on contact even
        -- when the floor-touch test (screen floor) is false, so accept a
        -- near-zero vy as "supported" too. This prevents hanging in ragdoll
        -- on top of windows.
        local supported = self.grounded or math.abs(self.phys.vy) < 6
        if (spd < 25 and supported and self.ragdoll_timer > 0.7)
           or self.ragdoll_timer > 3.5 then
            self:recover_from_ragdoll()
        end
    end

    -- Character logic (suppressed while ragdolled: the body is limp and the
    -- animation is frozen). Note the explicit early-out -- without it the
    -- motion-state block below would overwrite STATE_RAGDOLL.
    local obstacles = self:obstacles()
    if self.state ~= STATE_RAGDOLL then
        self.jump_timer = math.max(0, self.jump_timer - dt)

        -- Periodically pick a new wander direction.
        -- Pattern (DeskSaw behavior.gd wandering()): idle 4-8s → walk 0.5-2s → idle.
        -- The pet spends most of its time standing still.
        self.wander_timer = (self.wander_timer or 0) + dt
        if self.wander_phase == "idle" then
            if self.wander_timer >= self.wander_idle_for then
                self:pick_wander()
            end
        else -- "walk"
            if self.wander_timer >= self.wander_walk_for then
                self.dir = 0
                self.wander_phase = "idle"
                self.wander_timer = 0
                self.wander_idle_for = 4 + math.random() * 4  -- 4-8s idle
            end
        end

        -- Wall-stall: grounded, trying to walk (dir ~= 0), but not actually
        -- moving (vx ≈ 0) means the pet walked into a wall. Stand still and
        -- walk away from the wall on the next wander.
        if self.grounded and self.dir ~= 0 then
            if math.abs(self.phys.vx) < 30 then
                self.wall_stall_timer = (self.wall_stall_timer or 0) + dt
                if self.wall_stall_timer > 0.3 then
                    local blocked_dir = self.dir
                    self.dir = 0
                    self.next_wander_dir = -blocked_dir
                    self.wander_phase = "idle"
                    self.wander_timer = 0
                    self.wander_idle_for = 4 + math.random() * 4
                end
            else
                self.wall_stall_timer = 0
            end
        else
            self.wall_stall_timer = 0
        end

        -- Decide state from motion (DeskSaw movement_handler.gd): a grounded
        -- pet is moving whenever `dir` is non-zero. Do NOT compare the speed
        -- against maxspeed here -- the walk limit is maxspeed * walk_mult
        -- (384), so the old check flipped to idle at 240, braked, re-accelerated
        -- and stuttered.
        local speed = math.abs(self.phys.vx)
        if self.grounded then
            if self.dir ~= 0 then
                self.state = STATE_MOVING
            else
                self.state = STATE_IDLE
            end
        else
            if self.phys.vy < 0 then self.state = STATE_JUMPING
            else self.state = STATE_FALLING end
        end

        -- Animation clip + playback phase (DeskSaw clips from the scene).
        local clip
        if self.state == STATE_MOVING then clip = "moveF"
        elseif self.state == STATE_JUMPING then clip = "jump"
        elseif self.state == STATE_FALLING then clip = "fall"
        else clip = "idleagain" end
        if clip == "moveF" then
            local norm = math.min(1, math.abs(self.phys.vx) / math.max(1, self.config.maxspeed))
            -- Animation playback speed scales with velocity (DeskSaw
            -- movement_handler.gd: speed_scale = (norm/2 + 0.1) * ANIM_MULT).
            -- When the pet stops (e.g. against a wall) the cycle nearly
            -- stops, so it never freezes mid-stride. At full walk speed the
            -- 0.58s clip loops about once per second.
            local speed_scale = (norm * 0.5 + 0.1) * 0.945
            self.phase = self.phase + dt * speed_scale
        elseif clip == "jump" or clip == "fall" then
            self.phase = self.phase + dt
        else
            self.phase = 0
        end
        self.clip = clip

        -- Face system (ported from Scripts/impo/faceSys.gd).
        -- Blink: every 2-4s, close eyes for 0.15s (skipped while talking).
        face_dirty = false
        local old_eye, old_head = self.eye_texture, self.head_texture

        self.face_blink_timer = self.face_blink_timer + dt
        if not self.face_blinking and not self.face_talking then
            if self.face_blink_timer >= (self.face_blink_for > 0 and 0.15 or (2 + math.random() * 2)) then
                if self.face_blink_for == 0 then
                    self.face_blink_for = 2 + math.random() * 2
                end
            end
            if self.face_blink_timer >= self.face_blink_for then
                self.face_blinking = true
                self.face_blink_timer = 0
                self.face_blink_for = 0.15  -- blink duration
            end
        elseif self.face_blinking then
            self.face_blink_timer = self.face_blink_timer + dt
            if self.face_blink_timer >= 0.15 then
                self.face_blinking = false
                self.face_blink_timer = 0
                self.face_blink_for = 2 + math.random() * 2  -- next interval
            end
        end

        -- Lookback: cursor on the opposite side from facing -> "back" eye.
        -- Runs every frame (idle too) so the pet reacts to the mouse even
        -- while standing still.
        local mx, my = mouse.coords().x, mouse.coords().y
        local px = self.phys.cx
        local lookback = (mx < px and self.facing == 1) or (mx > px and self.facing == -1)
        self.face_lookback = lookback

        -- Eye direction update: every 0.2s, pick open/back eye (unless blinking).
        self.face_eye_timer = self.face_eye_timer + dt
        if self.face_eye_timer >= 0.2 then
            self.face_eye_timer = 0
            if not self.face_blinking then
                local e = self.face_eyes[self.face_emotion]
                self.eye_texture = lookback and e.back or e.open
            end
        end

        -- Talking: cycle head texture (default -> opensmall -> default).
        if self.face_talking then
            self.face_talk_timer = self.face_talk_timer + dt
            local t = self.face_talk_timer
            local cycle = 0.08 + math.random() * 0.10  -- mouth open/close cadence
            local phase = t % (cycle * 2)
            if phase < cycle then
                self.head_texture = self.face_heads.opensmall
            else
                self.head_texture = self.face_heads.default
            end
        else
            self.head_texture = self.face_heads.default
        end

        -- Force a wibox redraw when the face texture changed: when the pet is
        -- standing still nothing else moves, so the wibox would otherwise not
        -- redraw and the new eye/head texture would never appear on screen.
        if self.eye_texture ~= old_eye or self.head_texture ~= old_head then
            face_dirty = true
        end

        -- Lean from acceleration (DeskSaw live_body).
        local dv = self.phys.vx - self._last_vx
        local target = math.max(-self.config.lean_max,
                                math.min(self.config.lean_max,
                                         dv * self.config.lean_per_dv))
        self.lean = self.lean + (target - self.lean) * math.min(1, dt * self.config.lean_rate)

        -- Crouch on landing is applied in the ground-detection block below
        -- (which has the transition data _last_vy at the impact frame). Do not
        -- also accumulate it here while airborne -- both blocks would read
        -- the same _last_vy and double the crouch.
        self.crouch = self.crouch * (1 - math.min(1, dt * self.config.crouch_rate))

        -- Visual offset (picture lags behind body).
        self.visual_offset_x = self.visual_offset_x * (1 - math.min(1, dt * self.config.offset_rate))
        self.visual_offset_y = self.visual_offset_y * (1 - math.min(1, dt * self.config.offset_rate))

        -- Apply movement force.
        if self.state == STATE_MOVING then
            local v = self.phys.vx
            local limit = self.config.maxspeed * self.config.walk_mult
            local tempmax = limit
            if self.dir == -1 then tempmax = (self.config.maxspeed / 2 + 40) * self.config.walk_mult end
            local same_dir = (sign(self.phys.vx) == self.dir) or self.phys.vx == 0
            if same_dir or math.abs(v) < tempmax then
                v = v + self.dir * (self.config.accel) * dt
                if sign(v) == sign(self.dir) and math.abs(v) > tempmax then
                    v = self.dir * tempmax
                end
            end
            self.phys.vx = v
        elseif self.state == STATE_IDLE then
            self.phys.vx = self.phys.vx * (1 - self.config.slowdown)
        end

        -- Jump to climb a wall/ledge, like movement_handler.gd's checkJump():
        -- only when something blocks the path ahead AND there's headroom above
        -- (like Godot's heightDetect raycast). Random hopping made the pet
        -- spend most of its time airborne (which reads as "floating").
        if self.grounded and self.jump_timer <= 0 and self.dir ~= 0 then
            local a = self:workarea()
            local probe = self.phys.cx + self.dir * (self.hw + 2)
            local mid_top = self.phys.cy - self.hh * 0.5
            local mid_bot = self.phys.cy + self.hh * 0.5
            local blocked = false
            for _, ob in ipairs(obstacles) do
                -- Ignore the four screen-boundary walls: the pet should turn
                -- around at the screen edge, not bounce off it.
                if ob.x >= a.x - 1 and ob.x + ob.w <= a.x + a.width + 1
                   and ob.y >= a.y - 1 and ob.y + ob.h <= a.y + a.height + 1 then
                    if probe > ob.x and probe < ob.x + ob.w
                       and mid_bot > ob.y and mid_top < ob.y + ob.h then
                        blocked = true
                        break
                    end
                end
            end
            -- Headroom check (Godot heightDetect): only jump if there's clear
            -- space above the pet's head, otherwise it would jump into a
            -- ceiling or overhang.
            local headroom = true
            local head_top = self.phys.cy - self.hh
            for _, ob in ipairs(obstacles) do
                if ob.x >= a.x - 1 and ob.x + ob.w <= a.x + a.width + 1
                   and ob.y >= a.y - 1 and ob.y + ob.h <= a.y + a.height + 1 then
                    if self.phys.cx > ob.x and self.phys.cx < ob.x + ob.w
                       and ob.y + ob.h > head_top - 40 and ob.y < head_top then
                        headroom = false
                        break
                    end
                end
            end
            if blocked and headroom then
                self.phys.vy = (self.config.jump_power or -600)
                              * (self.config.jump_mult or 1.22)
                self.jump_timer = self.config.jump_cooldown
                self.since_grounded = self.config.coyote_time + 1
                self:play_impact(math.abs(self.phys.vy))
            end
        end
    end

    -- Ragdoll trigger. DeskSaw triggers this on HORIZONTAL speed (being flung
    -- sideways / dragged), not on vertical falls -- otherwise every jump or
    -- drop from a window would immediately go limp.
    if self.state ~= STATE_RAGDOLL then
        local spd = math.abs(self.phys.vx)
        if spd > self.config.ragdoll_speed then
            self:enter_ragdoll()
            self.ragdoll_timer = 0
        end
    end

    -- Integrate physics against obstacles.
    physics.step(self.phys, dt, obstacles, self.config)

    -- Ground detection. `phys.cy` is the box CENTRE (physics.new_box was
    -- placed one half-height above the feet), so the FEET -- the box's bottom
    -- edge -- are at `phys.cy + hh`. The physics solver rests bodies a few
    -- pixels above a surface (contact slop), so use a small tolerance.
    local a = self:workarea()
    local feet_y = self.phys.cy + self.hh
    local GROUND_EPS = 6
    local floor_touch = feet_y >= a.y + a.height - GROUND_EPS
    if floor_touch and self.phys.vy >= -2 then
        if not self.grounded then
            self.grounded = true
            self.since_grounded = 0
            -- landing crouch (not while ragdolled: there is no pose to absorb)
            if self.state ~= STATE_RAGDOLL
               and math.abs(self._last_vy) > self.config.land_min then
                self.crouch = math.min(self.crouch + math.abs(self._last_vy) * self.config.land_crouch,
                                       self.config.land_crouch_max)
            end
        end
    elseif math.abs(self.phys.vy) < 1 then
        -- Physics solver zeroed vertical velocity: body is resting on a
        -- surface (screen floor or floating window). Treat as grounded so the
        -- pet can keep walking. The jump apex briefly crosses vy == 0, but a
        -- single frame of grounded there is harmless -- gravity pulls the pet
        -- down again on the next sub-step.
        if not self.grounded then
            self.grounded = true
            self.since_grounded = 0
        end
    else
        self.grounded = false
        self.since_grounded = self.since_grounded + dt
    end

    -- A body resting on top of a floating window (not the screen floor) is
    -- supported by that window: treat it as grounded if the physics solver has
    -- zeroed its fall. Belt-and-suspenders fallback for the elseif above.
    if not self.grounded and self.state == STATE_RAGDOLL
       and math.abs(self.phys.vy) < 6 then
        self.grounded = true
        self.since_grounded = 0
    end

    -- Off-screen recovery. The box is centred on phys.cy, so its bottom (the
    -- feet) is at phys.cy + hh and its top at phys.cy - hh.
    local m = self.hw + 8
    local cx, cy = self.phys.cx, self.phys.cy
    local feet_y = cy + self.hh
    if cx < a.x - m or cx > a.x + a.width + m
       or feet_y < a.y - m or (cy - self.hh) > a.y + a.height + m then
        self.phys.cx = math.max(a.x + self.hw, math.min(cx, a.x + a.width - self.hw))
        self.phys.cy = math.max(a.y + self.h, math.min(cy, a.y + a.height - self.hh))
        self.phys.vx, self.phys.vy, self.phys.omega = 0, 0, 0
        self:recover_from_ragdoll()
    end

    self._last_vx = self.phys.vx
    self._last_vy = self.phys.vy

    -- Redraw when the face changed (blink, lookback, talking). Without this,
    -- the wibox only redraws when the pet moves, so a standing pet's eyes
    -- would never update. Re-setting the bgimage emits widget::redraw_needed
    -- which forces a repaint even when the geometry is unchanged.
    if face_dirty then
        self.wibox:set_bgimage(self._bgimage_func, nil, self)
    end

    self:set_geometry()
end

function box:set_geometry()
    if not self.wibox or not self.phys then return end

    -- `self.wibox` is a square of side `vis`. The skeleton origin (the FEET)
    -- must land at the box's BOTTOM edge, i.e. at
    --     screen (phys.cx, phys.cy + hh)
    -- and the wibox's own top-left is that minus half its side.
    local vis = self.wibox.width
    local half_vis = vis / 2
    local feet_x = self.phys.cx
    local feet_y = self.phys.cy + self.hh      -- box bottom = feet

    local x = math.floor(feet_x - half_vis + 0.5)
    local y = math.floor(feet_y - half_vis + 0.5)

    local moved = x ~= (self._gx or math.huge) or y ~= (self._gy or math.huge)
    if moved then
        self._gx, self._gy = x, y
        self.wibox:geometry { x = x, y = y }
    end

    local update_shape = (self.facing or 1) ~= (self._last_facing or 1)

    -- While ragdolled the bgimage animates limb angles every frame but phys.angle
    -- may stay constant, which would freeze the wibox shape mask on the first
    -- ragdoll frame. Rebuild the shape when the ragdoll limbs are still moving
    -- (angular velocity above epsilon or angle delta above threshold).
    if self.state == STATE_RAGDOLL and self.rag then
        local maxdw, maxda = 0, 0
        self._last_rag_a = self._last_rag_a or {}
        for b, s in pairs(self.rag) do
            if math.abs(s.w or 0) > maxdw then maxdw = math.abs(s.w) end
            local la = self._last_rag_a[b]
            local da = (la == nil) and 1 or math.abs((s.a or 0) - la)
            if da > maxda then maxda = da end
        end
        if maxdw > 0.02 or maxda > 0.03 then
            update_shape = true
        end
        -- Store snapshot for next frame's delta.
        for b, s in pairs(self.rag) do
            self._last_rag_a[b] = s.a or 0
        end
    end

    if update_shape then
        self.wibox:set_shape(self._shape_fn)
        self._last_facing = self.facing or 1
    end
end

-- ---------------------------------------------------------------------------
-- Grab / throw (pendulum model).
-- ---------------------------------------------------------------------------

function box:grab()
    if self.grabbing then return end
    self.grabbing = true
    self.grab_side = "center"

    local coords = mouse.coords()
    self.grab_mx, self.grab_my = coords.x, coords.y
    self.grab_lx, self.grab_ly = coords.x - self.phys.cx, coords.y - self.phys.cy
    self.pivot_samples = {}
    self.phys.vx, self.phys.vy, self.phys.omega = 0, 0, 0

    local ok = pcall(mousegrabber.run, function(m)
        self.grab_mx, self.grab_my = m.x, m.y
        if not m.buttons[1] then
            self:release_toss()
            self.grabbing = false
            self.grab_side = nil
            return false
        end
        return true
    end, "fleur")

    if not ok then
        self.grabbing = false
        self.grab_side = nil
    end
end

function box:grab_step(dt)
    dt = math.min(dt, 0.1)
    local px, py = self.grab_mx, self.grab_my

    -- Direct spring to cursor (Godot uses a PinJoint; a damped spring is the
    -- simple Lua equivalent). Pull the body centre toward the cursor each
    -- frame; zero velocity so there's no inertia fighting the cursor.
    local k = 24.0 -- spring stiffness (higher = tighter follow)
    self.phys.cx = self.phys.cx + (px - self.phys.cx) * k * dt
    self.phys.cy = self.phys.cy + (py - self.phys.cy) * k * dt
    self.phys.omega = 0

    -- Clamp horizontally to stay on screen; do not clamp vertically so the
    -- pet can be lifted off the ground. Bottom clamp keeps it from being
    -- pulled below the workarea.
    local a = self:workarea()
    self.phys.cx = math.max(a.x + self.hw, math.min(self.phys.cx, a.x + a.width - self.hw))
    self.phys.cy = math.min(self.phys.cy, a.y + a.height - self.hh)

    self.phys.vx, self.phys.vy = 0, 0
    self:set_geometry()
end

function box:release_toss()
    local spin = self.pend_w or 0
    self.pend_w = 0

    local pvx, pvy = 0, 0
    local s = self.pivot_samples
    if #s >= 2 then
        local last = s[#s]
        local first = 1
        local throw_window = (self.config and self.config.throw_window) or 0.12
        while first < #s and last.t - s[first].t > throw_window do
            first = first + 1
        end
        local dt = last.t - s[first].t
        if dt >= 0.01 then
            pvx = (last.x - s[first].x) / dt
            pvy = (last.y - s[first].y) / dt
        end
    end

    local throw_multiplier = (self.config and self.config.throw_multiplier) or 1.0
    local phi = physics.atan2(self.phys.cy - self.grab_my, self.phys.cx - self.grab_mx)
    self.phys.omega = spin * ((self.config and self.config.toss_spin) or 1.0)
    self.phys.vx = (-spin * 100 * math.sin(phi) + pvx) * throw_multiplier
    self.phys.vy = (spin * 100 * math.cos(phi) + pvy) * throw_multiplier

    local spd = math.sqrt(self.phys.vx ^ 2 + self.phys.vy ^ 2)
    if spd > (self.config.sound_min_speed or 100) then
        self:play_impact(spd)
    end
    -- entering ragdoll if thrown hard
    if spd > self.config.ragdoll_speed then
        self:enter_ragdoll()
        self.ragdoll_timer = 0
    end
end

return box
