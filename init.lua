-- init.lua
-- desksaw
-- Desktop pet overlay for AwesomeWM: a physics-driven sprite that lives on
-- top of your floating windows and collides with them.
--
-- Require from rc.lua:
--   local desksaw = require("desksaw")
--   awful.screen.connect_for_each_screen(function(s) desksaw.init(s) end)

local gears = require("gears")
local awful = require("awful")

-- Box (and everything under it) is loaded defensively: if the renderer fails
-- to load, desksaw degrades to "no pet" rather than erroring out of rc.lua.
local ok_box, box = pcall(require, "desksaw.box")
if not ok_box then
    print("[desksaw] failed to load box module: " .. tostring(box))
    box = nil
end

local module = {
    controllers = {},
    config = {},
    available = box ~= nil,
}

module.config = {
    -- Rendering.
    size = 200,
    sprite_scale = 4,      -- pixels per skeleton unit
    border_width = 0,

    -- Timing.
    fps = 60,

    -- Physics. gravity matches the scene's RigidBody2D gravity_scale = 0.8.
    gravity = 784,
    restitution = 0.55,
    damping = 0.9985,
    friction = 0.94,
    spin_damping = 0.995,
    rest_epsilon = 30,
    max_dt = 0.1,
    substeps = 8,
    wall_thickness = 8,
    render_angle_threshold = 0.01,

    -- Character movement (DeskSaw port).
    maxspeed = 240,
    walk_mult = 1.6,
    accel = 2600,
    air_accel = 900,
    slowdown = 0.18,
    jump_power = -600,
    jump_mult = 1.22,
    jump_cooldown = 4.0,
    coyote_time = 0.11,
    ragdoll_speed = 575,

    -- Lean / crouch / visual offset (live_body).
    lean_per_dv = 0.30,
    lean_rate = 5.0,
    lean_max = 25,
    land_crouch = 0.05,
    land_crouch_max = 55,
    land_min = 150,
    crouch_rate = 6.0,
    offset_rate = 8.0,

    -- Grab / throw (pendulum).
    pend_damping = 4.5,
    throw_window = 0.12,
    throw_multiplier = 1.0,
    toss_spin = 1.0,

    -- Sound (paplay).
    sound_enabled = true,
    sound_path = "/home/tsuchinoko/desksaw/assets/sounds/effects/thudswoosh.ogg",
    sound_cooldown = 0.15,
    sound_min_speed = 100,
}

-- (Re)create the actor on `screen`, with a fixed-timestep drive timer.
function module.set_mode(screen)
    if not box then
        print("[desksaw] renderer unavailable, not starting")
        return nil
    end
    local idx = screen.index
    local ctrl = module.controllers[idx]
    if ctrl and ctrl.actor then
        ctrl.actor:destroy()
        if ctrl.timer then ctrl.timer:stop() end
    end
    -- Invalidate cached debug menu for this screen so it gets recreated
    -- with the new actor reference.
    debug_menus[idx] = nil

    local actor = box.new(screen, module.config)
    actor.last_tick = box.now()

    local timer = gears.timer { timeout = 1 / module.config.fps }
    local step_dt = 1 / module.config.fps
    local acc = 0

    timer:connect_signal("timeout", function()
        if not actor or not actor.phys then return end
        local now = box.now()
        local dt = now - actor.last_tick
        actor.last_tick = now
        if dt <= 0 or dt > 0.5 then
            acc = 0
            return
        end
        acc = math.min(acc + dt, step_dt * 16)
        while acc >= step_dt do
            actor:step(step_dt)
            acc = acc - step_dt
        end
    end)
    timer:start()

    module.controllers[idx] = { screen = screen, actor = actor, timer = timer }
    return actor
end

-- Debug submenu items for rc.lua's mymainmenu desksaw submenu.
-- Returns an awful.menu-compatible items table. Each action looks up the
-- actor fresh at call time, so a cached menu stays correct after actor
-- recreation.
-- Menu cache, per screen index.
local debug_menus = {}

local function debug_menu_items(screen)
    local idx = screen and screen.index or 1
    local ctrl = module.controllers[idx]
    if not ctrl or not ctrl.actor then return {} end
    local actor = ctrl.actor

    local function set_state(state, dir)
        if state == STATE_RAGDOLL then
            actor:enter_ragdoll()
            return
        end
        if actor.state == STATE_RAGDOLL then
            actor:recover_from_ragdoll()
        end
        actor.dir = dir or 0
        if dir and dir ~= 0 then
            actor.facing = dir
        end
        actor.state = state
        actor.phase = 0
        actor.clip = state
        actor.wander_timer = 0
        actor.wander_phase = state == STATE_MOVING and "walk" or "idle"
    end

    return {
        { "Idle",           function() set_state("idle", 0) end },
        { "Move left",      function() set_state("moving", -1) end },
        { "Move right",     function() set_state("moving", 1) end },
        { "Jump",           function() set_state("jumping", 0) end },
        { "Fall",           function() set_state("falling", 0) end },
        { "Ragdoll",        function() set_state("ragdoll", 0) end },
        { "Recover",        function() set_state("idle", 0) end },
        { "Respawn",        function() actor:respawn() end },
    }
end

-- Toggle the debug submenu for the given screen.
-- Menu is created once per screen and reused (no spawning new instances).
function module.toggle_debug_menu(screen)
    local idx = screen and screen.index or 1
    local ctrl = module.controllers[idx]
    if not ctrl or not ctrl.actor then return end

    local menu = debug_menus[idx]
    if not menu then
        menu = awful.menu {
            items = debug_menu_items(screen),
        }
        debug_menus[idx] = menu
    end
    menu:toggle()
end

function module.init(screen)
    return module.set_mode(screen)
end

function module.show(screen)
    if screen then
        local ctrl = module.controllers[screen.index]
        if ctrl and ctrl.actor then ctrl.actor:set_visible(true) end
    else
        for _, ctrl in pairs(module.controllers) do
            if ctrl and ctrl.actor then ctrl.actor:set_visible(true) end
        end
    end
end

function module.hide(screen)
    if screen then
        local ctrl = module.controllers[screen.index]
        if ctrl and ctrl.actor then ctrl.actor:set_visible(false) end
    else
        for _, ctrl in pairs(module.controllers) do
            if ctrl and ctrl.actor then ctrl.actor:set_visible(false) end
        end
    end
end

function module.toggle(screen)
    if screen then
        local ctrl = module.controllers[screen.index]
        if ctrl and ctrl.actor and ctrl.actor.wibox then
            local vis = ctrl.actor.wibox.visible
            ctrl.actor:set_visible(not vis)
        end
    else
        local any = false
        for _, ctrl in pairs(module.controllers) do
            if ctrl and ctrl.actor and ctrl.actor.wibox and ctrl.actor.wibox.visible then
                any = true
                break
            end
        end
        if any then module.hide() else module.show() end
    end
end

function module.reset(screen)
    local ctrl = module.controllers[screen and screen.index]
    if ctrl and ctrl.actor then ctrl.actor:respawn() end
end

-- Screen lifecycle.
screen.connect_signal("added", function(s)
    module.init(s)
end)
screen.connect_signal("removed", function(s)
    local idx = s.index
    local ctrl = module.controllers[idx]
    if ctrl then
        if ctrl.actor then ctrl.actor:destroy() end
        if ctrl.timer then ctrl.timer:stop() end
    end
    module.controllers[idx] = nil
end)

return module
