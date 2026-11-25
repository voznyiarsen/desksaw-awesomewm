-- preview.lua
-- desksaw.preview
-- Renders the skeleton OFFLINE to a PNG so the pose can be checked without
-- loading the module into the running AwesomeWM.
--
-- Why this exists: a draw error raised inside a wibox shape/bgimage callback
-- runs in AwesomeWM's C draw path and is NOT catchable from Lua — it takes the
-- whole window manager down with it. So the draw code is always exercised here
-- first, under the same LGI/cairo the WM uses.
--
-- Usage (must run under luajit, since AwesomeWM uses LuaJIT's LGI):
--   luajit desksaw/preview.lua out.png [clip] [phase] [angle]
--   luajit desksaw/preview.lua out.png idleagain 0 0 rag
--   luajit desksaw/preview.lua out.png --sheet      (contact sheet)

package.path = (arg and arg[0]:match("^(.*)/[^/]+$") or ".") .. "/?.lua;"
              .. package.path

local lgi = require("lgi")
local cairo = lgi.cairo
local skeleton = require("desksaw.skeleton")

local args = {}
for i = 1, #arg do args[i] = arg[i] end

local scale = 4
local sheet = args[2] == "--sheet"

-- ---------------------------------------------------------------------------
-- Shared draw helper. pcall is deliberate: it is the only way a bad pose is
-- reported as an error instead of a segfault killing the test process.
-- ---------------------------------------------------------------------------

local function draw_one(cr, cx, cy, opts)
    local ok, err = pcall(skeleton.draw, cr, {
        x = cx, y = cy,
        angle = opts.angle or 0,
        scale = scale,
        facing = opts.facing or 1,
        clip = opts.clip,
        phase = opts.phase or 0,
        lean = opts.lean or 0,
        crouch = opts.crouch or 0,
        rag = opts.rag,
        rag_neutral = opts.rag_neutral,
        eye = opts.eye,
    })
    if not ok then
        io.stderr:write("DRAW ERROR: " .. tostring(err) .. "\n")
        return false
    end
    return true
end

-- ---------------------------------------------------------------------------
-- Contact sheet: every clip at 4 phases, to eyeball the whole animation set.
-- ---------------------------------------------------------------------------

if sheet then
    local clips = { "idleagain", "moveF", "moveB", "jump", "fall", "sit", "laydown" }
    local frames = 4
    local cw, ch = 300, 260
    local surf = cairo.ImageSurface(cairo.Format.ARGB32, cw * frames, ch * #clips)
    local cr = cairo.Context(surf)
    cr:set_source_rgba(0.16, 0.16, 0.20, 1)
    cr:paint()

    for row, clip in ipairs(clips) do
        local len = skeleton.clip_length(clip)
        for f = 0, frames - 1 do
            local t = (len > 0 and len or 1) * (f / frames)
            local cx = cw * f + cw / 2
            local cy = ch * (row - 1) + ch / 2 + 40
            if not draw_one(cr, cx, cy, { clip = clip, phase = t }) then
                os.exit(1)
            end
            cr:set_source_rgba(0.55, 0.55, 0.60, 1)
            cr:select_font_face("monospace")
            cr:set_font_size(12)
            cr:move_to(cw * f + 10, ch * (row - 1) + 20)
            cr:show_text(string.format("%s t=%.2f", clip, t))
        end
    end

    local out = args[1] or "/mnt/scratch/junk/sheet.png"
    surf:write_to_png(out)
    io.write(string.format("wrote %s  (%d clips x %d frames)\n", out, #clips, frames))
    os.exit(0)
end

-- ---------------------------------------------------------------------------
-- Single pose
-- ---------------------------------------------------------------------------

local out = args[1] or "/mnt/scratch/junk/preview.png"
local clip = args[2] or "idleagain"
local phase = tonumber(args[3] or "0")
local angle = tonumber(args[4] or "0")
local ragdoll = args[5] == "rag"

local surf = cairo.ImageSurface(cairo.Format.ARGB32, 900, 900)
local cr = cairo.Context(surf)
cr:set_source_rgba(0.16, 0.16, 0.20, 1)
cr:paint()

-- ground line, so "is it standing on the floor?" is visible at a glance
local floor_y = 620
cr:set_source_rgba(0.40, 0.40, 0.46, 1)
cr:set_line_width(2)
cr:move_to(0, floor_y)
cr:line_to(900, floor_y)
cr:stroke()

local opts = { clip = clip, phase = phase, angle = angle }
if ragdoll then
    local rag = skeleton.new_rag_state()
    -- kick the limbs so the flail is visible
    for _, b in ipairs({ "legF", "crusF", "legB", "crusB",
                         "fArm", "fLArm", "bArm", "bLArm", "head", "tail" }) do
        rag[b].a = (b:find("Arm") or b == "head" or b == "tail") and -0.8 or 0.9
    end
    opts.rag = rag
end

-- Place the pet so its feet sit on the ground line at rest.
local _, _, _, bcy = skeleton.extents()
if not draw_one(cr, 450, floor_y - bcy * scale, opts) then
    os.exit(1)
end

surf:write_to_png(out)

local minx, miny, maxx, maxy = skeleton.bounds()
io.write(string.format("wrote %s  (clip=%s t=%.2f angle=%.2f%s)\n",
                       out, clip, phase, angle, ragdoll and " ragdoll" or ""))
io.write(string.format("rest bbox: x %.1f..%.1f  y %.1f..%.1f  (%.1f x %.1f units)\n",
                       minx, maxx, miny, maxy, maxx - minx, maxy - miny))
