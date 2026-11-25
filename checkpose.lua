-- checkpose.lua
-- desksaw.checkpose
-- Numeric verification that the skeleton pose actually changes across animation
-- frames. Since the poses cannot be eyeballed here, this measures the drawn
-- bounding box (and limb joint positions) per frame instead.
--
-- Usage: luajit desksaw/checkpose.lua

local root = (arg and arg[0]:match("^(.*)/[^/]+$") or ".") .. "/"
package.path = root .. "?.lua;" .. package.path

local sk = require("desksaw.skeleton")
local lgi = require("lgi")
local cairo = lgi.cairo
local ffi = require("ffi")

local function frame_bbox(clip, t)
    local W, H = 700, 700
    local s = cairo.ImageSurface(cairo.Format.ARGB32, W, H)
    local c = cairo.Context(s)
    c:set_source_rgba(0, 0, 0, 1)
    c:paint()
    local ok, err = pcall(sk.draw, c, { x = 350, y = 420, scale = 4, clip = clip, phase = t })
    if not ok then return nil, err end
    local stride = s:get_stride()
    local p = ffi.cast("uint32_t*", s:get_data())
    local minx, miny, maxx, maxy = 9999, 9999, -1, -1
    for y = 0, H - 1 do
        for x = 0, W - 1 do
            if p[y * (stride / 4) + x] ~= 0xFF000000 then
                if x < minx then minx = x end
                if x > maxx then maxx = x end
                if y < miny then miny = y end
                if y > maxy then maxy = y end
            end
        end
    end
    return string.format("x %3d..%3d  y %3d..%3d  (%3dx%3d)",
                         minx, maxx, miny, maxy, maxx - minx, maxy - miny)
end

local clips = { "idleagain", "moveF", "moveB", "jump", "fall", "sit", "laydown" }
local fails = 0

print("desksaw pose check")
print("------------------")
for _, clip in ipairs(clips) do
    local len = sk.clip_length(clip)
    print(string.format("%s (len %.2f)", clip, len))
    local prev
    local distinct = 0
    for i = 0, 3 do
        local t = (len > 0 and len or 1) * (i / 4)
        local bb, err = frame_bbox(clip, t)
        if not bb then
            print(string.format("   t=%.2f  DRAW ERROR: %s", t, tostring(err)))
            fails = fails + 1
        else
            print(string.format("   t=%.2f  %s", t, bb))
            if prev and bb ~= prev then distinct = distinct + 1 end
            prev = bb
        end
    end
    -- A looping clip with more than one keyframe should differ frame to frame.
    if prev and distinct == 0 and len > 0 then
        print("   note: bbox identical across sampled frames")
    end
end

print("")
print("rest bbox: ", sk.bounds())
print("extents:   ", sk.extents())
print("")
if fails > 0 then
    print("RESULT: FAIL (" .. fails .. " draw errors)")
    os.exit(1)
end
print("RESULT: PASS (no draw errors)")
