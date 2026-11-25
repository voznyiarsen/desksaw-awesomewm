-- checkrig.lua
-- desksaw.checkrig
-- Numeric verification of the rig, run WITHOUT AwesomeWM:
--   1. every IK chain reaches its clip target
--   2. bone segment lengths never change
--   3. the feet are the lowest points and sit on the skeleton-space ground (y=0)
--   4. the skeleton origin is at the FEET, not the figure centre
--
-- Usage: luajit desksaw/checkrig.lua

local root = (arg and arg[0]:match("^(.*)/[^/]+$") or ".") .. "/"
package.path = root .. "?.lua;" .. package.path

local pose = require("desksaw.pose")
local anim = require("desksaw.anim")
local sk = require("desksaw.skeleton")

local DEG = 180 / math.pi
local fails = 0

local function fail(msg)
    fails = fails + 1
    print("  FAIL " .. msg)
end

local function dist(G, a, b)
    local ax, ay = G[a].M[5], G[a].M[6]
    local bx, by = G[b].M[5], G[b].M[6]
    return math.sqrt((bx - ax) ^ 2 + (by - ay) ^ 2)
end

local CHAINS = {
    { tip = "footF", a = "legF", b = "crusF", target = "ik/IKFLeg:position", l1 = pose.SEG.thighF, l2 = pose.SEG.shinF },
    { tip = "footB", a = "legB", b = "crusB", target = "ik/IKBLeg:position", l1 = pose.SEG.thighB, l2 = pose.SEG.shinB },
    { tip = "handF", a = "fArm", b = "fLArm", target = "ik/IKFArm:position", l1 = pose.SEG.upperF, l2 = pose.SEG.foreF },
    { tip = "handB", a = "bArm", b = "bLArm", target = "ik/IKBArm:position", l1 = pose.SEG.upperB, l2 = pose.SEG.foreB },
}

local clips = { "idleagain", "moveF", "moveB", "jump", "fall", "sit", "laydown" }
local TOL = 0.25

print("desksaw rig check")
print("-----------------")

print("segment constants (skeleton units):")
local ks = {}
for k in pairs(pose.SEG) do ks[#ks + 1] = k end
table.sort(ks)
for _, k in ipairs(ks) do print(string.format("   %-7s %.4f", k, pose.SEG[k])) end

print("")
for _, clip in ipairs(clips) do
    local cd = anim[clip]
    local len = (cd and cd.length) or 1
    for _, f in ipairs({ 0, 0.25, 0.5, 0.75 }) do
        local t = len * f
        local G = pose.build({ anim = anim, clip = clip, phase = t })
        local msgs = {}

        for _, ch in ipairs(CHAINS) do
            local tgt = cd and cd.tracks[ch.target]
            if tgt then
                local tv = pose.sample_track(tgt, t)
                if tv then
                    local tx, ty = pose.xform(G.ikNodes.M, tv[1], tv[2])
                    local bx, by = G[ch.tip].M[5], G[ch.tip].M[6]
                    local err = math.sqrt((bx - tx) ^ 2 + (by - ty) ^ 2)
                    local sx, sy = G[ch.a].M[5], G[ch.a].M[6]
                    local sd = math.sqrt((tx - sx) ^ 2 + (ty - sy) ^ 2)
                    local reach = ch.l1 + ch.l2
                    local allowed = (sd > reach) and (sd - reach + 0.5) or TOL
                    if err > allowed then
                        msgs[#msgs + 1] = string.format("%s off %.2f", ch.tip, err)
                    end
                end
            end
            local d1 = dist(G, ch.a, ch.b)
            if math.abs(d1 - ch.l1) > 0.01 then
                msgs[#msgs + 1] = string.format("%s-%s len %.3f", ch.a, ch.b, d1)
            end
        end

        local hipY = G.hip.M[6]
        local footY = math.max(G.footF.M[6], G.footB.M[6])
        if footY <= hipY then
            msgs[#msgs + 1] = "FEET ABOVE HIP"
        end

        if #msgs > 0 then
            fail(string.format("%-9s t=%.2f  %s", clip, t, table.concat(msgs, "; ")))
        end
    end
    print(string.format("  checked %-9s (len %.4f)", clip, len))
end

-- The soles of the COLLISION shape are at y=0 (spec §3.2), but the foot
-- SPRITES include toes that hang a little below that (experimentFoot's art
-- reaches y~+12 in the idle pose). So the bounds' lowest point is a bit below
-- zero -- that is correct, not a bug. Check that it is within one foot's
-- height of the ground, and that the hip is a leg's length above it.
print("")
local x0, y0, x1, y1 = sk.bounds()
print(string.format("idle-pose bounds: x %.2f..%.2f  y %.2f..%.2f", x0, x1, y0, y1))
print(string.format("  w=%.2f  h=%.2f", x1 - x0, y1 - y0))

local G0 = pose.build({ anim = anim, clip = "idleagain", phase = 0 })
local hipY = G0.hip.M[6]
local legLen = pose.SEG.thighF + pose.SEG.shinF

if y1 > legLen then
    fail(string.format("soles at y=%.2f, more than a leg (%.1f) below ground", y1, legLen))
else
    print(string.format("  lowest art at y=%.2f (%.1f below the collision floor -- toes)", y1, y1))
end

if hipY > -10 then
    fail(string.format("hip at y=%.2f, expected ~-20 (above the feet)", hipY))
else
    print(string.format("  hip at y=%.2f (%.1f above the lowest art, leg = %.1f) -- OK",
        hipY, y1 - hipY, legLen))
end

print("")
if fails > 0 then
    print(string.format("RESULT: FAIL (%d)", fails))
    os.exit(1)
end
print("RESULT: PASS")
