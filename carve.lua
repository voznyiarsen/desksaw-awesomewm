-- carve.lua
-- simplephys.carve
-- Axis-aligned rectangle subtraction, used to mask floating-window
-- obstacles against the fixed windows stacked above them: the box must
-- only collide with the parts of a floating window that are not obscured
-- by a fixed window. No awesome dependencies (plain lua, no trig).

local carve = {}

local function max2(a, b) return a > b and a or b end
local function min2(a, b) return a < b and a or b end

-- Carve the rectangles in `occ[1..oc]` out of `win` ({x,y,w,h}) and return
-- the surviving pieces as a list of {x,y,w,h} tables (`win` and the occ
-- entries are never mutated). Overlapping occluders are handled by splitting
-- each surviving piece in turn. If a piece is fully covered it produces no
-- strips and simply disappears.
--
-- The result is the set difference win \ (union of occ) APPROXIMATED to a
-- minimum feature size: `minsz` (default 2) discards pieces thinner than that
-- in either dimension -- including `win` itself, which the caller may well
-- hand in as a 1px sliver. Those slivers are the 1px collision seams along
-- window edges, and a body catching on one behaves worse than a body ignoring
-- it, so the approximation is deliberate. It does mean this is not an exact
-- set difference.
--
-- `scratch` (optional) is a caller-owned table used to recycle every
-- intermediate allocation: the piece tables, and the two alternating work
-- buffers. Pass the same table on every call to make a steady-state rebuild
-- allocation-free. The returned list is scratch-owned, so the caller must
-- consume it (copy the numbers out) before the next call with that scratch.
function carve.subtract(win, occ, oc, minsz, scratch)
    minsz = minsz or 2

    local store, work, nxt
    if scratch then
        scratch.store = scratch.store or {}
        scratch.a = scratch.a or {}
        scratch.b = scratch.b or {}
        store, work, nxt = scratch.store, scratch.a, scratch.b
    else
        store, work, nxt = {}, {}, {}
    end
    for i = #work, 1, -1 do work[i] = nil end
    for i = #nxt, 1, -1 do nxt[i] = nil end

    -- Filter the input rect too: with no occluders at all the old code
    -- returned `win` unexamined, so a sub-minsz window still became an
    -- obstacle.
    if win.w < minsz or win.h < minsz then
        return work
    end

    -- Piece tables are handed out from `store` in increasing order and never
    -- reordered, so a piece still referenced by a work buffer can never be
    -- recycled mid-call: every new piece comes from an index above all live
    -- ones. (Returning the store itself and shuffling entries into place would
    -- alias two live pieces onto one table -- two obstacles sharing a rect.)
    local np = 0
    local function piece(x, y, w, h)
        np = np + 1
        local p = store[np]
        if not p then p = {}; store[np] = p end
        p.x, p.y, p.w, p.h = x, y, w, h
        return p
    end
    -- `nxt` is reassigned on every swap below; the closure reads the variable,
    -- so it always appends to the current output buffer.
    local function add(x, y, w, h)
        if w >= minsz and h >= minsz then
            nxt[#nxt + 1] = piece(x, y, w, h)
        end
    end

    work[1] = piece(win.x, win.y, win.w, win.h)

    for i = 1, oc do
        local cv = occ[i]
        local cvx1 = cv.x + cv.w
        local cvy1 = cv.y + cv.h
        for j = 1, #work do
            local p = work[j]
            local px, py, pw, ph = p.x, p.y, p.w, p.h
            local px1 = px + pw
            local py1 = py + ph
            local ix0 = max2(px, cv.x)
            local iy0 = max2(py, cv.y)
            local ix1 = min2(px1, cvx1)
            local iy1 = min2(py1, cvy1)
            if ix0 < ix1 and iy0 < iy1 then
                -- Four surrounding strips: left, right, top, bottom.
                if px < ix0 then add(px, py, ix0 - px, ph) end
                if ix1 < px1 then add(ix1, py, px1 - ix1, ph) end
                if py < iy0 then add(ix0, py, ix1 - ix0, iy0 - py) end
                if iy1 < py1 then add(ix0, iy1, ix1 - ix0, py1 - iy1) end
            elseif pw >= minsz and ph >= minsz then
                nxt[#nxt + 1] = p
            end
        end
        -- Swap the buffers and clear the one that just became spare.
        work, nxt = nxt, work
        for k = #nxt, 1, -1 do nxt[k] = nil end
        if #work == 0 then break end
    end

    return work
end

return carve
