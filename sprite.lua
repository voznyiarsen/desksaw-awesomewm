-- sprite.lua
-- simplephys.sprite
-- Load a PNG and compute the tight bounding box of its opaque pixels, plus
-- the convex hull (DOP) of those pixels. A physics body collides as that
-- DOP polygon (SAT projects its own edge normals), and the sprite's alpha
-- channel doubles as the wibox shape mask -- painting the alpha onto the
-- 1-bit shape surface yields the EXACT silhouette (transparent holes
-- included), so no black pseudo-transparency background bleeds around or
-- inside the figure. Loading also returns a paint closure so the renderer
-- draws the sprite centred on the body and rotated about the body's pivot,
-- with NEAREST filtering for crisp pixel-art upscaling.
--
-- The physics pivot is the hull's CENTRE OF MASS, not the sprite's bbox
-- centre: the hull is shifted so its area centroid lands on the origin (so
-- gravity, torque arms and stable resting angles all refer to the real COM).
-- The paint closure adds back a compensating offset, so the sprite's bbox
-- centre is drawn at (-com_x, -com_y) in the body frame and the figure
-- still appears where the body is.
--
-- Uses lgi + cairo to decode the PNG and scan the alpha channel. On
-- LuaJIT (standard AwesomeWM) the raw pixel buffer is read directly via
-- FFI. On non-LuaJIT Awesome builds (C Lua 5.3/5.4) FFI is unavailable,
-- so the PNG is written to a temp file, its IDAT chunk is zlib-decoded
-- via GIO's ZlibDecompressor, and the alpha channel is scanned from the
-- rebuilt pixel buffer. If neither path can produce pixel data, poly is
-- nil and the caller falls back to a box body (with a warning).
--
-- NOTE: The collision hull is always a CONVEX hull (Andrew's monotone
-- chain). Concave transparent cutouts -- gaps between limbs, ears, etc --
-- are bridged by the convex boundary. The visual window shape mask still
-- uses the exact alpha channel (holes included); only the physics
-- collision shape is the convex envelope. This is a deliberate
-- performance/stability tradeoff: true concave decomposition would
-- require multiple convex polygons and multi-contact SAT, which is out
-- of scope for a single-body demo.

local cairo_ok, lgi = pcall(require, "lgi")
local cairo
if cairo_ok then cairo = lgi.cairo end
local ffi_ok, ffi = pcall(require, "ffi")

local Gio, GL = nil, nil
if cairo_ok then
    Gio = lgi.Gio
    GL = lgi.GLib
end

-- Portability helpers across Lua 5.1 (LuaJIT) .. 5.4.
-- Big-endian unsigned 32-bit read (replaces string.unpack(">I4", ...),
-- which is Lua 5.3+ only).
local function be_u32(s, i)
    local a, b, c, d = s:byte(i, i + 3)
    return ((a * 256 + b) * 256 + c) * 256 + d
end

-- Area centroid of a closed convex polygon given as a {x=, y=} list (the
-- shape Andrew's chain produces). The physics layer computes the same value
-- for its own bodies (physics.poly_centroid); this is the loader's copy so
-- the sprite hull can be recentred on its centroid without depending on the
-- physics module.
local function poly_centroid(pts)
    local n = #pts
    if n < 3 then return 0, 0 end
    local area2, cxs, cys = 0, 0, 0
    for i = 1, n do
        local j = i % n + 1
        local xi, yi = pts[i].x, pts[i].y
        local xj, yj = pts[j].x, pts[j].y
        local cross = xi * yj - xj * yi
        area2 = area2 + cross
        cxs = cxs + (xi + xj) * cross
        cys = cys + (yi + yj) * cross
    end
    if math.abs(area2) < 1e-9 then return 0, 0 end
    return cxs / (3 * area2), cys / (3 * area2)
end

local sprite = {}

-- Largest image the loader will scan. The bbox scan and hull construction are
-- O(pixels), so a stray 20000x20000 PNG would stall the whole main loop (and
-- with it the WM) for minutes. 4096x4096 is far above any plushie sprite.
sprite.max_pixels = 4096 * 4096

-- Alpha cutoff for treating a texel as opaque when scanning the collision
-- bounding box / hull. 0 = any non-transparent pixel counts (the original
-- behaviour). Raising it (e.g. 16) drops faint anti-aliased / halo edge
-- pixels from the collision shape, which also keeps a light PNG matte from
-- showing as a visible halo. It does NOT change the painted sprite.
sprite.alpha_threshold = 0

-- High-resolution PNGs make three things slow: the alpha scan + hull build
-- (O(pixels), on the main loop), the SAT cost of a dense convex hull, and the
-- per-frame Cairo paint of a huge surface. These knobs decouple PHYSICS cost
-- from VISUAL quality: the alpha scan and hull run on a downscaled "collision
-- surface" and the paint runs on a downscaled "render surface", while the
-- sprite's on-screen size and position stay the same.

-- Longest dimension used for the collision (alpha / hull) scan surface. Above
-- this, the scan and hull are extracted from a downscaled copy, so the
-- O(pixels) work is bounded by this size, not by the native PNG. 0 disables
-- collision downsampling (scan the full image).
sprite.max_collision_dim = 1024

-- If the opaque bbox area has more native pixels than this, skip the convex
-- hull entirely and let the caller fall back to a bounding-box physics body.
-- 0 culls every sprite to a box. Large by default so ordinary sprites keep
-- their hull; only pathologically big ones fall back to a box.
sprite.max_hull_pixels = 1024 * 1024

-- After building the hull, cap its vertex count by uniform decimation so SAT
-- cost stays bounded for the detailed silhouettes of huge sprites.
sprite.max_hull_verts = 64

-- Longest dimension used for painting. Above this, the sprite is painted from
-- a downscaled copy so Cairo samples a smaller texture each repaint. Visual
-- quality is unchanged at normal desktop sizes; 0 always uses the original.
sprite.max_render_dim = 2048

-- Paint filter for the (possibly downscaled) render surface. nil = automatic:
-- BILINEAR for smooth downscaled sprites, NEAREST for unscaled/pixel-art.
-- Override with cairo.Filter.NEAREST (pixel art) or cairo.Filter.BILINEAR.
sprite.render_filter = nil

-- Cairo stores ARGB32 in NATIVE byte order, so the alpha byte of pixel (x,y)
-- is at data + y*stride + x*4 + 3 on little-endian machines (B,G,R,A) and at
-- offset + 0 on big-endian ones (A,R,G,B). Probing the ABI once and picking the
-- offset keeps the FFI reader correct on both instead of silently reading the
-- blue channel as alpha on a big-endian host.
local alpha_byte = 3
if ffi_ok and ffi.abi then
    local ok_le, le = pcall(ffi.abi, "le")
    if ok_le and not le then alpha_byte = 0 end
end

local function pixel_alpha(p, stride, x, y)
    return p[y * stride + x * 4 + alpha_byte] + 0
end

-- Return a surface in ARGB32 (8 bits per channel, native byte order), painting
-- `surf` onto a fresh one if it is in any other format.
--
-- This matters more than it looks: cairo does NOT always decode a PNG into
-- ARGB32. A 16-bit-per-channel PNG (colortype 6, bitdepth 16 -- what many image
-- editors emit) is loaded into RGBA128F, i.e. four 32-bit FLOATS per pixel, so
-- the ARGB32 reader below (one byte at x*4 + 3) reads a slice of some other
-- pixel's mantissa as "alpha". The hull it produced was plausible-looking but
-- unrelated to the sprite's actual silhouette, and the FFI-free path had the
-- matching problem: it re-encoded that surface to PNG, got a 16-bit file back,
-- and parsed it as 8-bit.
-- Converting once, up front, makes every downstream reader correct for any
-- format cairo may hand us (RGB16_565, RGB30, RGB96F, RGBA128F, ...). The
-- ORIGINAL surface is still what gets painted, so display quality is untouched.
local function as_argb32(surf, sw, sh)
    if not (cairo and cairo.Format and cairo.ImageSurface) then return surf end
    local ok, fmt = pcall(function() return surf:get_format() end)
    if ok and fmt == cairo.Format.ARGB32 then return surf end
    local ok2, conv = pcall(function()
        local c = cairo.ImageSurface.create(cairo.Format.ARGB32, sw, sh)
        local cr = cairo.Context(c)
        cr:set_operator(cairo.Operator.SOURCE)
        cr:set_source_surface(surf, 0, 0)
        cr:paint()
        c:flush()
        return c
    end)
    if ok2 and conv then return conv end
    return surf
end

-- Build a downscaled copy of `src` for painting, so Cairo samples a much
-- smaller texture each repaint. Returns (surface, scale) where
-- `scale` = render_dim / native_dim (draw_scale = sprite_scale / scale).
-- If no downscale is required, returns (src, 1).
local function make_render_surface(src, sw, sh, max_dim)
    if not (cairo and cairo.ImageSurface) then
        return src, 1
    end
    if not max_dim or max_dim <= 0 then
        return src, 1
    end
    local largest = math.max(sw, sh)
    if largest <= max_dim then
        return src, 1
    end

    local s = max_dim / largest
    local w = math.max(1, math.floor(sw * s + 0.5))
    local h = math.max(1, math.floor(sh * s + 0.5))

    local ok, out = pcall(function()
        local cs = cairo.ImageSurface.create(cairo.Format.ARGB32, w, h)
        local cr = cairo.Context(cs)
        cr:set_operator(cairo.Operator.SOURCE)
        cr:scale(w / sw, h / sh)
        cr:set_source_surface(src, 0, 0)
        local pattern = cr:get_source()
        if pattern then
            -- GOOD (bilinear) downscaling preserves the smoothness of
            -- high-quality sprites; this is a one-time load cost paid back
            -- over every frame the sprite is painted.
            pattern:set_filter(cairo.Filter.GOOD)
        end
        cr:paint()
        cs:flush()
        return cs
    end)
    if ok and out then
        return out, s
    end
    return src, 1
end

-- Uniformly drop vertices from a convex polygon to cap its size. Keeps every
-- n-th vertex, which is good enough for a plushie collider and preserves the
-- winding order.
local function decimate_convex_hull(pts, max_verts)
    local n = #pts
    if n <= max_verts then
        return pts
    end
    local out = {}
    local step = n / max_verts
    for i = 0, max_verts - 1 do
        out[#out + 1] = pts[math.floor(i * step) + 1]
    end
    return out
end

-- =============================================================================
-- LGI fallback: decode a PNG surface into an alpha-channel pixel reader
-- without FFI, using GIO's ZlibDecompressor for IDAT decompression.
-- =============================================================================

-- PNG unfilter: reconstruct filter-byte-encoded scanlines into raw pixels.
-- PNG applies per-byte filters; we need the reconstructed data to read
-- alpha values accurately.
local function png_paeth(a, b, c)
    local p = a + b - c
    local pa, pb, pc = math.abs(p - a), math.abs(p - b), math.abs(p - c)
    if pa <= pb and pa <= pc then return a
    elseif pb <= pc then return b
    else return c end
end

-- Unfilter raw PNG IDAT data into a contiguous string of pixels
-- (no filter bytes, no row stride padding beyond the raw pixel bytes).
-- colortype determines bytes-per-pixel (bpp):
--   0 (grayscale)     -> 1, no alpha
--   2 (RGB)           -> 3, no alpha
--   3 (palette)       -> 1, index into PLTE (alpha comes from tRNS)
--   4 (gray + alpha)  -> 2, has alpha
--   6 (RGBA)          -> 4, has alpha
-- Returns: string, sw, sh, bpp, alpha_offset, has_alpha
local function png_unfilter(raw, width, height, colortype)
    local bpp
    local alpha_offset
    local has_alpha
    if colortype == 6 then
        bpp, alpha_offset, has_alpha = 4, 3, true
    elseif colortype == 4 then
        bpp, alpha_offset, has_alpha = 2, 1, true
    elseif colortype == 0 then
        bpp, alpha_offset, has_alpha = 1, 0, false
    elseif colortype == 2 then
        bpp, alpha_offset, has_alpha = 3, 0, false
    elseif colortype == 3 then
        -- Palette: the sample IS the palette index. Transparency, if any,
        -- comes from the tRNS chunk, which the caller resolves.
        bpp, alpha_offset, has_alpha = 1, 0, false
    else
        return nil, width, height, 0, 0, false
    end

    local out = {}
    local prev = {}  -- previous reconstructed row (as byte table)
    local i = 1  -- 1-indexed position in decompressed raw data
    for y = 0, height - 1 do
        local filter = raw:byte(i) or 0
        i = i + 1
        local row = {}
        for x = 0, width * bpp - 1 do
            local fb = raw:byte(i) or 0
            i = i + 1
            local a = (x >= bpp) and row[x - bpp + 1] or 0
            local b = prev[x + 1] or 0
            local c = (x >= bpp and prev[x - bpp + 1]) or 0
            local recon
            if filter == 0 then
                recon = fb
            elseif filter == 1 then
                recon = (fb + a) % 256
            elseif filter == 2 then
                recon = (fb + b) % 256
            elseif filter == 3 then
                recon = (fb + math.floor((a + b) / 2)) % 256
            elseif filter == 4 then
                recon = (fb + png_paeth(a, b, c)) % 256
            else
                recon = fb
            end
            row[x + 1] = recon
        end
        -- Build the row string without table.unpack, which is missing on
        -- Lua 5.1 / LuaJIT and can overflow the C stack for wide images.
        local chars = {}
        for k = 1, #row do
            chars[k] = string.char(row[k])
        end
        out[#out + 1] = table.concat(chars)
        prev = row
    end
    return table.concat(out), width, height, bpp, alpha_offset, has_alpha
end

-- Decode a cairo ImageSurface to an alpha-channel pixel reader via LGI.
-- Writes the surface to a temp PNG, parses chunks, decompresses IDAT with
-- GIO's ZlibDecompressor, unfilters, and returns a function (x, y) -> alpha.
-- Returns nil + error message on failure.
--
-- Temp files are created with Gio.File.new_tmp (mkstemp semantics: an
-- unpredictable name, mode 0600, created exclusively by us). The previous
-- math.random-based "/tmp/simplephys_<n>.png" was both guessable -- a local
-- attacker could pre-create the path as a symlink and redirect our write --
-- and collision-prone across concurrent loads, since math.random was never
-- seeded.
local function tmp_path(suffix)
    if Gio and Gio.File.new_tmp then
        local ok, f, stream = pcall(Gio.File.new_tmp, "simplephys_XXXXXX" .. suffix)
        if ok and f then
            -- The handle is only needed to reserve the name; the writers below
            -- reopen the path. Closing it now avoids leaking a descriptor.
            if stream then pcall(function() stream:close() end) end
            return f:get_path()
        end
    end
    -- Last resort: a name in the user's own runtime dir if we have one (not
    -- world-writable, so the symlink race does not apply), else /tmp.
    local dir = (GL and GL.get_user_runtime_dir and GL.get_user_runtime_dir())
                or os.getenv("XDG_RUNTIME_DIR") or "/tmp"
    return string.format("%s/simplephys_%d_%d%s", dir,
        (GL and GL.get_monotonic_time and GL.get_monotonic_time()) or os.time(),
        math.random(0, 999999), suffix)
end

local function lgi_pixel_reader(surf, path)
    if not (Gio and GL) then
        return nil, "GIO/GLib unavailable for FFI-free PNG decode"
    end

    local tmp_png = tmp_path(".png")
    local tmp_idat = tmp_path(".idat")

    local ok, result = pcall(function()
        -- 1. Write surface to PNG temp file
        surf:write_to_png(tmp_png)

        -- 2. Read PNG file contents
        local gf = Gio.File.new_for_path(tmp_png)
        local png_data = gf:load_contents(nil)

        -- 3. Parse PNG to find IHDR, tRNS and IDAT
        local offset = 9
        local ihdr_str, idat_str, trns_str
        while offset <= #png_data - 8 do
            local length = be_u32(png_data, offset)
            local chunk_type = png_data:sub(offset + 4, offset + 7)
            if offset + 8 + length + 4 > #png_data then break end
            if chunk_type == "IHDR" then
                ihdr_str = png_data:sub(offset + 8, offset + 8 + length - 1)
            elseif chunk_type == "IDAT" then
                idat_str = (idat_str or "") ..
                    png_data:sub(offset + 8, offset + 8 + length - 1)
            elseif chunk_type == "tRNS" then
                trns_str = png_data:sub(offset + 8, offset + 8 + length - 1)
            elseif chunk_type == "IEND" then
                break
            end
            offset = offset + 8 + length + 4
        end
        if not ihdr_str then error("no IHDR chunk") end

        local width = be_u32(ihdr_str, 1)
        local height = be_u32(ihdr_str, 5)
        local bitdepth = ihdr_str:byte(9)
        local colortype = ihdr_str:byte(10)
        local interlace = ihdr_str:byte(13)

        -- png_unfilter walks one filter byte plus width*bpp sample bytes per
        -- row, which is only the layout of a NON-interlaced, 8-bit image.
        -- Adam7 stores seven progressively larger sub-images and sub-8-bit
        -- depths pack several samples per byte, so decoding either as 8-bit
        -- non-interlaced produces a scrambled alpha channel -- and therefore a
        -- nonsense collision hull -- rather than an error. In practice the PNG
        -- parsed here is cairo's OWN re-encoding of the already-decoded
        -- surface (so it is non-interlaced 8-bit by construction, whatever the
        -- source file was), but nothing in the format guarantees that for a
        -- future cairo, and silently wrong physics is worse than a clear
        -- message.
        if interlace ~= 0 then
            error("interlaced (Adam7) PNGs are not supported by the FFI-free "
                  .. "decoder; re-save without interlacing")
        end
        if bitdepth ~= 8 then
            error("unsupported PNG bit depth " .. tostring(bitdepth)
                  .. " (only 8-bit is supported by the FFI-free decoder)")
        end

        -- 4. Write IDAT to temp file
        local tf = io.open(tmp_idat, "wb")
        if not tf then error("cannot open temp file " .. tostring(tmp_idat)) end
        tf:write(idat_str)
        tf:close()

          -- 5. Decompress IDAT zlib stream via GIO
          --    NOTE: use read_bytes (returns GLib.Bytes) + get_data rather
          --    than Gio.InputStream.read (returns string), because lgi's
          --    marshal of the out-parameters for read() triggers an internal
          --    assert in this environment ("marshal.c: assertion failed:
          --    size > 0") that aborts the process and bypasses pcall.
          local gf_idat = Gio.File.new_for_path(tmp_idat)
          local idat_bytes = gf_idat:load_bytes(nil)
          local mi = Gio.MemoryInputStream.new_from_bytes(idat_bytes)
          local zd = Gio.ZlibDecompressor.new(0)  -- 0 = GZ_FORMAT_ZLIB
          local cs = Gio.ConverterInputStream.new(mi, zd)

          -- A single read_bytes() only returns up to 64 KiB; a sprite whose
          -- decompressed scanlines are larger would otherwise be truncated,
          -- producing a wrong alpha/bbox/hull and a corrupted collision shape.
          -- Loop until the stream is exhausted.
          local chunks = {}
          while true do
              local db = cs:read_bytes(65536, nil)
              if not db then break end

              local data = db:get_data()
              if not data or #data == 0 then break end

              chunks[#chunks + 1] = data
          end
          local raw = table.concat(chunks)

        -- 6. Unfilter scanlines
        local pixels, pw, ph, bpp, alpha_off, has_alpha =
            png_unfilter(raw, width, height, colortype)
        if not pixels then error("unsupported PNG colortype: " .. colortype) end

        -- 7. Return pixel reader closure
        if has_alpha then
            return function(x, y)
                local idx = (y * pw + x) * bpp + alpha_off
                return pixels:byte(idx + 1) or 0
            end
        elseif colortype == 3 and trns_str then
            -- Palette transparency: tRNS holds one alpha byte per palette
            -- entry (entries beyond its length are opaque). Treating every
            -- palette pixel as opaque -- as this decoder used to -- gives a
            -- hull covering the sprite's whole bounding box for any
            -- paletted PNG with a transparent background.
            local trns = {}
            for i = 1, #trns_str do
                trns[i - 1] = trns_str:byte(i)
            end
            return function(x, y)
                local idx = pixels:byte(y * pw + x + 1) or 0
                local a = trns[idx]
                if a == nil then return 255 end
                return a
            end
        else
            -- Colortypes 0, 2 and palettes without tRNS: no transparency.
            return function() return 255 end
        end
    end)

    -- Cleanup temp files (the reader closure captures the in-memory pixel
    -- string, not the temp files, so this is safe to do immediately).
    pcall(function() os.remove(tmp_png) end)
    pcall(function() os.remove(tmp_idat) end)

    if not ok then
        return nil, tostring(result)
    end
    -- On success, `result` is the pixel-reader closure returned from
    -- inside the pcall.
    if type(result) == "function" then
        return result, nil
    end
    return nil, "unknown error in LGI PNG decode"
end

-- path: filesystem path to the PNG. scale: optional uniform scale (1 = native
-- bbox pixels). Returns nil + error on failure.
function sprite.load(path, scale)
    scale = scale or 1
    if not scale or scale <= 0 then
        return nil, "sprite scale must be positive"
    end
    if not cairo_ok or not cairo.ImageSurface then
        return nil, "lgi cairo unavailable"
    end
    local ok, surf = pcall(cairo.ImageSurface.create_from_png, path)
    if not ok or not surf then
        return nil, "could not load sprite: " .. tostring(path)
    end
    local sw, sh = surf:get_width(), surf:get_height()
    if sw == 0 or sh == 0 then
        return nil, "sprite loaded empty (check path): " .. tostring(path)
    end
    -- Guard against pathologically large images: everything below is O(pixels)
    -- and runs synchronously on the WM's main loop.
    if sw * sh > sprite.max_pixels then
        return nil, string.format(
            "sprite too large: %dx%d (%d pixels, limit %d)",
            sw, sh, sw * sh, sprite.max_pixels)
    end

    -- Collision LOD: bound the O(pixels) alpha scan / hull build by
    -- max_collision_dim. Crucially, this does NOT rescale the surface with
    -- cairo: a cairo downscale still reads every native pixel to produce the
    -- smaller surface, which is slower than scanning the native surface in the
    -- first place (cairo's per-pixel work + an extra full-surface copy). The
    -- real cost being avoided is the per-pixel Lua calls in the scan/hull loop
    -- (each pixel reader call is a Lua closure call, and there are up to
    -- sw*sh of them), so instead we keep the native surface and sample it
    -- through a strided reader whenever the sprite is larger than
    -- max_collision_dim. That turns an sw*sh call budget into col_w*col_h
    -- while reading the exact same pixels; the PNG decode itself is
    -- unavoidable (cairo decodes at full resolution up front). For small
    -- sprites col_scale == 1 and the strided reader is skipped entirely, so
    -- bbox and hull are byte-for-byte identical to the un-downscaled loader.
    local scan = as_argb32(surf, sw, sh)
    local col_alpha_native = nil
    if ffi_ok and scan.get_data then
        local ptr = scan:get_data()
        if ptr then
            local p = ffi.cast("uint8_t*", ptr)
            local stride = scan:get_stride()
            -- RGB24 surfaces have no alpha byte, so the ARGB32 reader would
            -- see alpha==0 for every pixel and treat the image as fully
            -- transparent. Detect that format and treat it as opaque.
            local format = scan:get_format()
            if format == cairo.Format.RGB24 then
                col_alpha_native = function() return 255 end
            else
                col_alpha_native = function(x, y)
                    return pixel_alpha(p, stride, x, y)
                end
            end
        end
    end

    if not col_alpha_native then
        -- FFI unavailable or get_data returned nil: LGI fallback decodes the
        -- native surface into a fast per-pixel reader (one-time GIO/zlib cost).
        local reader, lgi_err = lgi_pixel_reader(scan, path)
        if reader then
            col_alpha_native = reader
        else
            print("[simplephys] warning: no pixel data for sprite "
                  .. tostring(path) .. " (" .. tostring(lgi_err) .. "); "
                  .. "collision will fall back to full-image rectangle")
        end
    end

    -- Collision resolution: col_w x col_h samples; col_scale = col_dim/native
    -- maps collision pixels back to native pixels (and is 1 for small sprites).
    local col_scale = 1
    local col_w, col_h = sw, sh
    if sprite.max_collision_dim and sprite.max_collision_dim > 0 then
        local largest = math.max(sw, sh)
        if largest > sprite.max_collision_dim then
            col_scale = sprite.max_collision_dim / largest
            col_w = math.max(1, math.floor(sw * col_scale + 0.5))
            col_h = math.max(1, math.floor(sh * col_scale + 0.5))
        end
    end

    -- Strided reader: sample the NATIVE scan at collision resolution, so the
    -- bbox + hull loops run col_w*col_h times instead of sw*sh. When col_scale
    -- == 1 this branch is skipped and col_alpha IS the native reader, so the
    -- small-sprite path is unchanged.
    local col_alpha
    if col_alpha_native and col_scale < 1 then
        local sx, sy = sw / col_w, sh / col_h
        col_alpha = function(cx, cy)
            local nx = math.floor((cx + 0.5) * sx)
            local ny = math.floor((cy + 0.5) * sy)
            if nx >= sw then nx = sw - 1 end
            if ny >= sh then ny = sh - 1 end
            return col_alpha_native(nx, ny)
        end
    else
        col_alpha = col_alpha_native
    end

    -- Render LOD: paint from a downscaled surface so Cairo samples a smaller
    -- texture each frame. draw_scale converts native pixels to the render
    -- surface (paint at sprite_scale * draw_scale).
    local render_surf, render_scale = make_render_surface(
        surf, sw, sh, sprite.max_render_dim or 0)

    -- Tight bbox of opaque texels, scanned in COLLISION (downscaled) space and
    -- mapped back to native pixels.
    local bw, bh, bx0, by0 = sw, sh, 0, 0
    -- Collision-space bbox, reused by build_poly (which samples the native scan
    -- at collision resolution via col_alpha). Defaults cover the no-alpha and
    -- no-downscale cases so build_poly always sees valid bounds.
    local col_bx0, col_by0, col_bw, col_bh, col_cx, col_cy =
        0, 0, col_w, col_h, col_w / 2, col_h / 2
    if col_alpha then
        local minx, miny, maxx, maxy = col_w - 1, col_h - 1, -1, -1
        for y = 0, col_h - 1 do
            for x = 0, col_w - 1 do
                if col_alpha(x, y) > sprite.alpha_threshold then
                    if x < minx then minx = x end
                    if x > maxx then maxx = x end
                    if y < miny then miny = y end
                    if y > maxy then maxy = y end
                end
            end
        end
        if maxx >= 0 then
            -- col_scale < 1 when downscaled: invert to map collision -> native.
            local inv = col_scale > 0 and (1 / col_scale) or 1
            col_bx0, col_by0 = minx, miny
            col_bw = maxx - minx + 1
            col_bh = maxy - miny + 1
            col_cx = minx + col_bw / 2
            col_cy = miny + col_bh / 2
            -- Map the collision-space bbox back to native integer pixels. The
            -- start rounds to nearest; the end is ceiling-rounded so the mapped
            -- bbox never shrinks the (downscaled) silhouette. For col_scale == 1
            -- (no downscale, the default for ordinary sprites) the rounding is a
            -- no-op and these reduce to the exact integer bbox the scan found,
            -- so bbox-exact behaviour is unchanged. The clamp afterwards pins
            -- a full-image bbox (blahaj, no transparent border) to sw x sh.
            bx0 = math.floor(minx * inv + 0.5)
            by0 = math.floor(miny * inv + 0.5)
            bw = math.ceil((minx + col_bw) * inv) - bx0
            bh = math.ceil((miny + col_bh) * inv) - by0
            -- Clamp to the native surface: rounding/downscale blur can push a
            -- mapped coordinate just past the edge, and a full-image bbox
            -- (blahaj, no transparent border) must map back exactly to sw/sh.
            if bx0 < 0 then bx0 = 0 end
            if by0 < 0 then by0 = 0 end
            if bx0 + bw > sw then bw = sw - bx0 end
            if by0 + bh > sh then bh = sh - by0 end
            if bw <= 0 or bh <= 0 then
                return nil, "sprite bbox mapped out of bounds"
            end
        else
            -- No opaque texel: there is no meaningful collision body, so let
            -- the caller fall back to a coloured box rather than silently
            -- using the full image rectangle.
            return nil, "sprite has no opaque pixels"
        end
    end

    -- Bbox centre (native surface pixels): the rotation pivot when painted.
    local cx = bx0 + bw / 2
    local cy = by0 + bh / 2

    -- Convex hull (Andrew's monotone chain) of the opaque pixels' corners,
    -- mapped to local body coordinates: origin at the bbox centre, units
    -- already multiplied by `scale` (the same space the physics body uses).
    -- This is the sprite's DOP: a tight CONVEX polygon whose edges are the
    -- silhouette's own edge directions, used for BOTH the collision shape
    -- (physics.sat projects polygon edge normals) and the window shape mask
    -- (so no black pseudo-transparency background bleeds around the figure).
    --
    -- Limitation: concave sprites (transparent gaps between limbs, ears,
    -- etc.) are bridged by the convex hull. The visual mask still uses the
    -- exact alpha channel; only the physics collision shape is convex.
    local com_x, com_y = 0, 0  -- hull centroid in the bbox-centre frame
    local function build_poly()
        -- Only silhouette BOUNDARY pixels matter for the convex hull; interior
        -- opaque pixels can never be on the hull, so skipping them cuts the
        -- input size (and the sort cost) dramatically. All sampling happens in
        -- COLLISION (downscaled) space via col_alpha, which samples the native
        -- scan, so a huge sprite is scanned at collision resolution rather than
        -- native.
        local xlo, xhi = col_bx0, col_bx0 + col_bw - 1
        local ylo, yhi = col_by0, col_by0 + col_bh - 1
        local function is_opaque(x, y)
            if x < xlo or y < ylo or x > xhi or y > yhi then
                return false
            end
            return col_alpha(x, y) > sprite.alpha_threshold
        end
        local function is_boundary(x, y)
            return not is_opaque(x - 1, y)
                or not is_opaque(x + 1, y)
                or not is_opaque(x, y - 1)
                or not is_opaque(x, y + 1)
        end
        local pts = {}
        -- Deduplicate the corner points before sorting. Adjacent boundary
        -- pixels share corners, so a silhouette of B boundary pixels emits up
        -- to 4B points of which only ~B are distinct; the duplicates cost a
        -- larger table.sort (the dominant term here) and buy nothing, since a
        -- repeated point can never change a convex hull. Keyed on the integer
        -- lattice in collision space.
        local seen = {}
        local key_stride = col_bw + 2
        local function add_point(ix, iy, px, py)
            local key = (ix - col_bx0) + (iy - col_by0) * key_stride
            if not seen[key] then
                seen[key] = true
                pts[#pts + 1] = { x = px, y = py }
            end
        end
        -- Map a collision-space pixel to the body's coordinate frame:
        --   native   = collision / col_scale
        --   body     = (native - cx) * scale = (collision - col_cx) * (scale / col_scale)
        -- For unscaled sprites (col_scale == 1) this reduces to the original
        -- (x - cx) * scale, leaving native behaviour unchanged.
        local col_to_body = col_scale > 0 and (scale / col_scale) or scale
        for y = col_by0, col_by0 + col_bh - 1 do
            for x = col_bx0, col_bx0 + col_bw - 1 do
                if col_alpha(x, y) > sprite.alpha_threshold and is_boundary(x, y) then
                    local x0 = (x - col_cx) * col_to_body
                    local y0 = (y - col_cy) * col_to_body
                    local x1 = (x + 1 - col_cx) * col_to_body
                    local y1 = (y + 1 - col_cy) * col_to_body
                    add_point(x,     y,     x0, y0)
                    add_point(x + 1, y,     x1, y0)
                    add_point(x,     y + 1, x0, y1)
                    add_point(x + 1, y + 1, x1, y1)
                end
            end
        end
        if #pts < 4 then return nil end
        table.sort(pts, function(a, b)
            return a.x < b.x or (a.x == b.x and a.y < b.y)
        end)
        local function cross(o, a, b)
            return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
        end
        local lower, upper = {}, {}
        for _, p in ipairs(pts) do
            while #lower >= 2 and cross(lower[#lower - 1], lower[#lower], p) <= 0 do
                table.remove(lower)
            end
            lower[#lower + 1] = p
        end
        for i = #pts, 1, -1 do
            local p = pts[i]
            while #upper >= 2 and cross(upper[#upper - 1], upper[#upper], p) <= 0 do
                table.remove(upper)
            end
            upper[#upper + 1] = p
        end
        table.remove(upper)
        table.remove(lower)
        for i = 1, #upper do
            lower[#lower + 1] = upper[i]
        end
        if #lower < 3 then return nil end
        -- Recentre the hull on its area centroid: the physics body should
        -- pivot on its centre of mass (gravity torque, torque arms and
        -- stable resting angles are then all consistent), so the hull is
        -- shifted to make the centroid the origin. The shift is captured so
        -- the paint closure can offset the sprite by the same amount -- the
        -- bbox centre then sits at (-com_x, -com_y) in the body frame.
        com_x, com_y = poly_centroid(lower)
        for _, p in ipairs(lower) do
            p.x = p.x - com_x
            p.y = p.y - com_y
        end
        return lower
    end

    -- Paint the sprite onto `cr`, centred on its bbox centre.
    -- The caller is expected to have already translated to the body centre and
    -- rotated by the body angle (and to have clipped to the bbox if the
    -- transparent border should be masked out); this just drops the textured
    -- pixels in place at the sprite's stored scale. NEAREST filtering keeps
    -- pixel-art crisp when scaled up (no bilinear smudging).
    --
    -- The body pivot is the hull's centroid, not the bbox centre: the sprite
    -- must be painted with its bbox centre offset by (-com_x, -com_y) in the
    -- body frame so the figure tracks the COM-centred body correctly.
    --
    -- Paint from the render LOD surface (`render_surf`): it is render_scale x
    -- smaller than the native surface, so counter-scale to cover the same
    -- on-screen extent. render_scale == 1 for non-downscaled sprites, leaving
    -- behaviour identical to painting the native surface.
    local function paint(cr)
        cr:save()
        -- Replace the destination pixels (including transparent ones) with the
        -- source, instead of the default OVER blend. Because paint() covers
        -- the whole surface and the source pattern uses EXTEND_NONE,
        -- transparent sprite texels (and everything outside the sprite) ERASE
        -- the previous frame in one pass -- no separate CLEAR, no white edge
        -- flash, no "inverse ghosting".
        cr:set_operator(cairo.Operator.SOURCE)
        cr:translate(-com_x, -com_y)
        cr:translate(-cx * scale, -cy * scale)
        cr:scale(scale / render_scale, scale / render_scale)
        cr:set_source_surface(render_surf, 0, 0)
        local pattern = cr:get_source()
        if pattern then
            -- Outside the sprite surface must be transparent, never repeated
            -- or padded, or the edge texels would smear into a halo.
            pattern:set_extend(cairo.Extend.NONE)
            pattern:set_filter(sprite.render_filter
                or (render_scale < 1 and cairo.Filter.BILINEAR
                     or cairo.Filter.NEAREST))
        end
        cr:paint()
        cr:restore()
    end

    -- Collision LOD phase 2: skip the hull for dense bboxes (fall back to the
    -- box body the caller already has) and cap the vertex count of the hull
    -- we do keep, so SAT cost is bounded regardless of native sprite size.
    local poly
    if col_alpha then
        local hull_pixels = bw * bh
        local want_hull = hull_pixels <= (sprite.max_hull_pixels or math.huge)
        if want_hull then
            poly = build_poly()
            if poly and (sprite.max_hull_verts or 0) > 0
               and #poly > sprite.max_hull_verts then
                poly = decimate_convex_hull(poly, sprite.max_hull_verts)
                if #poly < 3 then
                    poly = nil
                else
                    -- build_poly() recentred the FULL hull on its centroid
                    -- (com_x/com_y) and shifted its points to match. Decimating
                    -- subsets vertices, which can move the centroid, so recompute
                    -- the decimated hull's centroid and re-centre it, keeping
                    -- com_x/com_y in sync with the rendered/pivoted hull.
                    local dx, dy = poly_centroid(poly)
                    if dx ~= 0 or dy ~= 0 then
                        com_x = com_x + dx
                        com_y = com_y + dy
                        for _, p in ipairs(poly) do
                            p.x = p.x - dx
                            p.y = p.y - dy
                        end
                    end
                end
            end
        end
    end

    return {
        surface = surf,
        sw = sw, sh = sh,
        bw = bw * scale, bh = bh * scale,   -- the body's collision extents
        cx = cx, cy = cy,                   -- bbox centre (native pixels)
        com_x = com_x, com_y = com_y,       -- hull centroid in bbox-centre
                                            -- frame (the shift applied to the
                                            -- hull; the bbox centre then sits
                                            -- at -com in the body frame)
        scale = scale,
        poly = poly,   -- DOP hull: {x,y}[] in body coords, centred on the COM
        paint = paint,
    }
end

-- Cached form of sprite.load, keyed on path + scale. Decoding a PNG and
-- rebuilding its hull is O(pixels) and happens on the main loop, so the menu's
-- "probe at scale 1, then load at the target scale" pattern used to pay for the
-- same decode twice per swap, and swapping back to a previous sprite paid for
-- it all over again. Sprite objects are immutable once built (the paint closure
-- only reads the surface), so they are safe to share between actors.
--
-- The cache holds `sprite.cache_max` entries; when it is full the whole table
-- is dropped rather than tracking LRU order, which keeps the bookkeeping free
-- and is perfectly adequate for a handful of sprites. Failures are not cached.
sprite.cache_max = 16
local cache, cache_n = {}, 0

function sprite.load_cached(path, scale)
    if type(path) ~= "string" then
        return sprite.load(path, scale)
    end
    local key = path .. "@" .. tostring(scale or 1)
    local hit = cache[key]
    if hit then return hit end

    local sp, err = sprite.load(path, scale)
    if not sp then return nil, err end

    if cache_n >= sprite.cache_max then
        cache, cache_n = {}, 0
    end
    cache[key] = sp
    cache_n = cache_n + 1
    return sp
end

-- Drop every cached sprite (e.g. after the sprite directory changed on disk).
function sprite.clear_cache()
    cache, cache_n = {}, 0
end

return sprite
