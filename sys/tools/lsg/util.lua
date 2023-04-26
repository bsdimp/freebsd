--
-- SPDX-License-Identifier: BSD-2-Clause
--
-- Copyright (c) 2023 Warner Losh <imp@bsdimp.com>
--

-- Derived in large part from makesyscalls.lua:
--
-- SPDX-License-Identifier: BSD-2-Clause-FreeBSD
--
-- Copyright (c) 2019 Kyle Evans <kevans@FreeBSD.org>

local util = {}

function util.trim(s, char)
	if s == nil then
		return nil
	end
	if char == nil then
		char = "%s"
	end
	return s:gsub("^" .. char .. "+", ""):gsub(char .. "+$", "")
end

-- Returns a table (list) of strings
function util.split(s, re)
	local t = { }

	for v in s:gmatch(re) do
		table.insert(t, v)
	end
	return t
end

function util.abort(status, msg)
	assert(io.stderr:write(msg .. "\n"))
	-- cleanup
	os.exit(status)
end

function util.Set(t)
	local s = { }
	for _,v in pairs(t) do s[v] = true end
	return s
end

function util.SetFromString(str, re)
	local s = { }

	for v in str:gmatch(re) do
		s[v] = true
	end
	return s
end

return util
