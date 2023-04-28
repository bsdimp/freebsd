#!/usr/libexec/flua
--
-- SPDX-License-Identifier: BSD-2-Clause
--
-- Copyright (c) 2023 Warner Losh <imp@bsdimp.com>
--
--
-- Thanks to Kyle Evans for his makesyscall.lua in FreeBSD which served as
-- inspiration for this, and as a source of code at times.
--
-- SPDX-License-Identifier: BSD-2-Clause-FreeBSD
--
-- Copyright (c) 2019 Kyle Evans <kevans@FreeBSD.org>
--

-- We generally assume that this script will be run by flua, however we've
-- carefully crafted modules for it that mimic interfaces provided by modules
-- available in ports.  Currently, this script is compatible with lua from
-- ports along with the compatible luafilesystem and lua-posix modules.

-- When we have a path, add it to the package.path (. is already in the list)
if arg[0]:match("/") then
	local a = arg[0]:gsub("/[^/]+.lua$", "")
	package.path = package.path .. ";" .. a .. "/?.lua"
end

-- The FreeBSD syscall generator
local FreeBSDSyscall = require("freebsd-syscall")

local config = require("config")		-- Common config file mgt

local generated_tag = "@" .. "generated"

-- Globals

-- Default configuration; any of these may get replaced by a configuration file
-- optionally specified. A lot of these are passed into the fbsd_sys parser and
-- the bsd_user code generator A bit tricky because a lot of the inherited code
-- has a global config table that it referrs to deep in the call tree... need to
-- make sure that all that code is converted to using one local to the object.
local cfg = {
	syscallprefix = "SYS_"
}

-- Libc has all the STD, NOSTD and SYSMUX system calls in it, as well as
-- replaced system calls dating back to FreeBSD 7. We are lucky that the
-- system call filename is just the base symbol name for it.
local function gen_syscalls_mk(tbl, cfg)
	local s = tbl.syscalls
	local max = 0

	print(string.format([[/*
 * System call numbers.
 *
 * DO NOT EDIT-- this file is automatically %s.
 */
]], generated_tag))

	for k, v in pairs(s) do
		local c = v:compat_level()
		if v.num > max then
			max = v.num
		end
		if      v.type.STD or
			v.type.NOSTD or
			v.type.SYSMUX or
			c >= 7 then
			print(string.format("#define\t%s%s\t%d", cfg.syscallprefix, v:symbol(), v.num))
		elseif c >= 0 then
			local s
			if c == 0 then
				s = "obsolete"
			elseif c == 3 then
				s = "old"
			else
				s = "freebsd" .. c
			end
			print(string.format("\t\t\t\t/* %d is %s %s */", v.num, s, v.name))
		elseif v.type.RESERVED then
			print(string.format("\t\t\t\t/* %d is reserved */", v.num))
		elseif v.type.UNIMP then
			print(string.format("\t\t\t\t/* %d is unimplemented %s */", v.num, v.name))
		else -- do nothing
		end
	end
	print(string.format("#define\t%sMAXSYSCALL\t%d", cfg.syscallprefix, max + 1))
end

-- Entry

if #arg < 1 or #arg > 2 then
	error("usage: " .. arg[0] .. " syscall.master")
end

local sysfile, configfile = arg[1], arg[2]

local cfg_mod = { }

-- process_config either returns nil and a message, or a table that we should
-- merge into the global config. XXX Seems like this should be in
-- config.something instead of bare code.
if configfile ~= nil then
	local res = assert(config.process(configfile))

	for k, v in pairs(res) do
		if v ~= cfg[k] then
			cfg[k] = v
			cfg_mod[k] = true
		end
	end
end

-- The parsed syscall table
local tbl = FreeBSDSyscall:new{sysfile = sysfile, config = cfg}

gen_syscalls_mk(tbl, cfg)
