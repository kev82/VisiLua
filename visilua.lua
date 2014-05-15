--[[
 Copyright 2014 Kevin Martin

 This file is part of VisiLua

 VisiLua is free software: you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation, either
 version 3 of the License, or (at your option) any later version.
 
 VisiLua is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library.  If not, see <http://www.gnu.org/licenses/>.
--]]

local module = {
 _VERSION = "alpha-20140515-1734",
 _DESCRIPTION =
  "VisiLua: A Lua interface to VisiLibity (http://www.visilibity.org)",
 _URL = "http://github.com/kev82/VisiLua",
 _COPYRIGHT = "Copyright 2014 Kevin Martin",
 _LICENSE = [[LGPL Version 3 or later (see http://www.gnu.org/licenses]]
}

local clib = ...

--I haven't decided on the module interface, yet but we need to use it
--in order to solve a problem. To start, We will expose the current
--C-interface under internalfuncs_v01

local t = {}
for k, v in pairs(clib) do
	t[k] = v
end

module.internalfuncs_v01 = t

return module

