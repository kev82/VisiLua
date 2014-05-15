VisiLua
=======

A Lua binding to the VisiLibity library (http://www.visilibity.org)

This is still under development, and everything is still subject to complete
redesign. The front interface is non existant, but the C objects/functions
can be accessed in the internalfuncs-v01 subtable. This will be removed once
a proper interface has nbeen created.

The following demonstrates some of the functionallity

--------------------------

local visilua = require("visilua").internalfuncs_v01

local pc = visilua.newPolyConstructor()
pc(0,1)
pc(1,1)
pc(1,0)
pc(0,0)
local poly1 = pc()

pc = visilua.newPolyConstructor()
pc(0.25,0.75)
pc(0.25,0.25)
pc(0.75,0.25)
pc(0.75,0.75)
local poly2 = pc()

pc = visilua.newPolyConstructor()
pc(0.15,0.65)
pc(0.15,0.15)
pc(0.65,0.15)
pc(0.65,0.65)
local poly3 = pc()

print(poly1.area)
print(poly2.area)

print(poly1.bb_left)

print(poly1:epContainsPoint(0.001, 0.1, 0.5))
print(poly2:epContainsPoint(0.001, 0.1, 0.5))

local ed = visilua.newEnvDefinition({
 boundary = poly1,
 holes = {poly2}})

local env = visilua.newEpsilonEnv(ed, 0.000001)
local vp = env:visibilityPolygon(0.1, 0.1)

print(vp:epContainsPoint(0.001, 0, 0))
print(vp:epContainsPoint(0.001, 1, 1))

local path = env:shortestPath(0.1, 0.1, 0.0, 0.0)
print(#path, path:pathLength())

path = env:shortestPath(0.1, 0.1, 0.9, 0.9)
print(#path, path:pathLength())
 
