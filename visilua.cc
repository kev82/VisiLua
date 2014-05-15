/*
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
*/
#include <lua.hpp>
#include <cassert>

extern "C" {
int luaopen_visilua(lua_State *l);
}

namespace {

using namespace VisiLibity;

namespace polygon {

class userdata
{
public:
	const Polygon &ccw() const {
		if(ccw_ == nullptr) {
			ccw_ = new Polygon(*cw_);
			ccw_->reverse();
		}
		return *ccw_;
	}

	const Polygon &cw() const {
		if(cw_ == nullptr) {
			cw_ = new Polygon(*ccw_);
			cw_->reverse();
		}
		return *cw_;
	}

	const Polygon &any() const {
		if(cw_) return *cw_;
		return *ccw_;
	}

	userdata(Polygon *cw, Polygon *ccw) :
	 cw_(cw), ccw_(ccw) {
		assert( (cw_ != nullptr) || (ccw_ != nullptr) );
	}

	~userdata() {
		delete ccw_;
		delete cw_;
	}
private:
	mutable Polygon *cw_;
	mutable Polygon *ccw_;
};

int mtgc(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	userdata *pud = (userdata *)lua_touserdata(l, 1);
	assert(pud != NULL);

	try {
		pud->~userdata();
	} catch(std::exception &e) {
		return luaL_error(l, "Couldn't destruct polygon::userdata '%s'",
		 e.what());
	}

	return 1;
}

int calcBoundingBox(lua_State *l) {
	lua_settop(l, 2);	//[ut]
	luaL_checktype(l, 1, LUA_TUSERDATA);
	luaL_checktype(l, 2, LUA_TTABLE);

	try {
		userdata *pud = (userdata *)lua_touserdata(l, 1);
		assert(pud != NULL);

		Bounding_Box b = pud->any().bbox();

		lua_pushnumber(l, b.x_min);
		lua_setfield(l, -2, "bb_left");

		lua_pushnumber(l, b.x_max);
		lua_setfield(l, -2, "bb_right");

		lua_pushnumber(l, b.y_max);
		lua_setfield(l, -2, "bb_top");

		lua_pushnumber(l, b.y_min);
		lua_setfield(l, -2, "bb_bottom");
	} catch(std::exception &e) {
		return luaL_error(l, "polygon::calcBoundingBox caught '%s'", e.what());
	}

	return 0;
}

int mtindex(lua_State *l) {
	lua_settop(l, 2);	//[us]
	luaL_checktype(l, 1, LUA_TUSERDATA);
	luaL_checktype(l, 2, LUA_TSTRING);

	//is it a value we need to calculate
	lua_getuservalue(l, 1);	//[ust]
	assert(lua_type(l, -1) == LUA_TTABLE);

	lua_pushvalue(l, 2);	//[usts]
	lua_rawget(l, -2);	//[ust?]
	if(lua_type(l, -1) != LUA_TNIL) {
		return 1;
	}
	lua_pop(l, 1);	//[ust]

	//is it a function in the metatable
	lua_getmetatable(l, 1);	//[ustt]
	assert(lua_gettop(l) == 4);
	lua_pushvalue(l, 2);	//[ustts]
	lua_rawget(l, -2);	//[ustt?]
	if(lua_type(l, -1) != LUA_TNIL) {
		assert(lua_type(l, -1) == LUA_TFUNCTION);
		return 1;
	}
	lua_pop(l, 2);	//[ust]

	int wantbb = 0;
	wantbb += (strcmp(lua_tostring(l, 2), "bb_top") == 0);
	wantbb += (strcmp(lua_tostring(l, 2), "bb_bottom") == 0);
	wantbb += (strcmp(lua_tostring(l, 2), "bb_left") == 0);
	wantbb += (strcmp(lua_tostring(l, 2), "bb_right") == 0);

	if(wantbb) {
		lua_pushcfunction(l, calcBoundingBox);	//[ustf]
	}

	if(lua_type(l, -1) == LUA_TFUNCTION) {
		lua_pushvalue(l, 1);	//[ustfu]
		lua_pushvalue(l, 3);	//[ustfut]
		lua_call(l, 2, 0);	//[ust]
	}

	lua_pushvalue(l, 2);	//[usts]
	lua_rawget(l, -2);	//[ust?]
	return 1;
}

int mtEpsilonContainsPoint(lua_State *l) {
	lua_settop(l, 4);	//[unnn]
	luaL_checktype(l, 1, LUA_TUSERDATA);
	luaL_checktype(l, 2, LUA_TNUMBER);
	luaL_checktype(l, 3, LUA_TNUMBER);
	luaL_checktype(l, 4, LUA_TNUMBER);

	try {
		userdata *pud = (userdata *)lua_touserdata(l, 1);
		const Polygon &p = pud->any();

		double epsilon = lua_tonumber(l, 2);

		if(!p.is_simple(epsilon)) {
			//no objects to destroy, so can return
			return luaL_error(l, "Polygon isn't epsilon-simple");
		}

		Point p2(lua_tonumber(l, 3), lua_tonumber(l, 4));
		if(p2.in(p, epsilon)) {
			lua_pushboolean(l, 1);
		} else {
			lua_pushboolean(l, 0);
		}

		return 1;
	} catch(std::exception &e) {
		return luaL_error(l,
		 "polygon::mtEpsilonContainsPoint caught '%s'", e.what());
	}
}

int setMetatable(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	lua_rawgetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	if(lua_type(l, -1) == LUA_TNIL) {
		lua_pop(l, 1);
		
		lua_newtable(l);

		lua_pushcfunction(l, mtgc);
		lua_setfield(l, -2, "__gc");

		lua_pushcfunction(l, mtindex);
		lua_setfield(l, -2, "__index");

		lua_pushcfunction(l, mtEpsilonContainsPoint);
		lua_setfield(l, -2, "epContainsPoint");

		lua_pushvalue(l, -1);
		lua_rawsetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	}
	assert(lua_type(l, -1) == LUA_TTABLE);

	lua_setmetatable(l, 1);
	assert(lua_gettop(l) == 1);
	return 1;
}

bool is(lua_State *l, int idx) {
	idx = lua_absindex(l, idx);
	if(lua_type(l, idx) != LUA_TUSERDATA) return false;
	
	int havemeta = lua_getmetatable(l, idx);
	if(havemeta == 0) return false;

	lua_rawgetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	int equal = lua_compare(l, -1, -2, LUA_OPEQ);
	lua_pop(l, 2);

	return equal == 1;
}

int commonConstruct(lua_State *l) {
	lua_settop(l, 1);	//[u]
	luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);

	try {
		userdata *pud =
		 (userdata *)lua_newuserdata(l, sizeof(userdata));	//[uu]

		Polygon *cw = nullptr;
		Polygon *ccw = (Polygon *)lua_touserdata(l, 1);

		double polyarea = ccw->area();
		if(polyarea < 0) {
			std::swap(cw, ccw);
			polyarea *= -1;
		} 

		lua_newtable(l);	//[uut]
		lua_pushnumber(l, polyarea);	//[uutn]
		lua_setfield(l, -2, "area");	//[uut]
		lua_setuservalue(l, 2);	//[uu]

		new (pud) userdata(cw, ccw);
	} catch(std::exception &e) {
		return luaL_error(l, "polygon::commonConstruct caught '%s'",
		 e.what());
	}

	lua_remove(l, 1);	//[u]
	lua_pushcfunction(l, setMetatable);	//[uf]
	lua_insert(l, 1);	//[fu]
	lua_call(l, 1, 1);	//[u]
	return 1;
}

int constructVecPoint(lua_State *l) {
	lua_settop(l, 1);	//[u]
	luaL_checktype(l, 1, LUA_TUSERDATA);

	try {
		std::vector<Point> *pvp =
		 (std::vector<Point> *)lua_touserdata(l, 1);
		assert(pvp != NULL);

		Polygon *p = new Polygon(*pvp);
		lua_pop(l, 1);	//[]
		lua_pushlightuserdata(l, p);	//[u]
	} catch(std::exception &e) {
		return luaL_error(l, "polygon::constructVecPoint caught '%s'",
		 e.what());
	}

	lua_pushcfunction(l, commonConstruct);	//[uf]
	lua_insert(l, 1);	//[fu]
	lua_call(l, 1, 1);
	return 1;
}

}

namespace polycon {

int newVertex(lua_State *l) {
	lua_settop(l, 3);
	luaL_checktype(l, 1, LUA_TUSERDATA);
	luaL_checktype(l, 2, LUA_TNUMBER);
	luaL_checktype(l, 3, LUA_TNUMBER);

	std::vector<Point> *pvp = (std::vector<Point> *)lua_touserdata(l, 1);
	assert(pvp != NULL);
	try {
		std::vector<Point> &vp = *pvp;
		vp.push_back(Point(lua_tonumber(l, 2), lua_tonumber(l, 3)));
	} catch(std::exception &e) {
		return luaL_error(l, "polycon::newVertex caught '%s'", e.what());
	}

	return 0;
}

int genPolygon(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	lua_pushcfunction(l, polygon::constructVecPoint);
	lua_insert(l, 1);
	lua_call(l, 1, 1);

	return 1;
}

int mtCall(lua_State *l) {
	if(lua_gettop(l) > 1) {
		lua_pushcfunction(l, newVertex);
	} else {
		lua_pushcfunction(l, genPolygon);
	}
	lua_insert(l, 1);
	lua_call(l, lua_gettop(l)-1, LUA_MULTRET);
	return lua_gettop(l);
}

int mtgc(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);
	
	std::vector<Point> *pvp = (std::vector<Point> *)lua_touserdata(l, 1);
	assert(pvp != NULL);
	
	try {
		std::vector<Point> &vp = *pvp;
		vp.~vector<Point>();
	} catch(std::exception &e) {
		return luaL_error(l, "polycon::mtgc caught '%s'", e.what());
	}

	return 0;
}

int setMetatable(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	lua_rawgetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	if(lua_type(l, -1) == LUA_TNIL) {
		lua_pop(l, 1);
		
		lua_newtable(l);

		lua_pushcfunction(l, mtgc);
		lua_setfield(l, -2, "__gc");

		lua_pushcfunction(l, mtCall);
		lua_setfield(l, -2, "__call");

		lua_pushvalue(l, -1);
		lua_rawsetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	}
	assert(lua_type(l, -1) == LUA_TTABLE);

	lua_setmetatable(l, 1);
	assert(lua_gettop(l) == 1);
	return 1;
}

int construct(lua_State *l) {
	lua_settop(l, 0);

	void *ud = lua_newuserdata(l, sizeof(std::vector<Point>));
	assert(ud != NULL);

	try {
		new (ud) std::vector<Point>();
	} catch(std::exception &e) {
		return luaL_error(l,
		 "Unable to use placement new in polycon::construct '%s'",
		 e.what());
	}

	//Can't set this metatable until we know the the vector has
	//been properly constructed, else, the destructor call will
	//crash
	lua_pushcfunction(l, setMetatable);
	lua_insert(l, 1);
	lua_call(l, 1, 1);

	return 1;
}

}

namespace envdef {

int mtgc(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	try {
		Environment *pe = (Environment *)lua_touserdata(l, 1);
		assert(pe != nullptr);
		pe->~Environment();
	} catch(std::exception &e) {
		return luaL_error(l, "envdef::mtgc caught '%s'", e.what());
	}
}

int setMetatable(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	lua_rawgetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	if(lua_type(l, -1) == LUA_TNIL) {
		lua_pop(l, 1);
		
		lua_newtable(l);

		lua_pushcfunction(l, mtgc);
		lua_setfield(l, -2, "__gc");

		lua_pushvalue(l, -1);
		lua_rawsetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	}
	assert(lua_type(l, -1) == LUA_TTABLE);

	lua_setmetatable(l, 1);
	assert(lua_gettop(l) == 1);
	return 1;
}

bool is(lua_State *l, int idx) {
	idx = lua_absindex(l, idx);
	if(lua_type(l, idx) != LUA_TUSERDATA) return false;
	
	int havemeta = lua_getmetatable(l, idx);
	if(havemeta == 0) return false;

	lua_rawgetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	int equal = lua_compare(l, -1, -2, LUA_OPEQ);
	lua_pop(l, 2);

	return equal == 1;
}

int construct(lua_State *l) {
	class LuaError
	{
	private:
		std::string error_;
	public:
		LuaError(const std::string &e) : error_(e) {}
		const std::string &what() const { return error_; }
	};

	lua_settop(l, 1);	//[t]
	luaL_checktype(l, 1, LUA_TTABLE);	

	Environment *pe =
	 (Environment *)lua_newuserdata(l, sizeof(Environment));	//[tu]

	try {
		std::vector<Polygon> def;

		lua_getfield(l, 1, "boundary");	//[tu?]
		if(!polygon::is(l, -1)) {
			throw LuaError("Boundary should be a polygon");
		}
		def.push_back( ((polygon::userdata *)lua_touserdata(l, -1))->ccw() );
		lua_pop(l, 1);	//[tu]

		lua_getfield(l, 1, "holes");	//[tu?]
		if(lua_type(l, -1) != LUA_TTABLE) {
			throw LuaError("Holes should be a table");
		}
		int holes = lua_rawlen(l, -1);
		for(int i=1;i<=holes;++i) {
			lua_rawgeti(l, -1, i);	//[tut?]
			if(!polygon::is(l, -1)) {
				throw LuaError("Each hole should be a polygon");
			}

			def.push_back(
			 ((polygon::userdata *)lua_touserdata(l, -1))->cw() );
			lua_pop(l, 1);	//[tut]
		}
		lua_pop(l, 1);	//[tu]

		new (pe) Environment(def);
	} catch(LuaError &le) {
		lua_pushstring(l, le.what().c_str());
		return lua_error(l);
	} catch(std::exception &e) {
		return luaL_error(l, "envdef::construct caught '%s'", e.what());
	}

	lua_pushcfunction(l, setMetatable);	//[tuf]
	lua_insert(l, 2);	//[tfu]
	lua_call(l, 1, 1);	//[tu]
	return 1;
}

}

namespace polyline {

int mtgc(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	try {
		Polyline *pp = (Polyline *)lua_touserdata(l, 1);
		assert(pp != nullptr);
		pp->~Polyline();
	} catch(std::exception &e) {
		return luaL_error(l, "polyline::mtgc caught '%s'", e.what());
	}
}

int mtLen(lua_State *l) {
	lua_settop(l, 1);	//[u]
	luaL_checktype(l, 1, LUA_TUSERDATA);

	try {
		Polyline *pp = (Polyline *)lua_touserdata(l, 1);
		assert(pp != nullptr);
		lua_pushinteger(l, pp->size());
		return 1;
	} catch(std::exception &e) {
		return luaL_error(l, "polyline::mtLen caught '%s'", e.what());
	}
}

int mtPathDist(lua_State *l) {
	lua_settop(l, 1);	//[u]
	luaL_checktype(l, 1, LUA_TUSERDATA);

	try {
		Polyline *pp = (Polyline *)lua_touserdata(l, 1);
		assert(pp != nullptr);
		lua_pushnumber(l, pp->length());
		return 1;
	} catch(std::exception &e) {
		return luaL_error(l, "polyline::mtPathDist caught '%s'", e.what());
	}
}

int setMetatable(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	lua_rawgetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	if(lua_type(l, -1) == LUA_TNIL) {
		lua_pop(l, 1);
		
		lua_newtable(l);

		lua_pushvalue(l, -1);
		lua_setfield(l, -2, "__index");

		lua_pushcfunction(l, mtgc);
		lua_setfield(l, -2, "__gc");

		lua_pushcfunction(l, mtLen);
		lua_setfield(l, -2, "__len");

		lua_pushcfunction(l, mtPathDist);
		lua_setfield(l, -2, "pathLength");

		lua_pushvalue(l, -1);
		lua_rawsetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	}
	assert(lua_type(l, -1) == LUA_TTABLE);

	lua_setmetatable(l, 1);
	assert(lua_gettop(l) == 1);
	return 1;
}

int construct(lua_State *l) {
	lua_settop(l, 0);	//[]

	Polyline *p =
	 (Polyline *)lua_newuserdata(l, sizeof(Polyline));	//[u]

	try {
		new (p) Polyline;
	} catch(std::exception &e) {
		return luaL_error(l, "polyline::construct caught '%s'", e.what());
	}

	lua_pushcfunction(l, setMetatable);	//[uf]
	lua_insert(l, 1);	//[fu]
	lua_call(l, 1, 1);	//[u]
	return 1;
}

}

namespace epsenv {

class userdata
{
public:
	const Environment *env;
	int envidx;

	double epsilon;

	const Visibility_Graph *graph;

	userdata(const Environment *env, int envidx,
	 double eps, const Visibility_Graph *vg) :
	 env(env), envidx(envidx), epsilon(eps), graph(vg) {}

	~userdata() {
		delete graph;
	}
};

int mtgc(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	try {
		userdata *pud = (userdata *)lua_touserdata(l, 1);
		assert(pud != nullptr);

		luaL_unref(l, LUA_REGISTRYINDEX, pud->envidx);
		pud->~userdata();
	} catch(std::exception &e) {
		return luaL_error(l, "epsenv::mtgc caught '%s'", e.what());
	}
}

int mtVisibilityPolygon(lua_State *l) {
	lua_settop(l, 3);	//[unn]
	luaL_checktype(l, 1, LUA_TUSERDATA);
	luaL_checktype(l, 2, LUA_TNUMBER);
	luaL_checktype(l, 3, LUA_TNUMBER);

	Polygon *vp = nullptr;
	try {
		userdata *pud = (userdata *)lua_touserdata(l, 1);

		Point p(lua_tonumber(l, 2), lua_tonumber(l, 3));
		p.snap_to_boundary_of(*(pud->env), pud->epsilon);
		p.snap_to_vertices_of(*(pud->env), pud->epsilon);

		vp = new Visibility_Polygon(p, *(pud->env), pud->epsilon);
	} catch(std::exception &e) {
		return luaL_error(l, "epsenv::mtVisibilityPolygon caught '%s'",
		 e.what());
	}

	lua_pushcfunction(l, polygon::commonConstruct);
	lua_pushlightuserdata(l, vp);
	lua_call(l, 1, 1);
	return 1;
}

int mtShortestPath(lua_State *l) {
	class LuaError
	{
	private:
		std::string error_;
	public:
		LuaError(const std::string &e) : error_(e) {}
		const std::string &what() const { return error_; }
	};

	lua_settop(l, 5);	//[unnnn]
	luaL_checktype(l, 1, LUA_TUSERDATA);
	luaL_checktype(l, 2, LUA_TNUMBER);
	luaL_checktype(l, 3, LUA_TNUMBER);
	luaL_checktype(l, 4, LUA_TNUMBER);
	luaL_checktype(l, 5, LUA_TNUMBER);

	try {
		userdata *pud = (userdata *)lua_touserdata(l, 1);

		if(!pud->graph) {
			pud->graph = new Visibility_Graph(*(pud->env), pud->epsilon);
		}

		Point start(lua_tonumber(l, 2), lua_tonumber(l, 3));
		Point finish(lua_tonumber(l, 4), lua_tonumber(l, 5));

		if(!start.in(*(pud->env), pud->epsilon)) {
			throw LuaError("Start is not in environment");
		}
		if(!finish.in(*(pud->env), pud->epsilon)) {
			throw LuaError("Finish is not in environment");
		}

		Polyline p = pud->env->shortest_path(
		 start, finish, *(pud->graph), pud->epsilon);

		lua_pushcfunction(l, polyline::construct);
		lua_call(l, 0, 1);
		Polyline *pp = (Polyline *)lua_touserdata(l, -1);
		*pp = std::move(p);

		return 1;
	} catch(std::exception &e) {
		return luaL_error(l, "epsenv::mtShortestPath caught '%s'",
		 e.what());
	}
}

int setMetatable(lua_State *l) {
	lua_settop(l, 1);
	luaL_checktype(l, 1, LUA_TUSERDATA);

	lua_rawgetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	if(lua_type(l, -1) == LUA_TNIL) {
		lua_pop(l, 1);
		
		lua_newtable(l);

		lua_pushcfunction(l, mtgc);
		lua_setfield(l, -2, "__gc");

		lua_pushvalue(l, -1);
		lua_setfield(l, -2, "__index");

		lua_pushcfunction(l, mtVisibilityPolygon);
		lua_setfield(l, -2, "visibilityPolygon");

		lua_pushcfunction(l, mtShortestPath);
		lua_setfield(l, -2, "shortestPath");

		lua_pushvalue(l, -1);
		lua_rawsetp(l, LUA_REGISTRYINDEX, (void *)setMetatable);
	}
	assert(lua_type(l, -1) == LUA_TTABLE);

	lua_setmetatable(l, 1);
	assert(lua_gettop(l) == 1);
	return 1;
}

int construct(lua_State *l) {
	class LuaError
	{
	private:
		std::string error_;
	public:
		LuaError(const std::string &e) : error_(e) {}
		const std::string &what() const { return error_; }
	};

	lua_settop(l, 2);	//[un]
	luaL_checktype(l, 1, LUA_TUSERDATA);
	luaL_checktype(l, 2, LUA_TNUMBER);

	if(!envdef::is(l, 1)) {
		return luaL_error(l, "Expected Environment Definition");
	}

	userdata *pud =
	 (userdata *)lua_newuserdata(l, sizeof(userdata));	//[unu]

	try {
		Environment *env = (Environment *)lua_touserdata(l, 1);
		double eps = lua_tonumber(l, 2);

		if(!env->is_valid(eps)) {
			throw LuaError("Environment Definition is not Epsilon valid");
		}

		lua_pushvalue(l, 1);	//[unuu]
		int envidx = luaL_ref(l, LUA_REGISTRYINDEX);	//[unu]

		new (pud) userdata(env, envidx, eps, nullptr);
	} catch(LuaError &le) {
		lua_pushstring(l, le.what().c_str());
		return lua_error(l);
	} catch(std::exception &e) {
		return luaL_error(l, "epsenv::construct caught '%s'", e.what());
	}

	lua_insert(l, 1);	//[uun]
	lua_pop(l, 2);	//[u]

	lua_pushcfunction(l, setMetatable);	//[uf]
	lua_insert(l, 1);	//[fu]
	lua_call(l, 1, 1);	//[u]
	return 1;
}

}

}

static luaL_Reg modfuncs[] = {
 {"newPolyConstructor", polycon::construct},
 {"newEnvDefinition", envdef::construct},
 {"newEpsilonEnv", epsenv::construct},
 {NULL, NULL}
};

int luaopen_visilua(lua_State *l) {
	int rc = luaL_loadbufferx(l, (const char *)vl_code,
	 sizeof(vl_code), "visilua", "b");
	if(rc != LUA_OK) {
		return luaL_error(l,
		 "Unable to load lua part '%s'", lua_tostring(l, 1));
	}

	luaL_newlib(l, modfuncs);
	lua_call(l, 1, 1);
	return 1;
}


	
