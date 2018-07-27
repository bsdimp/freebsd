# $FreeBSD$

# Common flags to build lua related files

LUA_CFLAGS+=	-I${LUASRC} -I${LDRSRC} -I${LIBLUASRC}
LUA_CFLAGS+=	-DLUA_FLOAT_TYPE=LUA_FLOAT_INT64
