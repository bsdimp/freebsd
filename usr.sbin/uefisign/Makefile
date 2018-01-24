# $FreeBSD$

PROG=	uefisign
SRCS=	uefisign.c child.c pe.c
MAN=	uefisign.8

LINKS=	${BINDIR}/uefisign ${BINDIR}/efisign
MLINKS=	uefisign.1 efisign.1

LIBADD=	crypto

WARNS=	6

.include <bsd.prog.mk>
