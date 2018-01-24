# $FreeBSD$

PROG=	efivar
MAN=	efivar.8

LIBADD= efivar geom

SRCS=	efivar.c efiutil.c

EFIBOOT=${SRCTOP}/stand/efi
CFLAGS+=	-I${EFIBOOT}/include

LINKS=	${BINDIR}/efivar ${BINDIR}/uefivar
MLINKS=	efivar.8 uefivar.8

.include <bsd.prog.mk>
