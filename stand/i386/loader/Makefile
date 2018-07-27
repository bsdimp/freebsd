# $FreeBSD$

HAVE_ZFS=		${MK_ZFS}

LOADER_NET_SUPPORT?=	yes
LOADER_NFS_SUPPORT?=	yes
LOADER_TFTP_SUPPORT?=	yes
LOADER_CD9660_SUPPORT?=	yes
LOADER_EXT2FS_SUPPORT?=	yes
LOADER_MSDOS_SUPPORT?=	yes
LOADER_UFS_SUPPORT?=	yes
LOADER_GZIP_SUPPORT?=	yes
LOADER_BZIP2_SUPPORT?=	yes

.include <bsd.init.mk>

LOADER?=	loader
INTERNALPROG=
NEWVERSWHAT?=	"bootstrap loader" x86
VERSION_FILE=	${.CURDIR}/../loader/version

.PATH:		${BOOTSRC}/i386/loader

# architecture-specific loader code
SRCS=		main.c conf.c vers.c chain.c

# Include bcache code.
HAVE_BCACHE=	yes

# Enable PnP and ISA-PnP code.
HAVE_PNP=	yes
HAVE_ISABUS=	yes

.if ${MK_LOADER_FIREWIRE} == "yes"
CFLAGS+=	-DLOADER_FIREWIRE_SUPPORT
LIBFIREWIRE=	${BOOTOBJ}/i386/libfirewire/libfirewire.a
.endif

.if exists(${.CURDIR}/help.i386)
HELP_FILES=	${.CURDIR}/help.i386
.endif

# Always add MI sources
.include	"${BOOTSRC}/loader.mk"

CLEANFILES+=	${LOADER} ${LOADER}.bin

CFLAGS+=	-Wall
LDFLAGS+=	-static -Ttext 0x0

# i386 standalone support library
LIBI386=	${BOOTOBJ}/i386/libi386/libi386.a
CFLAGS+=	-I${BOOTSRC}/i386

# Debug me!
#CFLAGS+=	-g
#LDFLAGS+=	-g

.if defined(PROG)
SRCS+=${SRCS.${PROG}}
DPADD=${LIBS.${PROG}}
LDADD=${LIBS.${PROG}}
.if "${PROG}" == "${DEF_LANG:tl}_${LOADER}.sym"
LINKS= ${BINDIR}/${DEF_LANG:tl}_${LOADER} ${BINDIR}/${LOADER}
.endif
.endif

.for i in ${LDR_LANGS}
PROGS+=		${i:tl}_${LOADER}.sym
LIBS.${i:tl}_${LOADER}.sym=${${i}_LIBS32}
SRCS.${i:tl}_${LOADER}.sym=${${i}_SRCS}

${i:tl}_${LOADER}: ${i:tl}_${LOADER}.bin ${BTXLDR} ${BTXKERN}
	btxld -v -f aout -e ${LOADER_ADDRESS} -o ${.TARGET} -l ${BTXLDR} \
		-b ${BTXKERN} ${i:tl}_${LOADER}.bin

${i:tl}_${LOADER}.bin: ${i:tl}_${LOADER}.sym
	strip -R .comment -R .note -o ${.TARGET} ${.ALLSRC}

FILES+=	${i:tl}_${LOADER}
# XXX INSTALLFLAGS_loader= -b
FILESMODE_${i:tl}_${LOADER}= ${BINMODE} -b
.endfor

# XXX crt0.o needs to be first for pxeboot(8) to work
OBJS=	${BTXCRT}

DPADD+=	${LIBFIREWIRE} ${LIBI386} ${LIBSA32}
LDADD+=	${LIBFIREWIRE} ${LIBI386} ${LIBSA32}

.if ${MACHINE_CPUARCH} == "amd64"
CFLAGS+=	-DLOADER_PREFER_AMD64
.endif

.include <bsd.progs.mk>
