# Makefile for syscall tables
#
# $FreeBSD$

# Don't use an OBJDIR
.OBJDIR: ${.CURDIR}

sysent:	v7_sysent.c v7_syscall.h v7_proto.h

v7_sysent.c v7_syscall.h v7_proto.h: ../../kern/makesyscalls.sh \
		syscalls.master syscalls.conf
	sh ../../kern/makesyscalls.sh syscalls.master syscalls.conf
