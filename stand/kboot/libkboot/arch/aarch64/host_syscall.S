#include "syscall_nr.h"
#include <machine/asm.h>

/*
 * Emulate the Linux system call interface. System call number in x8.
 * Args in x0, x1, x2, x3, x4 and x5. Return in x0.
 */
ENTRY(host_syscall)
	mov	x8, x0
	mov	x0, x1
	mov	x1, x2
	mov	x2, x3
	mov	x3, x4
	mov	x4, x5
	mov	x5, x6
	svc	0
	ret
/* Note: We're exposing the raw return value to the caller */
END(host_syscall)

ENTRY(__rt_restore)
	mov	x8, #SYS_rt_sigreturn
	svc	0
END(__rt_restore)
