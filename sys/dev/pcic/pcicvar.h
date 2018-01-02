/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2003-2018 Warner Losh.
 * Copyright (c) 2000,2001 Jonathan Chen.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 */

/*
 * Structure definitions for the PCIC ExCA ISA driver
 */
struct pcic_softc;
struct pcic_intrhand {
	driver_filter_t	*filt;
	driver_intr_t	*intr;
	void 		*arg;
	struct pcic_softc *sc;
	void		*cookie;
};

struct pcic_reslist {
	SLIST_ENTRY(pcic_reslist) link;
	struct	resource *res;
	int	type;
	int	rid;
		/* note: unlike the regular resource list, there can be
		 * duplicate rid's in the same list.  However, the
		 * combination of rid and res->r_dev should be unique.
		 */
	bus_addr_t cardaddr; /* for 16-bit pccard memory */
};

struct pcic_softc {
	device_t	dev;
	struct exca_softc exca[EXCA_NSLOTS];
	struct		resource *base_res;
	struct		resource *irq_res;
	void		*intrhand;
	bus_space_tag_t bst;
	bus_space_handle_t bsh;
	struct mtx	mtx;
	int		cardok;
	uint32_t	flags;				/* mtx */
#define PCIC_CHANGE_MASK	0x0000000f
#define	PCIC_CHANGE_OFFSET	0
#define PCIC_POWER_MASK		0x000000f0
#define	PCIC_POWER_OFFSET	4
#define	PCIC_KTHREAD_RUNNING	0x40000000
#define	PCIC_KTHREAD_DONE	0x80000000
	SLIST_HEAD(, pcic_reslist) rl;
	struct proc	*event_thread;
	void (*chipinit)(struct pcic_softc *);
	struct root_hold_token *sc_root_token;
};

/* result of detect_card */
#define	CARD_UKN_CARD	0x00
#define	CARD_5V_CARD	0x01
#define	CARD_3V_CARD	0x02

/* for power_socket */
#define	CARD_VCC(X)	(X)
#define CARD_VPP_VCC	0xf0
#define CARD_VCCMASK	0xf
#define CARD_VCCSHIFT	0
#define XV		2
#define YV		1

#define CARD_OFF	(CARD_VCC(0))
