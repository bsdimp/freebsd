/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2002-2018 M. Warner Losh.
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
 */

/*
 * Driver for ISA to PCMCIA bridges compliant with the Intel ExCA
 * specification.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/proc.h>
#include <sys/condvar.h>
#include <sys/errno.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/kthread.h>
#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <isa/isavar.h>

#include <dev/pccard/pccardreg.h>
#include <dev/pccard/pccardvar.h>

#include <dev/exca/excareg.h>
#include <dev/exca/excavar.h>

#include <dev/pcic/pcicreg.h>
#include <dev/pcic/pcicvar.h>

#include "power_if.h"
#include "card_if.h"

/*****************************************************************************
 * Configurable parameters.
 *****************************************************************************/

/* sysctl vars */
static SYSCTL_NODE(_hw, OID_AUTO, pcic, CTLFLAG_RD, 0, "PCIC parameters");

static int isa_intr_mask = EXCA_INT_MASK_ALLOWED;
SYSCTL_INT(_hw_pcic, OID_AUTO, intr_mask, CTLFLAG_RDTUN, &isa_intr_mask, 0,
    "Mask of allowable interrupts for this laptop.  The default is generally"
    " correct, but some laptops do not route all the IRQ pins to the bridge to"
    " save wires.  Sometimes you need a more restrictive mask because some of"
    " the hardware in your laptop may not have a driver so its IRQ might not be"
    " allocated.");

devclass_t pcic_devclass;

static u_long pcic_start_mem = 0xa000;
SYSCTL_ULONG(_hw_pcic, OID_AUTO, start_memory, CTLFLAG_RWTUN,
    &pcic_start_mem, 0xa000,
    "Starting address for memory allocations");

static u_long pcic_start_io = 0x100;
SYSCTL_ULONG(_hw_pcic, OID_AUTO, start_16_io, CTLFLAG_RWTUN,
    &pcic_start_io, 0x100,
    "Starting ioport for 16-bit cards");

static int pcic_debug = 10;
SYSCTL_INT(_hw_pcic, OID_AUTO, debug, CTLFLAG_RWTUN, &pcic_debug, 10,
    "Verbose PCIC bridge debugging");

#define PCIC_MEMALIGN	0x400

/*****************************************************************************
 * End of configurable parameters.
 *****************************************************************************/

#define	DPRINTF(x) do { if (pcic_debug) printf x; } while (0)
#define	DEVPRINTF(x) do { if (pcic_debug) device_printf x; } while (0)

static struct isa_pnp_id pcic_ids[] = {
	{EXCA_PNP_ACTIONTEC,		NULL},		/* AEI0218 */
	{EXCA_PNP_IBM3765,		NULL},		/* IBM3765 */
	{EXCA_PNP_82365,		NULL},		/* PNP0E00 */
	{EXCA_PNP_CL_PD6720,		NULL},		/* PNP0E01 */
	{EXCA_PNP_VLSI_82C146,		NULL},		/* PNP0E02 */
	{EXCA_PNP_82365_CARDBUS,	NULL},		/* PNP0E03 */
	{EXCA_PNP_SCM_SWAPBOX,		NULL},		/* SCM0469 */
	{0}
};

static int pcic_func_filt(void *arg);
static void pcic_func_intr(void *arg);

static int
pcic_slot(device_t child)
{
	int slot = 0;

	exca_get_slot(child, &slot);
	return (slot);
}


/************************************************************************/
/* Probe/Attach								*/
/************************************************************************/

static int
pcic_activate(device_t dev)
{
	struct pcic_softc *sc = device_get_softc(dev);
	struct resource *res;
	int rid;
	int i;

	/* A little bogus, but go ahead and get the irq for CSC events */
	rid = 0;
	res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (res == NULL) {
		/*
		 * No IRQ specified, find one.  This can be due to the PnP
		 * data not specifying any IRQ, or the default kernel not
		 * assinging an IRQ.
		 */
		for (i = 0; i < 16 && res == NULL; i++) {
			if (((1 << i) & isa_intr_mask) == 0)
				continue;
			res = bus_alloc_resource(dev, SYS_RES_IRQ, &rid, i, i,
			    1, RF_ACTIVE);
		}
	}
	if (res == NULL)
		return (ENXIO);
	sc->irq_res = res;
	rid = 0;
	res = bus_alloc_resource_any(dev, SYS_RES_IOPORT, &rid, RF_ACTIVE);
	if (res == NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		sc->irq_res = NULL;
		device_printf(dev, "Cannot allocate I/O\n");
		return (ENOMEM);
	}
	sc->bst = rman_get_bustag(res);
	sc->bsh = rman_get_bushandle(res);
	sc->base_res = res;
	return (0);
}

static void
pcic_deactivate(device_t dev)
{
	struct pcic_softc *sc = device_get_softc(dev);

	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	sc->irq_res = NULL;
	if (sc->base_res)
		bus_release_resource(dev, SYS_RES_IOPORT, 0, sc->base_res);
	sc->base_res = NULL;
}

static int
pcic_probe(device_t dev)
{
	int error;
	struct pcic_softc *sc = device_get_softc(dev);

	/*
	 * Check isapnp ids, but this just rules out PNP devices quickly
	 * that we know we don't match.
	 */
	error = ISA_PNP_PROBE(device_get_parent(dev), dev, pcic_ids);
	if (error != 0 && error != ENOENT)
		return (error);

	/*
	 * Activate resoures so probe message has them all listed.
	 */
	error = pcic_activate(dev);
	if (error != 0)
		return (error);

	/*
	 * Check to make sure that we have actual hardware. We could skip
	 * this step if we have a good PNP ID for the device, but experience
	 * suggests caution is in order to guard against devices that return
	 * a bad PPN ID for ISA cards.
	 */
	error = exca_probe_slots(dev, &sc->exca[0], sc->bst, sc->bsh);
	pcic_deactivate(dev);
	return (error);
}

static void
pcic_event_thread(void *arg)
{
	struct pcic_softc *sc = arg;
	uint32_t status, changed;
	int err;
	int i;

	/*
	 * We need to act as a power sequencer on startup.  Delay 2s/channel
	 * to ensure the other channels have had a chance to come up.  We likely
	 * should add a lock that's shared on a per-slot basis so that only
	 * one power event can happen per slot at a time.
	 */
	pause("pcicstart", hz * device_get_unit(sc->dev) * 2);
	mtx_lock_spin(&sc->mtx);
	sc->flags |= PCIC_KTHREAD_RUNNING;
	for (i = 0; i < EXCA_NSLOTS; i++)
		if (sc->exca[i].chipset != EXCA_BOGUS)
			sc->flags |= (1u <<  (i + PCIC_CHANGE_OFFSET));
	while ((sc->flags & PCIC_KTHREAD_DONE) == 0) {
		changed = sc->flags & PCIC_CHANGE_MASK;
		sc->flags &= ~PCIC_CHANGE_MASK;
		mtx_unlock_spin(&sc->mtx);
		for (i = 0; i < EXCA_NSLOTS; i++) {
			if ((changed & (1u << (i + PCIC_CHANGE_OFFSET))) == 0)
				continue;
			status = exca_getb(&sc->exca[i], EXCA_IF_STATUS);
			if ((status & EXCA_IF_STATUS_CARDDETECT_MASK) ==
			    EXCA_IF_STATUS_CARDDETECT_PRESENT)
				exca_insert(sc, i);
			else
				exca_removal(sc, i);
		}

		/*
		 * First time through we need to tell mountroot that we're
		 * done.
		 */
		if (sc->sc_root_token) {
			root_mount_rel(sc->sc_root_token);
			sc->sc_root_token = NULL;
		}

		/*
		 * Wait until it has been 250ms since the last time we
		 * get an interrupt.  We handle the rest of the interrupt
		 * at the top of the loop.  Although we clear the bit in the
		 * ISR, we signal sc->cv from the detach path after we've
		 * set the CBB_KTHREAD_DONE bit, so we can't do a simple
		 * 250ms sleep here. --- XXXX not sure that we're doing this.
		 *
		 * In our ISR, we turn off the card changed interrupt.  Turn
		 * them back on here before we wait for them to happen.  We
		 * turn them on/off so that we can tolerate a large latency
		 * between the time we signal cbb_event_thread and it gets
		 * a chance to run.
		 */
		mtx_lock_spin(&sc->mtx);
		// XXX enable card change? -- I think for Edge triggered interrupts
		// XXX we don't need to worry about.
		msleep_spin(&sc->intrhand, &sc->mtx, "-", 0);
		err = 0;
		while (err != EWOULDBLOCK &&
		    (sc->flags & PCIC_KTHREAD_DONE) == 0)
			err = msleep_spin(&sc->intrhand, &sc->mtx, "-", hz / 5);
	}
	DEVPRINTF((sc->dev, "Thread terminating\n"));
	sc->flags &= ~PCIC_KTHREAD_RUNNING;
	mtx_unlock_spin(&sc->mtx);
	kproc_exit(0);
}

static int
pcic_filt(void *arg)
{
	struct pcic_softc *sc = arg;
	uint8_t csc;
	int i, retval = FILTER_STRAY;

	for (i = 0; i < EXCA_NSLOTS; i++) {
		/*
		 * Check for power changes...
		 */
		csc = exca_getb(&sc->exca[i], EXCA_CSC);
		mtx_lock_spin(&sc->mtx);
		if (csc & EXCA_CSC_READY) {
			sc->flags |= (1 << (i + PCIC_POWER_OFFSET));
			wakeup((void *)&sc->flags);
			retval = FILTER_HANDLED;
		}

		/*
		 * Make a note of which one has a change interrupt and signal the
		 * main pcic thread to cope.
		 */
		if (csc & EXCA_CSC_CD) {
			sc->flags |= (1 << (i + PCIC_CHANGE_OFFSET));
			wakeup(&sc->intrhand);
			retval = FILTER_HANDLED;
		}
		mtx_unlock_spin(&sc->mtx);
	}

	// XXX do we need to handle interrupt sharing here? I don't think so.

	return retval;
}

static int
pcic_attach(device_t brdev)
{
	struct pcic_softc *sc = (struct pcic_softc *)device_get_softc(brdev);
//	struct sysctl_ctx_list *sctx;
//	struct sysctl_oid *soid;
	device_t parent;
	int i;
	int error;// XXX error checking

	parent = device_get_parent(brdev);
	mtx_init(&sc->mtx, device_get_nameunit(brdev), "cbb", MTX_SPIN);
	sc->dev = brdev;
	SLIST_INIT(&sc->rl);

	error = pcic_activate(brdev);
	if (error != 0) {
		DEVPRINTF((brdev, "WARNING: Can't activate PCIC %d\n", error));
		return (error);
	}

	/* Check to make sure that we have actual hardware */
	error = exca_probe_slots(brdev, &sc->exca[0], sc->bst, sc->bsh);
	sc->bst = rman_get_bustag(sc->base_res);
	sc->bsh = rman_get_bushandle(sc->base_res);

	for (i = 0; i < EXCA_NSLOTS; i++) {
		printf("Checking slot %d chipset %d pccarddev = %p\n",
		    i, sc->exca[i].chipset, sc->exca[i].pccarddev);
		if (sc->exca[i].chipset != EXCA_BOGUS && sc->exca[i].pccarddev)
			device_set_ivars(sc->exca[i].pccarddev, &sc->exca[i]);
	}

#if 0
	/*Sysctls*/
	sctx = device_get_sysctl_ctx(brdev);
	soid = device_get_sysctl_tree(brdev);
	SYSCTL_ADD_UINT(sctx, SYSCTL_CHILDREN(soid), OID_AUTO, "domain",
	    CTLFLAG_RD, &sc->domain, 0, "Domain number");
	SYSCTL_ADD_UINT(sctx, SYSCTL_CHILDREN(soid), OID_AUTO, "pribus",
	    CTLFLAG_RD, &sc->pribus, 0, "Primary bus number");
	SYSCTL_ADD_UINT(sctx, SYSCTL_CHILDREN(soid), OID_AUTO, "secbus",
	    CTLFLAG_RD, &sc->bus.sec, 0, "Secondary bus number");
	SYSCTL_ADD_UINT(sctx, SYSCTL_CHILDREN(soid), OID_AUTO, "subbus",
	    CTLFLAG_RD, &sc->bus.sub, 0, "Subordinate bus number");
#endif

	for (i = 0; i < EXCA_NSLOTS; i++) {
		if (sc->exca[i].chipset == EXCA_BOGUS)
			continue;

		/* reset 16-bit pcmcia bus */
		exca_clrb(&sc->exca[i], EXCA_INTR, EXCA_INTR_RESET);

		/* turn off power */
		exca_reset(&sc->exca[i], sc->exca[i].pccarddev);
	}

	if (bus_setup_intr(brdev, sc->irq_res, INTR_TYPE_AV | INTR_MPSAFE,
	    pcic_filt, NULL, sc, &sc->intrhand)) {
		device_printf(brdev, "couldn't establish interrupt\n");
		return ENXIO;// XXX resource leakage
	}

	/* Start the thread */
	if (kproc_create(pcic_event_thread, sc, &sc->event_thread, 0, 0,
	    "%s event thread", device_get_nameunit(brdev))) {
		device_printf(brdev, "unable to create event thread.\n");
		panic("pcic_create_event_thread");
	}
	sc->sc_root_token = root_mount_hold(device_get_nameunit(sc->dev));
	return (0);
}

static int
pcic_detach(device_t dev)
{

	return (ENXIO);
}

static int
pcic_suspend(device_t dev)
{

	return (0);
}

static int
pcic_resume(device_t dev)
{

	return (0);
}

static int
pcic_read_ivar(device_t brdev, device_t child, int which, uintptr_t *result)
{
	struct exca_softc *exca = device_get_ivars(child);

	switch (which) {
	case EXCA_IVAR_SLOT:
		if (exca == NULL)
			return (EINVAL);
		*result = exca->slot;
		return (0);
	}

	return (ENOENT);
}

static int
pcic_write_ivar(device_t brdev, device_t child, int which, uintptr_t value)
{

	switch (which) {
	case EXCA_IVAR_SLOT:
		return (EINVAL);
	}

	return (ENOENT);
}

static void
pcic_insert_res(struct pcic_softc *sc, struct resource *res, int type,
    int rid)
{
	struct pcic_reslist *rle;

	/*
	 * Need to record allocated resource so we can iterate through
	 * it later.
	 */
	rle = malloc(sizeof(struct pcic_reslist), M_DEVBUF, M_NOWAIT);
	if (rle == NULL)
		panic("pcic_insert_res: can't record entry!");
	rle->res = res;
	rle->type = type;
	rle->rid = rid;
	SLIST_INSERT_HEAD(&sc->rl, rle, link);
}

static void
pcic_remove_res(struct pcic_softc *sc, struct resource *res)
{
	struct pcic_reslist *rle;

	SLIST_FOREACH(rle, &sc->rl, link) {
		if (rle->res == res) {
			SLIST_REMOVE(&sc->rl, rle, pcic_reslist, link);
			free(rle, M_DEVBUF);
			return;
		}
	}
}

static struct resource *
pcic_find_res(struct pcic_softc *sc, int type, int rid)
{
	struct pcic_reslist *rle;

	SLIST_FOREACH(rle, &sc->rl, link)
		if (SYS_RES_MEMORY == rle->type && rid == rle->rid)
			return (rle->res);
	return (NULL);
}

static struct resource *
pcic_alloc_resource(device_t brdev, device_t child, int type, int *rid,
    rman_res_t start, rman_res_t end, rman_res_t count, u_int flags)
{
	struct resource *res = NULL;
	struct pcic_softc *sc = device_get_softc(brdev);
	int align;
	int tmp;

	switch (type) {
	case SYS_RES_MEMORY:
		if (start < pcic_start_mem)
			start = pcic_start_mem;
		if (end < start)
			end = start;
		if (count < PCIC_MEMALIGN)
			align = PCIC_MEMALIGN;
		else
			align = count;
		if (align > (1 << RF_ALIGNMENT(flags)))
			flags = (flags & ~RF_ALIGNMENT_MASK) | 
			    rman_make_alignment_flags(align);
		break;
	case SYS_RES_IOPORT:
		if (start < pcic_start_io)
			start = pcic_start_io;
		if (end < start)
			end = start;
		break;
	case SYS_RES_IRQ:
		tmp = rman_get_start(sc->irq_res);
		if (start > tmp || end < tmp || count != 1) {
			device_printf(child, "requested interrupt %jd-%jd,"
			    "count = %jd not supported by pcic\n",
			    start, end, count);
			return (NULL);
		}
		flags |= RF_SHAREABLE;
		start = end = rman_get_start(sc->irq_res);
		break;
	}
	res = BUS_ALLOC_RESOURCE(device_get_parent(brdev), child, type, rid,
	    start, end, count, flags & ~RF_ACTIVE);
	if (res == NULL)
		return (NULL);
	pcic_insert_res(sc, res, type, *rid);
	if (flags & RF_ACTIVE) {
		if (bus_activate_resource(child, type, *rid, res) != 0) {
			bus_release_resource(child, type, *rid, res);
			return (NULL);
		}
	}

	return (res);
}

static int
pcic_release_resource(device_t brdev, device_t child, int type,
    int rid, struct resource *res)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	int error;

	if (rman_get_flags(res) & RF_ACTIVE) {
		error = bus_deactivate_resource(child, type, rid, res);
		if (error != 0)
			return (error);
	}
	pcic_remove_res(sc, res);
	return (BUS_RELEASE_RESOURCE(device_get_parent(brdev), child,
	    type, rid, res));
}

static int
pcic_activate_resource(device_t brdev, device_t child, int type, int rid,
    struct resource *res)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	int slot;

	slot = pcic_slot(child);
	return (exca_activate_resource(&sc->exca[slot], child, type, rid, res));
}

static int
pcic_deactivate_resource(device_t brdev, device_t child, int type,
    int rid, struct resource *res)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	int slot;

	slot = pcic_slot(child);
	return (exca_deactivate_resource(&sc->exca[slot], child, type, rid, res));
}

static void
pcic_driver_added(device_t brdev, driver_t *driver)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	device_t *devlist;
	device_t dev;
	int tmp;
	int numdevs;
	int wake = 0;

	DEVICE_IDENTIFY(driver, brdev);
	tmp = device_get_children(brdev, &devlist, &numdevs);
	if (tmp != 0) {
		device_printf(brdev, "Cannot get children list, no reprobe\n");
		return;
	}
	for (tmp = 0; tmp < numdevs; tmp++) {
		dev = devlist[tmp];
		if (device_get_state(dev) == DS_NOTPRESENT &&
		    device_probe_and_attach(dev) == 0)
			wake++;
	}
	free(devlist, M_TEMP);

	if (wake > 0)
		wakeup(&sc->intrhand);
}

static void
pcic_child_detached(device_t brdev, device_t child)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	int slot;

	slot = pcic_slot(child);
	if (child != sc->exca[slot].pccarddev)
		device_printf(brdev, "Unknown child detached: %s\n",
		    device_get_nameunit(child));
}

/*
 * Enable function interrupts.  We turn on function interrupts when the card
 * requests an interrupt.
 */
static void
pcic_enable_func_intr(struct pcic_softc *sc, uint32_t irq)
{
	uint8_t reg;

// XXX assert?
	reg = (exca_getb(&sc->exca[0], EXCA_INTR) & ~EXCA_INTR_IRQ_MASK) | irq;
	exca_putb(&sc->exca[0], EXCA_INTR, reg);
}

static void
pcic_disable_func_intr(struct pcic_softc *sc)
{
	uint8_t reg;

	reg = (exca_getb(&sc->exca[0], EXCA_INTR) & ~EXCA_INTR_IRQ_MASK) | 
	    EXCA_INTR_IRQ_NONE;
	exca_putb(&sc->exca[0], EXCA_INTR, reg);
}

static int
pcic_setup_intr(device_t dev, device_t child, struct resource *irq,
    int flags, driver_filter_t *filt, driver_intr_t *intr, void *arg,
    void **cookiep)
{
	struct pcic_intrhand *ih;
	struct pcic_softc *sc = device_get_softc(dev);
	int err;

	if (filt == NULL && intr == NULL)
		return (EINVAL);
	ih = malloc(sizeof(struct pcic_intrhand), M_DEVBUF, M_NOWAIT);
	if (ih == NULL)
		return (ENOMEM);
	*cookiep = ih;
	ih->filt = filt;
	ih->intr = intr;
	ih->arg = arg;
	ih->sc = sc;
	err = BUS_SETUP_INTR(device_get_parent(dev), child, irq, flags,
	    filt ? pcic_func_filt : NULL, intr ? pcic_func_intr : NULL, ih,
	    &ih->cookie);
	if (err != 0) {
		free(ih, M_DEVBUF);
		return (err);
	}
	pcic_enable_func_intr(sc, rman_get_start(irq));
	sc->cardok = 1;
	return 0;
}

static int
pcic_teardown_intr(device_t dev, device_t child, struct resource *irq,
    void *cookie)
{
	struct pcic_intrhand *ih;
	struct pcic_softc *sc;
	int err;

	ih = (struct pcic_intrhand *) cookie;
	sc = ih->sc;
	err = BUS_TEARDOWN_INTR(device_get_parent(dev), child, irq,
	    ih->cookie);
	if (err != 0)
		return (err);
	free(ih, M_DEVBUF);
	pcic_disable_func_intr(sc);
	return (0);
}

static int
pcic_child_present(device_t parent, device_t child)
{
	struct pcic_softc *sc = device_get_softc(parent);
	uint32_t status;
	int slot;

	slot = pcic_slot(child);
	status = exca_getb(&sc->exca[slot], EXCA_IF_STATUS);
	return ((status & EXCA_IF_STATUS_CARDDETECT_MASK) ==
	    EXCA_IF_STATUS_CARDDETECT_PRESENT);
}

static int
pcic_set_res_flags(device_t brdev, device_t child, int type, int rid,
    u_long flags)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	struct resource *res;
	int slot;

	slot = pcic_slot(child);
	if (type != SYS_RES_MEMORY)
		return (EINVAL);
	res = pcic_find_res(sc, type, rid);
	if (res == NULL) {
		device_printf(brdev,
		    "set_res_flags: specified rid not found\n");
		return (ENOENT);
	}
	return (exca_mem_set_flags(&sc->exca[slot], res, flags));
}

static int
pcic_set_memory_offset(device_t brdev, device_t child, int rid,
    uint32_t cardaddr, uint32_t *deltap)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	struct resource *res;
	int slot;

	slot = pcic_slot(child);
	res = pcic_find_res(sc, SYS_RES_MEMORY, rid);
	if (res == NULL) {
		device_printf(brdev,
		    "set_memory_offset: specified rid not found\n");
		return (ENOENT);
	}
	return (exca_mem_set_offset(&sc->exca[slot], res, cardaddr, deltap));
}

static int
pcic_power_enable_socket(device_t brdev, device_t child)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	int slot;

	slot = pcic_slot(child);
	DPRINTF(("pcic_socket_enable:\n"));
	exca_power_on(&sc->exca[slot]);
	exca_reset(&sc->exca[slot], child);

	return (0);
}

static int
pcic_power_disable_socket(device_t brdev, device_t child)
{
	struct pcic_softc *sc = device_get_softc(brdev);
	int slot;

	slot = pcic_slot(child);
	DPRINTF(("pcic_socket_disable\n"));

	/* Turn off the card's interrupt and leave it in reset, wait 10ms */
	exca_putb(&sc->exca[slot], EXCA_INTR, 0);
	pause("pcicP1", hz / 100);

	/* power down the socket */
	exca_power_off(&sc->exca[slot]);

	/* wait 300ms until power fails (Tpf). */
	pause("pcicP2", hz * 300 / 1000);

	/* enable CSC interrupts */
	exca_putb(&sc->exca[slot], EXCA_INTR, EXCA_INTR_ENABLE);
	return (0);
}

static int
pcic_func_filt(void *arg)
{
	struct pcic_intrhand *ih = (struct pcic_intrhand *)arg;
	struct pcic_softc *sc = ih->sc;

	/*
	 * Make sure that the card is really there.
	 */
	if (!sc->cardok)
		return (FILTER_STRAY);
	if (!pcic_child_present(sc->dev, sc->dev)) {
		sc->cardok = 0;
		return (FILTER_HANDLED);
	}

	return ((*ih->filt)(ih->arg));
}

static void
pcic_func_intr(void *arg)
{
	struct pcic_intrhand *ih = (struct pcic_intrhand *)arg;
	struct pcic_softc *sc = ih->sc;

	/*
	 * While this check may seem redundant, it helps close a race
	 * condition.  If the card is ejected after the filter runs, but
	 * before this ISR can be scheduled, then we need to do the same
	 * filtering to prevent the card's ISR from being called.  One could
	 * argue that the card's ISR should be able to cope, but experience
	 * has shown they can't always.  This mitigates the problem by making
	 * the race quite a bit smaller.  Properly written client ISRs should
	 * cope with the card going away in the middle of the ISR.  We assume
	 * that drivers that are sophisticated enough to use filters don't
	 * need our protection.  This also allows us to ensure they *ARE*
	 * called if their filter said they needed to be called.
	 */
	if (ih->filt == NULL) {
		if (!sc->cardok)
			return;
		if (!pcic_child_present(sc->dev, sc->dev)) {
			sc->cardok = 0;
			return;
		}
	}

	/*
	 * Call the registered ithread interrupt handler.  This entire routine
	 * will be called with Giant if this isn't an MP safe driver, or not
	 * if it is.  Either way, we don't have to worry.
	 */
	ih->intr(ih->arg);
}

static device_method_t pcic_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,			pcic_probe),
	DEVMETHOD(device_attach,		pcic_attach),
	DEVMETHOD(device_detach,		pcic_detach),
	DEVMETHOD(device_suspend,		pcic_suspend),
	DEVMETHOD(device_resume,		pcic_resume),

	/* bus methods */
	DEVMETHOD(bus_read_ivar,		pcic_read_ivar),
	DEVMETHOD(bus_write_ivar,		pcic_write_ivar),
	DEVMETHOD(bus_alloc_resource,		pcic_alloc_resource),
	DEVMETHOD(bus_release_resource,		pcic_release_resource),
	DEVMETHOD(bus_activate_resource,	pcic_activate_resource),
	DEVMETHOD(bus_deactivate_resource,	pcic_deactivate_resource),
	DEVMETHOD(bus_driver_added,		pcic_driver_added),
	DEVMETHOD(bus_child_detached,		pcic_child_detached),
	DEVMETHOD(bus_setup_intr,		pcic_setup_intr),
	DEVMETHOD(bus_teardown_intr,		pcic_teardown_intr),
	DEVMETHOD(bus_child_present,		pcic_child_present),

	/* 16-bit card interface */
	DEVMETHOD(card_set_res_flags,		pcic_set_res_flags),
	DEVMETHOD(card_set_memory_offset,	pcic_set_memory_offset),

	/* power interface */
	DEVMETHOD(power_enable_socket,		pcic_power_enable_socket),
	DEVMETHOD(power_disable_socket,		pcic_power_disable_socket),

	DEVMETHOD_END
};

static driver_t pcic_driver = {
	"pcic",
	pcic_methods,
	sizeof(struct pcic_softc)
};

DRIVER_MODULE(pcic, isa, pcic_driver, pcic_devclass, 0, 0);
MODULE_DEPEND(pcic, exca, 1, 1, 1);
ISA_PNP_INFO(pcic_ids);
