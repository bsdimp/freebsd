/*	$OpenBSD: rtsx.c,v 1.21 2017/10/09 20:06:36 stsp Exp $	*/
/*	$OpenBSD: rtsx_pci.c,v 1.14 2017/09/06 13:07:38 jcs Exp $	*/

/*
 * Copyright (c) 2006 Uwe Stuehler <uwe@openbsd.org>
 * Copyright (c) 2012 Stefan Sperling <stsp@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * Realtek RTS52xx/RTL84xx Card Reader driver.
 */

/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 2019 M Warner Losh <imp@FreeBSD.org>
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

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/resource.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/rtsx/rtsxreg.h>
#include <dev/rtsx/rtsxvar.h>

#include <dev/mmc/bridge.h>
#include <dev/mmc/mmcbrvar.h>

#include "opt_mmccam.h"

#ifdef MMCCAM
#include <cam/cam.h>
#include <cam/cam_ccb.h>
#include <cam/cam_debug.h>
#include <cam/cam_sim.h>
#include <cam/cam_xpt_sim.h>
#endif

/* 
 * We use three DMA buffers: a command buffer, a data buffer, and a buffer for
 * ADMA transfer descriptors which describe scatter-gather (SG) I/O operations.
 *
 * The command buffer contains a command queue for the host controller,
 * which describes SD/MMC commands to run, and other parameters. The chip
 * runs the command queue when a special bit in the RTSX_HCBAR register is
 * set and signals completion with the TRANS_OK interrupt.
 * Each command is encoded as a 4 byte sequence containing command number
 * (read, write, or check a host controller register), a register address,
 * and a data bit-mask and value.
 * SD/MMC commands which do not transfer any data from/to the card only use
 * the command buffer.
 *
 * The smmmc stack provides DMA-safe buffers with data transfer commands.
 * In this case we write a list of descriptors to the ADMA descriptor buffer,
 * instructing the chip to transfer data directly from/to sdmmc DMA buffers.
 *
 * However, some sdmmc commands used during card initialization also carry
 * data, and these don't come with DMA-safe buffers. In this case, we transfer
 * data from/to the SD card via a DMA data bounce buffer.
 *
 * In both cases, data transfer is controlled via the RTSX_HDBAR register
 * and completion is signalled by the TRANS_OK interrupt.
 *
 * The chip is unable to perform DMA above 4GB.
 */

#define	RTSX_DMA_MAX_SEGSIZE	0x80000
#define	RTSX_HOSTCMD_MAX	256
#define	RTSX_HOSTCMD_BUFSIZE	(sizeof(uint32_t) * RTSX_HOSTCMD_MAX)
#define	RTSX_DMA_DATA_BUFSIZE	MAXPHYS
#define	RTSX_ADMA_DESC_SIZE	(sizeof(uint64_t) * SDMMC_MAXNSEGS)

#define READ4(sc, reg)							\
	(bus_space_read_4((sc)->iot, (sc)->ioh, (reg)))
#define WRITE4(sc, reg, val)						\
	bus_space_write_4((sc)->iot, (sc)->ioh, (reg), (val))

#define	RTSX_READ(sc, reg, val) 				\
	do { 							\
		int err = rtsx_read((sc), (reg), (val)); 	\
		if (err) 					\
			return (err);				\
	} while (0)

#define	RTSX_WRITE(sc, reg, val) 				\
	do { 							\
		int err = rtsx_write((sc), (reg), 0xff, (val));	\
		if (err) 					\
			return (err);				\
	} while (0)

#define	RTSX_CLR(sc, reg, bits)					\
	do { 							\
		int err = rtsx_write((sc), (reg), (bits), 0); 	\
		if (err) 					\
			return (err);				\
	} while (0)

#define	RTSX_SET(sc, reg, bits)					\
	do { 							\
		int err = rtsx_write((sc), (reg), (bits), 0xff);\
		if (err) 					\
			return (err);				\
	} while (0)

// XXX these are suspect
#define	RTSX_MEMRES		0
#define	RTSX_IRQRES		1
#define	RTSX_RESSZ		2
#define	RTSX_DMA_SEGS		(PAGE_SIZE / sizeof(struct rtsx_dma_desc))
#define	RTSX_DMA_DESC_SIZE	(sizeof(struct rtsx_dma_desc) * RTSX_DMA_SEGS)

#define	RTSX_RESET_RETRY	1000

struct rtsx_softc {
	device_t		rtsx_dev;

	int			rtsx_bus_busy;
	int			rtsx_resid;
	int			rtsx_timeout;
	int			rtsx_bar;
	struct callout		rtsx_timeoutc;
	struct mmc_host		rtsx_host;
#ifdef MMCCAM
	union ccb *		ccb;
	struct cam_devq *	devq;
	struct cam_sim * 	sim;
	struct mtx		sim_mtx;
#else
	struct mmc_request *	rtsx_req;
#endif
	struct mtx		rtsx_mtx;
	struct resource *	rtsx_res[RTSX_RESSZ];
	struct rtsx_conf *	rtsx_conf;
	uint32_t		rtsx_intr;
	uint32_t		rtsx_intr_wait;
	void *			rtsx_intrhand;

	/* Fields required for DMA access. */
	bus_addr_t	  	rtsx_dma_desc_phys;
	bus_dmamap_t		rtsx_dma_map;
	bus_dma_tag_t 		rtsx_dma_tag;
	void * 			rtsx_dma_desc;
	bus_dmamap_t		rtsx_dma_buf_map;
	bus_dma_tag_t		rtsx_dma_buf_tag;
	int			rtsx_dma_map_err;

	bus_space_tag_t	iot;		/* host register set tag */
	bus_space_handle_t ioh;		/* host register set handle */
	bus_dma_tag_t	dmat;		/* DMA tag from attachment driver */
	bus_dmamap_t	dmap_cmd;	/* DMA map for command transfer */
	bus_dmamap_t	dmap_data;	/* DMA map for data transfer */
	bus_dmamap_t	dmap_adma;	/* DMA map for ADMA SG descriptors */
	caddr_t		admabuf;	/* buffer for ADMA SG descriptors */
	bus_dma_segment_t adma_segs[1];	/* segments for ADMA SG buffer */
	int		flags;
	uint32_t 	intr_status;	/* soft interrupt status */
	uint8_t	regs[RTSX_NREG];/* host controller state */
	uint32_t	regs4[6];	/* host controller state */
};

static struct resource_spec rtsx_res_spec[] = {
	{ SYS_RES_MEMORY,	0,	RF_ACTIVE },
	{ SYS_RES_IRQ,		0,	RF_ACTIVE | RF_SHAREABLE },
	{ -1,			0,	0 }
};

static int rtsx_probe(device_t);
static int rtsx_attach(device_t);
static int rtsx_detach(device_t);
static int rtsx_setup_dma(struct rtsx_softc *);
static int rtsx_reset(struct rtsx_softc *);
static void rtsx_intr(void *);
static int rtsx_update_clock(struct rtsx_softc *, uint32_t);

static void rtsx_print_error(uint32_t);
static int rtsx_update_ios(device_t, device_t);
static int rtsx_request(device_t, device_t, struct mmc_request *);
static int rtsx_get_ro(device_t, device_t);
static int rtsx_acquire_host(device_t, device_t);
static int rtsx_release_host(device_t, device_t);
#ifdef MMCCAM
static void rtsx_cam_action(struct cam_sim *, union ccb *);
static void rtsx_cam_poll(struct cam_sim *);
static int rtsx_cam_settran_settings(struct rtsx_softc *, union ccb *);
static int rtsx_cam_request(struct rtsx_softc *, union ccb *);
static void rtsx_cam_handle_mmcio(struct cam_sim *, union ccb *);
#endif

static int rtsx_read(struct rtsx_softc *, uint16_t, uint8_t *);
static int rtsx_write(struct rtsx_softc *, uint16_t, uint8_t, uint8_t);
static int rtsx_write_phy(struct rtsx_softc *, uint8_t, uint16_t);
static int rtsx_read_cfg(struct rtsx_softc *, uint8_t, uint16_t, uint32_t *);

#define	RTSX_LOCK(_sc)	mtx_lock(&(_sc)->rtsx_mtx)
#define	RTSX_UNLOCK(_sc)	mtx_unlock(&(_sc)->rtsx_mtx)
#define	RTSX_READ_4(_sc, _reg)					\
	bus_read_4((_sc)->rtsx_res[RTSX_MEMRES], _reg)
#define	RTSX_WRITE_4(_sc, _reg, _value)				\
	bus_write_4((_sc)->rtsx_res[RTSX_MEMRES], _reg, _value)



#ifdef MMCCAM
static void
rtsx_cam_handle_mmcio(struct cam_sim *sim, union ccb *ccb)
{
	struct rtsx_softc *sc;

	sc = cam_sim_softc(sim);

	rtsx_cam_request(sc, ccb);
}

static void
rtsx_cam_action(struct cam_sim *sim, union ccb *ccb)
{
	struct rtsx_softc *sc;

	sc = cam_sim_softc(sim);
	if (sc == NULL) {
		ccb->ccb_h.status = CAM_SEL_TIMEOUT;
		xpt_done(ccb);
		return;
	}

	mtx_assert(&sc->sim_mtx, MA_OWNED);

	switch (ccb->ccb_h.func_code) {
	case XPT_PATH_INQ:
	{
		struct ccb_pathinq *cpi;

		cpi = &ccb->cpi;
		cpi->version_num = 1;
		cpi->hba_inquiry = 0;
		cpi->target_sprt = 0;
		cpi->hba_misc = PIM_NOBUSRESET | PIM_SEQSCAN;
		cpi->hba_eng_cnt = 0;
		cpi->max_target = 0;
		cpi->max_lun = 0;
		cpi->initiator_id = 1;
		cpi->maxio = (sc->rtsx_conf->dma_xferlen *
			      RTSX_DMA_SEGS) / MMC_SECTOR_SIZE;
		strncpy(cpi->sim_vid, "FreeBSD", SIM_IDLEN);
		strncpy(cpi->hba_vid, "Deglitch Networks", HBA_IDLEN);
		strncpy(cpi->dev_name, cam_sim_name(sim), DEV_IDLEN);
		cpi->unit_number = cam_sim_unit(sim);
		cpi->bus_id = cam_sim_bus(sim);
		cpi->protocol = PROTO_MMCSD;
		cpi->protocol_version = SCSI_REV_0;
		cpi->transport = XPORT_MMCSD;
		cpi->transport_version = 1;

		cpi->ccb_h.status = CAM_REQ_CMP;
		break;
	}
	case XPT_GET_TRAN_SETTINGS:
	{
		struct ccb_trans_settings *cts = &ccb->cts;

		if (bootverbose)
			device_printf(sc->rtsx_dev, "Got XPT_GET_TRAN_SETTINGS\n");

		cts->protocol = PROTO_MMCSD;
		cts->protocol_version = 1;
		cts->transport = XPORT_MMCSD;
		cts->transport_version = 1;
		cts->xport_specific.valid = 0;
		cts->proto_specific.mmc.host_ocr = sc->rtsx_host.host_ocr;
		cts->proto_specific.mmc.host_f_min = sc->rtsx_host.f_min;
		cts->proto_specific.mmc.host_f_max = sc->rtsx_host.f_max;
		cts->proto_specific.mmc.host_caps = sc->rtsx_host.caps;
		cts->proto_specific.mmc.host_max_data = (sc->rtsx_conf->dma_xferlen *
		    RTSX_DMA_SEGS) / MMC_SECTOR_SIZE;
		memcpy(&cts->proto_specific.mmc.ios, &sc->rtsx_host.ios, sizeof(struct mmc_ios));
		ccb->ccb_h.status = CAM_REQ_CMP;
		break;
	}
	case XPT_SET_TRAN_SETTINGS:
	{
		if (bootverbose)
			device_printf(sc->rtsx_dev, "Got XPT_SET_TRAN_SETTINGS\n");
		rtsx_cam_settran_settings(sc, ccb);
		ccb->ccb_h.status = CAM_REQ_CMP;
		break;
	}
	case XPT_RESET_BUS:
		if (bootverbose)
			device_printf(sc->rtsx_dev, "Got XPT_RESET_BUS, ACK it...\n");
		ccb->ccb_h.status = CAM_REQ_CMP;
		break;
	case XPT_MMC_IO:
		/*
		 * Here is the HW-dependent part of
		 * sending the command to the underlying h/w
		 * At some point in the future an interrupt comes.
		 * Then the request will be marked as completed.
		 */
		ccb->ccb_h.status = CAM_REQ_INPROG;

		rtsx_cam_handle_mmcio(sim, ccb);
		return;
		/* NOTREACHED */
		break;
	default:
		ccb->ccb_h.status = CAM_REQ_INVALID;
		break;
	}
	xpt_done(ccb);
	return;
}

static void
rtsx_cam_poll(struct cam_sim *sim)
{
	return;
}

static int
rtsx_cam_settran_settings(struct rtsx_softc *sc, union ccb *ccb)
{
	struct mmc_ios *ios;
	struct mmc_ios *new_ios;
	struct ccb_trans_settings_mmc *cts;

	ios = &sc->rtsx_host.ios;

	cts = &ccb->cts.proto_specific.mmc;
	new_ios = &cts->ios;

	/* Update only requested fields */
	if (cts->ios_valid & MMC_CLK) {
		ios->clock = new_ios->clock;
		device_printf(sc->rtsx_dev, "Clock => %d\n", ios->clock);
	}
	if (cts->ios_valid & MMC_VDD) {
		ios->vdd = new_ios->vdd;
		device_printf(sc->rtsx_dev, "VDD => %d\n", ios->vdd);
	}
	if (cts->ios_valid & MMC_CS) {
		ios->chip_select = new_ios->chip_select;
		device_printf(sc->rtsx_dev, "CS => %d\n", ios->chip_select);
	}
	if (cts->ios_valid & MMC_BW) {
		ios->bus_width = new_ios->bus_width;
		device_printf(sc->rtsx_dev, "Bus width => %d\n", ios->bus_width);
	}
	if (cts->ios_valid & MMC_PM) {
		ios->power_mode = new_ios->power_mode;
		device_printf(sc->rtsx_dev, "Power mode => %d\n", ios->power_mode);
	}
	if (cts->ios_valid & MMC_BT) {
		ios->timing = new_ios->timing;
		device_printf(sc->rtsx_dev, "Timing => %d\n", ios->timing);
	}
	if (cts->ios_valid & MMC_BM) {
		ios->bus_mode = new_ios->bus_mode;
		device_printf(sc->rtsx_dev, "Bus mode => %d\n", ios->bus_mode);
	}

	return (rtsx_update_ios(sc->rtsx_dev, NULL));
}

static int
rtsx_cam_request(struct rtsx_softc *sc, union ccb *ccb)
{
	struct ccb_mmcio *mmcio;

	mmcio = &ccb->mmcio;

	RTSX_LOCK(sc);

#ifdef DEBUG
	if (__predict_false(bootverbose)) {
		device_printf(sc->rtsx_dev, "CMD%u arg %#x flags %#x dlen %u dflags %#x\n",
			    mmcio->cmd.opcode, mmcio->cmd.arg, mmcio->cmd.flags,
			    mmcio->cmd.data != NULL ? (unsigned int) mmcio->cmd.data->len : 0,
			    mmcio->cmd.data != NULL ? mmcio->cmd.data->flags: 0);
	}
#endif
	if (mmcio->cmd.data != NULL) {
		if (mmcio->cmd.data->len == 0 || mmcio->cmd.data->flags == 0)
			panic("data->len = %d, data->flags = %d -- something is b0rked",
			      (int)mmcio->cmd.data->len, mmcio->cmd.data->flags);
	}
	if (sc->ccb != NULL) {
		device_printf(sc->rtsx_dev, "Controller still has an active command\n");
		return (EBUSY);
	}
	sc->ccb = ccb;
	/* rtsx_request locks again */
	RTSX_UNLOCK(sc);
	rtsx_request(sc->rtsx_dev, NULL, NULL);

	return (0);
}
#endif /* MMCCAM */

#define RTSX_PCI_BAR 		0x10
#define RTSX_PCI_BAR_525A 	0x14

#define RTSX_DEV(x, y, z, zz) { PCI_DEV(PCI_VENDOR_REALTEK, PCI_PRODUCT_REALTEK_##x), \
				PCI_DESCR(y), .driver_data = (z) | ((zz) << 16) }

struct pci_device_table rtsx_devs[] = {
	RTSX_DEV(RTS5209, "Realtek RTS5209", RTSX_F_5209, RTSX_PCI_BAR),
	RTSX_DEV(RTS5227, "Realtek RTS5227", 0, RTSX_PCI_BAR),
	RTSX_DEV(RTS5229, "Realtek RTS5229", RTSX_F_5229, RTSX_PCI_BAR),
	RTSX_DEV(RTS522A, "Realtek RTS522A", 0, RTSX_PCI_BAR),
	RTSX_DEV(RTS5249, "Realtek RTS5249", RTSX_F_5229, RTSX_PCI_BAR),
	RTSX_DEV(RTS525A, "Realtek RTS525A", RTSX_F_525A, RTSX_PCI_BAR_525A),
	RTSX_DEV(RTL8402, "Realtek RTL8402", 0, RTSX_PCI_BAR),
	RTSX_DEV(RTL8411, "Realtek RTL8411", 0, RTSX_PCI_BAR),
	RTSX_DEV(RTL8411B, "Realtek RTL8411B", 0, RTSX_PCI_BAR),
};

static int
rtsx_probe(device_t dev)
{
	const struct pci_device_table *tbl;

	tbl = PCI_MATCH(dev, rtsx_devs);
	if (tbl == NULL)
		return (ENXIO);
	device_set_desc(dev, tbl->descr);

	return (BUS_PROBE_DEFAULT);
}

static int
rtsx_init(struct rtsx_softc *sc, int attaching)
{
	uint32_t status;
	uint8_t version;
	int error;

	/* Read IC version from dummy register. */
	if (sc->flags & RTSX_F_5229) {
		RTSX_READ(sc, RTSX_DUMMY_REG, &version);
		switch (version & 0x0F) {
		case RTSX_IC_VERSION_A:
		case RTSX_IC_VERSION_B:
		case RTSX_IC_VERSION_D:
			break;
		case RTSX_IC_VERSION_C:
			sc->flags |= RTSX_F_5229_TYPE_C;
			break;
		default:
			printf("rtsx_init: unknown ic %02x\n", version);
			return (1);
		}
	}

	/* Enable interrupt write-clear (default is read-clear). */
	RTSX_CLR(sc, RTSX_NFTS_TX_CTRL, RTSX_INT_READ_CLR);

	/* Clear any pending interrupts. */
	status = READ4(sc, RTSX_BIPR);
	WRITE4(sc, RTSX_BIPR, status);

	/* Check for cards already inserted at attach time. */
	if (attaching && (status & RTSX_SD_EXIST))
		sc->flags |= RTSX_F_CARD_PRESENT;

	/* Enable interrupts. */
	WRITE4(sc, RTSX_BIER,
	    RTSX_TRANS_OK_INT_EN | RTSX_TRANS_FAIL_INT_EN | RTSX_SD_INT_EN);

	/* Power on SSC clock. */
	RTSX_CLR(sc, RTSX_FPDCTL, RTSX_SSC_POWER_DOWN);
	DELAY(200);

	/* XXX magic numbers from linux driver */
	if (sc->flags & RTSX_F_5209)
		error = rtsx_write_phy(sc, 0x00, 0xB966);
	else
		error = rtsx_write_phy(sc, 0x00, 0xBA42);
	if (error) {
		device_printf(sc->rtsx_dev, "cannot write phy register\n");
		return (1);
	}

	RTSX_SET(sc, RTSX_CLK_DIV, 0x07);

	/* Disable sleep mode. */
	RTSX_CLR(sc, RTSX_HOST_SLEEP_STATE,
	    RTSX_HOST_ENTER_S1 | RTSX_HOST_ENTER_S3);

	/* Disable card clock. */
	RTSX_CLR(sc, RTSX_CARD_CLK_EN, RTSX_CARD_CLK_EN_ALL);

	RTSX_CLR(sc, RTSX_CHANGE_LINK_STATE,
	    RTSX_FORCE_RST_CORE_EN | RTSX_NON_STICKY_RST_N_DBG | 0x04);
	RTSX_WRITE(sc, RTSX_SD30_DRIVE_SEL, RTSX_SD30_DRIVE_SEL_3V3);

	/* Enable SSC clock. */
	RTSX_WRITE(sc, RTSX_SSC_CTL1, RTSX_SSC_8X_EN | RTSX_SSC_SEL_4M);
	RTSX_WRITE(sc, RTSX_SSC_CTL2, 0x12);

	RTSX_SET(sc, RTSX_CHANGE_LINK_STATE, RTSX_MAC_PHY_RST_N_DBG);
	RTSX_SET(sc, RTSX_IRQSTAT0, RTSX_LINK_READY_INT);

	RTSX_WRITE(sc, RTSX_PERST_GLITCH_WIDTH, 0x80);

	/* Set RC oscillator to 400K. */
	RTSX_CLR(sc, RTSX_RCCTL, RTSX_RCCTL_F_2M);

	/* Request clock by driving CLKREQ pin to zero. */
	RTSX_SET(sc, RTSX_PETXCFG, RTSX_PETXCFG_CLKREQ_PIN);

	/* Set up LED GPIO. */
	if (sc->flags & RTSX_F_5209) {
		RTSX_WRITE(sc, RTSX_CARD_GPIO, 0x03);
		RTSX_WRITE(sc, RTSX_CARD_GPIO_DIR, 0x03);
	} else {
		RTSX_SET(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
		/* Switch LDO3318 source from DV33 to 3V3. */
		RTSX_CLR(sc, RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_DV33);
		RTSX_SET(sc, RTSX_LDO_PWR_SEL, RTSX_LDO_PWR_SEL_3V3);
		/* Set default OLT blink period. */
		RTSX_SET(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_PERIOD);
	}

	return (0);
}

static int
rtsx_attach(device_t dev)
{
	device_t child;
	const struct pci_device_table *tbl;
	struct rtsx_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *tree;
	uint32_t bus_width, max_freq;
	int error;
	uint32_t sdio_cfg;
	int rsegs;

	sc = device_get_softc(dev);
	sc->rtsx_dev = dev;
	tbl = PCI_MATCH(dev, rtsx_devs);
	sc->flags = tbl->driver_data & 0xffff;
	sc->rtsx_bar = tbl->driver_data >> 16;
#ifndef MMCCAM
	sc->rtsx_req = NULL;
#endif

	if (rtsx_init(sc, 1))
		return (ENXIO);

	if (rtsx_read_cfg(sc, 0, RTSX_SDIOCFG_REG, &sdio_cfg) == 0) {
		if ((sdio_cfg & RTSX_SDIOCFG_SDIO_ONLY) ||
		    (sdio_cfg & RTSX_SDIOCFG_HAVE_SDIO))
			sc->flags |= RTSX_F_SDIO_SUPPORT;
	}

	if (rtsx_setup_dma(sc) != 0)
		return (ENXIO);
/* attach children? */

	/* Now handle cards discovered during attachment. */
	if (ISSET(sc->flags, RTSX_F_CARD_PRESENT))
		rtsx_card_insert(sc);
	
	return 0;


/* XXX AW */
	if (bus_alloc_resources(dev, rtsx_res_spec, sc->rtsx_res) != 0) {
		device_printf(dev, "cannot allocate device resources\n");
		return (ENXIO);
	}
	if (bus_setup_intr(dev, sc->rtsx_res[RTSX_IRQRES],
	    INTR_TYPE_MISC | INTR_MPSAFE, NULL, rtsx_intr, sc,
	    &sc->rtsx_intrhand)) {
		bus_release_resources(dev, rtsx_res_spec, sc->rtsx_res);
		device_printf(dev, "cannot setup interrupt handler\n");
		return (ENXIO);
	}
	mtx_init(&sc->rtsx_mtx, device_get_nameunit(sc->rtsx_dev), "rtsx",
	    MTX_DEF);
	callout_init_mtx(&sc->rtsx_timeoutc, &sc->rtsx_mtx, 0);

	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "req_timeout", CTLFLAG_RW,
	    &sc->rtsx_timeout, 0, "Request timeout in seconds");

	/* Soft Reset controller. */
	if (rtsx_reset(sc) != 0) {
		device_printf(dev, "cannot reset the controller\n");
		goto fail;
	}

	if (rtsx_setup_dma(sc) != 0) {
		device_printf(sc->rtsx_dev, "Couldn't setup DMA!\n");
		goto fail;
	}

	sc->rtsx_host.f_min = 400000;

	max_freq = 52000000;
	sc->rtsx_host.f_max = max_freq;

	sc->rtsx_host.host_ocr = MMC_OCR_320_330 | MMC_OCR_330_340;
	sc->rtsx_host.caps = MMC_CAP_HSPEED | MMC_CAP_UHS_SDR12 |
			   MMC_CAP_UHS_SDR25 | MMC_CAP_UHS_SDR50 |
			   MMC_CAP_UHS_DDR50 | MMC_CAP_MMC_DDR52;

	sc->rtsx_host.caps |= MMC_CAP_SIGNALING_330 | MMC_CAP_SIGNALING_180;

	if (bus_width >= 4)
		sc->rtsx_host.caps |= MMC_CAP_4_BIT_DATA;
	if (bus_width >= 8)
		sc->rtsx_host.caps |= MMC_CAP_8_BIT_DATA;

#ifdef MMCCAM
	child = NULL; /* Not used by MMCCAM, need to silence compiler warnings */
	sc->ccb = NULL;
	if ((sc->devq = cam_simq_alloc(1)) == NULL) {
		goto fail;
	}

	mtx_init(&sc->sim_mtx, "awmmcsim", NULL, MTX_DEF);
	sc->sim = cam_sim_alloc(rtsx_cam_action, rtsx_cam_poll,
	    "rtsx_sim", sc, device_get_unit(dev),
	    &sc->sim_mtx, 1, 1, sc->devq);

	if (sc->sim == NULL) {
		cam_simq_free(sc->devq);
		device_printf(dev, "cannot allocate CAM SIM\n");
		goto fail;
	}

	mtx_lock(&sc->sim_mtx);
	if (xpt_bus_register(sc->sim, sc->rtsx_dev, 0) != 0) {
		device_printf(dev, "cannot register SCSI pass-through bus\n");
		cam_sim_free(sc->sim, FALSE);
		cam_simq_free(sc->devq);
		mtx_unlock(&sc->sim_mtx);
		goto fail;
	}

	mtx_unlock(&sc->sim_mtx);
#else /* !MMCCAM */
	child = device_add_child(dev, "mmc", -1);
	if (child == NULL) {
		device_printf(dev, "attaching MMC bus failed!\n");
		goto fail;
	}
	if (device_probe_and_attach(child) != 0) {
		device_printf(dev, "attaching MMC child failed!\n");
		device_delete_child(dev, child);
		goto fail;
	}
#endif /* MMCCAM */
	return (0);

fail:
	callout_drain(&sc->rtsx_timeoutc);
	mtx_destroy(&sc->rtsx_mtx);
	bus_teardown_intr(dev, sc->rtsx_res[RTSX_IRQRES], sc->rtsx_intrhand);
	bus_release_resources(dev, rtsx_res_spec, sc->rtsx_res);

#ifdef MMCCAM
	if (sc->sim != NULL) {
		mtx_lock(&sc->sim_mtx);
		xpt_bus_deregister(cam_sim_path(sc->sim));
		cam_sim_free(sc->sim, FALSE);
		mtx_unlock(&sc->sim_mtx);
	}

	if (sc->devq != NULL)
		cam_simq_free(sc->devq);
#endif
	return (ENXIO);
}

static int
rtsx_detach(device_t dev)
{

	return (EBUSY);
}

static void
rtsx_dma_desc_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int err)
{
	struct rtsx_softc *sc;

	sc = (struct rtsx_softc *)arg;
	if (err) {
		sc->rtsx_dma_map_err = err;
		return;
	}
	sc->rtsx_dma_desc_phys = segs[0].ds_addr;
}

static int
rtsx_setup_dma(struct rtsx_softc *sc)
{
	int error;

	/* Allocate the DMA descriptor memory. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->rtsx_dev),	/* parent */
	    RTSX_DMA_ALIGN, 0,			/* align, boundary */
	    BUS_SPACE_MAXADDR_32BIT,		/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg*/
	    RTSX_MAX_SEGSIZE, 1,		/* maxsize, nsegment */
	    RTSX_MAX_SEGSIZE,			/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lock, lockarg*/
	    &sc->rtsx_dma_tag);
	if (error)
		return (error);
	error = bus_dmamap_create(sc->rtsx_dma_tag, BUS_DMA_NOWAIT,
	    &sc->rtsx_dmap_cmd);
	/* We always leave the cmd dma map loaded */
	error = bus_dmamap_load(sc->rtsx_dma_tag, sc->rtsx_dmap_cmd,
	    XXX I AM HERE XXX);




	if (bus_dmamap_create(sc->rtsx_dma_tag, RTSX_HOSTCMD_BUFSIZE, 1,
	    RTSX_DMA_MAX_SEGSIZE, 0, BUS_DMA_NOWAIT,
	    &sc->dmap_cmd) != 0)
		return 1;
	if (bus_dmamap_create(sc->dmat, RTSX_DMA_DATA_BUFSIZE, 1,
	    RTSX_DMA_MAX_SEGSIZE, 0, BUS_DMA_NOWAIT,
	    &sc->dmap_data) != 0)
	    	goto destroy_cmd;
	if (bus_dmamap_create(sc->dmat, RTSX_ADMA_DESC_SIZE, 1,
	    RTSX_DMA_MAX_SEGSIZE, 0, BUS_DMA_NOWAIT,
	    &sc->dmap_adma) != 0)
	    	goto destroy_data;
	if (bus_dmamem_alloc(sc->dmat, RTSX_ADMA_DESC_SIZE, 0, 0,
	    sc->adma_segs, 1, &rsegs, BUS_DMA_WAITOK|BUS_DMA_ZERO))
	    	goto destroy_adma;
	if (bus_dmamem_map(sc->dmat, sc->adma_segs, rsegs, RTSX_ADMA_DESC_SIZE,
	    &sc->admabuf, BUS_DMA_WAITOK|BUS_DMA_COHERENT))
	    	goto free_adma;

unmap_adma:
	bus_dmamem_unmap(sc->dmat, sc->admabuf, RTSX_ADMA_DESC_SIZE);
free_adma:
	bus_dmamem_free(sc->dmat, sc->adma_segs, rsegs);
destroy_adma:
	bus_dmamap_destroy(sc->dmat, sc->dmap_adma);
destroy_data:
	bus_dmamap_destroy(sc->dmat, sc->dmap_data);
destroy_cmd:
	bus_dmamap_destroy(sc->dmat, sc->dmap_cmd);
	return 1;


	/* Allocate the DMA descriptor memory. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->rtsx_dev),	/* parent */
	    RTSX_DMA_ALIGN, 0,		/* align, boundary */
	    BUS_SPACE_MAXADDR_32BIT,		/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg*/
	    RTSX_DMA_DESC_SIZE, 1,		/* maxsize, nsegment */
	    RTSX_DMA_DESC_SIZE,		/* maxsegsize */
	    0,					/* flags */
	    NULL, NULL,				/* lock, lockarg*/
	    &sc->rtsx_dma_tag);
	if (error)
		return (error);

	error = bus_dmamem_alloc(sc->rtsx_dma_tag, &sc->rtsx_dma_desc,
	    BUS_DMA_COHERENT | BUS_DMA_WAITOK | BUS_DMA_ZERO,
	    &sc->rtsx_dma_map);
	if (error)
		return (error);

	error = bus_dmamap_load(sc->rtsx_dma_tag,
	    sc->rtsx_dma_map,
	    sc->rtsx_dma_desc, RTSX_DMA_DESC_SIZE,
	    rtsx_dma_desc_cb, sc, 0);
	if (error)
		return (error);
	if (sc->rtsx_dma_map_err)
		return (sc->rtsx_dma_map_err);

	/* Create the DMA map for data transfers. */
	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->rtsx_dev),	/* parent */
	    RTSX_DMA_ALIGN, 0,		/* align, boundary */
	    BUS_SPACE_MAXADDR_32BIT,		/* lowaddr */
	    BUS_SPACE_MAXADDR,			/* highaddr */
	    NULL, NULL,				/* filter, filterarg*/
	    sc->rtsx_conf->dma_xferlen *
	    RTSX_DMA_SEGS, RTSX_DMA_SEGS,	/* maxsize, nsegments */
	    sc->rtsx_conf->dma_xferlen,	/* maxsegsize */
	    BUS_DMA_ALLOCNOW,			/* flags */
	    NULL, NULL,				/* lock, lockarg*/
	    &sc->rtsx_dma_buf_tag);
	if (error)
		return (error);
	error = bus_dmamap_create(sc->rtsx_dma_buf_tag, 0,
	    &sc->rtsx_dma_buf_map);
	if (error)
		return (error);

	return (0);
}

static void
rtsx_dma_cb(void *arg, bus_dma_segment_t *segs, int nsegs, int err)
{
	int i;
	struct rtsx_dma_desc *dma_desc;
	struct rtsx_softc *sc;

	sc = (struct rtsx_softc *)arg;
	sc->rtsx_dma_map_err = err;

	if (err)
		return;

	dma_desc = sc->rtsx_dma_desc;
	for (i = 0; i < nsegs; i++) {
		if (segs[i].ds_len == sc->rtsx_conf->dma_xferlen)
			dma_desc[i].buf_size = 0;		/* Size of 0 indicate max len */
		else
			dma_desc[i].buf_size = segs[i].ds_len;
		dma_desc[i].buf_addr = segs[i].ds_addr;
		dma_desc[i].config = RTSX_DMA_CONFIG_CH |
			RTSX_DMA_CONFIG_OWN | RTSX_DMA_CONFIG_DIC;

		dma_desc[i].next = sc->rtsx_dma_desc_phys +
			((i + 1) * sizeof(struct rtsx_dma_desc));
	}

	dma_desc[0].config |= RTSX_DMA_CONFIG_FD;
	dma_desc[nsegs - 1].config |= RTSX_DMA_CONFIG_LD |
		RTSX_DMA_CONFIG_ER;
	dma_desc[nsegs - 1].config &= ~RTSX_DMA_CONFIG_DIC;
	dma_desc[nsegs - 1].next = 0;
}

static int
rtsx_prepare_dma(struct rtsx_softc *sc)
{
	bus_dmasync_op_t sync_op;
	int error;
	struct mmc_command *cmd;
	uint32_t val;

#ifdef MMCCAM
	cmd = &sc->ccb->mmcio.cmd;
#else
	cmd = sc->rtsx_req->cmd;
#endif
	if (cmd->data->len > (sc->rtsx_conf->dma_xferlen * RTSX_DMA_SEGS))
		return (EFBIG);
	error = bus_dmamap_load(sc->rtsx_dma_buf_tag, sc->rtsx_dma_buf_map,
	    cmd->data->data, cmd->data->len, rtsx_dma_cb, sc, 0);
	if (error)
		return (error);
	if (sc->rtsx_dma_map_err)
		return (sc->rtsx_dma_map_err);

	if (cmd->data->flags & MMC_DATA_WRITE)
		sync_op = BUS_DMASYNC_PREWRITE;
	else
		sync_op = BUS_DMASYNC_PREREAD;
	bus_dmamap_sync(sc->rtsx_dma_buf_tag, sc->rtsx_dma_buf_map, sync_op);
	bus_dmamap_sync(sc->rtsx_dma_tag, sc->rtsx_dma_map, BUS_DMASYNC_PREWRITE);

	/* Enable DMA */
	val = RTSX_READ_4(sc, RTSX_GCTL);
	val &= ~RTSX_GCTL_FIFO_AC_MOD;
	val |= RTSX_GCTL_DMA_ENB;
	RTSX_WRITE_4(sc, RTSX_GCTL, val);

	/* Reset DMA */
	val |= RTSX_GCTL_DMA_RST;
	RTSX_WRITE_4(sc, RTSX_GCTL, val);

	RTSX_WRITE_4(sc, RTSX_DMAC, RTSX_DMAC_IDMAC_SOFT_RST);
	RTSX_WRITE_4(sc, RTSX_DMAC,
	    RTSX_DMAC_IDMAC_IDMA_ON | RTSX_DMAC_IDMAC_FIX_BURST);

	/* Enable RX or TX DMA interrupt */
	val = RTSX_READ_4(sc, RTSX_IDIE);
	if (cmd->data->flags & MMC_DATA_WRITE)
		val |= RTSX_IDST_TX_INT;
	else
		val |= RTSX_IDST_RX_INT;
	RTSX_WRITE_4(sc, RTSX_IDIE, val);

	/* Set DMA descritptor list address */
	RTSX_WRITE_4(sc, RTSX_DLBA, sc->rtsx_dma_desc_phys);

	/* FIFO trigger level */
	RTSX_WRITE_4(sc, RTSX_FWLR, RTSX_DMA_FTRGLEVEL);

	return (0);
}

static int
rtsx_reset(struct rtsx_softc *sc)
{
	uint32_t reg;
	int timeout;

	reg = RTSX_READ_4(sc, RTSX_GCTL);
	reg |= RTSX_GCTL_RESET;
	RTSX_WRITE_4(sc, RTSX_GCTL, reg);
	timeout = RTSX_RESET_RETRY;
	while (--timeout > 0) {
		if ((RTSX_READ_4(sc, RTSX_GCTL) & RTSX_GCTL_RESET) == 0)
			break;
		DELAY(100);
	}
	if (timeout == 0)
		return (ETIMEDOUT);

	return (0);
}

static void
rtsx_req_done(struct rtsx_softc *sc)
{
	struct mmc_command *cmd;
#ifdef MMCCAM
	union ccb *ccb;
#else
	struct mmc_request *req;
#endif
	uint32_t val, mask;
	int retry;

#ifdef MMCCAM
	ccb = sc->ccb;
	cmd = &ccb->mmcio.cmd;
#else
	cmd = sc->rtsx_req->cmd;
#endif
#ifdef DEBUG
	if (bootverbose) {
		device_printf(sc->rtsx_dev, "%s: cmd %d err %d\n", __func__, cmd->opcode, cmd->error);
	}
#endif
	if (cmd->error != MMC_ERR_NONE) {
		/* Reset the FIFO and DMA engines. */
		mask = RTSX_GCTL_FIFO_RST | RTSX_GCTL_DMA_RST;
		val = RTSX_READ_4(sc, RTSX_GCTL);
		RTSX_WRITE_4(sc, RTSX_GCTL, val | mask);

		retry = RTSX_RESET_RETRY;
		while (--retry > 0) {
			if ((RTSX_READ_4(sc, RTSX_GCTL) &
			    RTSX_GCTL_RESET) == 0)
				break;
			DELAY(100);
		}
		if (retry == 0)
			device_printf(sc->rtsx_dev,
			    "timeout resetting DMA/FIFO\n");
		rtsx_update_clock(sc, 1);
	}

	callout_stop(&sc->rtsx_timeoutc);
	sc->rtsx_intr = 0;
	sc->rtsx_resid = 0;
	sc->rtsx_dma_map_err = 0;
	sc->rtsx_intr_wait = 0;
#ifdef MMCCAM
	sc->ccb = NULL;
	ccb->ccb_h.status =
		(ccb->mmcio.cmd.error == 0 ? CAM_REQ_CMP : CAM_REQ_CMP_ERR);
	xpt_done(ccb);
#else
	req = sc->rtsx_req;
	sc->rtsx_req = NULL;
	req->done(req);
#endif
}

static void
rtsx_req_ok(struct rtsx_softc *sc)
{
	int timeout;
	struct mmc_command *cmd;
	uint32_t status;

	timeout = 1000;
	while (--timeout > 0) {
		status = RTSX_READ_4(sc, RTSX_STAR);
		if ((status & RTSX_STAR_CARD_BUSY) == 0)
			break;
		DELAY(1000);
	}
#ifdef MMCCAM
	cmd = &sc->ccb->mmcio.cmd;
#else
	cmd = sc->rtsx_req->cmd;
#endif
	if (timeout == 0) {
		cmd->error = MMC_ERR_FAILED;
		rtsx_req_done(sc);
		return;
	}
	if (cmd->flags & MMC_RSP_PRESENT) {
		if (cmd->flags & MMC_RSP_136) {
			cmd->resp[0] = RTSX_READ_4(sc, RTSX_RESP3);
			cmd->resp[1] = RTSX_READ_4(sc, RTSX_RESP2);
			cmd->resp[2] = RTSX_READ_4(sc, RTSX_RESP1);
			cmd->resp[3] = RTSX_READ_4(sc, RTSX_RESP0);
		} else
			cmd->resp[0] = RTSX_READ_4(sc, RTSX_RESP0);
	}
	/* All data has been transferred ? */
	if (cmd->data != NULL && (sc->rtsx_resid << 2) < cmd->data->len)
		cmd->error = MMC_ERR_FAILED;
	rtsx_req_done(sc);
}


static inline void
set_mmc_error(struct rtsx_softc *sc, int error_code)
{
#ifdef MMCCAM
	sc->ccb->mmcio.cmd.error = error_code;
#else
	sc->rtsx_req->cmd->error = error_code;
#endif
}

static void
rtsx_timeout(void *arg)
{
	struct rtsx_softc *sc;

	sc = (struct rtsx_softc *)arg;
#ifdef MMCCAM
	if (sc->ccb != NULL) {
#else
	if (sc->rtsx_req != NULL) {
#endif
		device_printf(sc->rtsx_dev, "controller timeout\n");
		set_mmc_error(sc, MMC_ERR_TIMEOUT);
		rtsx_req_done(sc);
	} else
		device_printf(sc->rtsx_dev,
		    "Spurious timeout - no active request\n");
}

static void
rtsx_print_error(uint32_t err)
{
	if(err & RTSX_INT_RESP_ERR)
		printf("RTSX_INT_RESP_ERR ");
	if (err & RTSX_INT_RESP_CRC_ERR)
		printf("RTSX_INT_RESP_CRC_ERR ");
	if (err & RTSX_INT_DATA_CRC_ERR)
		printf("RTSX_INT_DATA_CRC_ERR ");
	if (err & RTSX_INT_RESP_TIMEOUT)
		printf("RTSX_INT_RESP_TIMEOUT ");
	if (err & RTSX_INT_FIFO_RUN_ERR)
		printf("RTSX_INT_FIFO_RUN_ERR ");
	if (err & RTSX_INT_CMD_BUSY)
		printf("RTSX_INT_CMD_BUSY ");
	if (err & RTSX_INT_DATA_START_ERR)
		printf("RTSX_INT_DATA_START_ERR ");
	if (err & RTSX_INT_DATA_END_BIT_ERR)
		printf("RTSX_INT_DATA_END_BIT_ERR");
	printf("\n");
}

static void
rtsx_intr(void *arg)
{
	bus_dmasync_op_t sync_op;
	struct rtsx_softc *sc;
	struct mmc_data *data;
	uint32_t idst, imask, rint;

	sc = (struct rtsx_softc *)arg;
	RTSX_LOCK(sc);
	rint = RTSX_READ_4(sc, RTSX_RISR);
	idst = RTSX_READ_4(sc, RTSX_IDST);
	imask = RTSX_READ_4(sc, RTSX_IMKR);
	if (idst == 0 && imask == 0 && rint == 0) {
		RTSX_UNLOCK(sc);
		return;
	}
#ifdef DEBUG
	device_printf(sc->rtsx_dev, "idst: %#x, imask: %#x, rint: %#x\n",
	    idst, imask, rint);
#endif
#ifdef MMCCAM
	if (sc->ccb == NULL) {
#else
	if (sc->rtsx_req == NULL) {
#endif
		device_printf(sc->rtsx_dev,
		    "Spurious interrupt - no active request, rint: 0x%08X\n",
		    rint);
		rtsx_print_error(rint);
		goto end;
	}
	if (rint & RTSX_INT_ERR_BIT) {
		if (bootverbose)
			device_printf(sc->rtsx_dev, "error rint: 0x%08X\n", rint);
		rtsx_print_error(rint);
		if (rint & RTSX_INT_RESP_TIMEOUT)
			set_mmc_error(sc, MMC_ERR_TIMEOUT);
		else
			set_mmc_error(sc, MMC_ERR_FAILED);
		rtsx_req_done(sc);
		goto end;
	}
	if (idst & RTSX_IDST_ERROR) {
		device_printf(sc->rtsx_dev, "error idst: 0x%08x\n", idst);
		set_mmc_error(sc, MMC_ERR_FAILED);
		rtsx_req_done(sc);
		goto end;
	}

	sc->rtsx_intr |= rint;
#ifdef MMCCAM
	data = sc->ccb->mmcio.cmd.data;
#else
	data = sc->rtsx_req->cmd->data;
#endif
	if (data != NULL && (idst & RTSX_IDST_COMPLETE) != 0) {
		if (data->flags & MMC_DATA_WRITE)
			sync_op = BUS_DMASYNC_POSTWRITE;
		else
			sync_op = BUS_DMASYNC_POSTREAD;
		bus_dmamap_sync(sc->rtsx_dma_buf_tag, sc->rtsx_dma_buf_map,
		    sync_op);
		bus_dmamap_sync(sc->rtsx_dma_tag, sc->rtsx_dma_map,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->rtsx_dma_buf_tag, sc->rtsx_dma_buf_map);
		sc->rtsx_resid = data->len >> 2;
	}
	if ((sc->rtsx_intr & sc->rtsx_intr_wait) == sc->rtsx_intr_wait)
		rtsx_req_ok(sc);

end:
	RTSX_WRITE_4(sc, RTSX_IDST, idst);
	RTSX_WRITE_4(sc, RTSX_RISR, rint);
	RTSX_UNLOCK(sc);
}

static int
rtsx_request(device_t bus, device_t child, struct mmc_request *req)
{
	int blksz;
	struct rtsx_softc *sc;
	struct mmc_command *cmd;
	uint32_t cmdreg, imask;
	int err;

	sc = device_get_softc(bus);

	RTSX_LOCK(sc);
#ifdef MMCCAM
	KASSERT(req == NULL, ("req should be NULL in MMCCAM case!"));
	/*
	 * For MMCCAM, sc->ccb has been NULL-checked and populated
	 * by rtsx_cam_request() already.
	 */
	cmd = &sc->ccb->mmcio.cmd;
#else
	if (sc->rtsx_req) {
		RTSX_UNLOCK(sc);
		return (EBUSY);
	}
	sc->rtsx_req = req;
	cmd = req->cmd;

#ifdef DEBUG
	if (bootverbose)
		device_printf(sc->rtsx_dev, "CMD%u arg %#x flags %#x dlen %u dflags %#x\n",
			      cmd->opcode, cmd->arg, cmd->flags,
			      cmd->data != NULL ? (unsigned int)cmd->data->len : 0,
			      cmd->data != NULL ? cmd->data->flags: 0);
#endif
#endif
	cmdreg = RTSX_CMDR_LOAD;
	imask = RTSX_INT_ERR_BIT;
	sc->rtsx_intr_wait = 0;
	sc->rtsx_intr = 0;
	sc->rtsx_resid = 0;
	cmd->error = MMC_ERR_NONE;

	if (cmd->opcode == MMC_GO_IDLE_STATE)
		cmdreg |= RTSX_CMDR_SEND_INIT_SEQ;

	if (cmd->flags & MMC_RSP_PRESENT)
		cmdreg |= RTSX_CMDR_RESP_RCV;
	if (cmd->flags & MMC_RSP_136)
		cmdreg |= RTSX_CMDR_LONG_RESP;
	if (cmd->flags & MMC_RSP_CRC)
		cmdreg |= RTSX_CMDR_CHK_RESP_CRC;

	if (cmd->data) {
		cmdreg |= RTSX_CMDR_DATA_TRANS | RTSX_CMDR_WAIT_PRE_OVER;

		if (cmd->data->flags & MMC_DATA_MULTI) {
			cmdreg |= RTSX_CMDR_STOP_CMD_FLAG;
			imask |= RTSX_INT_AUTO_STOP_DONE;
			sc->rtsx_intr_wait |= RTSX_INT_AUTO_STOP_DONE;
		} else {
			sc->rtsx_intr_wait |= RTSX_INT_DATA_OVER;
			imask |= RTSX_INT_DATA_OVER;
		}
		if (cmd->data->flags & MMC_DATA_WRITE)
			cmdreg |= RTSX_CMDR_DIR_WRITE;
#ifdef MMCCAM
		if (cmd->data->flags & MMC_DATA_BLOCK_SIZE) {
			RTSX_WRITE_4(sc, RTSX_BKSR, cmd->data->block_size);
			RTSX_WRITE_4(sc, RTSX_BYCR, cmd->data->len);
		} else
#endif
		{
			blksz = min(cmd->data->len, MMC_SECTOR_SIZE);
			RTSX_WRITE_4(sc, RTSX_BKSR, blksz);
			RTSX_WRITE_4(sc, RTSX_BYCR, cmd->data->len);
		}
	} else {
		imask |= RTSX_INT_CMD_DONE;
	}

	/* Enable the interrupts we are interested in */
	RTSX_WRITE_4(sc, RTSX_IMKR, imask);
	RTSX_WRITE_4(sc, RTSX_RISR, 0xffffffff);

	/* Enable auto stop if needed */
	RTSX_WRITE_4(sc, RTSX_A12A,
	    cmdreg & RTSX_CMDR_STOP_CMD_FLAG ? 0 : 0xffff);

	/* Write the command argument */
	RTSX_WRITE_4(sc, RTSX_CAGR, cmd->arg);

	/* 
	 * If we don't have data start the request
	 * if we do prepare the dma request and start the request
	 */
	if (cmd->data == NULL) {
		RTSX_WRITE_4(sc, RTSX_CMDR, cmdreg | cmd->opcode);
	} else {
		err = rtsx_prepare_dma(sc);
		if (err != 0)
			device_printf(sc->rtsx_dev, "prepare_dma failed: %d\n", err);

		RTSX_WRITE_4(sc, RTSX_CMDR, cmdreg | cmd->opcode);
	}

	callout_reset(&sc->rtsx_timeoutc, sc->rtsx_timeout * hz,
	    rtsx_timeout, sc);
	RTSX_UNLOCK(sc);

	return (0);
}

static int
rtsx_read_ivar(device_t bus, device_t child, int which,
    uintptr_t *result)
{
	struct rtsx_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		*(int *)result = sc->rtsx_host.ios.bus_mode;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		*(int *)result = sc->rtsx_host.ios.bus_width;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		*(int *)result = sc->rtsx_host.ios.chip_select;
		break;
	case MMCBR_IVAR_CLOCK:
		*(int *)result = sc->rtsx_host.ios.clock;
		break;
	case MMCBR_IVAR_F_MIN:
		*(int *)result = sc->rtsx_host.f_min;
		break;
	case MMCBR_IVAR_F_MAX:
		*(int *)result = sc->rtsx_host.f_max;
		break;
	case MMCBR_IVAR_HOST_OCR:
		*(int *)result = sc->rtsx_host.host_ocr;
		break;
	case MMCBR_IVAR_MODE:
		*(int *)result = sc->rtsx_host.mode;
		break;
	case MMCBR_IVAR_OCR:
		*(int *)result = sc->rtsx_host.ocr;
		break;
	case MMCBR_IVAR_POWER_MODE:
		*(int *)result = sc->rtsx_host.ios.power_mode;
		break;
	case MMCBR_IVAR_VDD:
		*(int *)result = sc->rtsx_host.ios.vdd;
		break;
	case MMCBR_IVAR_VCCQ:
		*(int *)result = sc->rtsx_host.ios.vccq;
		break;
	case MMCBR_IVAR_CAPS:
		*(int *)result = sc->rtsx_host.caps;
		break;
	case MMCBR_IVAR_TIMING:
		*(int *)result = sc->rtsx_host.ios.timing;
		break;
	case MMCBR_IVAR_MAX_DATA:
		*(int *)result = (sc->rtsx_conf->dma_xferlen *
		    RTSX_DMA_SEGS) / MMC_SECTOR_SIZE;
		break;
	case MMCBR_IVAR_RETUNE_REQ:
		*(int *)result = retune_req_none;
		break;
	}

	return (0);
}

static int
rtsx_write_ivar(device_t bus, device_t child, int which,
    uintptr_t value)
{
	struct rtsx_softc *sc;

	sc = device_get_softc(bus);
	switch (which) {
	default:
		return (EINVAL);
	case MMCBR_IVAR_BUS_MODE:
		sc->rtsx_host.ios.bus_mode = value;
		break;
	case MMCBR_IVAR_BUS_WIDTH:
		sc->rtsx_host.ios.bus_width = value;
		break;
	case MMCBR_IVAR_CHIP_SELECT:
		sc->rtsx_host.ios.chip_select = value;
		break;
	case MMCBR_IVAR_CLOCK:
		sc->rtsx_host.ios.clock = value;
		break;
	case MMCBR_IVAR_MODE:
		sc->rtsx_host.mode = value;
		break;
	case MMCBR_IVAR_OCR:
		sc->rtsx_host.ocr = value;
		break;
	case MMCBR_IVAR_POWER_MODE:
		sc->rtsx_host.ios.power_mode = value;
		break;
	case MMCBR_IVAR_VDD:
		sc->rtsx_host.ios.vdd = value;
		break;
	case MMCBR_IVAR_VCCQ:
		sc->rtsx_host.ios.vccq = value;
		break;
	case MMCBR_IVAR_TIMING:
		sc->rtsx_host.ios.timing = value;
		break;
	/* These are read-only */
	case MMCBR_IVAR_CAPS:
	case MMCBR_IVAR_HOST_OCR:
	case MMCBR_IVAR_F_MIN:
	case MMCBR_IVAR_F_MAX:
	case MMCBR_IVAR_MAX_DATA:
		return (EINVAL);
	}

	return (0);
}

static int
rtsx_update_clock(struct rtsx_softc *sc, uint32_t clkon)
{
	uint32_t reg;
	int retry;

	reg = RTSX_READ_4(sc, RTSX_CKCR);
	reg &= ~(RTSX_CKCR_ENB | RTSX_CKCR_LOW_POWER |
	    RTSX_CKCR_MASK_DATA0);

	if (clkon)
		reg |= RTSX_CKCR_ENB;
	if (sc->rtsx_conf->mask_data0)
		reg |= RTSX_CKCR_MASK_DATA0;

	RTSX_WRITE_4(sc, RTSX_CKCR, reg);

	reg = RTSX_CMDR_LOAD | RTSX_CMDR_PRG_CLK |
	    RTSX_CMDR_WAIT_PRE_OVER;
	RTSX_WRITE_4(sc, RTSX_CMDR, reg);
	retry = 0xfffff;

	while (reg & RTSX_CMDR_LOAD && --retry > 0) {
		reg = RTSX_READ_4(sc, RTSX_CMDR);
		DELAY(10);
	}
	RTSX_WRITE_4(sc, RTSX_RISR, 0xffffffff);

	if (reg & RTSX_CMDR_LOAD) {
		device_printf(sc->rtsx_dev, "timeout updating clock\n");
		return (ETIMEDOUT);
	}

	if (sc->rtsx_conf->mask_data0) {
		reg = RTSX_READ_4(sc, RTSX_CKCR);
		reg &= ~RTSX_CKCR_MASK_DATA0;
		RTSX_WRITE_4(sc, RTSX_CKCR, reg);
	}

	return (0);
}

static int
rtsx_switch_vccq(device_t bus, device_t child)
{
	struct rtsx_softc *sc;
	int uvolt, err;

	sc = device_get_softc(bus);

	if (sc->rtsx_reg_vqmmc == NULL)
		return EOPNOTSUPP;

	switch (sc->rtsx_host.ios.vccq) {
	case vccq_180:
		uvolt = 1800000;
		break;
	case vccq_330:
		uvolt = 3300000;
		break;
	default:
		return EINVAL;
	}

	err = regulator_set_voltage(sc->rtsx_reg_vqmmc, uvolt, uvolt);
	if (err != 0) {
		device_printf(sc->rtsx_dev,
		    "Cannot set vqmmc to %d<->%d\n",
		    uvolt,
		    uvolt);
		return (err);
	}

	return (0);
}

static int
rtsx_update_ios(device_t bus, device_t child)
{
	int error;
	struct rtsx_softc *sc;
	struct mmc_ios *ios;
	unsigned int clock;
	uint32_t reg, div = 1;

	sc = device_get_softc(bus);

	ios = &sc->rtsx_host.ios;

	/* Set the bus width. */
	switch (ios->bus_width) {
	case bus_width_1:
		RTSX_WRITE_4(sc, RTSX_BWDR, RTSX_BWDR1);
		break;
	case bus_width_4:
		RTSX_WRITE_4(sc, RTSX_BWDR, RTSX_BWDR4);
		break;
	case bus_width_8:
		RTSX_WRITE_4(sc, RTSX_BWDR, RTSX_BWDR8);
		break;
	}

	switch (ios->power_mode) {
	case power_on:
		break;
	case power_off:
		if (bootverbose)
			device_printf(sc->rtsx_dev, "Powering down sd/mmc\n");

		if (sc->rtsx_reg_vmmc)
			regulator_disable(sc->rtsx_reg_vmmc);
		if (sc->rtsx_reg_vqmmc)
			regulator_disable(sc->rtsx_reg_vqmmc);

		rtsx_reset(sc);
		break;
	case power_up:
		if (bootverbose)
			device_printf(sc->rtsx_dev, "Powering up sd/mmc\n");

		if (sc->rtsx_reg_vmmc)
			regulator_enable(sc->rtsx_reg_vmmc);
		if (sc->rtsx_reg_vqmmc)
			regulator_enable(sc->rtsx_reg_vqmmc);
		rtsx_init(sc);
		break;
	};

	/* Enable ddr mode if needed */
	reg = RTSX_READ_4(sc, RTSX_GCTL);
	if (ios->timing == bus_timing_uhs_ddr50 ||
	  ios->timing == bus_timing_mmc_ddr52)
		reg |= RTSX_GCTL_DDR_MOD_SEL;
	else
		reg &= ~RTSX_GCTL_DDR_MOD_SEL;
	RTSX_WRITE_4(sc, RTSX_GCTL, reg);

	if (ios->clock && ios->clock != sc->rtsx_clock) {
		sc->rtsx_clock = clock = ios->clock;

		/* Disable clock */
		error = rtsx_update_clock(sc, 0);
		if (error != 0)
			return (error);

		if (ios->timing == bus_timing_mmc_ddr52 &&
		    (sc->rtsx_conf->new_timing ||
		    ios->bus_width == bus_width_8)) {
			div = 2;
			clock <<= 1;
		}

		/* Reset the divider. */
		reg = RTSX_READ_4(sc, RTSX_CKCR);
		reg &= ~RTSX_CKCR_DIV;
		reg |= div - 1;
		RTSX_WRITE_4(sc, RTSX_CKCR, reg);

		/* New timing mode if needed */
		if (sc->rtsx_conf->new_timing) {
			reg = RTSX_READ_4(sc, RTSX_NTSR);
			reg |= RTSX_NTSR_MODE_SELECT;
			RTSX_WRITE_4(sc, RTSX_NTSR, reg);
		}

		/* Set the MMC clock. */
		error = clk_disable(sc->rtsx_clk_mmc);
		if (error != 0 && bootverbose)
			device_printf(sc->rtsx_dev,
			  "failed to disable mmc clock: %d\n", error);
		error = clk_set_freq(sc->rtsx_clk_mmc, clock,
		    CLK_SET_ROUND_DOWN);
		if (error != 0) {
			device_printf(sc->rtsx_dev,
			    "failed to set frequency to %u Hz: %d\n",
			    clock, error);
			return (error);
		}
		error = clk_enable(sc->rtsx_clk_mmc);
		if (error != 0 && bootverbose)
			device_printf(sc->rtsx_dev,
			  "failed to re-enable mmc clock: %d\n", error);

		if (sc->rtsx_conf->can_calibrate)
			RTSX_WRITE_4(sc, RTSX_SAMP_DL, RTSX_SAMP_DL_SW_EN);

		/* Enable clock. */
		error = rtsx_update_clock(sc, 1);
		if (error != 0)
			return (error);
	}


	return (0);
}

static int
rtsx_get_ro(device_t bus, device_t child)
{

	return (0);
}

static int
rtsx_acquire_host(device_t bus, device_t child)
{
	struct rtsx_softc *sc;
	int error;

	sc = device_get_softc(bus);
	RTSX_LOCK(sc);
	while (sc->rtsx_bus_busy) {
		error = msleep(sc, &sc->rtsx_mtx, PCATCH, "mmchw", 0);
		if (error != 0) {
			RTSX_UNLOCK(sc);
			return (error);
		}
	}
	sc->rtsx_bus_busy++;
	RTSX_UNLOCK(sc);

	return (0);
}


static int
rtsx_release_host(device_t bus, device_t child)
{
	struct rtsx_softc *sc;

	sc = device_get_softc(bus);
	RTSX_LOCK(sc);
	sc->rtsx_bus_busy--;
	wakeup(sc);
	RTSX_UNLOCK(sc);

	return (0);
}

// XXX rtsx.c stuff
int
rtsx_activate(struct device *self, int act)
{
	struct rtsx_softc *sc = (struct rtsx_softc *)self;
	int rv = 0;

	switch (act) {
	case DVACT_SUSPEND:
		rv = config_activate_children(self, act);
		rtsx_save_regs(sc);
		break;
	case DVACT_RESUME:
		rtsx_restore_regs(sc);

		/* Handle cards ejected/inserted during suspend. */
		if (READ4(sc, RTSX_BIPR) & RTSX_SD_EXIST)
			rtsx_card_insert(sc);
		else
			rtsx_card_eject(sc);

		rv = config_activate_children(self, act);
		break;
	default:
		rv = config_activate_children(self, act);
		break;
	}
	return (rv);
}

int
rtsx_led_enable(struct rtsx_softc *sc)
{
	if (sc->flags & RTSX_F_5209) {
		RTSX_CLR(sc, RTSX_CARD_GPIO, RTSX_CARD_GPIO_LED_OFF);
		RTSX_WRITE(sc, RTSX_CARD_AUTO_BLINK,
		    RTSX_LED_BLINK_EN | RTSX_LED_BLINK_SPEED);
	} else {
		RTSX_SET(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
		RTSX_SET(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_AUTOBLINK);
	}

	return 0;
}

int
rtsx_led_disable(struct rtsx_softc *sc)
{
	if (sc->flags & RTSX_F_5209) {
		RTSX_CLR(sc, RTSX_CARD_AUTO_BLINK, RTSX_LED_BLINK_EN);
		RTSX_WRITE(sc, RTSX_CARD_GPIO, RTSX_CARD_GPIO_LED_OFF);
	} else {
		RTSX_CLR(sc, RTSX_OLT_LED_CTL, RTSX_OLT_LED_AUTOBLINK);
		RTSX_CLR(sc, RTSX_GPIO_CTL, RTSX_GPIO_LED_ON);
	}

	return 0;
}

/*
 * Reset the host controller.  Called during initialization, when
 * cards are removed, upon resume, and during error recovery.
 */
int
rtsx_host_reset(sdmmc_chipset_handle_t sch)
{
	struct rtsx_softc *sc = sch;
	int s;

	DPRINTF(1,("%s: host reset\n", DEVNAME(sc)));

	s = splsdmmc();

	if (ISSET(sc->flags, RTSX_F_CARD_PRESENT))
		rtsx_soft_reset(sc);

	if (rtsx_init(sc, 0)) {
		splx(s);
		return 1;
	}

	splx(s);
	return 0;
}

uint32_t
rtsx_host_ocr(sdmmc_chipset_handle_t sch)
{
	return RTSX_SUPPORT_VOLTAGE;
}

int
rtsx_host_maxblklen(sdmmc_chipset_handle_t sch)
{
	return 512;
}

/*
 * Return non-zero if the card is currently inserted.
 */
int
rtsx_card_detect(sdmmc_chipset_handle_t sch)
{
	struct rtsx_softc *sc = sch;

	return ISSET(sc->flags, RTSX_F_CARD_PRESENT);
}

/*
 * Notice that the meaning of RTSX_PWR_GATE_CTRL changes between RTS5209 and
 * RTS5229. In RTS5209 it is a mask of disabled power gates, while in RTS5229
 * it is a mask of *enabled* gates.
 */

int
rtsx_bus_power_off(struct rtsx_softc *sc)
{
	int error;
	uint8_t disable3;

	error = rtsx_stop_sd_clock(sc);
	if (error)
		return error;

	/* Disable SD output. */
	RTSX_CLR(sc, RTSX_CARD_OE, RTSX_CARD_OUTPUT_EN);

	/* Turn off power. */
	disable3 = RTSX_PULL_CTL_DISABLE3;
	if (sc->flags & RTSX_F_5209)
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else {
		RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1 |
		    RTSX_LDO3318_VCC2);
		if (sc->flags & RTSX_F_5229_TYPE_C)
			disable3 = RTSX_PULL_CTL_DISABLE3_TYPE_C;
	}

	RTSX_SET(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_PMOS_STRG_800mA);

	/* Disable pull control. */
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_DISABLE12);
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_DISABLE12);
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, disable3);

	return 0;
}

int
rtsx_bus_power_on(struct rtsx_softc *sc)
{
	uint8_t enable3;
	int err;

	if (sc->flags & RTSX_F_525A) {
		err = rtsx_write(sc, RTSX_LDO_VCC_CFG1, RTSX_LDO_VCC_TUNE_MASK,
		    RTSX_LDO_VCC_3V3);
		if (err)
			return (err);
	}

	/* Select SD card. */
	RTSX_WRITE(sc, RTSX_CARD_SELECT, RTSX_SD_MOD_SEL);
	RTSX_WRITE(sc, RTSX_CARD_SHARE_MODE, RTSX_CARD_SHARE_48_SD);
	RTSX_SET(sc, RTSX_CARD_CLK_EN, RTSX_SD_CLK_EN);

	/* Enable pull control. */
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL1, RTSX_PULL_CTL_ENABLE12);
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL2, RTSX_PULL_CTL_ENABLE12);
	if (sc->flags & RTSX_F_5229_TYPE_C)
		enable3 = RTSX_PULL_CTL_ENABLE3_TYPE_C;
	else
		enable3 = RTSX_PULL_CTL_ENABLE3;
	RTSX_WRITE(sc, RTSX_CARD_PULL_CTL3, enable3);

	/*
	 * To avoid a current peak, enable card power in two phases with a
	 * delay in between.
	 */

	/* Partial power. */
	RTSX_SET(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PARTIAL_PWR_ON);
	if (sc->flags & RTSX_F_5209)
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_SUSPEND);
	else
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC1);

	DELAY(200);

	/* Full power. */
	RTSX_CLR(sc, RTSX_CARD_PWR_CTL, RTSX_SD_PWR_OFF);
	if (sc->flags & RTSX_F_5209)
		RTSX_CLR(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_OFF);
	else
		RTSX_SET(sc, RTSX_PWR_GATE_CTRL, RTSX_LDO3318_VCC2);

	/* Enable SD card output. */
	RTSX_WRITE(sc, RTSX_CARD_OE, RTSX_SD_OUTPUT_EN);

	return 0;
}

int
rtsx_set_bus_width(struct rtsx_softc *sc, int w)
{
	uint32_t bus_width;
	int error;

	switch (w) {
		case 8:
			bus_width = RTSX_BUS_WIDTH_8;
			break;
		case 4:
			bus_width = RTSX_BUS_WIDTH_4;
			break;
		case 1:
		default:
			bus_width = RTSX_BUS_WIDTH_1;
			break;
	}

	error = rtsx_write(sc, RTSX_SD_CFG1, RTSX_BUS_WIDTH_MASK, bus_width);
	return error;
}

int
rtsx_stop_sd_clock(struct rtsx_softc *sc)
{
	RTSX_CLR(sc, RTSX_CARD_CLK_EN, RTSX_CARD_CLK_EN_ALL);
	RTSX_SET(sc, RTSX_SD_BUS_STAT, RTSX_SD_CLK_FORCE_STOP);

	return 0;
}

int
rtsx_switch_sd_clock(struct rtsx_softc *sc, uint8_t n, int div, int mcu)
{
	/* Enable SD 2.0 mode. */
	RTSX_CLR(sc, RTSX_SD_CFG1, RTSX_SD_MODE_MASK);

	RTSX_SET(sc, RTSX_CLK_CTL, RTSX_CLK_LOW_FREQ);

	RTSX_WRITE(sc, RTSX_CARD_CLK_SOURCE,
	    RTSX_CRC_FIX_CLK | RTSX_SD30_VAR_CLK0 | RTSX_SAMPLE_VAR_CLK1);
	RTSX_CLR(sc, RTSX_SD_SAMPLE_POINT_CTL, RTSX_SD20_RX_SEL_MASK);
	RTSX_WRITE(sc, RTSX_SD_PUSH_POINT_CTL, RTSX_SD20_TX_NEG_EDGE);
	RTSX_WRITE(sc, RTSX_CLK_DIV, (div << 4) | mcu);
	RTSX_CLR(sc, RTSX_SSC_CTL1, RTSX_RSTB);
	RTSX_CLR(sc, RTSX_SSC_CTL2, RTSX_SSC_DEPTH_MASK);
	RTSX_WRITE(sc, RTSX_SSC_DIV_N_0, n);
	RTSX_SET(sc, RTSX_SSC_CTL1, RTSX_RSTB);
	DELAY(100);

	RTSX_CLR(sc, RTSX_CLK_CTL, RTSX_CLK_LOW_FREQ);

	return 0;
}

/*
 * Set or change SD bus voltage and enable or disable SD bus power.
 * Return zero on success.
 */
int
rtsx_bus_power(sdmmc_chipset_handle_t sch, uint32_t ocr)
{
	struct rtsx_softc *sc = sch;
	int s, error = 0;

	DPRINTF(1,("%s: voltage change ocr=0x%x\n", DEVNAME(sc), ocr));

	s = splsdmmc();

	/*
	 * Disable bus power before voltage change.
	 */
	error = rtsx_bus_power_off(sc);
	if (error)
		goto ret;

	DELAY(200);

	/* If power is disabled, reset the host and return now. */
	if (ocr == 0) {
		splx(s);
		(void)rtsx_host_reset(sc);
		return 0;
	}

	if (!ISSET(ocr, RTSX_SUPPORT_VOLTAGE)) {
		/* Unsupported voltage level requested. */
		DPRINTF(1,("%s: unsupported voltage ocr=0x%x\n",
		    DEVNAME(sc), ocr));
		error = EINVAL;
		goto ret;
	}

	error = rtsx_bus_power_on(sc);
	if (error)
		goto ret;

	error = rtsx_set_bus_width(sc, 1);
ret:
	splx(s);
	return error;
}

/*
 * Set or change SDCLK frequency or disable the SD clock.
 * Return zero on success.
 */
int
rtsx_bus_clock(sdmmc_chipset_handle_t sch, int freq, int timing)
{
	struct rtsx_softc *sc = sch;
	int s;
	uint8_t n;
	int div;
	int mcu;
	int error = 0;

	s = splsdmmc();

	if (freq == SDMMC_SDCLK_OFF) {
		error = rtsx_stop_sd_clock(sc);
		goto ret;
	}

	/* Round down to a supported frequency. */
	if (freq >= SDMMC_SDCLK_50MHZ)
		freq = SDMMC_SDCLK_50MHZ;
	else if (freq >= SDMMC_SDCLK_25MHZ)
		freq = SDMMC_SDCLK_25MHZ;
	else
		freq = SDMMC_SDCLK_400KHZ;

	/*
	 * Configure the clock frequency.
	 */
	switch (freq) {
	case SDMMC_SDCLK_400KHZ:
		n = 80; /* minimum */
		div = RTSX_CLK_DIV_8;
		mcu = 7;
		RTSX_SET(sc, RTSX_SD_CFG1, RTSX_CLK_DIVIDE_128);
		break;
	case SDMMC_SDCLK_25MHZ:
		n = 100;
		div = RTSX_CLK_DIV_4;
		mcu = 7;
		RTSX_CLR(sc, RTSX_SD_CFG1, RTSX_CLK_DIVIDE_MASK);
		break;
	case SDMMC_SDCLK_50MHZ:
		n = 100;
		div = RTSX_CLK_DIV_2;
		mcu = 7;
		RTSX_CLR(sc, RTSX_SD_CFG1, RTSX_CLK_DIVIDE_MASK);
		break;
	default:
		error = EINVAL;
		goto ret;
	}

	/*
	 * Enable SD clock.
	 */
	error = rtsx_switch_sd_clock(sc, n, div, mcu);
ret:
	splx(s);
	return error;
}

int
rtsx_bus_width(sdmmc_chipset_handle_t sch, int width)
{
	struct rtsx_softc *sc = sch;

	return rtsx_set_bus_width(sc, width);
}

static int
rtsx_read(struct rtsx_softc *sc, uint16_t addr, uint8_t *val)
{
	int tries = 1024;
	uint32_t reg;
	
	WRITE4(sc, RTSX_HAIMR, RTSX_HAIMR_BUSY |
	    (uint32_t)((addr & 0x3FFF) << 16));

	while (tries--) {
		reg = READ4(sc, RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY))
			break;
	}

	*val = (reg & 0xff);
	return (tries == 0) ? ETIMEDOUT : 0;
}

static int
rtsx_write(struct rtsx_softc *sc, uint16_t addr, uint8_t mask, uint8_t val)
{
	int tries = 1024;
	uint32_t reg;

	WRITE4(sc, RTSX_HAIMR,
	    RTSX_HAIMR_BUSY | RTSX_HAIMR_WRITE |
	    (uint32_t)(((addr & 0x3FFF) << 16) |
	    (mask << 8) | val));

	while (tries--) {
		reg = READ4(sc, RTSX_HAIMR);
		if (!(reg & RTSX_HAIMR_BUSY)) {
			if (val != (reg & 0xff))
				return EIO;
			return 0;
		}
	}

	return ETIMEDOUT;
}

#ifdef notyet
int
rtsx_read_phy(struct rtsx_softc *sc, uint8_t addr, uint16_t *val)
{
	int timeout = 100000;
	uint8_t data0;
	uint8_t data1;
	uint8_t rwctl;

	RTSX_WRITE(sc, RTSX_PHY_ADDR, addr);
	RTSX_WRITE(sc, RTSX_PHY_RWCTL, RTSX_PHY_BUSY|RTSX_PHY_READ);

	while (timeout--) {
		RTSX_READ(sc, RTSX_PHY_RWCTL, &rwctl);
		if (!(rwctl & RTSX_PHY_BUSY))
			break;
	}
	
	if (timeout == 0)
		return ETIMEDOUT;
		
	RTSX_READ(sc, RTSX_PHY_DATA0, &data0);
	RTSX_READ(sc, RTSX_PHY_DATA1, &data1);
	*val = data0 | (data1 << 8);

	return 0;
}
#endif

static int
rtsx_write_phy(struct rtsx_softc *sc, uint8_t addr, uint16_t val)
{
	int timeout = 100000;
	uint8_t rwctl;

	RTSX_WRITE(sc, RTSX_PHY_DATA0, val);
	RTSX_WRITE(sc, RTSX_PHY_DATA1, val >> 8);
	RTSX_WRITE(sc, RTSX_PHY_ADDR, addr);
	RTSX_WRITE(sc, RTSX_PHY_RWCTL, RTSX_PHY_BUSY|RTSX_PHY_WRITE);

	while (timeout--) {
		RTSX_READ(sc, RTSX_PHY_RWCTL, &rwctl);
		if (!(rwctl & RTSX_PHY_BUSY))
			break;
	}
	
	if (timeout == 0)
		return ETIMEDOUT;
		
	return 0;
}

static int
rtsx_read_cfg(struct rtsx_softc *sc, uint8_t func, uint16_t addr,
    uint32_t *val)
{
	int tries = 1024;
	uint8_t data0, data1, data2, data3, rwctl;

	RTSX_WRITE(sc, RTSX_CFGADDR0, addr);
	RTSX_WRITE(sc, RTSX_CFGADDR1, addr >> 8);
	RTSX_WRITE(sc, RTSX_CFGRWCTL, RTSX_CFG_BUSY | (func & 0x03 << 4));

	while (tries--) {
		RTSX_READ(sc, RTSX_CFGRWCTL, &rwctl);
		if (!(rwctl & RTSX_CFG_BUSY))
			break;
	}

	if (tries == 0)
		return EIO;
	
	RTSX_READ(sc, RTSX_CFGDATA0, &data0);
	RTSX_READ(sc, RTSX_CFGDATA1, &data1);
	RTSX_READ(sc, RTSX_CFGDATA2, &data2);
	RTSX_READ(sc, RTSX_CFGDATA3, &data3);

	*val = (data3 << 24) | (data2 << 16) | (data1 << 8) | data0;

	return 0;
}

#ifdef notyet
int
rtsx_write_cfg(struct rtsx_softc *sc, uint8_t func, uint16_t addr,
    uint32_t mask, uint32_t val)
{
	int i, writemask = 0, tries = 1024;
	uint8_t rwctl;

	for (i = 0; i < 4; i++) {
		if (mask & 0xff) {
			RTSX_WRITE(sc, RTSX_CFGDATA0 + i, val & mask & 0xff);
			writemask |= (1 << i);
		}
		mask >>= 8;
		val >>= 8;
	}

	if (writemask) {
		RTSX_WRITE(sc, RTSX_CFGADDR0, addr);
		RTSX_WRITE(sc, RTSX_CFGADDR1, addr >> 8);
		RTSX_WRITE(sc, RTSX_CFGRWCTL,
		    RTSX_CFG_BUSY | writemask | (func & 0x03 << 4));
	}

	while (tries--) {
		RTSX_READ(sc, RTSX_CFGRWCTL, &rwctl);
		if (!(rwctl & RTSX_CFG_BUSY))
			break;
	}

	if (tries == 0)
		return EIO;
	
	return 0;
}
#endif

/* Append a properly encoded host command to the host command buffer. */
void
rtsx_hostcmd(uint32_t *cmdbuf, int *n, uint8_t cmd, uint16_t reg,
    uint8_t mask, uint8_t data)
{
	KASSERT(*n < RTSX_HOSTCMD_MAX);

	cmdbuf[(*n)++] = htole32((uint32_t)(cmd & 0x3) << 30) |
	    ((uint32_t)(reg & 0x3fff) << 16) |
	    ((uint32_t)(mask) << 8) |
	    ((uint32_t)data);
}

void
rtsx_save_regs(struct rtsx_softc *sc)
{
	int s, i;
	uint16_t reg;

	s = splsdmmc();

	i = 0;
	for (reg = 0xFDA0; reg < 0xFDAE; reg++)
		(void)rtsx_read(sc, reg, &sc->regs[i++]);
	for (reg = 0xFD52; reg < 0xFD69; reg++)
		(void)rtsx_read(sc, reg, &sc->regs[i++]);
	for (reg = 0xFE20; reg < 0xFE34; reg++)
		(void)rtsx_read(sc, reg, &sc->regs[i++]);

	sc->regs4[0] = READ4(sc, RTSX_HCBAR);
	sc->regs4[1] = READ4(sc, RTSX_HCBCTLR);
	sc->regs4[2] = READ4(sc, RTSX_HDBAR);
	sc->regs4[3] = READ4(sc, RTSX_HDBCTLR);
	sc->regs4[4] = READ4(sc, RTSX_HAIMR);
	sc->regs4[5] = READ4(sc, RTSX_BIER);
	/* Not saving RTSX_BIPR. */

	splx(s);
}

void
rtsx_restore_regs(struct rtsx_softc *sc)
{
	int s, i;
	uint16_t reg;

	s = splsdmmc();

	WRITE4(sc, RTSX_HCBAR, sc->regs4[0]);
	WRITE4(sc, RTSX_HCBCTLR, sc->regs4[1]);
	WRITE4(sc, RTSX_HDBAR, sc->regs4[2]);
	WRITE4(sc, RTSX_HDBCTLR, sc->regs4[3]);
	WRITE4(sc, RTSX_HAIMR, sc->regs4[4]);
	WRITE4(sc, RTSX_BIER, sc->regs4[5]);
	/* Not writing RTSX_BIPR since doing so would clear it. */

	i = 0;
	for (reg = 0xFDA0; reg < 0xFDAE; reg++)
		(void)rtsx_write(sc, reg, 0xff, sc->regs[i++]);
	for (reg = 0xFD52; reg < 0xFD69; reg++)
		(void)rtsx_write(sc, reg, 0xff, sc->regs[i++]);
	for (reg = 0xFE20; reg < 0xFE34; reg++)
		(void)rtsx_write(sc, reg, 0xff, sc->regs[i++]);

	splx(s);
}

uint8_t
rtsx_response_type(uint16_t sdmmc_rsp)
{
	int i;
	struct rsp_type {
		uint16_t sdmmc_rsp;
		uint8_t rtsx_rsp;
	} rsp_types[] = {
		{ SCF_RSP_R0,	RTSX_SD_RSP_TYPE_R0 },
		{ SCF_RSP_R1,	RTSX_SD_RSP_TYPE_R1 },
		{ SCF_RSP_R1B,	RTSX_SD_RSP_TYPE_R1B },
		{ SCF_RSP_R2,	RTSX_SD_RSP_TYPE_R2 },
		{ SCF_RSP_R3,	RTSX_SD_RSP_TYPE_R3 },
		{ SCF_RSP_R4,	RTSX_SD_RSP_TYPE_R4 },
		{ SCF_RSP_R5,	RTSX_SD_RSP_TYPE_R5 },
		{ SCF_RSP_R6,	RTSX_SD_RSP_TYPE_R6 },
		{ SCF_RSP_R7,	RTSX_SD_RSP_TYPE_R7 }
	};

	for (i = 0; i < nitems(rsp_types); i++) {
		if (sdmmc_rsp == rsp_types[i].sdmmc_rsp)
			return rsp_types[i].rtsx_rsp;
	}

	return 0;
}

int
rtsx_hostcmd_send(struct rtsx_softc *sc, int ncmd)
{
	int s;

	s = splsdmmc();

	/* Tell the chip where the command buffer is and run the commands. */
	WRITE4(sc, RTSX_HCBAR, sc->dmap_cmd->dm_segs[0].ds_addr);
	WRITE4(sc, RTSX_HCBCTLR,
	    ((ncmd * 4) & 0x00ffffff) | RTSX_START_CMD | RTSX_HW_AUTO_RSP);

	splx(s);

	return 0;
}

int
rtsx_xfer_exec(struct rtsx_softc *sc, bus_dmamap_t dmap, int dmaflags)
{
	int s = splsdmmc();

	/* Tell the chip where the data buffer is and run the transfer. */
	WRITE4(sc, RTSX_HDBAR, dmap->dm_segs[0].ds_addr);
	WRITE4(sc, RTSX_HDBCTLR, dmaflags);

	splx(s);

	/* Wait for completion. */
	return rtsx_wait_intr(sc, RTSX_TRANS_OK_INT, 10*hz);
}

int
rtsx_xfer(struct rtsx_softc *sc, struct sdmmc_command *cmd, uint32_t *cmdbuf)
{
	int ncmd, dma_dir, error, tmode;
	int read = ISSET(cmd->c_flags, SCF_CMD_READ);
	uint8_t cfg2;

	DPRINTF(3,("%s: %s xfer: %d bytes with block size %d\n", DEVNAME(sc),
	    read ? "read" : "write",
	    cmd->c_datalen, cmd->c_blklen));

	if (cmd->c_datalen > RTSX_DMA_DATA_BUFSIZE) {
		DPRINTF(3, ("%s: cmd->c_datalen too large: %d > %d\n",
		    DEVNAME(sc), cmd->c_datalen, RTSX_DMA_DATA_BUFSIZE));
		return ENOMEM;
	}

	/* Configure DMA transfer mode parameters. */
	cfg2 = RTSX_SD_NO_CHECK_WAIT_CRC_TO | RTSX_SD_CHECK_CRC16 |
	    RTSX_SD_NO_WAIT_BUSY_END | RTSX_SD_RSP_LEN_0;
	if (read) {
		dma_dir = RTSX_DMA_DIR_FROM_CARD;
		/* Use transfer mode AUTO_READ3, which assumes we've already
		 * sent the read command and gotten the response, and will
		 * send CMD 12 manually after reading multiple blocks. */
		tmode = RTSX_TM_AUTO_READ3;
		cfg2 |= RTSX_SD_CALCULATE_CRC7 | RTSX_SD_CHECK_CRC7;
	} else {
		dma_dir = RTSX_DMA_DIR_TO_CARD;
		/* Use transfer mode AUTO_WRITE3, which assumes we've already
		 * sent the write command and gotten the response, and will
		 * send CMD 12 manually after writing multiple blocks. */
		tmode = RTSX_TM_AUTO_WRITE3;
		cfg2 |= RTSX_SD_NO_CALCULATE_CRC7 | RTSX_SD_NO_CHECK_CRC7;
	}

	ncmd = 0;

	rtsx_hostcmd(cmdbuf, &ncmd, RTSX_WRITE_REG_CMD, RTSX_SD_CFG2,
	    0xff, cfg2); 

	/* Queue commands to configure data transfer size. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_L, 0xff,
	    (cmd->c_blklen & 0xff));
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BYTE_CNT_H, 0xff,
	    (cmd->c_blklen >> 8));
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_L, 0xff,
	    ((cmd->c_datalen / cmd->c_blklen) & 0xff));
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_BLOCK_CNT_H, 0xff,
	    ((cmd->c_datalen / cmd->c_blklen) >> 8));

	/* Use the DMA ring buffer for commands which transfer data. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE, 0x01, RTSX_RING_BUFFER);

	/* Configure DMA controller. */
	rtsx_hostcmd(cmdbuf, &ncmd, RTSX_WRITE_REG_CMD, RTSX_IRQSTAT0,
	    RTSX_DMA_DONE_INT, RTSX_DMA_DONE_INT);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC3, 0xff, cmd->c_datalen >> 24);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC2, 0xff, cmd->c_datalen >> 16);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC1, 0xff, cmd->c_datalen >> 8);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMATC0, 0xff, cmd->c_datalen);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_DMACTL,
	    0x03 | RTSX_DMA_PACK_SIZE_MASK,
	    dma_dir | RTSX_DMA_EN | RTSX_DMA_512);

	/* Queue commands to perform SD transfer. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
	    0xff, tmode | RTSX_SD_TRANSFER_START);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
	    RTSX_SD_TRANSFER_END, RTSX_SD_TRANSFER_END);

	error = rtsx_hostcmd_send(sc, ncmd);
	if (error)
		goto ret;

	if (cmd->c_dmamap)
		error = rtsx_xfer_adma(sc, cmd);
	else
		error = rtsx_xfer_bounce(sc, cmd);
ret:
	DPRINTF(3,("%s: xfer done, error=%d\n", DEVNAME(sc), error));
	return error;
}

int
rtsx_xfer_bounce(struct rtsx_softc *sc, struct sdmmc_command *cmd)
{
    	caddr_t datakvap;
	bus_dma_segment_t segs;
	int rsegs, error;
	int read = ISSET(cmd->c_flags, SCF_CMD_READ);

	/* Allocate and map DMA bounce buffer for data transfer. */
	error = bus_dmamem_alloc(sc->dmat, cmd->c_datalen, 0, 0, &segs, 1,
	    &rsegs, BUS_DMA_WAITOK|BUS_DMA_ZERO);
	if (error) {
		DPRINTF(3, ("%s: could not allocate %d bytes\n",
		    DEVNAME(sc), cmd->c_datalen));
		return error;
	}
	error = bus_dmamem_map(sc->dmat, &segs, rsegs, cmd->c_datalen,
	    &datakvap, BUS_DMA_WAITOK|BUS_DMA_COHERENT);
	if (error) {
		DPRINTF(3, ("%s: could not map data buffer\n", DEVNAME(sc)));
		goto free_databuf;
	}

	/* If this is a write, copy data from sdmmc-provided buffer. */
	if (!read)
		memcpy(datakvap, cmd->c_data, cmd->c_datalen);

	/* Load the data buffer and sync it. */
	error = bus_dmamap_load(sc->dmat, sc->dmap_data, datakvap,
	    cmd->c_datalen, NULL, BUS_DMA_WAITOK);
	if (error) {
		DPRINTF(3, ("%s: could not load DMA map\n", DEVNAME(sc)));
		goto unmap_databuf;
	}
	bus_dmamap_sync(sc->dmat, sc->dmap_data, 0, cmd->c_datalen,
	    BUS_DMASYNC_PREREAD);
	bus_dmamap_sync(sc->dmat, sc->dmap_data, 0, cmd->c_datalen,
	    BUS_DMASYNC_PREWRITE);

	error = rtsx_xfer_exec(sc, sc->dmap_data,
	    RTSX_TRIG_DMA | (read ? RTSX_DMA_READ : 0) |
	    (cmd->c_datalen & 0x00ffffff));
	if (error)
		goto unload_databuf;

	/* Sync and unload data DMA buffer. */
	bus_dmamap_sync(sc->dmat, sc->dmap_data, 0, cmd->c_datalen,
	    BUS_DMASYNC_POSTREAD);
	bus_dmamap_sync(sc->dmat, sc->dmap_data, 0, cmd->c_datalen,
	    BUS_DMASYNC_POSTWRITE);

unload_databuf:
	bus_dmamap_unload(sc->dmat, sc->dmap_data);

	/* If this is a read, copy data into sdmmc-provided buffer. */
	if (error == 0 && read)
		memcpy(cmd->c_data, datakvap, cmd->c_datalen);

	/* Free DMA data buffer. */
unmap_databuf:
	bus_dmamem_unmap(sc->dmat, datakvap, cmd->c_datalen);
free_databuf:
	bus_dmamem_free(sc->dmat, &segs, rsegs);
	return error;
}

int
rtsx_xfer_adma(struct rtsx_softc *sc, struct sdmmc_command *cmd)
{
	int i, error;
	uint64_t *descp;
	int read = ISSET(cmd->c_flags, SCF_CMD_READ);

	/* Initialize scatter-gather transfer descriptors. */
	descp = (uint64_t *)sc->admabuf;
	for (i = 0; i < cmd->c_dmamap->dm_nsegs; i++) {
		uint64_t paddr = cmd->c_dmamap->dm_segs[i].ds_addr;
		uint64_t len = cmd->c_dmamap->dm_segs[i].ds_len;
		uint8_t sgflags = RTSX_SG_VALID | RTSX_SG_TRANS_DATA;
		uint64_t desc;

		if (i == cmd->c_dmamap->dm_nsegs - 1)
			sgflags |= RTSX_SG_END;
		len &= 0x00ffffff;
		desc = htole64((paddr << 32) | (len << 12) | sgflags);
		memcpy(descp, &desc, sizeof(*descp));
		descp++;
	}

	error = bus_dmamap_load(sc->dmat, sc->dmap_adma, sc->admabuf,
	    RTSX_ADMA_DESC_SIZE, NULL, BUS_DMA_WAITOK);
	if (error) {
		DPRINTF(3, ("%s: could not load DMA map\n", DEVNAME(sc)));
		return error;
	}
	bus_dmamap_sync(sc->dmat, sc->dmap_adma, 0, RTSX_ADMA_DESC_SIZE,
	    	BUS_DMASYNC_PREWRITE);

	error = rtsx_xfer_exec(sc, sc->dmap_adma,
	    RTSX_ADMA_MODE | RTSX_TRIG_DMA | (read ? RTSX_DMA_READ : 0));

	bus_dmamap_sync(sc->dmat, sc->dmap_adma, 0, RTSX_ADMA_DESC_SIZE,
	    	BUS_DMASYNC_POSTWRITE);

	bus_dmamap_unload(sc->dmat, sc->dmap_adma);
	return error;
}

void
rtsx_exec_command(sdmmc_chipset_handle_t sch, struct sdmmc_command *cmd)
{
	struct rtsx_softc *sc = sch;
	bus_dma_segment_t segs;
	int rsegs;
	caddr_t cmdkvap;
	uint32_t *cmdbuf;
	uint8_t rsp_type;
	uint16_t r;
	int ncmd;
	int error = 0;

	DPRINTF(3,("%s: executing cmd %hu\n", DEVNAME(sc), cmd->c_opcode));

	/* Refuse SDIO probe if the chip doesn't support SDIO. */
	if (cmd->c_opcode == SD_IO_SEND_OP_COND &&
	    !ISSET(sc->flags, RTSX_F_SDIO_SUPPORT)) {
		error = ENOTSUP;
		goto ret;
	}

	rsp_type = rtsx_response_type(cmd->c_flags & 0xff00);
	if (rsp_type == 0) {
		printf("%s: unknown response type 0x%x\n", DEVNAME(sc),
			(cmd->c_flags & 0xff00));
		error = EINVAL;
		goto ret;
	}

	/* Allocate and map the host command buffer. */
	error = bus_dmamem_alloc(sc->dmat, RTSX_HOSTCMD_BUFSIZE, 0, 0, &segs, 1,
	    &rsegs, BUS_DMA_WAITOK|BUS_DMA_ZERO);
	if (error)
		goto ret;
	error = bus_dmamem_map(sc->dmat, &segs, rsegs, RTSX_HOSTCMD_BUFSIZE,
	    &cmdkvap, BUS_DMA_WAITOK|BUS_DMA_COHERENT);
	if (error)
		goto free_cmdbuf;

	/* The command buffer queues commands the host controller will
	 * run asynchronously. */
	cmdbuf = (uint32_t *)cmdkvap;
	ncmd = 0;

	/* Queue commands to set SD command index and argument. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD0, 0xff, 0x40 | cmd->c_opcode); 
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD1, 0xff, cmd->c_arg >> 24);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD2, 0xff, cmd->c_arg >> 16);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD3, 0xff, cmd->c_arg >> 8);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CMD4, 0xff, cmd->c_arg);

	/* Queue command to set response type. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_CFG2, 0xff, rsp_type);

	/* Use the ping-pong buffer for commands which do not transfer data. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_CARD_DATA_SOURCE,
	    0x01, RTSX_PINGPONG_BUFFER);

	/* Queue commands to perform SD transfer. */
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_WRITE_REG_CMD, RTSX_SD_TRANSFER,
	    0xff, RTSX_TM_CMD_RSP | RTSX_SD_TRANSFER_START);
	rtsx_hostcmd(cmdbuf, &ncmd,
	    RTSX_CHECK_REG_CMD, RTSX_SD_TRANSFER,
	    RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE,
	    RTSX_SD_TRANSFER_END|RTSX_SD_STAT_IDLE);

	/* Queue commands to read back card status response.*/
	if (rsp_type == RTSX_SD_RSP_TYPE_R2) {
		for (r = RTSX_PPBUF_BASE2 + 15; r > RTSX_PPBUF_BASE2; r--)
			rtsx_hostcmd(cmdbuf, &ncmd, RTSX_READ_REG_CMD, r, 0, 0);
		rtsx_hostcmd(cmdbuf, &ncmd, RTSX_READ_REG_CMD, RTSX_SD_CMD5,
		    0, 0);
	} else if (rsp_type != RTSX_SD_RSP_TYPE_R0) {
		for (r = RTSX_SD_CMD0; r <= RTSX_SD_CMD4; r++)
			rtsx_hostcmd(cmdbuf, &ncmd, RTSX_READ_REG_CMD, r, 0, 0);
	}

	/* Load and sync command DMA buffer. */
	error = bus_dmamap_load(sc->dmat, sc->dmap_cmd, cmdkvap,
	    RTSX_HOSTCMD_BUFSIZE, NULL, BUS_DMA_WAITOK);
	if (error)
		goto unmap_cmdbuf;

	bus_dmamap_sync(sc->dmat, sc->dmap_cmd, 0, RTSX_HOSTCMD_BUFSIZE,
	    BUS_DMASYNC_PREREAD);
	bus_dmamap_sync(sc->dmat, sc->dmap_cmd, 0, RTSX_HOSTCMD_BUFSIZE,
	    BUS_DMASYNC_PREWRITE);

	/* Run the command queue and wait for completion. */
	error = rtsx_hostcmd_send(sc, ncmd);
	if (error == 0)
		error = rtsx_wait_intr(sc, RTSX_TRANS_OK_INT, hz);
	if (error)
		goto unload_cmdbuf;

	bus_dmamap_sync(sc->dmat, sc->dmap_cmd, 0, RTSX_HOSTCMD_BUFSIZE,
	    BUS_DMASYNC_POSTREAD);
	bus_dmamap_sync(sc->dmat, sc->dmap_cmd, 0, RTSX_HOSTCMD_BUFSIZE,
	    BUS_DMASYNC_POSTWRITE);

	/* Copy card response into sdmmc response buffer. */
	if (ISSET(cmd->c_flags, SCF_RSP_PRESENT)) {
		/* Copy bytes like sdhc(4), which on little-endian uses
		 * different byte order for short and long responses... */
		if (ISSET(cmd->c_flags, SCF_RSP_136)) {
			memcpy(cmd->c_resp, cmdkvap + 1, sizeof(cmd->c_resp));
		} else {
			/* First byte is CHECK_REG_CMD return value, second
			 * one is the command op code -- we skip those. */
			cmd->c_resp[0] =
			    ((betoh32(cmdbuf[0]) & 0x0000ffff) << 16) |
			    ((betoh32(cmdbuf[1]) & 0xffff0000) >> 16);
		}
	}

	if (cmd->c_data) {
		error = rtsx_xfer(sc, cmd, cmdbuf);
		if (error) {
			uint8_t stat1;

			if (rtsx_read(sc, RTSX_SD_STAT1, &stat1) == 0 &&
			    (stat1 & RTSX_SD_CRC_ERR))
				printf("%s: CRC error\n", DEVNAME(sc));
		}
	}

unload_cmdbuf:
	bus_dmamap_unload(sc->dmat, sc->dmap_cmd);
unmap_cmdbuf:
	bus_dmamem_unmap(sc->dmat, cmdkvap, RTSX_HOSTCMD_BUFSIZE);
free_cmdbuf:
	bus_dmamem_free(sc->dmat, &segs, rsegs);
ret:
	SET(cmd->c_flags, SCF_ITSDONE);
	cmd->c_error = error;
}

/* Prepare for another command. */
void
rtsx_soft_reset(struct rtsx_softc *sc)
{
	DPRINTF(1,("%s: soft reset\n", DEVNAME(sc)));

	/* Stop command transfer. */
	WRITE4(sc, RTSX_HCBCTLR, RTSX_STOP_CMD);

	(void)rtsx_write(sc, RTSX_CARD_STOP, RTSX_SD_STOP|RTSX_SD_CLR_ERR,
		    RTSX_SD_STOP|RTSX_SD_CLR_ERR);

	/* Stop DMA transfer. */
	WRITE4(sc, RTSX_HDBCTLR, RTSX_STOP_DMA);
	(void)rtsx_write(sc, RTSX_DMACTL, RTSX_DMA_RST, RTSX_DMA_RST);

	(void)rtsx_write(sc, RTSX_RBCTL, RTSX_RB_FLUSH, RTSX_RB_FLUSH);
}

int
rtsx_wait_intr(struct rtsx_softc *sc, int mask, int timo)
{
	int status;
	int error = 0;
	int s;

	mask |= RTSX_TRANS_FAIL_INT;

	s = splsdmmc();
	status = sc->intr_status & mask;
	while (status == 0) {
		if (tsleep(&sc->intr_status, PRIBIO, "rtsxintr", timo)
		    == EWOULDBLOCK) {
			rtsx_soft_reset(sc);
			error = ETIMEDOUT;
			break;
		}
		status = sc->intr_status & mask;
	}
	sc->intr_status &= ~status;

	/* Has the card disappeared? */
	if (!ISSET(sc->flags, RTSX_F_CARD_PRESENT))
		error = ENODEV;

	splx(s);

	if (error == 0 && (status & RTSX_TRANS_FAIL_INT))
		error = EIO;

	return error;
}

void
rtsx_card_insert(struct rtsx_softc *sc)
{
	DPRINTF(1, ("%s: card inserted\n", DEVNAME(sc)));

	sc->flags |= RTSX_F_CARD_PRESENT;
	(void)rtsx_led_enable(sc);

	/* Schedule card discovery task. */
	sdmmc_needs_discover(sc->sdmmc);
}

void
rtsx_card_eject(struct rtsx_softc *sc)
{
	DPRINTF(1, ("%s: card ejected\n", DEVNAME(sc)));

	sc->flags &= ~RTSX_F_CARD_PRESENT;
	(void)rtsx_led_disable(sc);

	/* Schedule card discovery task. */
	sdmmc_needs_discover(sc->sdmmc);
}

/*
 * Established by attachment driver at interrupt priority IPL_SDMMC.
 */
int
rtsx_intr(void *arg)
{
	struct rtsx_softc *sc = arg;
	uint32_t enabled, status;

	enabled = READ4(sc, RTSX_BIER);
	status = READ4(sc, RTSX_BIPR);

	/* Ack interrupts. */
	WRITE4(sc, RTSX_BIPR, status);

	if (((enabled & status) == 0) || status == 0xffffffff)
		return 0;

	if (status & RTSX_SD_INT) {
		if (status & RTSX_SD_EXIST) {
			if (!ISSET(sc->flags, RTSX_F_CARD_PRESENT))
				rtsx_card_insert(sc);
		} else {
			rtsx_card_eject(sc);
		}
	}

	if (status & (RTSX_TRANS_OK_INT | RTSX_TRANS_FAIL_INT)) {
		sc->intr_status |= status;
		wakeup(&sc->intr_status);
	}

	return 1;
}

static device_method_t rtsx_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		rtsx_probe),
	DEVMETHOD(device_attach,	rtsx_attach),
	DEVMETHOD(device_detach,	rtsx_detach),

	/* Bus interface */
	DEVMETHOD(bus_read_ivar,	rtsx_read_ivar),
	DEVMETHOD(bus_write_ivar,	rtsx_write_ivar),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	rtsx_update_ios),
//	DEVMETHOD(mmcbr_tune,		sdhci_generic_tune),
//	DEVMETHOD(mmcbr_retune,		sdhci_generic_retune),
	DEVMETHOD(mmcbr_request,	rtsx_request),
	DEVMETHOD(mmcbr_get_ro,		rtsx_get_ro),
	DEVMETHOD(mmcbr_switch_vccq,	rtsx_switch_vccq),
	DEVMETHOD(mmcbr_acquire_host,	rtsx_acquire_host),
	DEVMETHOD(mmcbr_release_host,	rtsx_release_host),

	DEVMETHOD_END
};

static devclass_t rtsx_devclass;

static driver_t rtsx_driver = {
	"rtsx",
	rtsx_methods,
	sizeof(struct rtsx_softc),
};

DRIVER_MODULE(rtsx, pci, rtsx_driver, rtsx_devclass, NULL, NULL);
#ifndef MMCCAM
MMC_DECLARE_BRIDGE(rtsx);
#endif
