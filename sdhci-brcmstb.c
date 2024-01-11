#include <sys/param.h>
#include <sys/kernel.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/extres/clk/clk.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/module.h>
#include <sys/kdb.h>
#include <machine/bus.h>
#include <dev/mmc/bridge.h>
#include <dev/extres/regulator/regulator.h>
#include <dev/mmc/mmc_fdt_helpers.h>
#include <dev/mmc/mmcreg.h>
#include <dev/mmc/mmcbrvar.h>
#include <dev/sdhci/sdhci.h>
#include "mmcbr_if.h"
#include "sdhci_if.h"
#include "bcm2835_dma.h"
#include <arm/broadcom/bcm2835/bcm2835_vcbus.h>
#include <arm/broadcom/bcm2835/bcm2835_mbox_prop.h>

#define	BCM_SDHCI_BUFFER_SIZE		512
/*
 * NUM_DMA_SEGS is the number of DMA segments we want to accommodate on average.
 * We add in a number of segments based on how much we may need to spill into
 * another segment due to crossing page boundaries.  e.g. up to PAGE_SIZE, an
 * extra page is needed as we can cross a page boundary exactly once.
 */
#define	NUM_DMA_SEGS			1
#define	NUM_DMA_SPILL_SEGS		\
	((((NUM_DMA_SEGS * BCM_SDHCI_BUFFER_SIZE) - 1) / PAGE_SIZE) + 1)
#define	ALLOCATED_DMA_SEGS		(NUM_DMA_SEGS +	NUM_DMA_SPILL_SEGS)
#define	BCM_DMA_MAXSIZE			(NUM_DMA_SEGS * BCM_SDHCI_BUFFER_SIZE)
static void
bcm_sdhci_write_1(device_t dev, struct sdhci_slot *slot, bus_size_t off,
		  uint8_t val);
static void bcm_sdhci_dma_intr(int ch, void *arg);

#define WR4(sc, off,  val)						\
	bus_space_write_4(sc->sc_bst_reg, sc->sc_bsh_reg, off, val)
#define WR1(sc, off,  val)						\
	bus_space_write_1(sc->sc_bst_reg, sc->sc_bsh_reg, off, val)

#define RD4(sc, reg) \
  bus_space_read_4(sc->sc_bst_reg, sc->sc_bsh_reg, reg);

#define WR4_CFG(sc, off,  val) \
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, off, val)
#define RD4_CFG(sc, reg)				\
	bus_space_read_4(sc->sc_bst, sc->sc_bsh, reg)

static int brcmstb_probe(device_t);
static int brcmstb_attach(device_t);
static int brcmstb_detach(device_t);
static struct ofw_compat_data compat_data[] = {
	{"brcm,bcm2712-sdhci",		1},
	{NULL,				0}
};
static void brcmstb_intr(void *);
struct brcmstb_softc {
	device_t		sc_dev;
	struct resource *	sc_mem_res[5];
	struct resource *	sc_irq_res;
	struct mmc_fdt_helper	mmc_helper;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	bus_space_tag_t		sc_bst_reg;
	bus_space_handle_t	sc_bsh_reg;
	void *			sc_intrhand;
	uint8_t                 sc_power;
	struct mmc_request *	sc_req;
	struct sdhci_slot	sc_slot;
	struct mmc_host         brcmstb_host;
	clk_t                   clk_emmc2;
	struct mtx		mtx;
	int8_t		brcmstb_ios_bus_width;/* current host.ios.bus_width */
	int32_t		brcmstb_ios_clock;/* current host.ios.clock */
	int8_t		brcmstb_ios_power_mode;/* current host.ios.power mode */
	int8_t		brcmstb_ios_timing;	/* current host.ios.timing */
	int8_t		brcmstb_ios_vccq;	/* current host.ios.vccq */
	char			cmdbusy;
	char			mmc_app_cmd;

	u_int32_t		sdhci_int_status;
	u_int32_t		sdhci_signal_enable;
	u_int32_t		sdhci_present_state;
	u_int32_t		sdhci_blocksize;
	u_int32_t		sdhci_blockcount;
	u_int32_t		sdcard_rca;
	uint32_t		blksz_and_count;
	uint32_t                cmd_and_mode;
	bool			need_update_blk;

	int			sc_dma_ch;
	bus_dma_tag_t		sc_dma_tag;
	bus_dmamap_t		sc_dma_map;
	vm_paddr_t		sc_sdhci_buffer_phys;
	bus_addr_t		dmamap_seg_addrs[ALLOCATED_DMA_SEGS];
	bus_size_t		dmamap_seg_sizes[ALLOCATED_DMA_SEGS];
	int			dmamap_seg_count;
	int			dmamap_seg_index;
	int			dmamap_status;
};

static struct resource_spec brcmstb_sdhci_res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ SYS_RES_MEMORY, 1, RF_ACTIVE },
	{ SYS_RES_MEMORY, 2, RF_ACTIVE },
	{ SYS_RES_MEMORY, 3, RF_ACTIVE },
	{ SYS_RES_IRQ, 0, RF_ACTIVE },
	{ -1, 0, 0 }
};

static int
brcmstb_probe(device_t dev)
{


	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "Broadcom 2712 SDHOST controller");

	return (BUS_PROBE_DEFAULT);
}

static int
brcmstb_attach(device_t dev)
{
	int err;
	struct brcmstb_softc *sc = device_get_softc(dev);
	phandle_t node;
	regulator_t vdd_sd_io;
	regulator_t vcc_sd;
	u_int default_freq=0;
	sc->sc_dev = dev;
	sc->sc_req = NULL;

	sc->cmdbusy = 0;
	sc->mmc_app_cmd = 0;
	sc->sdhci_int_status = 0;
	sc->sdhci_signal_enable = 0;
	sc->sdhci_present_state = 0;
	sc->sdhci_blocksize = 0;
	sc->sdhci_blockcount = 0;

	sc->sdcard_rca = 0;
	err = bcm2835_mbox_set_power_state(BCM2835_MBOX_POWER_ID_EMMC, TRUE);
	if (err != 0) {
		if (bootverbose)
			device_printf(dev, "Unable to enable the power\n");
		return (err);
	}
	err = bcm2835_mbox_get_clock_rate(BCM2835_MBOX_CLOCK_ID_EMMC, &default_freq);
	if (err == 0) {
		/* Convert to MHz */
		default_freq /= 1000000;
	}
	device_printf(dev, "%s:%d SDHCI frequency: %dMHz\n",__FILE__,__LINE__,default_freq);
	mtx_init(&sc->mtx, "BRCM SDHOST mtx", "brcmstb_sdhost",
	    MTX_DEF | MTX_RECURSE);
	err=bus_alloc_resources(dev,brcmstb_sdhci_res_spec,sc->sc_mem_res);
	if (err!=0) {
		device_printf(dev, "cannot allocate memory window\n");
		err = ENXIO;
		goto fail;
	}
	err = 0;
	sc->sc_bst = rman_get_bustag(sc->sc_mem_res[1]);
	sc->sc_bsh = rman_get_bushandle(sc->sc_mem_res[1]);
	sc->sc_bst_reg = rman_get_bustag(sc->sc_mem_res[0]);
	sc->sc_bsh_reg = rman_get_bushandle(sc->sc_mem_res[0]);

	sc->sc_irq_res = sc->sc_mem_res[4];
	if (!sc->sc_irq_res) {
		device_printf(dev, "cannot allocate interrupt\n");
		err = ENXIO;
		goto fail;
	}
	if (bus_setup_intr(dev, sc->sc_irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL,brcmstb_intr , sc, &sc->sc_intrhand)) {
		device_printf(dev, "cannot setup interrupt handler\n");
		err = ENXIO;
		goto fail;
		}
	node = ofw_bus_get_node(sc->sc_dev);
	err = clk_get_by_ofw_index(sc->sc_dev,node,0,&sc->clk_emmc2);

	if (err != 0) {
		device_printf(sc->sc_dev, "Cannot get 'emmc2' clock\n");
		err = ENXIO;
		goto fail;
	}

	err = clk_enable(sc->clk_emmc2);
	if (err != 0) {
		device_printf(sc->sc_dev, "Cannot enable 'emmc2' clock\n");
		err = ENXIO;
		goto fail;
	}
	sc->sc_slot.max_clk=0;
	sc->sc_slot.quirks =0;
	sc->sc_slot.quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;
	sc->sc_slot.opt=0;
	sc->sc_slot.opt|=SDHCI_HAVE_DMA;




	node = ofw_bus_get_node(sc->sc_dev);
	err = clk_get_by_ofw_index(sc->sc_dev,node,0,&sc->clk_emmc2);

	if (err != 0) {
		device_printf(sc->sc_dev, "Cannot get 'emmc2' clock\n");
		err = ENXIO;
		goto fail;
	}

	err = clk_enable(sc->clk_emmc2);
	if (err != 0) {
		device_printf(sc->sc_dev, "Cannot enable 'emmc2' clock\n");
		err = ENXIO;
		goto fail;
	}
	regulator_get_by_name(sc->sc_dev,"vdd-sd-io",&vdd_sd_io);
	regulator_get_by_name(sc->sc_dev,"vcc-sd",&vcc_sd);

	regulator_enable(vdd_sd_io);
	regulator_enable(vcc_sd);

	sc->sc_power=0x0;
	sc->sc_slot.quirks=0;

	sc->sc_slot.timeout_clk=0;

	sc->sc_slot.opt |= SDHCI_HAVE_DMA | SDHCI_TUNING_ENABLED;
	sdhci_init_slot(sc->sc_dev,&sc->sc_slot,0);
	mmc_fdt_parse(sc->sc_dev,node,&sc->mmc_helper,&sc->sc_slot.host);

	// Allocate bus_dma resources.
	err = bus_dma_tag_create(bus_get_dma_tag(dev),
	    1, 0, bcm283x_dmabus_peripheral_lowaddr(),
	    BUS_SPACE_MAXADDR, NULL, NULL,
	    BCM_DMA_MAXSIZE, ALLOCATED_DMA_SEGS, BCM_SDHCI_BUFFER_SIZE,
	    BUS_DMA_ALLOCNOW, NULL, NULL,
	    &sc->sc_dma_tag);

	if (err) {
		device_printf(dev, "failed allocate DMA tag");
		goto fail;
	}

	err = bus_dmamap_create(sc->sc_dma_tag, 0, &sc->sc_dma_map);
	if (err) {
		device_printf(dev, "bus_dmamap_create failed\n");
		goto fail;
	}


	sdhci_start_slot(&sc->sc_slot);
	sc->blksz_and_count = SDHCI_READ_4(dev, &sc->sc_slot, SDHCI_BLOCK_SIZE);
	sc->cmd_and_mode = SDHCI_READ_4(dev, &sc->sc_slot, SDHCI_TRANSFER_MODE);
	return (0);
    fail:
	if (sc->sc_intrhand)
		bus_teardown_intr(dev, sc->sc_irq_res, sc->sc_intrhand);
	if (sc->sc_irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sc_irq_res);
	if (sc->sc_mem_res[0])
		bus_release_resources(dev, brcmstb_sdhci_res_spec, sc->sc_mem_res);

	return (ENXIO);

}
static int
brcmstb_detach(device_t dev)
{


	return (EBUSY);
}

static void
brcmstb_intr(void *arg)
{
	struct brcmstb_softc *sc = arg;
	struct sdhci_slot *slot = &sc->sc_slot;

	sdhci_generic_intr(slot);
}

static uint8_t
bcm_sdhci_read_1(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct brcmstb_softc *sc = device_get_softc(dev);
	uint32_t val = RD4(sc, off & ~3);

	return ((val >> (off & 3)*8) & 0xff);
}

static uint16_t
bcm_sdhci_read_2(device_t dev, struct sdhci_slot *slot, bus_size_t off)
{
	struct brcmstb_softc *sc = device_get_softc(dev);
	uint32_t val32;

	/*
	 * Standard 32-bit handling of command and transfer mode, as
	 * well as block size and count.
	 */
	if ((off == SDHCI_BLOCK_SIZE || off == SDHCI_BLOCK_COUNT) &&
	    sc->need_update_blk)
		val32 = sc->blksz_and_count;
	else if (off == SDHCI_TRANSFER_MODE || off == SDHCI_COMMAND_FLAGS)
		val32 = sc->cmd_and_mode;
	else
		val32 = RD4(sc, off & ~3);

	return ((val32 >> (off & 3)*8) & 0xffff);
}

static uint32_t
bcm_sdhci_read_4(device_t dev, struct sdhci_slot *slot, bus_size_t reg)
{
	struct brcmstb_softc*sc = device_get_softc(dev);

	uint32_t val = RD4(sc,reg);
	return val;

}

static void
bcm_sdhci_read_multi_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint32_t *data, bus_size_t count)
{
	struct brcmstb_softc*sc = device_get_softc(dev);

	bus_space_read_multi_4(sc->sc_bst, sc->sc_bsh, off, data, count);
}

static void
bcm_sdhci_write_1(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint8_t val)
{


	struct brcmstb_softc*sc = device_get_softc(dev);


	uint32_t oldval = RD4(sc, (off & ~3));
	uint32_t byte_shift = ((int)off << 3 &  0x18);
	uint32_t mask = 0xff << byte_shift;
	uint32_t newval = (oldval & ~mask) | (val << byte_shift);
	WR4(sc,((off) & ~3),newval);


}

static void
bcm_sdhci_write_2(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint16_t val)
{
	struct brcmstb_softc *sc = device_get_softc(dev);
	uint32_t val32;

	/*
	 * If we have a queued up 16bit value for blk size or count, use and
	 * update the saved value rather than doing any real register access.
	 * If we did not touch either since the last write, then read from
	 * register as at least block count can change.
	 * Similarly, if we are about to issue a command, always use the saved
	 * value for transfer mode as we can never write that without issuing
	 * a command.
	 */
	if ((off == SDHCI_BLOCK_SIZE || off == SDHCI_BLOCK_COUNT) &&
	    sc->need_update_blk)
		val32 = sc->blksz_and_count;
	else if (off == SDHCI_COMMAND_FLAGS)
		val32 = sc->cmd_and_mode;
	else
		val32 = RD4(sc, off & ~3);

	val32 &= ~(0xffff << (off & 3)*8);
	val32 |= (val << (off & 3)*8);

	if (off == SDHCI_TRANSFER_MODE)
		sc->cmd_and_mode = val32;
	else if (off == SDHCI_BLOCK_SIZE || off == SDHCI_BLOCK_COUNT) {
		sc->blksz_and_count = val32;
		sc->need_update_blk = true;
	} else {
		if (off == SDHCI_COMMAND_FLAGS) {
			/* If we saved blk writes, do them now before cmd. */
			if (sc->need_update_blk) {
				WR4(sc, SDHCI_BLOCK_SIZE, sc->blksz_and_count);
				sc->need_update_blk = false;
			}
			/* Always save cmd and mode registers. */
			sc->cmd_and_mode = val32;
		}
		WR4(sc, off & ~3, val32);
	}
}


static void bcm_sdhci_write_4(device_t dev, struct sdhci_slot *slot, bus_size_t reg,
    uint32_t val)
{
	struct brcmstb_softc*sc = device_get_softc(dev);
	WR4(sc,reg,val);
}

static void
bcm_sdhci_write_multi_4(device_t dev, struct sdhci_slot *slot, bus_size_t off,
    uint32_t *data, bus_size_t count)
{
	struct brcmstb_softc*sc = device_get_softc(dev);

	bus_space_write_multi_4(sc->sc_bst, sc->sc_bsh, off, data, count);
}

static int
brcmstb_switch_vccq(device_t dev, device_t reqdev) {
	struct brcmstb_softc *sc = device_get_softc(dev);
	int uvolt=0;
	regulator_get_voltage(sc->mmc_helper.vqmmc_supply,&uvolt);
	if (regulator_check_voltage(sc->mmc_helper.vqmmc_supply, 3300000) == 0) {
		if(sc->sc_slot.host.ios.vccq==2 && uvolt !=3300000) {
			regulator_set_voltage(sc->mmc_helper.vqmmc_supply, 3300000,3300000);
			DELAY(5000);
		}
	}

	regulator_get_voltage(sc->mmc_helper.vmmc_supply,&uvolt);
	if (regulator_check_voltage(sc->mmc_helper.vmmc_supply, 3300000) == 0) {
		if( uvolt !=3300000) {
			regulator_set_voltage(sc->mmc_helper.vmmc_supply, 3300000,330000);
			DELAY(5000);
	 }
 }


return 0;
}

static int
bcm_sdhci_get_ro(device_t bus, device_t child)
{

	return (0);
}
static bool
brcmstb_fdt_get_card_present(device_t dev, struct sdhci_slot *slot)

{
	struct brcmstb_softc *sc = device_get_softc(dev);
	uint32_t val = 	RD4_CFG(sc,0x24);

	return val;

}
static device_method_t brcmstb_methods[] = {
            /* Methods from the device interface */
	DEVMETHOD(device_probe,         brcmstb_probe),
	DEVMETHOD(device_attach,        brcmstb_attach),
	DEVMETHOD(device_detach,        brcmstb_detach),

	/* MMC bridge interface */
	DEVMETHOD(mmcbr_update_ios,	sdhci_generic_update_ios),
	DEVMETHOD(mmcbr_acquire_host,	sdhci_generic_acquire_host),
	DEVMETHOD(mmcbr_release_host,	sdhci_generic_release_host),
	DEVMETHOD(mmcbr_request,	sdhci_generic_request),
	DEVMETHOD(mmcbr_switch_vccq,	brcmstb_switch_vccq),
	DEVMETHOD(mmcbr_tune,		sdhci_generic_tune),
	DEVMETHOD(mmcbr_tune,           sdhci_generic_tune),
	DEVMETHOD(bus_read_ivar,	sdhci_generic_read_ivar),
	DEVMETHOD(bus_write_ivar,	sdhci_generic_write_ivar),
	DEVMETHOD(bus_add_child,	bus_generic_add_child),
	DEVMETHOD(mmcbr_get_ro,		bcm_sdhci_get_ro),
	DEVMETHOD(mmcbr_release_host,	sdhci_generic_release_host),
	DEVMETHOD(sdhci_get_card_present,brcmstb_fdt_get_card_present),
	DEVMETHOD(sdhci_set_uhs_timing,sdhci_generic_set_uhs_timing),
	DEVMETHOD(sdhci_read_1,		bcm_sdhci_read_1),
	DEVMETHOD(sdhci_read_2,		bcm_sdhci_read_2),
	DEVMETHOD(sdhci_read_4,		bcm_sdhci_read_4),
	DEVMETHOD(sdhci_read_multi_4,	bcm_sdhci_read_multi_4),
	DEVMETHOD(sdhci_write_1,	bcm_sdhci_write_1),
	DEVMETHOD(sdhci_write_2,	bcm_sdhci_write_2),
	DEVMETHOD(sdhci_write_4,	bcm_sdhci_write_4),
	DEVMETHOD(sdhci_write_multi_4,	bcm_sdhci_write_multi_4),
	DEVMETHOD_END
 };

 static driver_t brcmstb_driver = {
	 "brcmstb",
	 brcmstb_methods,
	 sizeof(struct brcmstb_softc)
 };

DRIVER_MODULE(brcmstb, simplebus, brcmstb_driver, NULL, NULL);
SDHCI_DEPEND(brcmstb);
#ifndef MMCCAM
MMC_DECLARE_BRIDGE(brcmstb);
#endif
