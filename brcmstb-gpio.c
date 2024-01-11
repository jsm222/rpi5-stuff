
#include <sys/cdefs.h>
#include "opt_platform.h"
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/fdt/fdt_pinctrl.h>
#include "gpio_if.h"
#define BRCMSTB_GPIO_PINS 36
#define	BRCMSTB_GPIO_WRITE(_sc, _off, _val)		\
    bus_space_write_4((_sc)->sc_bst, (_sc)->sc_bsh, _off, _val)
#define	BRCMSTB_GPIO_READ(_sc, _off)		\
    bus_space_read_4((_sc)->sc_bst, (_sc)->sc_bsh, _off)
#define	BRCMSTB_GPIO_READ_1(_sc, _off)		\
    bus_space_read_1((_sc)->sc_bst, (_sc)->sc_bsh, _off)
#define	BRCMSTB_GPIO_READ_2(_sc, _off)		\
    bus_space_read_2((_sc)->sc_bst, (_sc)->sc_bsh, _off)
static int
brcmstb_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val);
static int
brcmstb_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value);
static struct resource_spec brcmstb_gpio_res_spec[] = {
	{ SYS_RES_MEMORY, 0, RF_ACTIVE },
	{ -1, 0, 0 }
};
struct brcmstb_gpio_softc {
	device_t		sc_dev;
	device_t		sc_busdev;
	struct mtx		sc_mtx;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	struct resource *	sc_res[1];
	u_int			sc_maxpins;
	int			sc_gpio_npins;
	int			sc_ro_npins;
	int			sc_ro_pins[BRCMSTB_GPIO_PINS];
	struct gpio_pin		sc_gpio_pins[BRCMSTB_GPIO_PINS];

};



static struct ofw_compat_data compat_data[] = {	{"brcm,brcmstb-gpio",	1},
	{NULL,			0}
};
static struct brcmstb_gpio_softc *brcmstb_gpio_sc = NULL;


static int
brcmstb_gpio_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "brcmstb GPIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
brcmstb_gpio_attach(device_t dev) {
        phandle_t gpio;
	struct brcmstb_gpio_softc *sc;


	brcmstb_gpio_sc = sc = device_get_softc(dev);

	sc->sc_dev = dev;
	mtx_init(&sc->sc_mtx, "bcrmstb gpio", "gpio", MTX_SPIN);

	if (bus_alloc_resources(dev, brcmstb_gpio_res_spec, sc->sc_res) !=0) {
		device_printf(dev, "cannot allocate resources\n");
		goto fail;
	}
	sc->sc_bst = rman_get_bustag(sc->sc_res[0]);
	sc->sc_bsh = rman_get_bushandle(sc->sc_res[0]);
	/* Find our node. */
	gpio = ofw_bus_get_node(sc->sc_dev);


	if (!OF_hasprop(gpio, "gpio-controller"))
		goto fail;
sc->sc_busdev = gpiobus_attach_bus(dev);
	if (sc->sc_busdev == NULL)
		goto fail;
	fdt_pinctrl_register(dev, "pins");
	fdt_pinctrl_configure_tree(dev);



	return(0);

fail:
	bus_release_resources(dev, brcmstb_gpio_res_spec, sc->sc_res);
	mtx_destroy(&sc->sc_mtx);

	return (ENXIO);
}
static int
brcmstb_gpio_detach(device_t dev)
{
	return (EBUSY);
}

static device_t
brcmstb_gpio_get_bus(device_t dev)
{
	struct brcmstb_gpio_softc *sc;

	sc = device_get_softc(dev);

	return (sc->sc_busdev);
}
static int
brcmstb_gpio_pin_get(device_t dev, uint32_t pin, unsigned int *val) {
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
		if(BRCMSTB_GPIO_READ(sc,0x4) & (1UL <<pin))
			*val =0x1;
		else
			*val = 0x0;
	return (0);


}

static int
brcmstb_gpio_pin_set(device_t dev, uint32_t pin, unsigned int value) {
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
	uint32_t val;

	val=BRCMSTB_GPIO_READ(sc,0x4);

	if(value)
		val= val | (1UL << pin);
	else
		val = val & ~(1UL << pin);

	BRCMSTB_GPIO_WRITE(sc,0x4,val);
	return (0);
}

static int
brcmstb_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = 31;
	return (0);
}
static int
brcmstb_gpio_configure_pins(device_t dev, phandle_t cfgxref)
{
	phandle_t cfgnode;

	uint32_t *pins;
	cfgnode = OF_node_from_xref(cfgxref);

	pins = NULL;;
	char name[32];
	OF_getprop(cfgnode, "name", &name, sizeof(name));
	device_printf(dev, "set pin %d", pins[0]);
	return (0);
}
static int
brcmstb_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct brcmstb_gpio_softc *sc = device_get_softc(dev);
	uint32_t val;
	val = BRCMSTB_GPIO_READ(sc,0x8);

	if(flags & GPIO_PIN_OUTPUT)  {
		val = val & ~(1UL<<pin);
	}
	else if(flags & GPIO_PIN_INPUT)  {
		val = val | (1UL<<pin);
	}
	BRCMSTB_GPIO_WRITE(sc,0x8,val);
	return(0);
}
static device_method_t brcmstb_gpio_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		brcmstb_gpio_probe),
	DEVMETHOD(device_attach,	brcmstb_gpio_attach),
	DEVMETHOD(device_detach,	brcmstb_gpio_detach),
	/* gpio interface */
	DEVMETHOD(gpio_get_bus,		brcmstb_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		brcmstb_gpio_pin_max),
	DEVMETHOD(gpio_pin_get,		brcmstb_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		brcmstb_gpio_pin_set),
	DEVMETHOD(gpio_pin_setflags,	brcmstb_gpio_pin_setflags),
	/*pinctrl*/
	DEVMETHOD(fdt_pinctrl_configure, brcmstb_gpio_configure_pins),
	DEVMETHOD_END
};

static driver_t brcmstb_gpio_driver = {
	"gpio",
	brcmstb_gpio_methods,
	sizeof(struct brcmstb_gpio_softc),
};



EARLY_DRIVER_MODULE(brcmstb_gpio, simplebus, brcmstb_gpio_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
