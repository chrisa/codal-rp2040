#include "CodalConfig.h"
#include "CodalDmesg.h"
#include "ZSingleWireSerial.h"
#include "Event.h"

#include "stdio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "jacdac.pio.h"
#include "dma.h"
#include "ram.h"
#include "hardware/structs/iobank0.h"

#include "codal_target_hal.h"

using namespace codal;

#define COPY __attribute__((section(".time_critical.sws")))

__attribute__((used)) COPY static void pulse_log(void)
{
    gpio_put(2, 1);
    gpio_put(2, 0);
}

namespace codal
{

COPY void gpio_set_function_(uint gpio, enum gpio_function fn)
{
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    invalid_params_if(GPIO, ((uint32_t)fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB) &
                                ~IO_BANK0_GPIO0_CTRL_FUNCSEL_BITS);
    // Set input enable on, output disable off
    hw_write_masked(&padsbank0_hw->io[gpio], PADS_BANK0_GPIO0_IE_BITS,
                    PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
    // Zero all fields apart from fsel; we want this IO to do what the peripheral tells it.
    // This doesn't affect e.g. pullup/pulldown, as these are in pad controls.
    iobank0_hw->io[gpio].ctrl = fn << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

COPY void gpio_set_pulls_(uint gpio, bool up, bool down)
{
    invalid_params_if(GPIO, gpio >= NUM_BANK0_GPIOS);
    hw_write_masked(&padsbank0_hw->io[gpio],
                    (bool_to_bit(up) << PADS_BANK0_GPIO0_PUE_LSB) |
                        (bool_to_bit(down) << PADS_BANK0_GPIO0_PDE_LSB),
                    PADS_BANK0_GPIO0_PUE_BITS | PADS_BANK0_GPIO0_PDE_BITS);
}

static inline void pio_gpio_init_(PIO pio, uint pin)
{
    check_pio_param(pio);
    valid_params_if(PIO, pin < 32);
    gpio_set_function_(pin, pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1);
}

COPY void pio_sm_set_consecutive_pindirs_(PIO pio, uint sm, uint pin, uint count, bool is_out)
{
    check_pio_param(pio);
    check_sm_param(sm);
    valid_params_if(PIO, pin < 32u);
    uint32_t pinctrl_saved = pio->sm[sm].pinctrl;
    uint pindir_val = is_out ? 0x1f : 0;
    while (count > 5)
    {
        pio->sm[sm].pinctrl =
            (5u << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (pin << PIO_SM0_PINCTRL_SET_BASE_LSB);
        pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, pindir_val));
        count -= 5;
        pin = (pin + 5) & 0x1f;
    }
    pio->sm[sm].pinctrl =
        (count << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (pin << PIO_SM0_PINCTRL_SET_BASE_LSB);
    pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, pindir_val));
    pio->sm[sm].pinctrl = pinctrl_saved;
}

COPY void pio_sm_set_pins_with_mask_(PIO pio, uint sm, uint32_t pinvals, uint32_t pin_mask)
{
    check_pio_param(pio);
    check_sm_param(sm);
    uint32_t pinctrl_saved = pio->sm[sm].pinctrl;
    while (pin_mask)
    {
        uint base = (uint)__builtin_ctz(pin_mask);
        pio->sm[sm].pinctrl =
            (1u << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (base << PIO_SM0_PINCTRL_SET_BASE_LSB);
        pio_sm_exec(pio, sm, pio_encode_set(pio_pins, (pinvals >> base) & 0x1u));
        pin_mask &= pin_mask - 1;
    }
    pio->sm[sm].pinctrl = pinctrl_saved;
}

COPY void pio_sm_set_pindirs_with_mask_(PIO pio, uint sm, uint32_t pindirs, uint32_t pin_mask)
{
    check_pio_param(pio);
    check_sm_param(sm);
    uint32_t pinctrl_saved = pio->sm[sm].pinctrl;
    while (pin_mask)
    {
        uint base = (uint)__builtin_ctz(pin_mask);
        pio->sm[sm].pinctrl =
            (1u << PIO_SM0_PINCTRL_SET_COUNT_LSB) | (base << PIO_SM0_PINCTRL_SET_BASE_LSB);
        pio_sm_exec(pio, sm, pio_encode_set(pio_pindirs, (pindirs >> base) & 0x1u));
        pin_mask &= pin_mask - 1;
    }
    pio->sm[sm].pinctrl = pinctrl_saved;
}

COPY void pio_sm_init_(PIO pio, uint sm, uint initial_pc, const pio_sm_config *config)
{
    valid_params_if(PIO, initial_pc < PIO_INSTRUCTION_COUNT);
    // Halt the machine, set some sensible defaults
    pio_sm_set_enabled(pio, sm, false);

    if (config)
    {
        pio_sm_set_config(pio, sm, config);
    }
    else
    {
        pio_sm_config c = pio_get_default_sm_config();
        pio_sm_set_config(pio, sm, &c);
    }

    pio_sm_clear_fifos(pio, sm);

    // Clear FIFO debug flags
    const uint32_t fdebug_sm_mask = (1u << PIO_FDEBUG_TXOVER_LSB) | (1u << PIO_FDEBUG_RXUNDER_LSB) |
                                    (1u << PIO_FDEBUG_TXSTALL_LSB) | (1u << PIO_FDEBUG_RXSTALL_LSB);
    pio->fdebug = fdebug_sm_mask << sm;

    // Finally, clear some internal SM state
    pio_sm_restart(pio, sm);
    pio_sm_clkdiv_restart(pio, sm);
    pio_sm_exec(pio, sm, pio_encode_jmp(initial_pc));
}

} // namespace codal

#define STATUS_IDLE 0
#define STATUS_TX 0x10
#define STATUS_RX 0x20

#define PIO_BREAK_IRQ 0x2
// #define DEBUG_PIN

int dmachTx = -1;
int dmachRx = -1;

REAL_TIME_FUNC
static void rx_handler(void *p)
{
    ZSingleWireSerial *inst = (ZSingleWireSerial *)p;
    if (inst && inst->cb && (inst->status & STATUS_RX))
        inst->cb(SWS_EVT_DATA_RECEIVED);
}

REAL_TIME_FUNC
static void tx_handler(void *p)
{
    ZSingleWireSerial *inst = (ZSingleWireSerial *)p;
    if (inst && inst->cb)
    {
        // first clear any pending stalls
        pio0->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + inst->smtx));

        // wait for the data to be actually sent - i.e. stall
        while (!(pio0->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + inst->smtx))))
            ;
        inst->cb(SWS_EVT_DATA_SENT);
    }
}

static ZSingleWireSerial *inst;
extern "C" REAL_TIME_FUNC void isr_pio0_0()
{
    uint32_t n = pio0->irq;
    pio0->irq = n;
    if (n & PIO_BREAK_IRQ)
    {
        inst->cb(SWS_EVT_DATA_RECEIVED);
    }
}

REAL_TIME_FUNC
static void jd_tx_arm_pin(PIO pio, uint sm, uint pin)
{
    pio_sm_set_pins_with_mask_(pio, sm, 1u << pin, 1u << pin);
    pio_sm_set_pindirs_with_mask_(pio, sm, 1u << pin, 1u << pin);
    pio_gpio_init_(pio, pin);
}

REAL_TIME_FUNC
static void jd_tx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud)
{
    jd_tx_arm_pin(pio, sm, pin);
    pio_sm_config c = jd_tx_program_get_default_config(offset);
    sm_config_set_out_shift(&c, true, false, 32);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_sideset_pins(&c, pin);
    // sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    // limit the size of the TX fifo - it's filled by DMA anyway and we have to busy-wait for it
    // flush
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init_(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, false); // enable when need
}

REAL_TIME_FUNC
static void jd_rx_arm_pin(PIO pio, uint sm, uint pin)
{
#ifdef DEBUG_PIN
    // for debug pin 29
    pio_sm_set_pins_with_mask_(pio, sm, 1u << 29, 1u << 29); // init high
    pio_sm_set_pindirs_with_mask_(pio, sm, (0u << pin) | (1u << 29), (1u << pin) | (1u << 29));
    pio_gpio_init_(pio, 29);
    gpio_set_outover(29, GPIO_OVERRIDE_NORMAL);
#else
    pio_sm_set_consecutive_pindirs_(pio, sm, pin, 1, false);
#endif
    pio_gpio_init_(pio, pin);
    gpio_set_pulls_(pin, true, false);
}

REAL_TIME_FUNC
static void jd_rx_program_init(PIO pio, uint sm, uint offset, uint pin, uint baud)
{
    jd_rx_arm_pin(pio, sm, pin);
    pio_sm_config c = jd_rx_program_get_default_config(offset);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_in_shift(&c, true, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
#ifdef DEBUG_PIN
    sm_config_set_sideset_pins(&c, 29);
#endif
    float div = (float)clock_get_hz(clk_sys) / (8 * baud);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init_(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, false); // enable when need
}

ZSingleWireSerial::ZSingleWireSerial(Pin &p) : DMASingleWireSerial(p)
{
    inst = this;
    txprog = pio_add_program(pio0, &jd_tx_program);
    rxprog = pio_add_program(pio0, &jd_rx_program);

    jd_tx_program_init(pio0, smtx, txprog, p.name, baudrate);
    jd_rx_program_init(pio0, smrx, rxprog, p.name, baudrate);

    // fixed dma channels
    dmachRx = dma_claim_unused_channel(true);
    dmachTx = dma_claim_unused_channel(true);

    // init dma

    dma_channel_config c = dma_channel_get_default_config(dmachRx);
    channel_config_set_bswap(&c, 1);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, DREQ_PIO0_RX0 + smrx);
    uint8_t *rxPtr = (uint8_t *)&pio0->rxf[smrx] + 3;
    dma_channel_configure(dmachRx, &c,
                          NULL,  // dest
                          rxPtr, // src
                          0, false);

    c = dma_channel_get_default_config(dmachTx);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, pio_get_dreq(pio0, smtx, true));
    dma_channel_configure(dmachTx, &c, &pio0->txf[smtx], NULL, 0, false);

    // pio irq for dma rx break handling
    ram_irq_set_priority(DMA_IRQ_0, 0);
    ram_irq_set_priority(PIO0_IRQ_0, 0);

    ram_irq_set_enabled(PIO0_IRQ_0, true);
}

REAL_TIME_FUNC
int ZSingleWireSerial::setMode(SingleWireMode sw)
{
    target_disable_irq();

    // either enable rx or tx program
    if (sw == SingleWireRx)
    {
        status = STATUS_RX;
        jd_rx_arm_pin(pio0, smrx, p.name);
        // jd_rx_program_init(pio0, smrx, rxprog, p.name, baudrate);
        pio_sm_set_enabled(pio0, smrx, true);
    }
    else if (sw == SingleWireTx)
    {
        status = STATUS_TX;
        jd_tx_arm_pin(pio0, smtx, p.name);
        // jd_tx_program_init(pio0, smtx, txprog, p.name, baudrate);
        pio_sm_set_enabled(pio0, smtx, true);
    }
    else
    {
        status = STATUS_IDLE;
        gpio_set_function_(p.name, GPIO_FUNC_SIO); // release gpio
        pio_sm_set_enabled(pio0, smtx, false);
        pio_sm_set_enabled(pio0, smrx, false);
        pio_set_irq0_source_enabled(pio0, pis_interrupt1, false);
    }

    target_enable_irq();

    return DEVICE_OK;
}

REAL_TIME_FUNC
void ZSingleWireSerial::configureRxInterrupt(int enable) {}

REAL_TIME_FUNC
int ZSingleWireSerial::configureTx(int enable)
{
    return setMode(enable ? SingleWireTx : SingleWireDisconnected);
}

REAL_TIME_FUNC
int ZSingleWireSerial::configureRx(int enable)
{
    return setMode(enable ? SingleWireRx : SingleWireDisconnected);
}

int ZSingleWireSerial::putc(char c)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::getc()
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::send(uint8_t *data, int len)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::receive(uint8_t *data, int len)
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::setBaud(uint32_t baud)
{
    baudrate = baud;
    return DEVICE_OK;
}

uint32_t ZSingleWireSerial::getBaud()
{
    return baudrate;
}

REAL_TIME_FUNC
int ZSingleWireSerial::sendDMA(uint8_t *data, int len)
{
    if (status != STATUS_TX)
        setMode(SingleWireTx);

    DMA_SetChannelCallback(dmachTx, tx_handler, this);
    dma_channel_transfer_from_buffer_now(dmachTx, data, len);

    // clear stall if any
    pio0->fdebug = (1u << (PIO_FDEBUG_TXSTALL_LSB + smtx));

    return DEVICE_OK;
}

REAL_TIME_FUNC
int ZSingleWireSerial::receiveDMA(uint8_t *data, int len)
{
    if (status != STATUS_RX)
        setMode(SingleWireRx);

    pio0->irq = PIO_BREAK_IRQ; // clear irq
    DMA_SetChannelCallback(dmachRx, rx_handler, this);
    dma_channel_transfer_to_buffer_now(dmachRx, data, len);
    pio_set_irq0_source_enabled(pio0, pis_interrupt1, true);

    return DEVICE_OK;
}

int ZSingleWireSerial::abortDMA()
{
    dma_hw->abort = (1 << dmachRx) | (1 << dmachTx);

    if (!(status & (STATUS_RX | STATUS_TX)))
        return DEVICE_INVALID_PARAMETER;

    setMode(SingleWireDisconnected);

    return DEVICE_OK;
}

int ZSingleWireSerial::sendBreak()
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::getBytesReceived()
{
    return DEVICE_NOT_IMPLEMENTED;
}

int ZSingleWireSerial::getBytesTransmitted()
{
    return DEVICE_NOT_IMPLEMENTED;
}
