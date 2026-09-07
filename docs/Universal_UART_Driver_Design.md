# Generic UART Low-Level Design (uart_drv)

> A generic, multi-instance UART low-level layer: DMA transmit, ring-DMA receive consumed by the application,
> and optional silence (IDLE) framing — Modbus/frame mode is just a parameter. Implementation = `software/uartdrv/`.

## Design (software/uartdrv/uart_drv.c/h)

### Positioning and layering

```
Application (CLI / Modbus master-slave / any protocol)
   │  byte-stream mode: uart_read / uart_rx_count        ← "app reads it away, advance the consume pointer"
   │  frame mode:       uart_read_frame (whole-frame delivery)
┌──┴─────────────────────────────┐
│ uart_drv  (generic low-level, multi-instance) │  ← this design, done right once
│  TX: DMA + TC callback (DE timing) │
│  RX: ring DMA always spinning      │
│  framing: cfg.framing parameter    │
│    = STREAM: no frame concept      │
│    = FRAME_IDLE: IDLE marks frame boundary │
│  errors: count + raise flag, heal in main loop │
└──┬─────────────────────────────┘
   HAL (UART+DMA)
```

### Configuration struct ("pass params the way HAL does")

```c
typedef struct {
  USART_TypeDef      *instance;      /* USART1..UART8 */
  uint32_t            baud;
  uint8_t             parity;        /* UART_PARITY_NONE/ODD/EVEN */
  uint8_t             stopbits;
  uint8_t             swap;          /* on-chip TX/RX swap */
  /* 485 direction: 0=none (full duplex) 1=hardware DE (HAL DEM, recommended) 2=GPIO manual */
  uint8_t             de_mode;
  GPIO_TypeDef       *de_port;  uint16_t de_pin;   /* used when de_mode=2 */
  /* DMA resources (draw a number from resource table §6; NULL = degrade to interrupt mode, saves a channel for very-low-speed ports) */
  DMA_Stream_TypeDef *dma_rx;   uint32_t req_rx;
  DMA_Stream_TypeDef *dma_tx;   uint32_t req_tx;
  /* buffers (provided by the caller, sized per port conditions; ring DMA writes here directly, app consumes here directly = zero copy) */
  uint8_t            *rxbuf;    uint16_t rxlen;
  uint8_t            *txbuf;    uint16_t txlen;
  /* framing mode */
  uint8_t             framing;      /* UARTDRV_STREAM / UARTDRV_FRAME_IDLE */
  uint16_t            max_frames;   /* frame descriptor ring depth (used by FRAME_IDLE, e.g. 8) */
} uart_cfg_t;
```

GPIO pins do **not** go into cfg: pin AF attachment must be "done all at once after all relevant peripherals are enabled", handled by the board-level `pins_attach()`—the low-level driver never touches pins.

### Key mechanisms

1. **Zero-copy frame delivery**: the IDLE callback moves no data, it only records a `{offset,len}` entry into a **small frame-descriptor ring**; `uart_read_frame` copies straight from the DMA ring to the application per descriptor. This saves one whole-frame move and naturally supports back-to-back multi-frame queuing.
2. **Honest overflow**: HT/TC/IDLE—all three events advance the write shadow; `write shadow − read pointer > rxlen` or the frame-descriptor ring is full → `stats.overrun++`, full ring + descriptor reset. Better to drop it plainly and clearly.
3. **Frame-end = IDLE hardware event**: no software timer; `Size % rxlen` wrap modulo.
4. **TX contract**: `uart_write` copies into txbuf, starts DMA, and returns; `uart_flush(timeout)` waits for TC; writing again while busy = returns 0 (rejected). The TC callback = the sole moment to drop DE.
5. **Error self-heal**: ErrorCallback only does `stats.hw_errs++` + raises the rx_dead flag; the application main loop calls `uart_heal()` to restart receiving in thread context. **Never restart a peripheral inside an interrupt callback.**
6. **Diagnostics as standard**: `uart_stats_t{ rx_bytes, tx_bytes, frames, overrun, hw_errs }`—every port is observable by birth, one CLI command dumps it all.
7. **Concurrency contract (iron rule in the header comment)**: each instance = single reader + single writer; callbacks run in IRQ context and touch only this instance's fields; cross-task dispatch belongs to the upper layer.

### API at a glance

```c
int      uart_open (uart_drv_t *u, const uart_cfg_t *cfg);
int      uart_close(uart_drv_t *u);
uint16_t uart_write(uart_drv_t *u, const void *d, uint16_t n);   /* non-blocking, returns 0 when busy */
int      uart_flush(uart_drv_t *u, uint32_t timeout_ms);          /* wait for TC (frame truly off the wire) */
int      uart_tx_busy(uart_drv_t *u);
/* STREAM mode */
uint16_t uart_rx_count(uart_drv_t *u);
uint16_t uart_read (uart_drv_t *u, void *d, uint16_t n);          /* consuming advances the read pointer */
/* FRAME_IDLE mode */
uint16_t uart_read_frame(uart_drv_t *u, void *d, uint16_t cap);   /* 0 = no complete frame */
/* maintenance */
void     uart_heal(uart_drv_t *u);                                /* call from main loop: restart RX after error */
void     uart_stats(uart_drv_t *u, uart_stats_t *out);
```

### A direct answer to the Modbus question

"Does this program become useless if it's not on Modbus?"—in the new version **Modbus is not a special case but a parameter combination**: `FRAME_IDLE` framing is generic to any "silence-framed" protocol (DMX, many custom instrument protocols likewise); pure streaming protocols (GPS NMEA, transparent transmission, console) use `STREAM` and let the application segment by itself. Protocol knowledge (CRC, address, the exact t3.5 value) all stays in the application layer; the low-level layer only knows "bytes" and "silence."
