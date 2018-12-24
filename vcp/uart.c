/*
  Nuvoton NUC126 version of the UART API in: https://github.com/ataradov/vcp
*/

#include "NUC126.h"
#include "clk.h"
#include "sys.h"
#include "uart.h"
#include "usb_cdc.h"

/*- Definitions -------------------------------------------------------------*/
#define UART_BUF_SIZE            256

#define UART_BAUD_MODE2     (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk) /*!< Set UART Baudrate Mode is Mode2 */
#define UART_BAUD_MODE2_DIVIDER(u32SrcFreq, u32BaudRate)    ((((u32SrcFreq) + ((u32BaudRate)/2)) / (u32BaudRate))-2)

#define UART_WORD_LEN_5     (0) /*!< UART_LINE setting to set UART word length to 5 bits */
#define UART_WORD_LEN_6     (1) /*!< UART_LINE setting to set UART word length to 6 bits */
#define UART_WORD_LEN_7     (2) /*!< UART_LINE setting to set UART word length to 7 bits */
#define UART_WORD_LEN_8     (3) /*!< UART_LINE setting to set UART word length to 8 bits */

#define UART_PARITY_NONE    (0x0 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as no parity   */
#define UART_PARITY_ODD     (0x1 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as odd parity  */
#define UART_PARITY_EVEN    (0x3 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as even parity */
#define UART_PARITY_MARK    (0x5 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to keep parity bit as '1'  */
#define UART_PARITY_SPACE   (0x7 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to keep parity bit as '0'  */

#define UART_STOP_BIT_1     (0x0 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for one stop bit  */
#define UART_STOP_BIT_1_5   (0x1 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for 1.5 stop bit when 5-bit word length  */
#define UART_STOP_BIT_2     (0x1 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for two stop bit when 6, 7, 8-bit word length */

/*- Types ------------------------------------------------------------------*/
typedef struct
{
  int       wr;
  int       rd;
  uint8_t   data[UART_BUF_SIZE];
} fifo_buffer_t;

/*- Variables --------------------------------------------------------------*/
static volatile fifo_buffer_t uart_rx_fifo;
static volatile fifo_buffer_t uart_tx_fifo;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
void uart_init(usb_cdc_line_coding_t *line_coding)
{
  /* Reset IP */
  SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
  SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

  /* Configure UART0 and set UART0 Baudrate */
  UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
  UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

  UART0->INTEN = UART_INTEN_TOCNTEN_Msk | UART_INTEN_RDAIEN_Msk;

  uint32_t u32Reg, u32Tmp, u32SysTmp;
  uint32_t u32Div = 0;

  u32Tmp = 65 * line_coding->dwDTERate;
  u32SysTmp = __HIRC / 1000;
  /* Check we need to divide PLL clock. Note:
     It may not work when baudrate is very small,e.g. 110 bps */
  if(u32SysTmp > u32Tmp)
      u32Div = u32SysTmp / u32Tmp;


  /* Update UART peripheral clock frequency. */
  u32SysTmp = __HIRC / (u32Div + 1);

  // Reset hardware fifo
  UART0->FIFO = UART_FIFO_TXRST_Msk | UART_FIFO_RXRST_Msk;

  // Set baudrate, clock source and clock divider
  UART0->BAUD = 0x30000000 + ((u32SysTmp + line_coding->dwDTERate / 2) / line_coding->dwDTERate - 2);
  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(u32Div + 1));

  // Set parity
  if(line_coding->bParityType == 0)
      u32Reg = 0; // none parity
  else if(line_coding->bParityType == 1)
      u32Reg = 0x08; // odd parity
  else if(line_coding->bParityType == 2)
      u32Reg = 0x18; // even parity
  else
      u32Reg = 0;

  /* bit width */
  switch(line_coding->bDataBits)
  {
      case 5:
          u32Reg |= 0;
          break;
      case 6:
          u32Reg |= 1;
          break;
      case 7:
          u32Reg |= 2;
          break;
      case 8:
          u32Reg |= 3;
          break;
      default:
          break;
  }

  /* stop bit */
  if(line_coding->bCharFormat > 0)
      u32Reg |= 0x4; // 2 or 1.5 bits

  UART0->LINE = u32Reg;

  NVIC_EnableIRQ(UART02_IRQn);
}

//-----------------------------------------------------------------------------
bool uart_write_byte(int byte)
{
  int wr = (uart_tx_fifo.wr + 1) % UART_BUF_SIZE;
  bool res = false;

  NVIC_DisableIRQ(UART02_IRQn);

  if (wr != uart_tx_fifo.rd)
  {
    uart_tx_fifo.data[uart_tx_fifo.wr] = byte;
    uart_tx_fifo.wr = wr;
    res = true;

    UART0->INTEN |= UART_INTEN_THREIEN_Msk;
  }

  NVIC_EnableIRQ(UART02_IRQn);

  return res;
}

//-----------------------------------------------------------------------------
bool uart_read_byte(int *byte)
{
  bool res = false;

  NVIC_DisableIRQ(UART02_IRQn);

  if (uart_rx_fifo.rd != uart_rx_fifo.wr)
  {
    *byte = uart_rx_fifo.data[uart_rx_fifo.rd];
    uart_rx_fifo.rd = (uart_rx_fifo.rd + 1) % UART_BUF_SIZE;
    res = true;
  }

  NVIC_EnableIRQ(UART02_IRQn);

  return res;
}

//-----------------------------------------------------------------------------
void UART02_IRQHandler(void)
{
  uint32_t u32IntStatus;
  uint8_t byte;

  u32IntStatus = UART0->INTSTS;

  if((u32IntStatus & 0x1 /* RDAIF */) || (u32IntStatus & 0x10 /* TOUT_IF */))
  {
    /* Receiver FIFO threashold level is reached or RX time out */

    /* Get all the input characters */
    while((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
    {
      /* Get the character from UART Buffer */
      byte = UART0->DAT;
      int wr = (uart_rx_fifo.wr + 1) % UART_BUF_SIZE;

      if (wr == uart_rx_fifo.rd)
      {
        uart_serial_state_update(USB_CDC_SERIAL_STATE_OVERRUN);
      }
      else
      {
        uart_rx_fifo.data[uart_rx_fifo.wr] = byte;
        uart_rx_fifo.wr = wr;
      }
    }
  }

  if(u32IntStatus & 0x2 /* THRE_IF */)
  {
    if (uart_tx_fifo.rd == uart_tx_fifo.wr)
    {
      UART0->INTEN &= (~UART_INTEN_THREIEN_Msk);
    }
    else
    {
      UART0->DAT = uart_tx_fifo.data[uart_tx_fifo.rd];
      uart_tx_fifo.rd = (uart_tx_fifo.rd + 1) % UART_BUF_SIZE;
    }
  }
}
