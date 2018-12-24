/*
  modified to be used with Nuvoton NUC126 instead of Atmel SAMDx1
  original is available here: https://github.com/ataradov/vcp
  modified version is here: https://github.com/majbthrd/NUC126usb
*/

/*
 * Copyright (c) 2017, Alex Taradov <alex@taradov.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*- Includes ----------------------------------------------------------------*/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "NUC126.h"
#include "usb.h"
#include "uart.h"
#include "clk.h"
#include "sys.h"

/*- Definitions -------------------------------------------------------------*/
#define USB_BUFFER_SIZE        64
#define UART_WAIT_TIMEOUT      10 // ms

#define CRYSTAL_LESS /* system will be 48MHz when defined, otherwise, system is 72MHz */
#define HIRC48_AUTO_TRIM    SYS_IRCTCTL1_REFCKSEL_Msk | (1UL << SYS_IRCTCTL1_LOOPSEL_Pos) | (2UL << SYS_IRCTCTL1_FREQSEL_Pos)
#define TRIM_INIT           (SYS_BASE+0x118)

/*- Variables ---------------------------------------------------------------*/
static uint8_t app_recv_buffer[USB_BUFFER_SIZE];
static uint8_t app_send_buffer[USB_BUFFER_SIZE];
static int app_recv_buffer_size = 0;
static int app_recv_buffer_ptr = 0;
static int app_send_buffer_ptr = 0;
static bool app_send_buffer_free = true;
static bool app_send_zlp = false;
static int app_system_time = 0;
static int app_uart_timeout = 0;
static bool app_status = false;
static int app_status_timeout = 0;

/*- Implementations ---------------------------------------------------------*/

//-----------------------------------------------------------------------------
static uint32_t fmc_uid(void)
{
  uint32_t uid = 0;

  /* unlock sequence to allow access to FMC->ISPCTL */
  SYS->REGLCTL = 0x59;
  SYS->REGLCTL = 0x16;
  SYS->REGLCTL = 0x88;

  CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
  FMC->ISPCTL = FMC_ISPCTL_ISPEN_Msk;

  /* clear any existing fail flag */
  FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;

  /* perform ID command */
  for (int addr = 0; addr < 12; addr+=4)
  {
    FMC->ISPCMD = 0x04 /* read unique ID */;
    FMC->ISPADDR = addr;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);

    if (FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
      break;

    uid ^= FMC->ISPDAT;
  }

  FMC->ISPCTL = 0;

  return uid;
}

//-----------------------------------------------------------------------------
static void sys_init(void)
{
  /* Unlock protected registers */
  SYS_UnlockReg();

  /*---------------------------------------------------------------------------------------------------------*/
  /* Init System Clock                                                                                       */
  /*---------------------------------------------------------------------------------------------------------*/

  /* Enable Internal RC 22.1184 MHz clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

  /* Waiting for Internal RC clock ready */
  CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

  /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

#ifndef CRYSTAL_LESS
  /* Enable external XTAL 12 MHz clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

  /* Waiting for external XTAL clock ready */
  CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

  /* Set core clock */
  CLK_SetCoreClock(72000000);

  /* Use HIRC as UART clock source */
  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

  /* Use PLL as USB clock source */
  CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));

#else
  /* Enable Internal RC 48MHz clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

  /* Waiting for Internal RC clock ready */
  CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

  /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC48, CLK_CLKDIV0_HCLK(1));

  /* Use HIRC as UART clock source */
  CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

  /* Use HIRC48 as USB clock source */
  CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC48, CLK_CLKDIV0_USB(1));
#endif

  /* Enable module clock */
  CLK_EnableModuleClock(UART0_MODULE);
  CLK_EnableModuleClock(USBD_MODULE);


  /*---------------------------------------------------------------------------------------------------------*/
  /* Init I/O Multi-function                                                                                 */
  /*---------------------------------------------------------------------------------------------------------*/

  /* Set P3 multi-function pins for UART0 RXD and TXD */
  SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
  SYS->GPD_MFPL |= SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD;

  uint32_t sn = fmc_uid();

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[9] = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_init(void)
{
  SysTick->VAL = 0;
  SysTick->LOAD = __HIRC / 1000ul;
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;
  app_system_time = 0;
}

//-----------------------------------------------------------------------------
static void sys_time_task(void)
{
  if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
    app_system_time++;
}

//-----------------------------------------------------------------------------
static int get_system_time(void)
{
  return app_system_time;
}

//-----------------------------------------------------------------------------
static void set_status_state(bool active)
{
  (void)active;
}

//-----------------------------------------------------------------------------
static void update_status(bool status)
{
#if STATUS_RISING_EDGE > 0
  if (false == app_status && true == status)
  {
    set_status_state(true);
    app_status_timeout = get_system_time() + STATUS_RISING_EDGE;
  }
#endif

#if STATUS_FALLING_EDGE > 0
  if (true == app_status && false == status)
  {
    set_status_state(true);
    app_status_timeout = get_system_time() + STATUS_FALLING_EDGE;
  }
#endif

#if STATUS_RISING_EDGE == 0 && STATUS_FALLING_EDGE == 0
  set_status_state(status);
#endif

  app_status = status;
}

//-----------------------------------------------------------------------------
static void status_task(void)
{
  if (app_status_timeout && get_system_time() > app_status_timeout)
  {
    set_status_state(false);
    app_status_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
void usb_cdc_send_callback(void)
{
  app_send_buffer_free = true;
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_send_buffer_free = false;
  app_send_zlp = (USB_BUFFER_SIZE == app_send_buffer_ptr);

  usb_cdc_send(app_send_buffer, app_send_buffer_ptr);

  app_send_buffer_ptr = 0;
}

//-----------------------------------------------------------------------------
void usb_cdc_recv_callback(int size)
{
  app_recv_buffer_ptr = 0;
  app_recv_buffer_size = size;
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  (void)config;
}

//-----------------------------------------------------------------------------
void usb_cdc_line_coding_updated(usb_cdc_line_coding_t *line_coding)
{
  uart_init(line_coding);
}

//-----------------------------------------------------------------------------
void usb_cdc_control_line_state_update(int line_state)
{
  update_status(line_state & USB_CDC_CTRL_SIGNAL_DTE_PRESENT);
}

//-----------------------------------------------------------------------------
static void tx_task(void)
{
  while (app_recv_buffer_size)
  {
    if (!uart_write_byte(app_recv_buffer[app_recv_buffer_ptr]))
      break;

    app_recv_buffer_ptr++;
    app_recv_buffer_size--;

    if (0 == app_recv_buffer_size)
      usb_cdc_recv(app_recv_buffer, sizeof(app_recv_buffer));
  }
}

//-----------------------------------------------------------------------------
static void rx_task(void)
{
  int byte;

  if (!app_send_buffer_free)
    return;

  while (uart_read_byte(&byte))
  {
    app_uart_timeout = get_system_time() + UART_WAIT_TIMEOUT;
    app_send_buffer[app_send_buffer_ptr++] = byte;

    if (USB_BUFFER_SIZE == app_send_buffer_ptr)
    {
      send_buffer();
      break;
    }
  }
}

//-----------------------------------------------------------------------------
static void uart_timer_task(void)
{
  if (app_uart_timeout && get_system_time() > app_uart_timeout)
  {
    if (app_send_zlp || app_send_buffer_ptr)
      send_buffer();

    app_uart_timeout = 0;
  }
}

//-----------------------------------------------------------------------------
void uart_serial_state_update(int state)
{
  usb_cdc_set_state(state);
}

//-----------------------------------------------------------------------------
int main(void)
{
  uint32_t TrimInit;

  sys_init();
  sys_time_init();
  usb_hw_init();
  usb_cdc_init();
#ifdef CRYSTAL_LESS
    /* Backup init trim */
    TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less */
    SYS->IRCTCTL1 = HIRC48_AUTO_TRIM;
#endif
  set_status_state(false);

  while (1)
  {
#ifdef CRYSTAL_LESS
    /* Re-start crystal-less when any error found */
    if(SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk))
    {
      /* USB clock trim fail. Just retry */
      SYS->IRCTCTL1 = 0;  /* Disable crystal-less */
      SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk;
      
      /* Init TRIM */
      M32(TRIM_INIT) = TrimInit;
      
      /* Waiting for USB bus stable */
      USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
      while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);
      
      SYS->IRCTCTL1 = HIRC48_AUTO_TRIM; /* Re-enable crystal-less */
    }
#endif
    sys_time_task();
    usb_task();
    tx_task();
    rx_task();
    uart_timer_task();
    status_task();
  }

  return 0;
}
