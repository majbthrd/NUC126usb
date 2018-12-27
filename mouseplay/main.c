/*
  This provides USB mouse functionality similar to "mouseplay" in the "example-apps" directory of:
  https://github.com/majbthrd/PIC16F1-USB-DFU-Bootloader
  as well as the "example-apps" directory of:
  https://github.com/majbthrd/SAMDx1-USB-DFU-Bootloader
*/

/*
 * Copyright (c) 2018, Peter Lawrence
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
#include "clk.h"
#include "sys.h"

/*- Definitions -------------------------------------------------------------*/

#define CRYSTAL_LESS /* system will be 48MHz when defined, otherwise, system is 72MHz */
#define HIRC48_AUTO_TRIM    SYS_IRCTCTL1_REFCKSEL_Msk | (1UL << SYS_IRCTCTL1_LOOPSEL_Pos) | (2UL << SYS_IRCTCTL1_FREQSEL_Pos)
#define TRIM_INIT           (SYS_BASE+0x118)

/*- Variables ---------------------------------------------------------------*/
static uint8_t app_hid_report[64];
static bool app_hid_report_free = true;
static unsigned tick_count;

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

  /* Use PLL as USB clock source */
  CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_PLL, CLK_CLKDIV0_USB(3));

#else
  /* Enable Internal RC 48MHz clock */
  CLK_EnableXtalRC(CLK_PWRCTL_HIRC48EN_Msk);

  /* Waiting for Internal RC clock ready */
  CLK_WaitClockReady(CLK_STATUS_HIRC48STB_Msk);

  /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
  CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC48, CLK_CLKDIV0_HCLK(1));

  /* Use HIRC48 as USB clock source */
  CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL3_USBDSEL_HIRC48, CLK_CLKDIV0_USB(1));
#endif

  /* Enable module clock */
  CLK_EnableModuleClock(USBD_MODULE);


  uint32_t sn = fmc_uid();

  for (int i = 0; i < 8; i++)
    usb_serial_number[i] = "0123456789ABCDEF"[(sn >> (i * 4)) & 0xf];

  usb_serial_number[9] = 0;
}

//-----------------------------------------------------------------------------
void usb_hid_send_callback(void)
{
  app_hid_report_free = true;
}

//-----------------------------------------------------------------------------
static void send_buffer(void)
{
  app_hid_report_free = false;

  usb_hid_send(app_hid_report, 3);
}

//-----------------------------------------------------------------------------
void usb_configuration_callback(int config)
{
  (void)config;
  usb_hid_send_callback();
}

//-----------------------------------------------------------------------------
void usb_sof_callback(void)
{
  tick_count++;
}

//-----------------------------------------------------------------------------
static void hid_task(void)
{
  static unsigned table_index = 0;

  /* arbitrary mouse movement pattern to play back */
  const int8_t move_table[]=
  {
          /* 
          X, Y, (at time 0)
          X, Y, (at time 1)
          X, Y, (at time 2)
          ...
          */
          6, -2,
          2, -6,
          -2, -6,
          -6, -2,
          -6, 2,
          -2, 6,
          2, 6,
          6, 2,
  };

  if (tick_count < 64)
    return;

  if (!app_hid_report_free)
    return;

  /* table_index modulus 16 *AND* make table_index an even number */
  table_index &= 0xE;

  app_hid_report[0] = 0;
  app_hid_report[1] = move_table[table_index++];
  app_hid_report[2] = move_table[table_index++];

  send_buffer();
  tick_count = 0;
}

//-----------------------------------------------------------------------------
int main(void)
{
  uint32_t TrimInit;

  sys_init();
  usb_hw_init();
  usb_hid_init();
#ifdef CRYSTAL_LESS
    /* Backup init trim */
    TrimInit = M32(TRIM_INIT);

    /* Waiting for USB bus stable */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
    while((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

    /* Enable USB crystal-less */
    SYS->IRCTCTL1 = HIRC48_AUTO_TRIM;
#endif

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
    usb_task();
    hid_task();
  }

  return 0;
}
