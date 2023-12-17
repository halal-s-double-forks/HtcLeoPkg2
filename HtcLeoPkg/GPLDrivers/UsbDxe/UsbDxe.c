/*
 * Copyright (c) 2008 Texas Instruments
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * (C) Copyright 2009 Stefan Roese <sr@denx.de>, DENX Software Engineering
 *
 * Original Author Guenter Gebhardt
 * Copyright (C) 2006 Micronas GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/TimerLib.h>
#include <Library/LKEnvLib.h>
#include <Library/reg.h> //move to chipset
#include <Chipset/timer.h>

//#include <Protocol/HtcLeoUsb.h>
#include <Protocol/GpioTlmm.h>
#include <Library/Usb.h>

#ifdef USE_PROC_COMM
#include <Library/pcom_clients.h>
#endif

// Cached copy of the Hardware Gpio protocol instance
TLMM_GPIO *gGpio = NULL;

UINT8 usbhs_ulpi_phy_read_reg(UINT32 address)
{
    UINT32 cmd;
    UINT32 tmp;
    UINT32 port = 0;
    cmd = ((address << USB_OTG_HS_ULPI_VIEWPORT__ULPIADDR___S) |    // reg addr
           (port  << USB_OTG_HS_ULPI_VIEWPORT__ULPIPORT___S) |      // port
            USB_OTG_HS_ULPI_VIEWPORT__ULPIRUN___M );                // run bit
    MmioWrite32(USBH1_USB_OTG_HS_ULPI_VIEWPORT, cmd);
    do
    {
       tmp = MmioRead32(USBH1_USB_OTG_HS_ULPI_VIEWPORT);
    }  while ((tmp & USB_OTG_HS_ULPI_VIEWPORT__ULPIRUN___M) != 0);
    tmp &= USB_OTG_HS_ULPI_VIEWPORT__ULPIDATRD___M;
    tmp = tmp >> USB_OTG_HS_ULPI_VIEWPORT__ULPIDATRD___S;
    return(tmp);
}

void usbhs_ulpi_phy_write_reg(UINT32 address, UINT32 write_data)
{
    UINT32 cmd;
    UINT32 tmp;
    UINT32 port = 0;
    cmd = ((address << USB_OTG_HS_ULPI_VIEWPORT__ULPIADDR___S) |    // reg addr
           (port  << USB_OTG_HS_ULPI_VIEWPORT__ULPIPORT___S) |      // port
            USB_OTG_HS_ULPI_VIEWPORT__ULPIRW___M |                  // write
            (write_data & 0xFF) |                                   // write data
            USB_OTG_HS_ULPI_VIEWPORT__ULPIRUN___M );                // run bit
    MmioWrite32(USBH1_USB_OTG_HS_ULPI_VIEWPORT, cmd);
    do
    {
       tmp = MmioRead32(USBH1_USB_OTG_HS_ULPI_VIEWPORT);
    }  while ((tmp & USB_OTG_HS_ULPI_VIEWPORT__ULPIRUN___M) != 0);
}

void usbhs_ulpi_phy_init(void)
{
    // Set core to ULPI mode, clear connect status change (CSC)
    MmioWrite32(USBH1_USB_OTG_HS_PORTSC, 0x80000002);
#if defined(USE_PROC_COMM) && defined(USE_pcom_USB_PHY_RESET)
    DEBUG((EFI_D_ERROR, "Using PROC_COMM for PHY RESET\n"));
    //assumption: resets both usb host and phy (in APPS_RESET)
    pcom_usb_reset_phy();
    NanoSecondDelay(1000); //delay still needed to release USBHS core reset.
#else
    DEBUG((EFI_D_ERROR, "Not using PROC_COMM for PHY RESET\n"));
    // Reset USBHS core
    MmioWrite32(APPS_RESET, (MmioRead32(APPS_RESET) | APPS_RESET__USBH___M));
    // Reset USB PHY
    MmioWrite32(APPS_RESET, (MmioRead32(APPS_RESET) | APPS_RESET__USB_PHY___M));
    NanoSecondDelay(1000);
    // Release PHY reset
    MmioWrite32(APPS_RESET, (MmioRead32(APPS_RESET) & ~APPS_RESET__USB_PHY___M));
    // Release USBHS core reset
    MmioWrite32(APPS_RESET, (MmioRead32(APPS_RESET) & ~APPS_RESET__USBH___M));
#endif /* defined(USE_PROC_COMM) && defined(USE_pcom_USB_PHY_RESET) */
    // Set core to ULPI mode, clear connect status change (CSC)
    MmioWrite32(USBH1_USB_OTG_HS_PORTSC, 0x80000002);
    NanoSecondDelay(1000);
}

int ehci_hcd_init_qc(UINT32 *hccr_ptr, UINT32 *hcor_ptr)
{
#ifndef USE_PROC_COMM
    UINT32 reg;
#endif /*USE_PROC_COMM*/
    // Initialize capability register structure pointer
    *hccr_ptr = USBH1_USB_OTG_HS_CAPLENGTH;
    // Initialize operational register structure pointer
    *hcor_ptr = *hccr_ptr + MmioRead8(USBH1_USB_OTG_HS_CAPLENGTH);
#ifndef USE_PROC_COMM
    // Enable USBH1 clock  (ZZZZ smem)
    reg = MmioRead32(USBH_NS_REG);
    reg &= 0xFFFFF0FF;
    reg |= 0x00000B00;    // enable USBH1 clock
    MmioWrite32(USBH_NS_REG, reg);
#else /*USE_PROC_COMM defined */
    DEBUG((EFI_D_ERROR, "BEFORE:: USBH_NS_REG=0x%08x\n", MmioRead32(USBH_NS_REG)));
    DEBUG((EFI_D_ERROR, "BEFORE:: GLBL_CLK_ENA=0x%08x\n", MmioRead32(GLBL_CLK_ENA)));
    pcom_enable_hsusb_clk();
    DEBUG((EFI_D_ERROR, "AFTER_ENABLE:: USBH_NS_REG=0x%08x\n", MmioRead32(USBH_NS_REG)));
    DEBUG((EFI_D_ERROR, "AFTER_ENABLE:: GLBL_CLK_ENA=0x%08x\n", MmioRead32(GLBL_CLK_ENA)));
#endif /*USE_PROC_COMM*/
    // Initialize the PHY
    // ZZZZ replace this with an smem_proc_comm call???
    usbhs_ulpi_phy_init();
    // Enable IDpullup to start ID pin sampling
    usbhs_ulpi_phy_write_reg(0xB, 0x01);
    // Vbus power select = 1, stream disable mode,
    // host controller mode, setup lockouts off
    MmioWrite32(USBH1_USB_OTG_HS_USBMODE, 0x3B);
    // Use the AHB transactor
    MmioWrite32(USBH1_USB_OTG_HS_AHB_MODE, 0x0);
    // Check to see if ULPI is in normal sync state, if not perform a wake-up
    if ((MmioRead32(USBH1_USB_OTG_HS_ULPI_VIEWPORT) & USB_OTG_HS_ULPI_VIEWPORT__ULPISS___M) == 0)
    {
        // Wake up the ULPI (assume port 0)
        MmioWrite32(USBH1_USB_OTG_HS_ULPI_VIEWPORT, USB_OTG_HS_ULPI_VIEWPORT__ULPIWU___M);
        while (MmioRead32(USBH1_USB_OTG_HS_ULPI_VIEWPORT) & USB_OTG_HS_ULPI_VIEWPORT__ULPIWU___M);
        // Check again if in normal sync state,
        if ((MmioRead32(USBH1_USB_OTG_HS_ULPI_VIEWPORT) & USB_OTG_HS_ULPI_VIEWPORT__ULPISS___M) == 0)
        {
            DEBUG((EFI_D_ERROR, "ERROR: ULPI not in normal sync state\n"));
            return(-1);
        }
    }
    // Without PROC_COMM assume it has been turned on before U-boot.
#ifdef USE_PROC_COMM
    pcom_usb_vbus_power(1);
#endif /*USE_PROC_COMM*/
    // Set Power Power bit in the port status/control reg
    MmioWrite32(USBH1_USB_OTG_HS_PORTSC, (MmioRead32(USBH1_USB_OTG_HS_PORTSC) | USB_OTG_HS_PORTSC__PP___M));
    return 0;
}

int ehci_hcd_stop_qc(UINT32 hccr, UINT32 hcor)
{
#ifndef USE_PROC_COMM
    UINT32 reg;
#endif /*USE_PROC_COMM*/
#ifdef USE_PROC_COMM
    pcom_usb_vbus_power(0);
    DEBUG((EFI_D_ERROR, "BEFORE:: USBH_NS_REG=0x%08x\n", MmioRead32(USBH_NS_REG)));
    DEBUG((EFI_D_ERROR, "BEFORE:: GLBL_CLK_ENA=0x%08x\n", MmioRead32(GLBL_CLK_ENA)));
    pcom_disable_hsusb_clk();
    DEBUG((EFI_D_ERROR, "AFTER_DISABLE:: USBH_NS_REG=0x%08x\n", MmioRead32(USBH_NS_REG)));
    DEBUG((EFI_D_ERROR, "AFTER_DISABLE:: GLBL_CLK_ENA=0x%08x\n", MmioRead32(GLBL_CLK_ENA)));
#else /*USE_PROC_COMM not defined */
    //cant turn off vbus without proc_comm
    // Disable USBH1 clock
    reg = MmioRead32(USBH_NS_REG);
    reg &= 0xFFFFF0FF;    // disable USBH1-clock
    MmioWrite32(USBH_NS_REG, reg);
#endif /*USE_PROC_COMM*/
    return 0;
}

/*HTCLEO_USB_PROTOCOL gHtcLeoUsbProtocol = {
  Usb_i2c_write,
  Usb_i2c_read
};*/

EFI_STATUS
EFIAPI
UsbDxeInitialize(
	IN EFI_HANDLE         ImageHandle,
	IN EFI_SYSTEM_TABLE   *SystemTable
)
{
	EFI_STATUS  Status = EFI_SUCCESS;
	//EFI_HANDLE Handle = NULL;
    UINT32 hccr; 
    UINT32 hcor;

    // Find the gpio controller protocol.  ASSERT if not found.
    Status = gBS->LocateProtocol (&gTlmmGpioProtocolGuid, NULL, (VOID **)&gGpio);
    ASSERT_EFI_ERROR (Status);

    Status = ehci_hcd_init_qc(&hccr, &hcor);

	//PROBE
    if(Status) {
        DEBUG((EFI_D_ERROR, "Usb init failed!!"));
        CpuDeadLoop();
    }
    else
    {
        DEBUG((EFI_D_ERROR, "Usb init SUCCESS!"));
    }

	/*if (TRUE) 
	{
		Status = gBS->InstallMultipleProtocolInterfaces(
		&Handle, &gHtcLeoUsbProtocolGuid, &gHtcLeoUsbProtocol, NULL);
		ASSERT_EFI_ERROR(Status);
	}*/

	return Status;
}