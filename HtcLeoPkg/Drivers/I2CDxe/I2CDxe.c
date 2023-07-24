/** @file
  Clock driver for QSD8250

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  **/

#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/BaseMemoryLib.h>

#include <Protocol/BlockIo.h>
#include <Protocol/DevicePath.h>

#include <Library/adm.h>
//#include <Library/MsmSdccLib.h>


#include <Library/MsmI2cLib.h>
#include <Library/MicroPLib.h>


EFI_STATUS
EFIAPI
I2CDxeInitialize(
	IN EFI_HANDLE         ImageHandle,
	IN EFI_SYSTEM_TABLE   *SystemTable
)
{
	EFI_STATUS  Status = EFI_SUCCESS;
  EFI_TPL               OldTpl;


	DEBUG((EFI_D_ERROR, "I2CDxe init start \n"));
  OldTpl = gBS->RaiseTPL(TPL_HIGH_LEVEL);
	 Status = MsmI2cInitialize();
   ASSERT_EFI_ERROR(Status);

   Status = MicroPInitialize();
   ASSERT_EFI_ERROR(Status);
   gBS->RestoreTPL(OldTpl);

	DEBUG((EFI_D_ERROR, "I2CDxe init done! \n"));
  

	return Status;
}