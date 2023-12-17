/* LcdFbDxe: Simple FrameBuffer */
#include <PiDxe.h>
#include <Uefi.h>

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/FrameBufferBltLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/pcom_clients.h>

#include <Protocol/GraphicsOutput.h>
#include <Protocol/EmbeddedClock.h>

#include "LcdFbDxe.h"

/// Defines
#define FB_BITS_PER_PIXEL (LCD_BPP)
#define FB_BYTES_PER_PIXEL (FB_BITS_PER_PIXEL / 8)
#define DISPLAYDXE_PHYSICALADDRESS32(_x_) (UINTN)((_x_)&0xFFFFFFFF)

#define DISPLAYDXE_RED_MASK 0xFF0000
#define DISPLAYDXE_GREEN_MASK 0x00FF00
#define DISPLAYDXE_BLUE_MASK 0x0000FF
#define DISPLAYDXE_ALPHA_MASK 0x000000

// Cached copy of the Embedded Clock protocol instance
EMBEDDED_CLOCK_PROTOCOL  *gClock = NULL;

/*
 * Bits per pixel selector. Each value n is such that the bits-per-pixel is
 * 2 ^ n
 */
enum video_log2_bpp {
  VIDEO_BPP1 = 0,
  VIDEO_BPP2,
  VIDEO_BPP4,
  VIDEO_BPP8,
  VIDEO_BPP16,
  VIDEO_BPP32,
};

typedef struct {
  VENDOR_DEVICE_PATH DisplayDevicePath;
  EFI_DEVICE_PATH    EndDevicePath;
} DISPLAY_DEVICE_PATH;

DISPLAY_DEVICE_PATH mDisplayDevicePath = {
    {{HARDWARE_DEVICE_PATH,
      HW_VENDOR_DP,
      {
          (UINT8)(sizeof(VENDOR_DEVICE_PATH)),
          (UINT8)((sizeof(VENDOR_DEVICE_PATH)) >> 8),
      }},
     EFI_GRAPHICS_OUTPUT_PROTOCOL_GUID},
    {END_DEVICE_PATH_TYPE,
     END_ENTIRE_DEVICE_PATH_SUBTYPE,
     {sizeof(EFI_DEVICE_PATH_PROTOCOL), 0}}};

/// Declares

STATIC FRAME_BUFFER_CONFIGURE *mFrameBufferBltLibConfigure;
STATIC UINTN mFrameBufferBltLibConfigureSize;

STATIC
EFI_STATUS
EFIAPI
DisplayQueryMode(
    IN EFI_GRAPHICS_OUTPUT_PROTOCOL *This, IN UINT32 ModeNumber,
    OUT UINTN *SizeOfInfo, OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION **Info);

STATIC
EFI_STATUS
EFIAPI
DisplaySetMode(IN EFI_GRAPHICS_OUTPUT_PROTOCOL *This, IN UINT32 ModeNumber);

STATIC
EFI_STATUS
EFIAPI
DisplayBlt(
    IN EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
    IN EFI_GRAPHICS_OUTPUT_BLT_PIXEL *BltBuffer,
    OPTIONAL IN EFI_GRAPHICS_OUTPUT_BLT_OPERATION BltOperation,
    IN UINTN SourceX, IN UINTN SourceY, IN UINTN DestinationX,
    IN UINTN DestinationY, IN UINTN Width, IN UINTN Height,
    IN UINTN Delta OPTIONAL);

STATIC EFI_GRAPHICS_OUTPUT_PROTOCOL mDisplay = {
    DisplayQueryMode, DisplaySetMode, DisplayBlt, NULL};

STATIC
EFI_STATUS
EFIAPI
DisplayQueryMode(
    IN EFI_GRAPHICS_OUTPUT_PROTOCOL *This, IN UINT32 ModeNumber,
    OUT UINTN *SizeOfInfo, OUT EFI_GRAPHICS_OUTPUT_MODE_INFORMATION **Info)
{
  EFI_STATUS Status;
  Status = gBS->AllocatePool(
      EfiBootServicesData, sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION),
      (VOID **)Info);

  ASSERT_EFI_ERROR(Status);

  *SizeOfInfo                   = sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);
  (*Info)->Version              = This->Mode->Info->Version;
  (*Info)->HorizontalResolution = This->Mode->Info->HorizontalResolution;
  (*Info)->VerticalResolution   = This->Mode->Info->VerticalResolution;
  (*Info)->PixelFormat          = This->Mode->Info->PixelFormat;
  (*Info)->PixelsPerScanLine    = This->Mode->Info->PixelsPerScanLine;

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
DisplaySetMode(IN EFI_GRAPHICS_OUTPUT_PROTOCOL *This, IN UINT32 ModeNumber)
{
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
DisplayBlt(
    IN EFI_GRAPHICS_OUTPUT_PROTOCOL *This,
    IN EFI_GRAPHICS_OUTPUT_BLT_PIXEL *BltBuffer,
    OPTIONAL IN EFI_GRAPHICS_OUTPUT_BLT_OPERATION BltOperation,
    IN UINTN SourceX, IN UINTN SourceY, IN UINTN DestinationX,
    IN UINTN DestinationY, IN UINTN Width, IN UINTN Height,
    IN UINTN Delta OPTIONAL)
{
  RETURN_STATUS Status;
  EFI_TPL       Tpl;
  //
  // We have to raise to TPL_NOTIFY, so we make an atomic write to the frame
  // buffer. We would not want a timer based event (Cursor, ...) to come in
  // while we are doing this operation.
  //
  Tpl    = gBS->RaiseTPL(TPL_NOTIFY);
  Status = FrameBufferBlt(
      mFrameBufferBltLibConfigure, BltBuffer, BltOperation, SourceX, SourceY,
      DestinationX, DestinationY, Width, Height, Delta);
  gBS->RestoreTPL(Tpl);

  return RETURN_ERROR(Status) ? EFI_INVALID_PARAMETER : EFI_SUCCESS;
}

/*
 *  LCDC init routine
 */
void LcdcInit(void)
{
   unsigned int X = 0;
   unsigned int Y = 0;
   unsigned int width = LCDC_vl_col;
   unsigned int height = LCDC_vl_row;

   int hsync_period;
   int vsync_period;

   int hactive_start_x;
   int hactive_end_x;
   int vactive_start_y;
   int vactive_end_y;

   int hsync_width;
   int vsync_width;
   int vsync_starty;
   int vsync_endy;

   UINT32 dma_cfg = 0;

   hsync_period = LCDC_vl_sync_width + LCDC_vl_hfp + LCDC_vl_hbp;
   vsync_period = LCDC_vl_sync_height + LCDC_vl_vfp + LCDC_vl_vbp;
   hsync_width  = LCDC_vl_hsync_width;
   vsync_width  = LCDC_vl_vsync_width * hsync_period;
   vsync_starty = LCDC_vl_vbp * hsync_period;
   vsync_endy   = (((LCDC_vl_vbp + LCDC_vl_sync_height) * hsync_period) - 1);

   // Active area of display
   hactive_start_x = X + LCDC_vl_hbp;
   hactive_end_x   = hactive_start_x + width - 1;
   vactive_start_y = (Y + LCDC_vl_vbp) * hsync_period;
   vactive_end_y   = vactive_start_y + (height * hsync_period) - 1;

  /*
   * clk_enable(lcdc->mdp_clk);
	 * clk_enable(lcdc->pclk);
	 * clk_enable(lcdc->pad_pclk);
  */
  gClock->ClkEnable(14);//MDP_CLK);
#ifdef USE_PROC_COMM
  pcom_enable_lcdc_clk();
  pcom_enable_lcdc_pad_clk();
  pcom_set_lcdc_clk(LCD_CLK_PCOM_MHZ);
  //pcom_enable_lcdc_pad_clk();
  //pcom_enable_lcdc_clk();

#else
   MmioWrite32(LCD_NS_REG, LCD_NS_VAL_MHZ);
   MmioWrite32(LCD_MD_REG, LCD_MD_VAL_MHZ);
#endif

   // Stop any previous transfers
   MmioWrite32(MDP_LCDC_EN, 0x0);

   // Write the registers (disabled for now, breaks display)
   /*
   MmioWrite32(MDP_LCDC_HSYNC_CTL,          ((hsync_period << 16) | hsync_width));
   MmioWrite32(MDP_LCDC_VSYNC_PERIOD,       (vsync_period * hsync_period));
   MmioWrite32(MDP_LCDC_VSYNC_PULSE_WIDTH,  vsync_width);
   MmioWrite32(MDP_LCDC_DISPLAY_HCTL,       (((hsync_period - LCDC_vl_hfp - 1) << 16) | LCDC_vl_hbp));
   MmioWrite32(MDP_LCDC_DISPLAY_V_START,    vsync_starty);
   MmioWrite32(MDP_LCDC_DISPLAY_V_END,      vsync_endy);

   MmioWrite32(MDP_LCDC_ACTIVE_HCTL,        ((hactive_end_x << 16) | hactive_start_x | (1 << 31)));
   MmioWrite32(MDP_LCDC_ACTIVE_V_END,       vactive_end_y);
   MmioWrite32(MDP_LCDC_ACTIVE_V_START,     (vactive_start_y | (1 << 31)));
   MmioWrite32(MDP_LCDC_BORDER_CLR,         0x00000000);
   MmioWrite32(MDP_LCDC_UNDERFLOW_CTL,      0x80000000);
   MmioWrite32(MDP_LCDC_HSYNC_SKEW,         0x00000000);
   MmioWrite32(MDP_LCDC_CTL_POLARITY,       0x00000000);*/

   // Select the DMA channel for LCDC
   // Format (24bpp RGB)
  /*MmioWrite32(MDP_DMA_P_CONFIG, DMA_PACK_ALIGN_LSB|DMA_DITHER_EN|DMA_PACK_PATTERN_RGB|
              DMA_OUT_SEL_LCDC|DMA_IBUF_FORMAT_RGB888|
              DMA_DSTC0G_8BITS|DMA_DSTC1B_8BITS|DMA_DSTC2R_8BITS);*/

  dma_cfg |= (DMA_PACK_ALIGN_MSB |
		   DMA_PACK_PATTERN_RGB |
		   DMA_DITHER_EN);
	dma_cfg |= DMA_OUT_SEL_LCDC;
	dma_cfg |= DMA_IBUF_FORMAT_RGB888;
	//dma_cfg &= ~DMA_DITHER_EN; // solve color banding isue -- marc1706
	dma_cfg &= ~DMA_DST_BITS_MASK;
  dma_cfg |= DMA_DSTC0G_8BITS|DMA_DSTC1B_8BITS|DMA_DSTC2R_8BITS;

  MmioWrite32(MDP_DMA_P_SIZE,          ((height<<16) | width));
  MmioWrite32(MDP_DMA_P_IBUF_ADDR,     LCDC_FB_ADDR);
  MmioWrite32(MDP_DMA_P_IBUF_Y_STRIDE, width * FB_BYTES_PER_PIXEL);
  MmioWrite32(MDP_DMA_P_OUT_XY,        0x0);         // This must be 0

   // Enable
  MmioWrite32(MDP_LCDC_EN, 1);
}

/*
 *  LCDC init routine (stripped, working)
 *
void LcdcInit(void)
{
  unsigned int width = LCDC_vl_col;
  unsigned int height = LCDC_vl_row;
  
  // Stop any previous transfers
  MmioWrite32(MDP_LCDC_EN, 0x0);
  
  // Select the DMA channel for LCDC and set format (24bpp RGB)
  MmioWrite32(MDP_DMA_P_CONFIG, DMA_PACK_ALIGN_LSB|DMA_DITHER_EN|DMA_PACK_PATTERN_RGB|
              DMA_OUT_SEL_LCDC|DMA_IBUF_FORMAT_RGB888|
              DMA_DSTC0G_8BITS|DMA_DSTC1B_8BITS|DMA_DSTC2R_8BITS);
  MmioWrite32(MDP_DMA_P_SIZE,          ((height<<16) | width));
  MmioWrite32(MDP_DMA_P_IBUF_ADDR,     LCDC_FB_ADDR);
  MmioWrite32(MDP_DMA_P_IBUF_Y_STRIDE, width * 3);
  MmioWrite32(MDP_DMA_P_OUT_XY,        0x0);         // This must be 0

  // Enable
  MmioWrite32(MDP_LCDC_EN, 1);
}*/


EFI_STATUS
EFIAPI
LcdFbDxeInitialize(
    IN EFI_HANDLE ImageHandle, IN EFI_SYSTEM_TABLE *SystemTable)
{

  EFI_STATUS Status             = EFI_SUCCESS;
  EFI_HANDLE hUEFIDisplayHandle = NULL;

  // Find the clock controller protocol.  ASSERT if not found.
  Status = gBS->LocateProtocol (&gEmbeddedClockProtocolGuid, NULL, (VOID **)&gClock);
  ASSERT_EFI_ERROR (Status);

  // Init/reinit LCD
  LcdcInit();

  /* Retrieve frame buffer from pre-SEC bootloader */
  DEBUG(
      (EFI_D_INFO,
       "LcdFbDxe: Retrieve MIPI FrameBuffer parameters\n"));
  UINT32 MipiFrameBufferAddr   = LCDC_FB_ADDR;//FixedPcdGet32(PcdMipiFrameBufferAddress);
  UINT32 MipiFrameBufferWidth  = LCDC_vl_row;//FixedPcdGet32(PcdMipiFrameBufferWidth);
  UINT32 MipiFrameBufferHeight = LCDC_vl_col;//FixedPcdGet32(PcdMipiFrameBufferHeight);

  /* Sanity check */
  if (MipiFrameBufferAddr == 0 || MipiFrameBufferWidth == 0 ||
      MipiFrameBufferHeight == 0) {
    DEBUG((EFI_D_ERROR, "LcdFbDxe: Invalid FrameBuffer parameters\n"));
    return EFI_DEVICE_ERROR;
  }

  /* Prepare struct */
  if (mDisplay.Mode == NULL) {
    Status = gBS->AllocatePool(
        EfiBootServicesData, sizeof(EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE),
        (VOID **)&mDisplay.Mode);

    ASSERT_EFI_ERROR(Status);
    if (EFI_ERROR(Status))
      return Status;

    ZeroMem(mDisplay.Mode, sizeof(EFI_GRAPHICS_OUTPUT_PROTOCOL_MODE));
  }

  if (mDisplay.Mode->Info == NULL) {
    Status = gBS->AllocatePool(
        EfiBootServicesData, sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION),
        (VOID **)&mDisplay.Mode->Info);

    ASSERT_EFI_ERROR(Status);
    if (EFI_ERROR(Status))
      return Status;

    ZeroMem(mDisplay.Mode->Info, sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION));
  }

  /* Set information */
  mDisplay.Mode->MaxMode       = 1;
  mDisplay.Mode->Mode          = 0;
  mDisplay.Mode->Info->Version = 0;

  mDisplay.Mode->Info->HorizontalResolution = MipiFrameBufferWidth;
  mDisplay.Mode->Info->VerticalResolution   = MipiFrameBufferHeight;

  /* SimpleFB runs on r8g8b8 (VIDEO_BPP24) for HTC HD2 */
  UINT32               LineLength = MipiFrameBufferWidth * FB_BYTES_PER_PIXEL;
  UINT32               FrameBufferSize    = LineLength * MipiFrameBufferHeight;
  EFI_PHYSICAL_ADDRESS FrameBufferAddress = MipiFrameBufferAddr;

  mDisplay.Mode->Info->PixelsPerScanLine = MipiFrameBufferWidth;
  mDisplay.Mode->Info->PixelFormat = PixelBlueGreenRedReserved8BitPerColor;
  mDisplay.Mode->SizeOfInfo      = sizeof(EFI_GRAPHICS_OUTPUT_MODE_INFORMATION);
  mDisplay.Mode->FrameBufferBase = FrameBufferAddress;
  mDisplay.Mode->FrameBufferSize = FrameBufferSize;

  /* Create the FrameBufferBltLib configuration. */
  Status = FrameBufferBltConfigure(
      (VOID *)(UINTN)mDisplay.Mode->FrameBufferBase, mDisplay.Mode->Info,
      mFrameBufferBltLibConfigure, &mFrameBufferBltLibConfigureSize);

  if (Status == RETURN_BUFFER_TOO_SMALL) {
    mFrameBufferBltLibConfigure = AllocatePool(mFrameBufferBltLibConfigureSize);
    if (mFrameBufferBltLibConfigure != NULL) {
      Status = FrameBufferBltConfigure(
          (VOID *)(UINTN)mDisplay.Mode->FrameBufferBase, mDisplay.Mode->Info,
          mFrameBufferBltLibConfigure, &mFrameBufferBltLibConfigureSize);
    }
  }

  ASSERT_EFI_ERROR(Status);
  /* Causes gcc to error with "-Werror=int-to-pointer-cast" */
   //ZeroMem(LCDC_FB_ADDR, FrameBufferSize); 

  /* Register handle */
  Status = gBS->InstallMultipleProtocolInterfaces(
      &hUEFIDisplayHandle, &gEfiDevicePathProtocolGuid, &mDisplayDevicePath,
      &gEfiGraphicsOutputProtocolGuid, &mDisplay, NULL);

  ASSERT_EFI_ERROR(Status);

  return Status;
}