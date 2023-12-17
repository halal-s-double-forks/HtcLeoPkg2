#include <Library/reg.h>

/*
 *  LCDC configuration
 */

/* LCDC framebuffer is at MM HEAP1 in SMI
 * Note: Update as memory map changes
 */
#define CONFIG_LCD
#define CONFIG_QSD8X50_LCDC
#define	LCD_BPP		24

#   define LCDC_FB_ADDR		0x02A00000
#   define LCDC_FB_SIZE		0x00300000

/* 800x480x24 @ 60 Hz */
#   define LCDC_vl_col		800
#   define LCDC_vl_row		480
#   define LCDC_vl_sync_width	800
#   define LCDC_vl_sync_height	600
#   define LCDC_vl_hbp		216
#   define LCDC_vl_hfp		40
#   define LCDC_vl_vbp		27
#   define LCDC_vl_vfp		1
#   define LCDC_vl_hsync_width	136
#   define LCDC_vl_vsync_width	3

#   define LCD_MD_VAL_MHZ	0x0005FFCF //40 Mhz
#   define LCD_NS_VAL_MHZ	0xFFD41B49 //40 Mhz
#   define LCD_CLK_PCOM_MHZ	40000000