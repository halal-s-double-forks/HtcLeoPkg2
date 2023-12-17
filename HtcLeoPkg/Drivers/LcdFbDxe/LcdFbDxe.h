#include <Library/reg.h>

/*
 *  LCDC configuration
 */

/* LCDC framebuffer is at MM HEAP1 in SMI
 * Note: Update as memory map changes
 */
#define CONFIG_LCD
#define CONFIG_QSD8X50_LCDC
#define	LCD_BPP		32

#   define LCDC_FB_ADDR		0x02A00000
#   define LCDC_FB_SIZE		0x00300000

/* 800x480x32 @ 60 Hz */
#   define LCDC_vl_col		800
#   define LCDC_vl_row		480
#   define LCDC_vl_sync_width	800
#   define LCDC_vl_sync_height	600
#   define LCDC_vl_hbp		216
#   define LCDC_vl_hfp		40
#   define LCDC_vl_vbp		27
#   define LCDC_vl_vfp		1
#   define LCDC_vl_hsync_width	136
#   define LCDC_vl_vsync_width	4

#   define LCD_MD_VAL_MHZ	0x0005FFCF //40 Mhz
#   define LCD_NS_VAL_MHZ	0xFFD41B49 //40 Mhz
#   define LCD_CLK_PCOM_MHZ	40000000

// PCOM
#define USE_PROC_COMM

/* MDP-related defines */
#define MSM_MDP_BASE1 	0xAA200000
#define LCDC_BASE     	0xE0000

/* MDP 3.1 */
#define DMA_DSTC0G_8BITS (3<<0)
#define DMA_DSTC1B_8BITS (3<<2)
#define DMA_DSTC2R_8BITS (3<<4)

#define CLR_G 0x0
#define CLR_B 0x1
#define CLR_R 0x2
#define CLR_ALPHA 0x3

#define MDP_GET_PACK_PATTERN(a,x,y,z,bit) (((a)<<(bit*3))|((x)<<(bit*2))|((y)<<bit)|(z))
#define DMA_PACK_TIGHT                      (1 << 6)
#define DMA_PACK_LOOSE                      0
#define DMA_PACK_ALIGN_LSB                  0
#define DMA_PACK_ALIGN_MSB (1<<7)

#define DMA_PACK_PATTERN_RGB				\
        (MDP_GET_PACK_PATTERN(0,CLR_R,CLR_G,CLR_B, 2)<<8)
#define DMA_PACK_PATTERN_BGR \
        (MDP_GET_PACK_PATTERN(0, CLR_B, CLR_G, CLR_R, 2)<<8)

#define DMA_PACK_PATTERN_BGRA \
        (MDP_GET_PACK_PATTERN(CLR_ALPHA, CLR_B, CLR_G, CLR_R, 2)<<8)

#define DMA_DITHER_EN                         (1 << 24)
#define DMA_OUT_SEL_LCDC                      (1 << 20)
#define DMA_IBUF_FORMAT_RGB888			          (0 << 25)
#define DMA_IBUF_FORMAT_RGB565			          (1 << 25)
#define DMA_IBUF_FORMAT_XRGB8888		          (2 << 25)
#define DMA_IBUF_FORMAT_xRGB8888_OR_ARGB8888  (1 << 26)
#define DMA_IBUF_FORMAT_MASK			            (3 << 25)
#define DMA_DST_BITS_MASK 0x3F