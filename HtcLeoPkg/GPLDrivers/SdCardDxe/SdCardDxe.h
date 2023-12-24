// driver configs
#define USE_PROC_COMM
#define USE_DM

#include <Library/UbootEnvLib.h>
// must come in order
#include <Library/part.h>
#include <Library/mmc.h>
#include <Library/adm.h>
//#include <asm-armv7Scorpion/mmu.h>
#ifdef USE_PROC_COMM
#include <Library/pcom_clients.h>
#endif /*USE_PROC_COMM */

#include <Library/gpio.h>
#include <Chipset/iomap.h>
#include <Library/reg.h>
#include <Chipset/gpio.h>


#ifdef USE_DM
  #define NUM_BLOCKS_MULT    256
#else
  #define NUM_BLOCKS_MULT    1
#endif
#define NUM_BLOCKS_STATUS  1024

#ifdef USE_PROC_COMM
//the desired duty cycle is 50%,
//using proc_comm with 45Mhz possibly giving too low duty cycle,
//breaking it.
enum SD_MCLK_speed
{	MCLK_144KHz = 144000,
	MCLK_400KHz = 400000,
	MCLK_25MHz = 25000000,
	MCLK_48MHz = 49152000, //true 48Mhz not supported, use next highest
	MCLK_49MHz = 49152000,
	MCLK_50MHz = 50000000,
};
#else /*USE_PROC_COMM defined*/
enum SD_MCLK_speed
{
//	MCLK_144KHz, //not implemented w/o proc_comm
	MCLK_400KHz,
	MCLK_25MHz,
	MCLK_48MHz,
//	MCLK_49MHz, //not implemented w/o proc_comm
	MCLK_50MHz
};
#endif /*USE_PROC_COMM */

#define BLOCK_SIZE 512
#define SDC_INSTANCE 2

// Function prototypes
//void mmc_decode_cid(uint32_t * resp);
void mmc_decode_csd(uint32_t * resp);
int sdcc_send_cmd(uint16_t cmd, uint32_t arg, uint32_t response[]);
int check_clear_read_status(void);
int check_clear_write_status(void);
int card_set_block_size(uint32_t size);
int read_SCR_register(uint16_t rca);
int read_SD_status(uint16_t rca);
int switch_mode(uint16_t rca);
int card_identification_selection(uint32_t cid[], uint16_t* rca, uint8_t* num_of_io_func);
int card_transfer_init(uint16_t rca, uint32_t csd[], uint32_t cid[]);
int read_a_block(uint32_t block_number, uint32_t read_buffer[]);
int read_a_block_dm(uint32_t block_number, uint32_t num_blocks, uint32_t read_buffer[]);
int write_a_block(uint32_t block_number, uint32_t write_buffer[], uint16_t rca);
int write_a_block_dm(uint32_t block_number, uint32_t num_blocks, uint32_t write_buffer[], uint16_t rca);
int SD_MCLK_set(enum SD_MCLK_speed speed);
int SDCn_init(uint32_t instance);
void sdcard_gpio_config(int instance);
ulong mmc_bread(ulong blknr, lbaint_t blkcnt, void *dst);
block_dev_desc_t *mmc_get_dev();

int mmc_legacy_init(int verbose);