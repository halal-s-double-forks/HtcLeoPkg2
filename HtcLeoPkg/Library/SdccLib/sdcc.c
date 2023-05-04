/*
Copyright (c) 2010, Code Aurora Forum. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.
*/

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

#include <Library/reg.h>//has sdc2_base defines
#include <Library/adm.h>
#include <Library/pcom_clients.h>

#include <Chipset/timer.h>
#include <Chipset/clock.h>

#include <Library/LKEnvLib.h>
/*#include <Library/list.h>
#include <Library/part.h>
#include <Library/mmc.h>*/

#include <Library/SdccLib.h>

#define USE_PROC_COMM 1

int  mmc_is_ready; // Will be set to 1 if sdcard is ready

static unsigned char sdcc_use_dm;//=1
// Structures for use with ADM
static UINT32 sd_adm_cmd_ptr_list[8] __attribute__ ((aligned(8))); //  Must aligned on 8 byte boundary
static UINT32 sd_box_mode_entry[8] __attribute__ ((aligned(8))); //  Must aligned on 8 byte boundary

// clear the status bit.
#define MCI_CLEAR_STATUS(base, mask) do{ writel(mask, base + MCI_CLEAR); } while(readl(base + MCI_STATUS) & mask);

// Any one of these will be set when command processing is complete.
#define MCI_STATUS_CMD_COMPLETE_MASK        (MCI_STATUS__CMD_SENT___M | \
                                             MCI_STATUS__CMD_RESPONSE_END___M | \
                                             MCI_STATUS__CMD_TIMEOUT___M | \
                                             MCI_STATUS__CMD_CRC_FAIL___M)

// Any one of these will be set during data read processing.
#define MCI_STATUS_DATA_READ_COMPLETE_MASK  (MCI_STATUS_BLK_END_MASK | \
                                             MCI_STATUS_DATA_RD_ERR_MASK)

#define MCI_STATUS_DATA_RD_ERR_MASK         (MCI_STATUS__DATA_CRC_FAIL___M | \
                                             MCI_STATUS__DATA_TIMEOUT___M  | \
                                             MCI_STATUS__RX_OVERRUN___M    | \
                                             MCI_STATUS__START_BIT_ERR___M)

#define MCI_STATUS_BLK_END_MASK             (MCI_STATUS__DATA_BLK_END___M |  \
                                             MCI_STATUS__DATAEND___M)

#define MCI_STATUS_READ_DATA_MASK           (MCI_STATUS__RXDATA_AVLBL___M|  \
                                             MCI_STATUS__RXACTIVE___M)

// Writes to MCI port are not effective for 3 ticks of PCLK.
// The min pclk is 144KHz which gives 6.94 us/tick.
// Thus 21us == 3 ticks.
#define PORT_RUSH_TIMEOUT (21)

/* verify if data read was successful and clear up status bits */
int sdcc_read_data_cleanup(mmc_t *mmc)
{
    int err = 0;
    UINT32 base   = ((sd_parms_t*)(mmc->priv))->base;
    UINT32 status = readl(base + MCI_STATUS);

    if(status & MCI_STATUS__RX_OVERRUN___M) {
        err = SDCC_ERR_RX_OVERRUN;
    } else 
	if (status & MCI_STATUS__DATA_TIMEOUT___M) {
        err = SDCC_ERR_DATA_TIMEOUT;
    } else
	if (status & MCI_STATUS__DATA_CRC_FAIL___M) {
        err = SDCC_ERR_DATA_CRC_FAIL;
    } else 
	if (status & MCI_STATUS__START_BIT_ERR___M ) {
        err = SDCC_ERR_DATA_START_BIT;
    } else {
        // Wait for the blk status bits to be set.
        //while (!(readl(base + MCI_STATUS) & MCI_STATUS__DATAEND___M));
    }

    // Clear the status bits.
    MCI_CLEAR_STATUS(base, MCI_STATUS_DATA_READ_COMPLETE_MASK);

    return err;
}

/* Read data from controller fifo */
int sdcc_read_data(mmc_t *mmc, mmc_cmd_t *cmd, mmc_data_t *data)
{
    DEBUG((EFI_D_ERROR, "Start sdcc_read_data()\n"));

    UINT32 status;
    UINT16 byte_count   = 0;
    UINT32 *dest_ptr    = (UINT32 *)(data->dest);
    UINT32 base         = ((sd_parms_t*)(mmc->priv))->base;
    UINT32 adm_crci_num = ((sd_parms_t*)(mmc->priv))->adm_crci_num; //  ADM CRCI number

    if(sdcc_use_dm) {
        UINT32 num_rows;
        UINT32 addr_shft;
        UINT32 rows_per_block;
        UINT16 row_len;

        row_len = SDCC_FIFO_SIZE;
        rows_per_block = (data->blocksize / SDCC_FIFO_SIZE);
        num_rows = rows_per_block * data->blocks;
        UINT32 tx_dst = (UINT32)data->dest;

        DEBUG((EFI_D_ERROR, "Data dest before : %p\n", data->dest));

        while( num_rows != 0 ) {
            UINT32 tx_size = 0;
            // Check to see if the attempted transfer size is more than 0xFFFF
            // If it is we need to do more than one transfer.
            if( num_rows > 0xFFFF ) {
                tx_size   = 0xFFFF;
                num_rows -= 0xFFFF;
            } else {
                tx_size  = num_rows;
                num_rows = 0;
            }
			
            // Initialize the DM Box mode command entry (single entry)
            sd_box_mode_entry[0] = (ADM_CMD_LIST_LC | (adm_crci_num << 3) | ADM_ADDR_MODE_BOX);
            sd_box_mode_entry[1] = base + MCI_FIFO;                  	// SRC addr
            sd_box_mode_entry[2] = (UINT32) tx_dst;                	// DST addr
            sd_box_mode_entry[3] = ((row_len << 16) | (row_len << 0));  // SRC/DST row len
            sd_box_mode_entry[4] = ((tx_size << 16) | (tx_size << 0)); 	// SRC/DST num rows
            sd_box_mode_entry[5] = ((0 << 16) | (SDCC_FIFO_SIZE << 0)); // SRC/DST offset
            
			// Initialize the DM Command Pointer List (single entry)
            DEBUG((EFI_D_ERROR, "Initialize the DM Command Pointer List (single entry)\n"));
            addr_shft = ((UINT32)(&sd_box_mode_entry[0])) >> 3;
            sd_adm_cmd_ptr_list[0] = (ADM_CMD_PTR_LP | ADM_CMD_PTR_CMD_LIST | addr_shft);

            // Start ADM transfer, this transfer waits until it finishes
            // before returing
            DEBUG((EFI_D_ERROR, "Start ADM transfer\n"));
            if (adm_start_transfer(ADM_AARM_SD_CHN, sd_adm_cmd_ptr_list) != 0) {
               return SDCC_ERR_DATA_ADM_ERR;
               DEBUG((EFI_D_ERROR, "ADM transfer failed...\n"));
		       MicroSecondDelay(2000000);
            }
            // Add the amount we have transfered to the destination
            tx_dst += (tx_size*row_len);
        }
        
        // Try to read the data
        int read_len = (data->blocks) * (data->blocksize);

        DEBUG((EFI_D_ERROR, "DATA before reading: %p\n", *dest_ptr));
        //MicroSecondDelay(10000000);

        while((status = readl(base + MCI_STATUS)) & MCI_STATUS_READ_DATA_MASK) {
            // rx data available bit is not cleared immidiately.
            // read only the requested amount of data and wait for
            // the bit to be cleared.
            if(byte_count < read_len) {
                if(status & MCI_STATUS__RXDATA_AVLBL___M) {
                    *dest_ptr = readl(base + MCI_FIFO);// + (byte_count % SDCC_FIFO_SIZE));//+ (byte_count % MCI_FIFOSIZE)
                    DEBUG((EFI_D_ERROR, "DATA: %p\n", *dest_ptr));//E1A00110
                    dest_ptr++;
                    byte_count += 4;
                }
            }
        }

        DEBUG((EFI_D_ERROR, "data dest after reading : %p\n", data->dest));
        MicroSecondDelay(10000000);

    } else {
        int read_len = (data->blocks) * (data->blocksize);

        DEBUG((EFI_D_ERROR, "Data read not using Data Mover\n"));
        DEBUG((EFI_D_ERROR, "DATA before reading: %p\n", *dest_ptr));
        //MicroSecondDelay(10000000);

        while((status = readl(base + MCI_STATUS)) & MCI_STATUS_READ_DATA_MASK) {
            // rx data available bit is not cleared immidiately.
            // read only the requested amount of data and wait for
            // the bit to be cleared.
            if(byte_count < read_len) {
                if(status & MCI_STATUS__RXDATA_AVLBL___M) {
                    *dest_ptr = readl(base + MCI_FIFO + (byte_count % SDCC_FIFO_SIZE));//+ (byte_count % MCI_FIFOSIZE)
                    DEBUG((EFI_D_ERROR, "DATA: %p\n", *dest_ptr));
                    dest_ptr++;
                    byte_count += 4;
                }
            }
        }
    }
	
    //DEBUG((EFI_D_ERROR, "sdcc_read_data: call sdcc_read_data_cleanup()\n"));
	//MicroSecondDelay(2000000);
    //return sdcc_read_data_cleanup(mmc);
    return 0;
}

/* Set SD MCLK speed */
static int sdcc_mclk_set(int instance, enum SD_MCLK_speed speed)
{
#ifdef USE_PROC_COMM
    pcom_set_sdcard_clk(instance, speed);
    pcom_enable_sdcard_clk(instance);
#else
    if (instance == 1) {
		clk_set_rate(SDC1_CLK, MCLK_400KHz);
	} else
	if (instance == 2) {
		clk_set_rate(SDC2_CLK, MCLK_400KHz);
	} else
	if (instance == 3) {
		clk_set_rate(SDC3_CLK, MCLK_400KHz);
	} else
	if (instance == 4) {
		clk_set_rate(SDC4_CLK, MCLK_400KHz);
	}
#endif

   return 0;
}

/* Set bus width and bus clock speed */
void sdcc_set_ios(mmc_t *mmc)
{
    UINT32 clk_reg;
    UINT32 base     = ((sd_parms_t*)(mmc->priv))->base;
    UINT32 instance = ((sd_parms_t*)(mmc->priv))->instance;

    if(mmc->bus_width == 1) {
        clk_reg = readl(base + MCI_CLK) & ~MCI_CLK__WIDEBUS___M;
        writel((clk_reg | (BUS_WIDTH_1 << MCI_CLK__WIDEBUS___S)), base + MCI_CLK);
    } else
	if(mmc->bus_width == 4) {
        clk_reg = readl(base + MCI_CLK) & ~MCI_CLK__WIDEBUS___M;
        writel((clk_reg | (BUS_WIDTH_4 << MCI_CLK__WIDEBUS___S)), base + MCI_CLK);
    } else
	if(mmc->bus_width == 8) {
        clk_reg = readl(base + MCI_CLK) & ~MCI_CLK__WIDEBUS___M;
        writel((clk_reg | (BUS_WIDTH_8 << MCI_CLK__WIDEBUS___S)), base + MCI_CLK);
    }
    if(mmc->clock <= 25000000) {
        UINT32 temp32;

        sdcc_mclk_set(instance, mmc->clock);

        // Latch data on falling edge
        temp32 = readl(base + MCI_CLK) & ~MCI_CLK__SELECT_IN___M;
        temp32 |= (MCI_CLK__SELECT_IN__ON_THE_FALLING_EDGE_OF_MCICLOCK << MCI_CLK__SELECT_IN___S);
        writel(temp32, base + MCI_CLK);
    } else {
        UINT32 temp32;

        sdcc_mclk_set(instance, mmc->clock);

        // Card is in high speed mode, use feedback clock.
        temp32 = readl(base + MCI_CLK);
        temp32 &= ~(MCI_CLK__SELECT_IN___M);
        temp32 |= (MCI_CLK__SELECT_IN__USING_FEEDBACK_CLOCK << MCI_CLK__SELECT_IN___S);
        writel(temp32, base + MCI_CLK);
    }
}

/* Program the data control registers */
void sdcc_start_data(mmc_t *mmc, mmc_cmd_t *cmd, mmc_data_t *data)
{
    UINT32 data_ctrl;
    UINT32 base = ((sd_parms_t*)(mmc->priv))->base;
    UINT32 xfer_size = data->blocksize * data->blocks;;

    data_ctrl = MCI_DATA_CTL__ENABLE___M | (data->blocksize << MCI_DATA_CTL__BLOCKSIZE___S);

    if ( (xfer_size < SDCC_FIFO_SIZE) || (xfer_size % SDCC_FIFO_SIZE) ) {
        // can't use data mover.
        sdcc_use_dm = 0;
    } else {
        sdcc_use_dm = 1;
        data_ctrl |= MCI_DATA_CTL__DM_ENABLE___M;
    }

    if(data->flags == MMC_DATA_READ) {
        data_ctrl |= MCI_DATA_CTL__DIRECTION___M;
    }

    // Set timeout and data length
    writel(RD_DATA_TIMEOUT, base + MCI_DATA_TIMER);
    writel(xfer_size, base + MCI_DATA_LENGTH);

    // Delay for previous param to be applied.
    udelay(PORT_RUSH_TIMEOUT);
    writel(data_ctrl, base + MCI_DATA_CTL);
    // Delay before adm
    udelay(PORT_RUSH_TIMEOUT);
}

/* Set proper bit for the cmd register value */
static void sdcc_get_cmd_reg_value(mmc_cmd_t *cmd, mmc_data_t *data, UINT16 *creg)
{
    *creg = cmd->cmdidx | MCI_CMD__ENABLE___M;

    if(cmd->resp_type & MMC_RSP_PRESENT) {
        *creg |= MCI_CMD__RESPONSE___M;

        if(cmd->resp_type & MMC_RSP_136)
            *creg |= MCI_CMD__LONGRSP___M;
    }
    if(cmd->resp_type & MMC_RSP_BUSY) {
        *creg |= MCI_CMD__PROG_ENA___M;
    }

    // Set the DAT_CMD bit for data commands
    if( (cmd->cmdidx == CMD17) ||
        (cmd->cmdidx == CMD18) ||
        (cmd->cmdidx == CMD24) ||
        (cmd->cmdidx == CMD25) ||
        (cmd->cmdidx == CMD53) ) {
        // We don't need to set this bit for switch function (6) and
        // read scr (51) commands in SD case.
        *creg |= MCI_CMD__DAT_CMD___M;
    }

    if(cmd->resp_type & MMC_RSP_OPCODE) {
        // Don't know what is this for. Seems to work fine
        // without doing anything for this response type.
    }
}

/* Program the cmd registers */
void sdcc_start_command(mmc_t *mmc, mmc_cmd_t *cmd, mmc_data_t *data)
{
    UINT16 creg;
    UINT32 base = ((sd_parms_t*)(mmc->priv))->base;

    sdcc_get_cmd_reg_value(cmd, data, &creg);

    writel(cmd->cmdarg, base + MCI_ARGUMENT);
    udelay(PORT_RUSH_TIMEOUT);
    writel(creg, base + MCI_CMD);
    udelay(PORT_RUSH_TIMEOUT);
}

/* Send command as well as send/receive data */
int sdcc_send_cmd(mmc_t *mmc, mmc_cmd_t *cmd, mmc_data_t *data)
{
    int err = 0;
    UINT32 status;
    UINT32 base = ((sd_parms_t*)(mmc->priv))->base;

    /*if((status = readl(base + MCI_STATUS))) {
        // Some implementation error.
        // must always start with all status bits cleared.
        //printf("%s: Invalid status on entry: status = 0x%x, cmd = %d\n", __FUNCTION__, status, cmd->cmdidx);

        return SDCC_ERR_INVALID_STATUS;
    }*/

    if(data) {
        if(data->flags == MMC_DATA_WRITE)
            return SDCC_ERR_DATA_WRITE; // write not yet implemented.

        sdcc_start_data(mmc, cmd, data);
    }
    sdcc_start_command(mmc, cmd, data);

    // Wait for cmd completion (any response included).
    DEBUG((EFI_D_ERROR, "Wait for cmd completion (any response included)\n"));
    while( !((status = readl(base + MCI_STATUS)) & MCI_STATUS_CMD_COMPLETE_MASK) );

    DEBUG((EFI_D_ERROR, "Read response registers\n"));
    cmd->response[0] = readl(base + MCI_RESPn(0));
    cmd->response[1] = readl(base + MCI_RESPn(1));
    cmd->response[2] = readl(base + MCI_RESPn(2));
    cmd->response[3] = readl(base + MCI_RESPn(3));

    if(status & MCI_STATUS__CMD_CRC_FAIL___M) {
        if(cmd->resp_type & MMC_RSP_CRC) {
            // failure.
            DEBUG((EFI_D_ERROR, "SDCC_ERR_CRC_FAIL\n"));
            err = SDCC_ERR_CRC_FAIL;
        }
        // clear status bit.
        MCI_CLEAR_STATUS(base, MCI_STATUS__CMD_CRC_FAIL___M);
    } else
	if(status & MCI_STATUS__CMD_TIMEOUT___M) {
        // failure.
        err = SDCC_ERR_TIMEOUT;

        // clear timeout status bit.
        MCI_CLEAR_STATUS(base, MCI_STATUS__CMD_TIMEOUT___M);
    } else 
	if(status & MCI_STATUS__CMD_SENT___M) {
        // clear CMD_SENT status bit.
        MCI_CLEAR_STATUS(base, MCI_STATUS__CMD_SENT___M);
    } else 
	if(status & MCI_STATUS__CMD_RESPONSE_END___M) {
        // clear CMD_RESP_END status bit.
        MCI_CLEAR_STATUS(base, MCI_STATUS__CMD_RESPONSE_END___M);
    }

    if(cmd->resp_type & MMC_RSP_BUSY) {
        // Must wait for PROG_DONE status to be set before returning.
        while( !(readl(base + MCI_STATUS) & MCI_STATUS__PROG_DONE___M) );

        // now clear that bit.
        MCI_CLEAR_STATUS(base, MCI_CLEAR__PROG_DONE_CLR___M);
    }

    // if this was one of the data read/write cmds, handle the data.
    DEBUG((EFI_D_ERROR, "if this was one of the data read/write cmds, handle the data\n"));
	MicroSecondDelay(2000000);
    if(data) {
        if(err) {
            // there was some error while sending the cmd. cancel the data operation.
            DEBUG((EFI_D_ERROR, "there was some error while sending the cmd. cancel the data operation.\n"));
	        MicroSecondDelay(2000000);
            writel(0x0, base + MCI_DATA_CTL);
            udelay(PORT_RUSH_TIMEOUT);
        } else {
            if(data->flags == MMC_DATA_READ) {
                DEBUG((EFI_D_ERROR, "SEND THE READ COMMAND\n"));
	            MicroSecondDelay(2000000);
                err = sdcc_read_data(mmc, cmd, data);//this probably retyrns
            } else {
                // write data is not implemented.
                // placeholder for future update.
                while(1);
            }
        }
    }

    if(err) {
        status = readl(base + MCI_STATUS);
        //printf("%s: cmd = %d, err = %d status = 0x%x, response = 0x%x\n", __FUNCTION__, cmd->cmdidx, err, status, cmd->response[0]);
        //17, -8, 0x2
        //if(data)
            //printf("%s: blocksize = %d blocks = %d\n", __FUNCTION__, data->blocksize, data->blocks);
        DEBUG((EFI_D_ERROR, "%s: cmd = %d, err = %d status = 0x%x, response = 0x%x\n", __FUNCTION__, cmd->cmdidx, err, status, cmd->response[0]));
	    MicroSecondDelay(4000000);
			
        return err;
    }

    // read status bits to verify any condition that was not handled.
    // SDIO_INTR bit is set on D0 line status and is valid only in
    // case of SDIO interface. So it can be safely ignored.
    if(readl(base + MCI_STATUS) & MCI_STATUS__SDIO_INTR___M) {
        MCI_CLEAR_STATUS(base, MCI_CLEAR__SDIO_INTR_CLR___M);
    }

    if((status = readl(base + MCI_STATUS))) {
        // Some implementation error.
        // must always exit with all status bits cleared.
        //printf("%s: Invalid status on exit: status = 0x%x, cmd = %d\n", __FUNCTION__, status, cmd->cmdidx);

        //return SDCC_ERR_INVALID_STATUS;
    }

    return err;
}

//sttaic, last two: lbain_t
int mmc_read_blocks(struct mmc *mmc, void *dst, int start, int blkcnt)
{
	struct mmc_cmd cmd;
	struct mmc_data data;

    DEBUG((EFI_D_ERROR, "Data dest before read %d\n", &dst));

	if (blkcnt > 1)
    {
		cmd.cmdidx = MMC_CMD_READ_MULTIPLE_BLOCK;
    }
	else
    {
		cmd.cmdidx = MMC_CMD_READ_SINGLE_BLOCK;
        DEBUG((EFI_D_ERROR, "Set cmdid - single block\n"));
    }
	/*if (mmc->high_capacity) {
		cmd.cmdarg = start;
        DEBUG((EFI_D_ERROR, "Set start\n"));
	    MicroSecondDelay(8000000);
    }
	else
    {
		cmd.cmdarg = start * 512;//mmc->read_bl_len;
        DEBUG((EFI_D_ERROR, "Set start * 512\n"));
	    MicroSecondDelay(8000000);
    }*/
    cmd.cmdarg = start;
    DEBUG((EFI_D_ERROR, "Set start\n"));

	cmd.resp_type = MMC_RSP_R1;

	data.dest = dst;
	data.blocks = blkcnt;
	data.blocksize = 512;
	data.flags = MMC_DATA_READ;

    DEBUG((EFI_D_ERROR, "Stuff filed in\n"));

	if (sdcc_send_cmd(mmc, &cmd, &data))
    {
        DEBUG((EFI_D_ERROR, "Send cmd returned\n"));
		return 1;
    }

	if (blkcnt > 1) {
		cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
		cmd.cmdarg = 0;
		cmd.resp_type = MMC_RSP_R1b;
		if (sdcc_send_cmd(mmc, &cmd, NULL)) {
//			pr_err("mmc fail to send stop cmd\n");
			return 0;
		}
	}
    DEBUG((EFI_D_ERROR, "Data dest after read %d\n", &dst));

	//return blkcnt;
    return 0;
}

/* Initialize sd controller block */
void sdcc_controller_init(sd_parms_t *sd)
{
    UINT32 mci_clk;

    // Disable all interrupts sources
    writel(0x0, sd->base + MCI_INT_MASK0);
    writel(0x0, sd->base + MCI_INT_MASK1);

    // Clear all status bits
    writel(0x07FFFFFF, sd->base + MCI_CLEAR);
    udelay(PORT_RUSH_TIMEOUT);

    // Power control to the card, enable MCICLK with power save mode
    // disabled, otherwise the initialization clock cycles will be
    // shut off and the card will not initialize.
    writel(MCI_POWER__CONTROL__POWERON, sd->base + MCI_POWER);

    mci_clk = MCI_CLK__FLOW_ENA___M
			|(MCI_CLK__SELECT_IN__ON_THE_FALLING_EDGE_OF_MCICLOCK << MCI_CLK__SELECT_IN___S)
			| MCI_CLK__ENABLE___M;
    writel(mci_clk, sd->base + MCI_CLK);
}

/* Called during each scan of mmc devices */
int sdcc_init(mmc_t *mmc)
{
    sd_parms_t *sd = (sd_parms_t*)(mmc->priv);

#ifdef USE_PROC_COMM
	pcom_sdcard_power(1); //enable
	
    // Enable clock
    pcom_enable_sdcard_pclk(sd->instance);

    // Set the interface clock
    pcom_set_sdcard_clk(sd->instance, MCLK_400KHz);
    pcom_enable_sdcard_clk(sd->instance);
#else
    // Set the interface clock
	if (sd->instance == 1) {
		clk_set_rate(SDC1_CLK, MCLK_400KHz);
	} else
	if (sd->instance == 2) {
		clk_set_rate(SDC2_CLK, MCLK_400KHz);
	} else
	if (sd->instance == 3) {
		clk_set_rate(SDC3_CLK, MCLK_400KHz);
	} else
	if (sd->instance == 4) {
		clk_set_rate(SDC4_CLK, MCLK_400KHz);
	}
#endif
	// GPIO config
	pcom_sdcard_gpio_config(sd->instance);
	
    // Initialize controller
    sdcc_controller_init(sd);
	mmc_is_ready = 1;
	
    return 0;
}

static int mmc_send_cmd(uint16_t cmd, uint32_t arg, uint32_t response[])
{
    /* Hacks included : 
     * SDC2_BASE used instead of selection
     */
    uint8_t cmd_timeout = 0, cmd_crc_fail = 0, cmd_response_end = 0, n;
	uint8_t cmd_index = cmd & MCI_CMD__CMD_INDEX___M;
	uint32_t mci_status;

	// Program command argument before programming command register
    DEBUG((EFI_D_ERROR, "Program command argument before programming command register\n"));
	writel(arg, SDC2_BASE + MCI_ARGUMENT);

	// Program command index and command control bits
    DEBUG((EFI_D_ERROR, "Program command index and command control bits\n"));
	writel(cmd, SDC2_BASE + MCI_CMD);

	// Check response if enabled
    DEBUG((EFI_D_ERROR, "Check response if enabled\n"));
	if (cmd & MCI_CMD__RESPONSE___M) {
      // This condition has to be there because CmdCrcFail flag
      // sometimes gets set before CmdRespEnd gets set
      // Wait till one of the CMD flags is set
		while(!(cmd_crc_fail || cmd_timeout || cmd_response_end ||
              ((cmd_index == CMD5 || cmd_index == ACMD41 || cmd_index == CMD1) && cmd_crc_fail)))
		{
			mci_status = readl(SDC2_BASE + MCI_STATUS);
            DEBUG((EFI_D_ERROR, "MCI status: %p\n", mci_status));
			// command crc failed if flag is set
			cmd_crc_fail = (mci_status & MCI_STATUS__CMD_CRC_FAIL___M) >> MCI_STATUS__CMD_CRC_FAIL___S;
			// command response received w/o error
			cmd_response_end = (mci_status & MCI_STATUS__CMD_RESPONSE_END___M) >> MCI_STATUS__CMD_RESPONSE_END___S;
			// if CPSM intr disabled ==> timeout enabled
			if (!(cmd & MCI_CMD__INTERRUPT___M)) {
				// command timed out flag is set
                DEBUG((EFI_D_ERROR, "command timed out flag is set\n"));
				cmd_timeout = (mci_status & MCI_STATUS__CMD_TIMEOUT___M) >>	MCI_STATUS__CMD_TIMEOUT___S;
                break;
			}
		}

		// clear 'CmdRespEnd' status bit
        DEBUG((EFI_D_ERROR, "clear 'CmdRespEnd' status bit\n"));
		writel(MCI_CLEAR__CMD_RESP_END_CLT___M, SDC2_BASE + MCI_CLEAR);

		// Wait till CMD_RESP_END flag is cleared to handle slow 'mclks'
        DEBUG((EFI_D_ERROR, "Wait till CMD_RESP_END flag is cleared to handle slow 'mclks'\n"));
		while ((readl(SDC2_BASE + MCI_STATUS) & MCI_STATUS__CMD_RESPONSE_END___M));

		// Clear both just in case
        DEBUG((EFI_D_ERROR, "Clear both just in case\n"));
		writel(MCI_CLEAR__CMD_TIMOUT_CLR___M, SDC2_BASE + MCI_CLEAR);
		writel(MCI_CLEAR__CMD_CRC_FAIL_CLR___M, SDC2_BASE + MCI_CLEAR);

		// Read the contents of 4 response registers (irrespective of
		// long/short response)
        DEBUG((EFI_D_ERROR, "Read the contents of 4 response registers\n"));
		for(n = 0; n < 4; n++) {
			response[n] = readl(SDC2_BASE + MCI_RESPn(n));
		}
		if ((cmd_response_end == 1)
		|| ((cmd_crc_fail == 1) && (cmd_index == CMD5 || cmd_index == ACMD41 || cmd_index == CMD1)))
		{
			return(1);
		} else
		// Assuming argument (or RCA) value of 0x0 will be used for CMD5 and
		// CMD55 before card identification/initialization
		if ((cmd_index == CMD5  && arg == 0 && cmd_timeout == 1)
		|| (cmd_index == CMD55 && arg == 0 && cmd_timeout == 1))
		{
			return(1);
		} else {	
			return(0);
		}
	} else /* No response is required */ {
        DEBUG((EFI_D_ERROR, "No response required\n"));
        //DEBUG((EFI_D_ERROR, "Wait for 'CmdSent' flag to be set in status register\n"));
		// Wait for 'CmdSent' flag to be set in status register
        // HACK: This waits forever so disable for now
		//while(!(readl(SDC2_BASE + MCI_STATUS) & MCI_STATUS__CMD_SENT___M));

		// clear 'CmdSent' status bit
        DEBUG((EFI_D_ERROR, "clear 'CmdSent' status bit\n"));
		writel(MCI_CLEAR__CMD_SENT_CLR___M, SDC2_BASE + MCI_CLEAR);

		// Wait till CMD_SENT flag is cleared. To handle slow 'mclks'
        // HACK: This waits forever so disable for now
        //DEBUG((EFI_D_ERROR, "Wait till CMD_SENT flag is cleared. To handle slow 'mclks'\n"));
		//while(readl(SDC2_BASE + MCI_STATUS) & MCI_STATUS__CMD_SENT___M);

		//return(1);
        DEBUG((EFI_D_ERROR, "Return 0 instead\n"));
        return (0);
	}
}

/* Function to check the card status
 * Maybe read some info too
 */
int sdcc_check_status(mmc_t *mmc)
{
    uint16_t cmd;
    uint32_t response[4] = {0};
    uint32_t i;
    //uint32_t mci_status;

    DEBUG((EFI_D_ERROR, "SDcard test function()\n"));
	MicroSecondDelay(2000000);

    //cmd = ACMD13 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
    //#define ACMD13  0x0D    // SD_STATUS

    // ACMD13  SD_STATUS
    DEBUG((EFI_D_ERROR, "Send ACMD13 (SD status)\n"));
	cmd = ACMD13 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
	if (!mmc_send_cmd(cmd, 0, response))
		return(0);

    DEBUG((EFI_D_ERROR, "ACDM13 done, printing response: \n"));
    int j = 0;
    while(j < 4) {
        DEBUG((EFI_D_ERROR, "ACDM13 %d response: %d\n", j, response[j]));
        j++;
    }

	/*i = 0;
	while(i < 16) {
		mci_status = readl(SDC2_BASE + MCI_STATUS);

		if ((mci_status & MCI_STATUS__RXDATA_AVLBL___M) != 0) {
			data[i] = readl(SDC2_BASE + MCI_FIFO);
			i++;
		} else
		if ((mci_status & MCI_STATUS__RXACTIVE___M) == 0) {
			// Unexpected status on SD status read.
			return(0);
		}
	}*/

    return 0;
}

int card_identification_selection(uint32_t cid[], uint16_t* rca, uint8_t* num_of_io_func)
{
	uint32_t i;
	uint16_t cmd;
	uint32_t response[4] = {0};
	uint32_t arg;
	uint32_t hc_support = 0;
    unsigned int high_capacity = 0;

	// CMD0 - put the card in the idle state
	DEBUG((EFI_D_ERROR,"CMD0 - put the card in the idle state\n"));
	cmd = CMD0 | MCI_CMD__ENABLE___M;
    mmc_send_cmd(cmd, 0x00000000, response);
	/*if (!mmc_send_cmd(cmd, 0x00000000, response))
		DEBUG((EFI_D_ERROR,"CMD0 - fail\n"));
		return(0);*/

	// CMD8 - send interface condition
	// Attempt to init SD v2.0 features
	// voltage range 2.7-3.6V, check pattern AA
	DEBUG((EFI_D_ERROR,"CMD8\n"));
	arg = 0x000001AA;
	cmd = CMD8 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
	if (!mmc_send_cmd(cmd, arg, response))
    {
		hc_support = 0;          // no response, not high capacity
        DEBUG((EFI_D_ERROR,"Error: Card is not HC!!\n"));
        for(;;){};
    }
	else {
		hc_support = (1 << 30);  // HCS bit for ACMD41
        DEBUG((EFI_D_ERROR,"Card is HC!!\n"));
    }

	udelay(1000);
	
	// CMD55
	// Send before any application specific command
	// Use RCA address = 0 during initialization
	DEBUG((EFI_D_ERROR,"CMD55\n"));
	cmd = CMD55 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
	if (!mmc_send_cmd(cmd, 0x00000000, response))
		return(0);

	// ACMD41
	// Reads OCR register contents
	DEBUG((EFI_D_ERROR, "ACMD41\n"));
	arg = 0x00FF8000 | hc_support;
	cmd = ACMD41 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
	if (!mmc_send_cmd(cmd, arg, response))
		return(0);

	// If stuck in an infinite loop after CMD55 or ACMD41 -
	// the card might have gone into inactive state w/o accepting vdd range
	// sent with ACMD41
	// Loop till power up status (busy) bit is set
	while (!(response[0] & 0x80000000))	{
		// A short delay here after the ACMD41 will prevent the next
		// command from failing with a CRC error when I-cache is enabled.
		udelay(1000);

		cmd = CMD55 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
		if (!mmc_send_cmd(cmd, 0x00000000, response))
			return(0);

		cmd = ACMD41 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
		if (!mmc_send_cmd(cmd, arg, response))
			return(0);
	}

	// Check to see if this is a high capacity SD (SDHC) card.
	DEBUG((EFI_D_ERROR, "Check to see if this is a high capacity SD (SDHC) card.\n"));
	if ((response[0] & hc_support) != 0)
		high_capacity = 1;
		DEBUG((EFI_D_ERROR, "It's a high capacity SD (SDHC) card.\n"));

	// A short delay here after the ACMD41 will prevent the next
	// command from failing with a CRC error when I-cache is enabled.
	udelay(1000);

	// CMD2
	// Reads CID register contents - long response R2
	cmd = CMD2 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M | MCI_CMD__LONGRSP___M;
	if (!mmc_send_cmd(cmd, 0x00000000, response))
		return(0);
	for (i = 0; i < 4; i++)
		cid[i] = response[i];

	// CMD3
	cmd = CMD3 | MCI_CMD__ENABLE___M | MCI_CMD__RESPONSE___M;
	if (!mmc_send_cmd(cmd, 0x00000000, response))
		return(0);
	*rca = response[0] >> 16;

	return(1);
}

int mmc_init()
{
    int rc = 0;// -ENODEV;
    uint32_t cid[4] = {0};
    uint32_t csd[4] = {0};
    uint16_t rca;
    uint8_t  dummy;
    uint32_t buffer[128];
    uint32_t temp32;

    DEBUG((EFI_D_ERROR, "Run card ID sequence\n"));
    if (!card_identification_selection(cid, &rca, &dummy)) {
        DEBUG((EFI_D_ERROR, "Card ID sequence failed!\n"));
        for(;;){};
    }

    return 0;
}


EFI_STATUS
EFIAPI
SdccLibInitialize(
	VOID
)
{
    EFI_STATUS Status = EFI_SUCCESS;

    msm_clock_init();

    return Status;
}