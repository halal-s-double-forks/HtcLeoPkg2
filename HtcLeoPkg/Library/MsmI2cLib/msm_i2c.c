/* msm-i2c.c
 *
 * LK port: Copyright (C) 2011 Alexander Tarasikov <alexander.tarasikov@gmail.com>
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <Library/DebugLib.h>
#include <Library/reg.h>
#include <Chipset/msm_i2c.h>
#include <Library/pcom.h>
#include <Library/gpio.h>
#include <Chipset/iomap.h>
#include <Chipset/timer.h>
#include <Chipset/clock.h>
#include <Library/InterruptsLib.h>
#include <Library/LKEnvLib.h>
#include <Library/HtcLeoGpio.h>
#include <Library/MsmI2cLib.h>
#include <Library/IoLib.h>
#include <Library/BaseLib.h>
#include <Uefi.h>
#include <Library/UefiBootServicesTableLib.h>
//#include <Kernel/kernelthread.h>

//EFI_BOOT_SERVICES *BootServices = SystemTable->BootServices;

#define DEBUG_I2C 1


	#define I2C_DBG(lvl, fmt, x...)
	#define I2C_ERR(fmt, x...)
	#define I2C_DBG_FUNC_LINE()

enum {
	I2C_WRITE_DATA 			= 0x00,
	I2C_CLK_CTL 			= 0x04,
	I2C_STATUS 				= 0x08,
	I2C_READ_DATA 			= 0x0c,
	I2C_INTERFACE_SELECT 	= 0x10,

	I2C_WRITE_DATA_DATA_BYTE 			= 0xff,
	I2C_WRITE_DATA_ADDR_BYTE 			= 1U << 8,
	I2C_WRITE_DATA_LAST_BYTE			= 1U << 9,

	I2C_CLK_CTL_FS_DIVIDER_VALUE 		= 0xff,
	I2C_CLK_CTL_HS_DIVIDER_VALUE 		= 7U << 8,

	I2C_STATUS_WR_BUFFER_FULL 			= 1U << 0,
	I2C_STATUS_RD_BUFFER_FULL 			= 1U << 1,
	I2C_STATUS_BUS_ERROR 				= 1U << 2,
	I2C_STATUS_PACKET_NACKED 			= 1U << 3,
	I2C_STATUS_ARB_LOST 				= 1U << 4,
	I2C_STATUS_INVALID_WRITE 			= 1U << 5,
	I2C_STATUS_FAILED 					= 3U << 6,
	I2C_STATUS_BUS_ACTIVE 				= 1U << 8,
	I2C_STATUS_BUS_MASTER 				= 1U << 9,
	I2C_STATUS_ERROR_MASK 				= 0xfc,

	I2C_INTERFACE_SELECT_INTF_SELECT	= 1U << 0,
	I2C_INTERFACE_SELECT_SCL 			= 1U << 8,
	I2C_INTERFACE_SELECT_SDA 			= 1U << 9,
};

static struct msm_i2c_dev dev;
static int timeout;

#if DEBUG_I2C
static void dump_status(uint32_t status)
{
	//DEBUG((EFI_D_ERROR, "I2C STATUS DUMP\n"));

	//I2C_DBG(//DEBUGLEVEL, "STATUS (0x%.8x) ", status);
	//DEBUG((EFI_D_ERROR, "STATUS (0x%.8x) ", status));
	if (status & I2C_STATUS_BUS_MASTER){
		//I2C_DBG(//DEBUGLEVEL, "MST ");
		//DEBUG((EFI_D_ERROR, "MST "));
		}
	if (status & I2C_STATUS_BUS_ACTIVE){
		//I2C_DBG(//DEBUGLEVEL, "ACT ");
		//DEBUG((EFI_D_ERROR, "ACT "));
	}
	if (status & I2C_STATUS_INVALID_WRITE){
		//I2C_DBG(//DEBUGLEVEL, "INV_WR ");
		//DEBUG((EFI_D_ERROR, "INV_WR "));
		}
	if (status & I2C_STATUS_ARB_LOST){
		//I2C_DBG(//DEBUGLEVEL, "ARB_LST ");
		//DEBUG((EFI_D_ERROR, "ARB_LST "));
		}
	if (status & I2C_STATUS_PACKET_NACKED){
		//I2C_DBG(//DEBUGLEVEL, "NAK ");
		//DEBUG((EFI_D_ERROR, "NAK "));
		}
	if (status & I2C_STATUS_BUS_ERROR){
		//I2C_DBG(//DEBUGLEVEL, "BUS_ERR ");
		//DEBUG((EFI_D_ERROR, "BUS_ERR "));
		}
	if (status & I2C_STATUS_RD_BUFFER_FULL){
		//I2C_DBG(//DEBUGLEVEL, "RD_FULL ");
		//DEBUG((EFI_D_ERROR, "RD_FULL "));
		}
	if (status & I2C_STATUS_WR_BUFFER_FULL){
		//I2C_DBG(//DEBUGLEVEL, "WR_FULL ");
		//DEBUG((EFI_D_ERROR, "WR_FULL "));
		}
	if (status & I2C_STATUS_FAILED) {
		//I2C_DBG(//DEBUGLEVEL, "FAIL 0x%x", (status & I2C_STATUS_FAILED));
		//DEBUG((EFI_D_ERROR, "FAIL 0x%x", (status & I2C_STATUS_FAILED)));
		}
	//I2C_DBG(//DEBUGLEVEL, "\n");
	//DEBUG((EFI_D_ERROR, "\n"));
	//MicroSecondDelay(2000);
}
#endif

static void msm_i2c_write_delay(void)
{
	/* If scl is still high we have >4us (for 100kbps) to write the data
	 * register before we risk hitting a bug where the controller releases
	 * scl to soon after driving sda low. Writing the data after the
	 * scheduled release time for scl also avoids the bug.
	 */
	if (readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT) & I2C_INTERFACE_SELECT_SCL){
		return;
	}
		
	NanoSecondDelay(6);
}

static bool msm_i2c_fill_write_buffer(void)
{
	//DEBUG((EFI_D_ERROR, "MSM_I2C_FILL_WRITE_BUFFER FUNCTION\n"));
	uint16_t val;
	if (dev.pos < 0) {
		//DEBUG((EFI_D_ERROR, "IF IN LINE 155 TRUE\n"));
		//DEBUG((EFI_D_ERROR, "dev.msg->addr(%x)\n", dev.msg->addr));
		val = I2C_WRITE_DATA_ADDR_BYTE | dev.msg->addr << 1;
		if (dev.msg->flags & I2C_M_RD)
			val |= 1;
			
		if (dev.rem == 1 && dev.msg->len == 0)
			val |= I2C_WRITE_DATA_LAST_BYTE;
				//DEBUG((EFI_D_ERROR, "MSM_I2C_WRITE_DELAY ABOUT TO HAPPEN\n"));
		msm_i2c_write_delay();
		//DEBUG((EFI_D_ERROR, "MSM_I2C_WRITE_DELAY HAPPENED\n"));

		DEBUG((EFI_D_ERROR, "CRITICAL: writel(val, dev.pdata->i2c_base + I2C_WRITE_DATA);\n"));
		DEBUG((EFI_D_ERROR, "CRITICAL: writel(%x, %x + %x)\n", val, dev.pdata->i2c_base, I2C_WRITE_DATA));
		MicroSecondDelay(2000);
		writel(val, dev.pdata->i2c_base + I2C_WRITE_DATA);
		dev.pos++;
		//DEBUG((EFI_D_ERROR, "writel(val, dev.pdata->i2c_base + I2C_WRITE_DATA); HAPPENED\n"));
		//MicroSecondDelay(10000);
		return true;
	}else {
		//DEBUG((EFI_D_ERROR, "IF IN LINE 138 WAS NOT TRUE\n"));
	}

	if (dev.msg->flags & I2C_M_RD){
		return false;
		}

	if (!dev.cnt){
		return false;
		}

	/* Ready to take a byte */
	val = dev.msg->buf[dev.pos];
	if (dev.cnt == 1 && dev.rem == 1)
		val |= I2C_WRITE_DATA_LAST_BYTE;
	msm_i2c_write_delay();
	DEBUG((EFI_D_ERROR, "writel(%x, %x + %x)\n", val, dev.pdata->i2c_base, I2C_WRITE_DATA));
	MicroSecondDelay(1000);
	writel(val, dev.pdata->i2c_base + I2C_WRITE_DATA);
	dev.pos++;
	dev.cnt--;
	
	return true;
}

static void msm_i2c_read_buffer(void)
{
	/*
	 * Theres something in the FIFO.
	 * Are we expecting data or flush crap?
	 */
	//I2C_DBG_FUNC_LINE();
	if ((dev.msg->flags & I2C_M_RD) && dev.pos >= 0 && dev.cnt) {
		switch (dev.cnt) {
			case 1:
				if (dev.pos != 0)
					break;
				dev.need_flush = true;
				/* fall-trough */
			case 2:
				writel(I2C_WRITE_DATA_LAST_BYTE, dev.pdata->i2c_base + I2C_WRITE_DATA);
		}
		dev.msg->buf[dev.pos] = readl(dev.pdata->i2c_base + I2C_READ_DATA);
		dev.cnt--;
		dev.pos++;
	} else { /* FLUSH */
		if (dev.flush_cnt & 1) {
			/*
			* Stop requests are sometimes ignored, but writing
			* more than one request generates a write error.
			*/
			writel(I2C_WRITE_DATA_LAST_BYTE, dev.pdata->i2c_base + I2C_WRITE_DATA);
		}
		
		readl(dev.pdata->i2c_base + I2C_READ_DATA);
		
		if (dev.need_flush)
			dev.need_flush = false;
		else
			dev.flush_cnt++;
	}
}

static void msm_i2c_interrupt_locked(void)
{
	//DEBUG((EFI_D_ERROR, "MSM_I2C_INTERRUPT_LOCKED FUNCTION\n"));
	//MicroSecondDelay(2000);
	uint32_t status	= readl(dev.pdata->i2c_base + I2C_STATUS);
	bool not_done = true;

#if DEBUG_I2C
	dump_status(status);
#endif
	if (!dev.msg) {
		//I2C_DBG(DEBUGLEVEL, "IRQ but nothing to do!, status %x\n", status);
		//DEBUG((EFI_D_ERROR, "IRQ but nothing to do!, status %x\n", status));
		return;
	}
	
	if (status & I2C_STATUS_ERROR_MASK){
		//DEBUG((EFI_D_ERROR, "ERROR DETECTED ENTERING ERROR STATE\n"));
		//MicroSecondDelay(2000);
		goto out_err;
		}

	if (!(status & I2C_STATUS_WR_BUFFER_FULL)){
	//DEBUG((EFI_D_ERROR, "FILL WRITE BUFFER ABOUT TO HAPPEN\n"));
	//MicroSecondDelay(5000);
		not_done = msm_i2c_fill_write_buffer();
		//DEBUG((EFI_D_ERROR, "FILL WRITE BUFFER HAPPENED\n"));
		//MicroSecondDelay(5000);
		}
	if (status & I2C_STATUS_RD_BUFFER_FULL){
		//DEBUG((EFI_D_ERROR, "READ I2C BUFFER ABOUT TO HAPPEN\n"));
		msm_i2c_read_buffer();
		//DEBUG((EFI_D_ERROR, "READ I2C BUFFER HAPPENED\n"));
		}

	if (dev.pos >= 0 && dev.cnt == 0) {
		//DEBUG((EFI_D_ERROR, "IF IN LINE 242 TRUE\n"));
		if (dev.rem > 1) {
			//DEBUG((EFI_D_ERROR, "IF IN LINE 244 TRUE\n"));
			dev.rem--;
			dev.msg++;
			dev.pos = -1;
			dev.cnt = dev.msg->len;
		}
		else if (!not_done && !dev.need_flush) {
			//DEBUG((EFI_D_ERROR, "IF IN LINE 256 TRUE\n"));
			timeout = 0;
			return;
		}
	}
	//DEBUG((EFI_D_ERROR, "INTERRUPT RETURNING NOW\n"));
	//MicroSecondDelay(2000);
	return;

out_err:
	I2C_ERR("error, status %x\n", status);
	//DEBUG((EFI_D_ERROR, "error, status %x\n", status));
	dev.ret = ERROR;
	timeout = ERR_TIMED_OUT;
}

static enum handler_return msm_i2c_isr(void *arg) {
	//enter_critical_section();
	//OldTpl1 = gBS->RaiseTPL (TPL_HIGH_LEVEL);
	//EfiAcquireLock(&MySpinLock);
	msm_i2c_interrupt_locked();
	//EfiReleaseLock(&MySpinLock);
	//gBS->RestoreTPL (OldTpl1);
	//exit_critical_section();
	
	return INT_RESCHEDULE;
}

static int msm_i2c_poll_notbusy(int warn)
{
	//DEBUG((EFI_D_ERROR, "MSM_I2C_POLL_NOTBUSY FUNCTION \n"));
	//MicroSecondDelay(3000);
	uint32_t retries = 0;
	while (retries != 200) {
		uint32_t status = readl(dev.pdata->i2c_base + I2C_STATUS);
		uint32_t bit_value = (status >> 7) & 0x01;
		DEBUG((EFI_D_ERROR, "Status AND I2C_STATUS_BUS_ACTIVE %u\n", !(status & I2C_STATUS_BUS_ACTIVE)));
		if (!(status & I2C_STATUS_BUS_ACTIVE)) {

			if (retries && warn){
				//I2C_DBG(DEBUGLEVEL, "Warning bus was busy (%d)\n", retries);
				DEBUG((EFI_D_ERROR, "Warning bus was busy retries=%d\n", retries));
				
			}
			return 0;
		}
		
		if (retries > 100){
			MicroSecondDelay(10000);
			}
		retries++;
	}
	
	//I2C_ERR("Error waiting for notbusy\n");
	DEBUG((EFI_D_ERROR, "Error waiting for notbusy \n"));
	//MicroSecondDelay(2000);
	return ERR_TIMED_OUT;
}

static int msm_i2c_recover_bus_busy(void)
{
	int i;
	bool gpio_clk_status = false;
	uint32_t status = readl(dev.pdata->i2c_base + I2C_STATUS);
	//I2C_DBG_FUNC_LINE();

	if (!(status & (I2C_STATUS_BUS_ACTIVE | I2C_STATUS_WR_BUFFER_FULL)))
		return 0;

	if (dev.pdata->set_mux_to_i2c)
		dev.pdata->set_mux_to_i2c(0);

	if (status & I2C_STATUS_RD_BUFFER_FULL) {
		//I2C_DBG(//DEBUGLEVEL, "Read buffer full, status %x, intf %x\n",  status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT));
		//DEBUG((EFI_D_ERROR, "Read buffer full, status %x, intf %x\n",  status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT)));
		writel(I2C_WRITE_DATA_LAST_BYTE, dev.pdata->i2c_base + I2C_WRITE_DATA);
		//IoWrite16((UINTN)dev.pdata->i2c_base + I2C_WRITE_DATA, (UINT16)I2C_WRITE_DATA_LAST_BYTE);
		readl(dev.pdata->i2c_base + I2C_READ_DATA);
	}
	else if (status & I2C_STATUS_BUS_MASTER) {
		//I2C_DBG(//DEBUGLEVEL, "Still the bus master, status %x, intf %x\n", status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT));
		//DEBUG((EFI_D_ERROR, "Still the bus master, status %x, intf %x\n", status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT)));
		writel(I2C_WRITE_DATA_LAST_BYTE | 0xff, dev.pdata->i2c_base + I2C_WRITE_DATA);
		//IoWrite16((UINTN)dev.pdata->i2c_base + I2C_WRITE_DATA, (UINT16)I2C_WRITE_DATA_LAST_BYTE | 0xff);
	}

	//I2C_DBG(//DEBUGLEVEL, "i2c_scl: %d, i2c_sda: %d\n", gpio_get(dev.pdata->scl_gpio), gpio_get(dev.pdata->sda_gpio));
	//DEBUG((EFI_D_ERROR, "i2c_scl: %d, i2c_sda: %d\n", gpio_get(dev.pdata->scl_gpio), gpio_get(dev.pdata->sda_gpio)));

	for (i = 0; i < 9; i++) {
		if (gpio_get(dev.pdata->sda_gpio) && gpio_clk_status)
			break;
			
		gpio_set(dev.pdata->scl_gpio, 0);
		NanoSecondDelay(5);
		
		gpio_set(dev.pdata->sda_gpio, 0);
		NanoSecondDelay(5);
		
		gpio_config(dev.pdata->scl_gpio, GPIO_INPUT);
		NanoSecondDelay(5);
		
		if (!gpio_get(dev.pdata->scl_gpio))
			NanoSecondDelay(20);
			
		if (!gpio_get(dev.pdata->scl_gpio))
			//MicroSecondDelay(10);
			
		gpio_clk_status = gpio_get(dev.pdata->scl_gpio);
		gpio_config(dev.pdata->sda_gpio, GPIO_INPUT);
		NanoSecondDelay(5);
	}
	
	if (dev.pdata->set_mux_to_i2c)
		dev.pdata->set_mux_to_i2c(1);

	NanoSecondDelay(10);

	status = readl(dev.pdata->i2c_base + I2C_STATUS);
	if (!(status & I2C_STATUS_BUS_ACTIVE)) {
		//I2C_DBG(//DEBUGLEVEL, "Bus busy cleared after %d clock cycles, status %x, intf %x\n", i, status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT));
		//DEBUG((EFI_D_ERROR, "Bus busy cleared after %d clock cycles, status %x, intf %x\n", i, status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT)));
		return 0;
	}
	
	//I2C_DBG(//DEBUGLEVEL, "Bus still busy, status %x, intf %x\n", status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT));
	//DEBUG((EFI_D_ERROR, "Bus still busy, status %x, intf %x\n", status, readl(dev.pdata->i2c_base + I2C_INTERFACE_SELECT)));
	return ERR_NOT_READY;
}

int msm_i2c_xfer(struct i2c_msg msgs[], int num)
{
	//DEBUG((EFI_D_ERROR, "MSM_I2C_XFER FUNCTION \n"));
	int ret, ret_wait;
	//DEBUG((EFI_D_ERROR, "ABOUT TO ENABLE SOME CLOCK \n"));
	clk_enable(dev.pdata->clk_nr);
	//DEBUG((EFI_D_ERROR, "CLOCK ENABLED \n"));
	//DEBUG((EFI_D_ERROR, "ABOUT TO UNMASK SOME INTERRUPT \n"));
	unmask_interrupt(dev.pdata->irq_nr);
	//DEBUG((EFI_D_ERROR, "INTTERRUPT UNMASKED \n"));

	ret = msm_i2c_poll_notbusy(1);
	DEBUG((EFI_D_ERROR, "MSM_I2C_POLL_NOTBUSY RETURNED %d\n", ret));
	//MicroSecondDelay(2000);

	if (ret) {
		//DEBUG((EFI_D_ERROR, "ABOUT TO RECOVER BUS BUSY \n", ret));
			//MicroSecondDelay(2000);
		ret = msm_i2c_recover_bus_busy();
		//DEBUG((EFI_D_ERROR, "BUS BUSY RECOVERD \n", ret));
			//MicroSecondDelay(2000);
		if (ret){
			//DEBUG((EFI_D_ERROR, "RECOVER FAILED ENTERING ERROR STATE \n", ret));
			//MicroSecondDelay(20000);
			goto err;
			}
	}

	//enter_critical_section();
	//OldTpl2 = gBS->RaiseTPL (TPL_HIGH_LEVEL);
	//EfiAcquireLock(&MySpinLock);
	if (dev.flush_cnt) {
		//I2C_DBG(//DEBUGLEVEL, "%d unrequested bytes read\n", dev.flush_cnt);
		//DEBUG((EFI_D_ERROR, "%d unrequested bytes read\n", dev.flush_cnt));
	}
	dev.msg = msgs;
	dev.rem = num;
	dev.pos = -1;
	dev.ret = num;
	dev.need_flush = false;
	dev.flush_cnt = 0;
	dev.cnt = msgs->len;
	//DEBUG((EFI_D_ERROR, "ABOUT TO LOCK INTERRUPT\n"));
	msm_i2c_interrupt_locked();
	//DEBUG((EFI_D_ERROR, "INTERRUPT LOCKED\n"));
	//MicroSecondDelay(5000);

	//exit_critical_section();
	//DEBUG((EFI_D_ERROR, "INTERRUPT LOCKED 2\n"));
	//MicroSecondDelay(5000);
	//EfiReleaseLock(&MySpinLock);

	/*
	 * Now that we've setup the xfer, the ISR will transfer the data
	 * and wake us up with dev.err set if there was an error
	 */
	ret_wait = msm_i2c_poll_notbusy(0); /* Read may not have stopped in time */
	//DEBUG((EFI_D_ERROR, "RET VALUE IS HERE FROM NOTBUSY %d\n", ret_wait));
	//MicroSecondDelay(2000);
	
	//enter_critical_section();
	//OldTpl3 = gBS->RaiseTPL (TPL_HIGH_LEVEL);
	//EfiAcquireLock(&MySpinLock);
	if (dev.flush_cnt) {
		//I2C_DBG(//DEBUGLEVEL, "%d unrequested bytes read\n", dev.flush_cnt);
		//DEBUG((EFI_D_ERROR, "%d unrequested bytes read\n", dev.flush_cnt));
		//MicroSecondDelay(2000);
	}
	ret = dev.ret;
	dev.msg = NULL;
	dev.rem = 0;
	dev.pos = 0;
	dev.ret = 0;
	dev.flush_cnt = 0;
	dev.cnt = 0;
	//exit_critical_section();
	//gBS->RestoreTPL (OldTpl3);
	//EfiReleaseLock(&MySpinLock);

	if (ret_wait != 0) {
		//I2C_DBG(//DEBUGLEVEL, "Still busy after xfer completion\n");
		//DEBUG((EFI_D_ERROR, "Still busy after xfer completion\n"));
		//MicroSecondDelay(2000);
		ret_wait = msm_i2c_recover_bus_busy();
		if (ret_wait != 0){
			DEBUG((EFI_D_ERROR, "ERROR IN RECOVERING THE BUS\n"));
			//MicroSecondDelay(5000);
			goto err;
		}
			
	}
	if (timeout == ERR_TIMED_OUT) {
		//I2C_DBG(//DEBUGLEVEL, "Transaction timed out\n");
		//DEBUG((EFI_D_ERROR, "Transaction timed out\n"));
		//MicroSecondDelay(2000);
		ret = ERR_TIMED_OUT;
	}
	if (ret < 0) {
		I2C_ERR("Error during data xfer (%d)\n", ret);
		//DEBUG((EFI_D_ERROR, "Error during data xfer (%d)\n", ret));
		//MicroSecondDelay(2000);
		msm_i2c_recover_bus_busy();
	}
	/* if (timeout == ERR_TIMED_OUT) {
		//I2C_DBG(//DEBUGLEVEL, "Transaction timed out\n");
		ret = 2;
		msm_i2c_recover_bus_busy();
	}
	if (timeout == ERR_TIMED_OUT) {
		//I2C_DBG(//DEBUGLEVEL, "Transaction timed out\n");
		ret = ERR_TIMED_OUT;
	}
	if (ret < 0) {
		I2C_ERR("Error during data xfer (%d)\n", ret);
		msm_i2c_recover_bus_busy();
	} */
	return ret;
err:
DEBUG((EFI_D_ERROR, "ERROR STATE ENTERED I2C XFER"));
// MicroSecondDelay(20000);
	mask_interrupt(dev.pdata->irq_nr);
	clk_disable(dev.pdata->clk_nr);
	
	return ret;
}

//struct mutex msm_i2c_rw_mutex;
int msm_i2c_write(int chip, void *buf, size_t count)
{
	int rc = ERR_NOT_READY;
	if (!dev.pdata) {
		//I2C_DBG(ALWAYS, "[MSM-I2C]: %s: called when driver is not installed\n", __func__);
		//DEBUG((EFI_D_ERROR, "[MSM-I2C]: %s: called when driver is not installed\n", __func__));
		//MicroSecondDelay(2000);
		return rc;
	}

	struct i2c_msg msg[] = {
		{.addr = chip,	.flags = 0,		.len = count,	.buf = buf,}
	};
	
	int retry;
	for (retry = 0; retry <= MSM_I2C_WRITE_RETRY_TIMES; retry++) {
		if (msm_i2c_xfer(msg, 1) == 1) {
			rc = 0;
			break;
		}
		//MicroSecondDelay(10);
		//I2C_DBG(//DEBUGLEVEL, "%s, i2c write retry\n", __func__);
	}

	return rc;
}

int msm_i2c_read(int chip, uint8_t reg, void *buf, size_t count)
{
	int rc = ERR_NOT_READY;
	if (!dev.pdata) {
		//I2C_DBG(ALWAYS, "[MSM-I2C]: %s: called when driver is not installed\n", __func__);
		//DEBUG((EFI_D_ERROR, "[MSM-I2C]: %s: called when driver is not installed\n", __func__));
		//MicroSecondDelay(2000);
		return rc;
	}

	struct i2c_msg msgs[] = {
		{.addr = chip,	.flags = 0,			.len = 1,		.buf = &reg,},
		{.addr = chip,	.flags = I2C_M_RD,	.len = count,	.buf = buf, },
	};
	
	int retry;
	for (retry = 0; retry <= MSM_I2C_READ_RETRY_TIMES; retry++) {
		if (msm_i2c_xfer(msgs, 2) == 2) {
			rc = 0;
			break;
		}
		//MicroSecondDelay(10);
		//I2C_DBG(//DEBUGLEVEL, "%s, i2c read retry\n", __func__);
	}
	
	return rc;
}

int msm_i2c_probe(struct msm_i2c_pdata* pdata)
{
	if (dev.pdata) {
		//I2C_DBG(ALWAYS, "[MSM-I2C]: already installed\n");
		//DEBUG((EFI_D_ERROR, "[MSM-I2C]: already installed\n"));
		//MicroSecondDelay(2000);
		return -1;
	}
	if (!pdata->set_mux_to_i2c)
		return -1;

	dev.pdata = pdata;
	
	//enter_critical_section();
	//OldTpl4 = gBS->RaiseTPL (TPL_HIGH_LEVEL);
	
	//EfiAcquireLock(&MySpinLock);
	mask_interrupt(dev.pdata->irq_nr);
	dev.pdata->set_mux_to_i2c(0);
	clk_enable(dev.pdata->clk_nr);
	
	int i2c_clk 	= 19200000;
	int target_clk 	= 100000;
	if (dev.pdata->i2c_clock) {
		if (dev.pdata->i2c_clock < 100000 || dev.pdata->i2c_clock > 400000)
			target_clk = 100000;
		else
			target_clk = dev.pdata->i2c_clock;
	}
	int fs_div = ((i2c_clk / target_clk) / 2) - 3;
	int hs_div = 3;
	int clk_ctl = ((hs_div & 0x7) << 8) | (fs_div & 0xff);
	//DEBUG((EFI_D_ERROR, "writel(%x, %x + %x)\n", clk_ctl, dev.pdata->i2c_base + I2C_CLK_CTL));
	//MicroSecondDelay(1000);
	writel(clk_ctl, dev.pdata->i2c_base + I2C_CLK_CTL);
	//IoWrite16((UINTN)dev.pdata->i2c_base + I2C_CLK_CTL, (UINT16)clk_ctl)
	//I2C_DBG(//DEBUGLEVEL, "msm_i2c_probe: clk_ctl %x, %d Hz\n", clk_ctl, i2c_clk / (2 * ((clk_ctl & 0xff) + 3)));
	//DEBUG((EFI_D_ERROR, "msm_i2c_probe: clk_ctl %x, %d Hz\n", clk_ctl, i2c_clk / (2 * ((clk_ctl & 0xff) + 3))));

	clk_disable(dev.pdata->clk_nr);
	register_int_handler(dev.pdata->irq_nr, msm_i2c_isr, NULL);
	unmask_interrupt(dev.pdata->irq_nr);
	//exit_critical_section();
	//gBS->RestoreTPL (OldTpl4);
	//EfiReleaseLock(&MySpinLock);

	return 0;
}

void msm_i2c_remove() {
	if (!dev.pdata)
		return;//if driver is not installed
		
	//enter_critical_section();
	//OldTpl5 = gBS->RaiseTPL (TPL_HIGH_LEVEL);
	//EfiAcquireLock(&MySpinLock);
	mask_interrupt(dev.pdata->irq_nr);
	clk_disable(dev.pdata->clk_nr);
	dev.pdata->set_mux_to_i2c(0);
	dev.pdata = NULL;
	//exit_critical_section();
	//gBS->RestoreTPL (OldTpl5);
	//EfiReleaseLock(&MySpinLock);
}
