#define USE_PROC_COMM

//functions
UINT8 usbhs_ulpi_phy_read_reg(UINT32 address);
void usbhs_ulpi_phy_write_reg(UINT32 address, UINT32 write_data);
void usbhs_ulpi_phy_init(void);
int ehci_hcd_init_qc(UINT32 *hccr_ptr, UINT32 *hcor_ptr);
int ehci_hcd_stop_qc(UINT32 hccr, UINT32 hcor);