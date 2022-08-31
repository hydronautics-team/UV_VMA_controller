
uint8_t lsm_read(uint16_t Addr, uint8_t Reg);

//Ghi 1 byte tu thanh ghi lsm303dlhc
void lsm_write(uint16_t Addr, uint8_t Reg, uint8_t Value);


//Cau hinh co ban cho lsm303dlhc
void lsm_init(void);

//Khoi dong laj bo nho
void lsm_reboot(void);


//Doc XYZ Acc
void lsm_accxyz(float *pfData);

//Doc xyz Mag
void lsm_magxyz(float* pfData);
