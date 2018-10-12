#ifndef FLASH_H_
#define FLASH_H_


void d_flash_init(void);

void d_flash_write(uint32_t addr, const void * const p_src, uint32_t len);

void d_flash_read(const uint32_t addr, void * const p_dest, uint32_t len);

uint32_t *d_flash_get_data_store_addr(void);

void d_flash_erase(uint32_t addr, uint32_t len);

#endif