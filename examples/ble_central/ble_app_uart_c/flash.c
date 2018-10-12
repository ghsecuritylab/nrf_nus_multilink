#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_fstorage_sd.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "flash.h"


#define PAGE_SIZE	 		(4*1024)

#define STORAGE_SIZE 		(5*PAGE_SIZE)
#define START_ADDR	 		(uint32_t)(d_flash_end_addr_get()-STORAGE_SIZE)
#define END_ADDR	 		(uint32_t)(d_flash_end_addr_get())

#define APP_DATA_ADDR		(uint32_t *)(START_ADDR - 0*PAGE_SIZE)
#define PROGRAM_UNIT		 4	
#define FLASH_DATA_LEN(len)	 ((len%PROGRAM_UNIT)?((len/PROGRAM_UNIT + 1)*PROGRAM_UNIT):len)


static nrf_fstorage_t fstorage;



static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
}

static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        
    }
}

static uint32_t d_flash_end_addr_get(void)
{
    uint32_t const bootloader_addr = NRF_UICR->NRFFW[0];
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d:%X bytes",      p_fstorage->p_flash_info->erase_unit,p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d:%X bytes",    p_fstorage->p_flash_info->program_unit,p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

void d_flash_init(void)
{
   	static nrf_fstorage_api_t * p_fs_api;
	p_fs_api = &nrf_fstorage_sd;
	
  	fstorage.evt_handler =  fstorage_evt_handler;
    fstorage.start_addr  =  START_ADDR;
    fstorage.end_addr    =  END_ADDR;
    
	if(NRF_SUCCESS != nrf_fstorage_init(&fstorage, p_fs_api, NULL))
	{
		
	}
	print_flash_info(&fstorage);
}

void d_flash_write(uint32_t addr, const void * const p_src, uint32_t len)
{
  	uint32_t rc  = NRF_SUCCESS;
	
 	rc = nrf_fstorage_erase(&fstorage, addr, 1, NULL);
	if(NRF_SUCCESS != rc)
	{
		
	}
	wait_for_flash_ready(&fstorage);
  	rc = nrf_fstorage_write(&fstorage, addr, p_src, FLASH_DATA_LEN(len), NULL);
	if(NRF_SUCCESS != rc)
	{
		
	}
	wait_for_flash_ready(&fstorage);
}

void d_flash_erase(uint32_t addr, uint32_t len)
{
	uint32_t rc  = NRF_SUCCESS;
  	rc = nrf_fstorage_erase(&fstorage, addr, 1, NULL);
	if(NRF_SUCCESS != rc)
	{
		
	}
	wait_for_flash_ready(&fstorage);
}

uint32_t *d_flash_get_data_store_addr(void)
{
	return APP_DATA_ADDR;
}

void d_flash_read(const uint32_t addr, void * const p_dest, uint32_t len)
{
  	ret_code_t rc = NRF_SUCCESS;
	nrf_fstorage_read(&fstorage, addr, p_dest, FLASH_DATA_LEN(len));
	if(NRF_SUCCESS != rc)
	{

	}
}