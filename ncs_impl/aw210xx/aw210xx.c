#include <stdio.h>
#include "stdlib.h"
#include "string.h"

#include "aw210xx.h"
#include "aw210xx_reg.h"

#include <nrfx_twi.h>
#include <nrfx_twim.h>
#include <nrfx_twis.h>

#define AW210XX_NRF_DRV_VER             "V0.0.1"
#define TWI_INST_ID                     "0"

static volatile bool m_xfer_done = false;
static const nrfx_twis_t m_twis_inst = NRFX_TWIS_INSTANCE(TWI_INST_ID);

//  Child Event Handler
static void twis_handler(nrfx_twis_evt_t const * p_event)
{
    nrfx_err_t status;
    (void)status;

    /* Variable to store register number sent in the last TX. */
    static uint8_t reg_buff;

    switch (p_event->type)
    {
        case NRFX_TWIS_EVT_WRITE_REQ:
            status = nrfx_twis_rx_prepare(&m_twis_inst, &reg_buff, sizeof(reg_buff));
            NRFX_ASSERT(status == NRFX_SUCCESS);
            break;

        case NRFX_TWIS_EVT_READ_REQ:
            status = nrfx_twis_tx_prepare(&m_twis_inst,
                                          &m_drone_reg.bytes[reg_buff],
                                          sizeof(m_drone_reg) - reg_buff);
            NRFX_ASSERT(status == NRFX_SUCCESS);
            break;

        default:
            break;
    }
}


void twi_master_init(void);


bool aw210xx_init(void);
int32_t aw210xx_hw_reset(void);
int32_t aw210xx_en_pull_high(void);
int32_t aw210xx_en_pull_low(void);
void aw210xx_sw_reset(void);