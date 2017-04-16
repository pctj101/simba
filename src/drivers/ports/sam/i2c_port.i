/**
 * @section License
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2014-2016, Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * This file is part of the Simba project.
 */

/* SAM: TWI Address */
/* from sam3.h */
/*
#define SAM_TWI0       ((volatile struct sam_twi_t    *)0x4008c000u)
#define SAM_TWI1       ((volatile struct sam_twi_t    *)0x40090000u)
*/

/* SAM: Register Mapping */
#define SAM_TWI_CR      0x00 /* W/O */

#define SAM_TWI_MMR     0x04 /* R/W */
#define SAM_TWI_SMR     0x08 /* R/W */
#define SAM_TWI_IADR    0x0C /* R/W */
#define SAM_TWI_CWGR    0x10 /* R/W */

#define SAM_TWI_SR      0x20 /* R/O */

#define SAM_TWI_IER     0x24 /* W/O */
#define SAM_TWI_IDR     0x28 /* W/O */

#define SAM_TWI_IMR     0x2C /* R/O */
#define SAM_TWI_RHR     0x30 /* R/O */

#define SAM_TWI_THR     0x34 /* W/O */

/* TWI_CR */
#define SAM_TWI_CR_START    (1)
#define SAM_TWI_CR_STOP     (1<<1)
#define SAM_TWI_CR_MSEN     (1<<2) /* Enable TWI Master Mode (Iff MSDIS == 0)  */
#define SAM_TWI_CR_MSDIS    (1<<3)
#define SAM_TWI_CR_SVEN     (1<<4)
#define SAM_TWI_CR_SVDIS    (1<<5)
#define SAM_TWI_CR_QUICK    (1<<6)
#define SAM_TWI_CR_SWRST    (1<<7)

/* TWI_MMR */
#define SAM_TWI_MMR_MREAD       (1 << 12)
#define SAM_TWI_MMR_DADR_MASK   (0b01111111 << 16) /* Device Address Mask */
#define SAM_TWI_MMR_IADRSZ_0    (0b00000000 << 8)  /* Internal Device Address Size: None */
#define SAM_TWI_MMR_IADRSZ_1    (0b00000001 << 8)  /* Internal Device Address Size: 1 Byte */
#define SAM_TWI_MMR_IADRSZ_2    (0b00000010 << 8)  /* Internal Device Address Size: 2 Byte */
#define SAM_TWI_MMR_IADRSZ_3    (0b00000011 << 8)  /* Internal Device Address Size: 3 Byte */

/* TWI_SMR */
#define SAM_TWI_SMR_SADR_MASK   (0b01111111 << 16) /* Device Address Mask */

/* TWI_IADR */
#define SAM_TWI_IADR_MASK       (0xFF << 16) | (0xFF << 8) | (0xFF) /* 0, 1, 2 or 3 bytes depending on IADRSZ. */

/* TWI_CWGR */
#define SAM_TWI_CWGR_CKDIV_MASK       (0b111 << 16)
#define SAM_TWI_CWGR_CHDIV_MASK       (0xff  << 8)
#define SAM_TWI_CWGR_CLDIV_MASK       (0xff)

/* TWI_SR */
#define SAM_TWI_SR_TXCOMP       (1)      /* Transmission Completed (automatically set / reset) */
#define SAM_TWI_SR_RXRDY        (1 << 1) /* A byte has been received in the TWI_RHR since the last read */
#define SAM_TWI_SR_TXRDY        (1 << 2) /* Transmit Holding Register Ready (automatically set / reset) */
#define SAM_TWI_SR_SVREAD       (1 << 3) /* Slave Read (automatically set / reset) 
                                            Indicates that a read access is performed by a Master. */
#define SAM_TWI_SR_SVACC        (1 << 4) /* Slave Access (automatically set / reset)
                                            Indicates that the address decoding sequence has matched (A Master has sent SADR). 
                                            SVACC remains high until a NACK or a STOP condition is detected. */
#define SAM_TWI_SR_GACC         (1 << 5) /* GACC: General Call Access (clear on read) */
#define SAM_TWI_SR_OVRE         (1 << 6) /* OVRE: Overrun Error (clear on read) 
                                            (1 = TWI_RHR has been loaded while RXRDY was set. 
                                             Reset by read in TWI_SR when TXCOMP is set.) */
 
#define SAM_TWI_SR_NACK         (1 << 8) /* Master:  A data byte has not been acknowledged by the slave component. Set at the same time as TXCOMP */
#define SAM_TWI_SR_ARBLST       (1 << 9) /* Arbitration lost. Another master of the TWI bus has won the multi-master arbitration. TXCOMP is set at the same time. */
#define SAM_TWI_SR_SCLWS        (1 << 10) /* Slave: Clock Wait State (automatically set / reset) - Clock is Stretched */
#define SAM_TWI_SR_EOSACC       (1 << 11) /* End of Slave Access */
#define SAM_TWI_SR_ENDRX        (1 << 12) /* Master: End of RX Buffer - Receive Counter Register has reached 0 since the last write in TWI_RCR or TWI_RNCR.*/
#define SAM_TWI_SR_ENDTX        (1 << 13) /* Master: End of TX Buffer - Transmit Counter Register has reached 0 since the last write in TWI_TCR or TWI_TNCR */
#define SAM_TWI_SR_RXBUFF       (1 << 14) /* Master: RX Buffer Full:  Both TWI_RCR and TWI_RNCR have a value of 0. */
#define SAM_TWI_SR_TXBUFF       (1 << 15) /* Master: TX Buffer Full:  Both TWI_TCR and TWI_TNCR have a value of 0. */


/* TWI_IER - Interrupt Enable Register */
#define SAM_TWI_IER_TXCOMP      (1)
#define SAM_TWI_IER_RXRDY       (1 << 1)
#define SAM_TWI_IER_TXRDY       (1 << 2)

#define SAM_TWI_IER_SVACC       (1 << 4)
#define SAM_TWI_IER_GACC        (1 << 5)
#define SAM_TWI_IER_OVRE        (1 << 6)

#define SAM_TWI_IER_NACK        (1 << 8)
#define SAM_TWI_IER_ARBLST      (1 << 9)
#define SAM_TWI_IER_SCLWS       (1 << 10)
#define SAM_TWI_IER_EOSACC      (1 << 11) 
#define SAM_TWI_IER_ENDRX       (1 << 12)
#define SAM_TWI_IER_ENDTX       (1 << 13)
#define SAM_TWI_IER_RXBUFF      (1 << 14)
#define SAM_TWI_IER_TXBUFF      (1 << 15)


/* TWI_IDR - Interrupt Disable Register */
#define SAM_TWI_IDR_TXCOMP      (1)
#define SAM_TWI_IDR_RXRDY       (1 << 1)
#define SAM_TWI_IDR_TXRDY       (1 << 2)

#define SAM_TWI_IDR_SVACC       (1 << 4)
#define SAM_TWI_IDR_GACC        (1 << 5)
#define SAM_TWI_IDR_OVRE        (1 << 6)

#define SAM_TWI_IDR_NACK        (1 << 8)
#define SAM_TWI_IDR_ARBLST      (1 << 9)
#define SAM_TWI_IDR_SCLWS       (1 << 10)
#define SAM_TWI_IDR_EOSACC      (1 << 11) 
#define SAM_TWI_IDR_ENDRX       (1 << 12)
#define SAM_TWI_IDR_ENDTX       (1 << 13)
#define SAM_TWI_IDR_RXBUFF      (1 << 14)
#define SAM_TWI_IDR_TXBUFF      (1 << 15)

/* TWI_IMR - Interrupt Mask Register */
#define SAM_TWI_IMR_TXCOMP      (1)
#define SAM_TWI_IMR_RXRDY       (1 << 1)
#define SAM_TWI_IMR_TXRDY       (1 << 2)

#define SAM_TWI_IMR_SVACC       (1 << 4)
#define SAM_TWI_IMR_GACC        (1 << 5)
#define SAM_TWI_IMR_OVRE        (1 << 6)

#define SAM_TWI_IMR_NACK        (1 << 8)
#define SAM_TWI_IMR_ARBLST      (1 << 9)
#define SAM_TWI_IMR_SCLWS       (1 << 10)
#define SAM_TWI_IMR_EOSACC      (1 << 11) 
#define SAM_TWI_IMR_ENDRX       (1 << 12)
#define SAM_TWI_IMR_ENDTX       (1 << 13)
#define SAM_TWI_IMR_RXBUFF      (1 << 14)
#define SAM_TWI_IMR_TXBUFF      (1 << 15)

/* TWI_RHR - RXDATA: Master or Slave Receive Holding Data*/
#define SAM_TWI_RHR_MASK        (0xff)

/* TWI_THR - TXDATA: Master or Slave Transmit Holding Data*/
#define SAM_TWI_THR_MASK        (0xff)



/* There are 2 TWI Vects for SAM: 
    NVIC TWI0 ID #22 - ISR(twi0)
    NVIC TWI1 ID #23 - ISR(twi1)
*/
#define SAM_TWI0_SR       ((volatile uint32_t*)0x4008c020u)
static void isr(int index)
{
    struct i2c_device_t *dev_p = &i2c_device[index]; /* NVIC TWI0 ID #22 */
    struct i2c_driver_t *drv_p = dev_p->drv_p;
    uint32_t status;

    if (drv_p == NULL) {
        return;
    }

    /* status = (dev_p->regs_p->SR & 0xffff); */
    status = *SAM_TWI0_SR;
    std_printf(FSTR("isr_twi %ld: %ld\n"), index, status);
            thrd_resume_isr(drv_p->thrd_p, 0);
    return;

    if (status & SAM_TWI_SR_TXCOMP) {
        std_printf(FSTR("SAM_TWI_SR_TXCOMP\n"));
            thrd_resume_isr(drv_p->thrd_p, 0);
    } else if (status & SAM_TWI_SR_RXRDY) {
        std_printf(FSTR("SAM_TWI_SR_RXRDY\n"));
            /* Read a byte */
            *drv_p->buf_p++ = (uint8_t)(0xff & dev_p->regs_p->RHR); /* Automatically resets RXRDY */
            drv_p->size--;

            if(drv_p->size == 1) {
                drv_p->dev_p->regs_p->CR = SAM_TWI_CR_STOP; /* Issue a stop for last byte */
            }
    } else if (status & SAM_TWI_SR_TXRDY) {
        std_printf(FSTR("SAM_TWI_SR_TXRDY\n"));
            drv_p->dev_p->regs_p->THR = *drv_p->buf_p++;
            drv_p->size--;

            if(drv_p->size == 1) {
                drv_p->dev_p->regs_p->CR = SAM_TWI_CR_STOP; /* Issue a stop for last byte */
            }
    } else {
        std_printf(FSTR("SAM_TWI_SR-Other\n"));
        thrd_resume_isr(drv_p->thrd_p, 0);
    }


    return;
    switch (status) {
        case SAM_TWI_SR_TXCOMP:
            /* All done */
            if(drv_p->size == 0) {
                thrd_resume_isr(drv_p->thrd_p, 0);
            }
            break;
        case SAM_TWI_SR_RXRDY:
            /* Read a byte */
            *drv_p->buf_p++ = (uint8_t)(0xff & dev_p->regs_p->RHR); /* Automatically resets RXRDY */
            drv_p->size--;

            if(drv_p->size == 1) {
                drv_p->dev_p->regs_p->CR = SAM_TWI_CR_STOP; /* Issue a stop for last byte */
            }

            break;
        case SAM_TWI_SR_TXRDY:
            drv_p->dev_p->regs_p->THR = *drv_p->buf_p++;
            drv_p->size--;

            if(drv_p->size == 1) {
                drv_p->dev_p->regs_p->CR = SAM_TWI_CR_STOP; /* Issue a stop for last byte */
            }
            break;
        case SAM_TWI_SR_SVREAD:
            /* Slave: Note yet handled */
            break;
        case SAM_TWI_SR_SVACC:
            /* Slave: Note yet handled */
            break;
        case SAM_TWI_SR_GACC:
            /* TODO: Unsure why this would occur */
            break;
        case SAM_TWI_SR_OVRE:
            /* Error */
            break;
        case SAM_TWI_SR_NACK:
            /* Error */
            thrd_resume_isr(drv_p->thrd_p, 0);
            break;
        case SAM_TWI_SR_ARBLST:
            /* MultiMaster: Note yet handled */
            break;
        case SAM_TWI_SR_SCLWS:
            /* Slave: Note yet handled */
            break;
        case SAM_TWI_SR_EOSACC:
            /* Slave: Note yet handled */
            break;
        case SAM_TWI_SR_ENDRX:
            break;
        case SAM_TWI_SR_ENDTX:
            break;
        case SAM_TWI_SR_RXBUFF:
            break;
        case SAM_TWI_SR_TXBUFF:
            break;
        default:
            /* Error */
            thrd_resume_isr(drv_p->thrd_p, 0);
            break;
    }
}

#define TWI_ISR(vector, index)                 \
    ISR(vector) {                               \
        isr(index);                             \
    }                                           \

#if (I2C_DEVICE_MAX >= 1)
TWI_ISR(twi0, 0)
#endif

#if (I2C_DEVICE_MAX >= 2)
TWI_ISR(twi1, 1)
#endif



int i2c_port_module_init()
{
    return (0);
}

int i2c_port_init(struct i2c_driver_t *self_p,
                  struct i2c_device_t *dev_p,
                  int baudrate,
                  int address)
{
    self_p->dev_p = dev_p;
    self_p->twbr = baudrate;
    self_p->address = address;

    return (0);
}

int i2c_port_start(struct i2c_driver_t *self_p)
{
    self_p->dev_p->drv_p = self_p;
    self_p->thrd_p = NULL;

    
    self_p->dev_p->regs_p->IDR = (0xfffffffful); /* Disable TWI Interrupts */
    pmc_peripheral_clock_enable(self_p->dev_p->id);
    nvic_enable_interrupt(self_p->dev_p->id);
    /* Enable NVIC */


    /* Set TWI Clock (CLDIV, CHDIV, CKDIV) in TWI_CWGR
        Tmck from PowerManagementController
            The Main Clock has two sources:
            * 4/8/12 MHz Fast RC Oscillator which starts very quickly and is used at startup.
                * After reset, the 4/8/12 MHz Fast RC Oscillator is enabled with the 4 MHz frequency selected and it is selected as the source of MAINCK. 
                  MAINCK is the default clock selected to start up the system. 
            * 3 to 20 MHz Crystal or Ceramic Resonator-based Oscillator which can be bypassed.

        SCL Low Period = Tlow = ((CLDIV * 2^CKDIV) + 4) * Tmck
        SCL High Period = Thigh = ((CHDIV * 2^CKDIV) + 4) * Tmck

        Tmck 4mhz = 25us period (12.5us high/low)
        @100khz => 10ms period (5ms high/low) or 40*TmckTime

        CKDIV = 3
        2^CKDIV = 8 (slow speed 100khz)
        20 = CHDIV * 2^CKDIV  = CHDIV * 8
        20/8 = ~2.5 = CHDIV 
    */
    self_p->dev_p->regs_p->CWGR = (SAM_TWI_CWGR_CKDIV_MASK & (3 << 16)) |
                                        (SAM_TWI_CWGR_CHDIV_MASK & ( 0x02 << 8) ) |
                                        (SAM_TWI_CWGR_CLDIV_MASK & 0x02);

    /* Master Enable: TWI_CR = MSEN + SVDIS */
    self_p->dev_p->regs_p->CR = SAM_TWI_CR_MSEN & SAM_TWI_CR_SVDIS;

    /* Master Mode: 
        Later on Read/Write Set:
            - Device Slave Address
            - Internal Address Size if IADR Used
                - if IADRSZ then TWI_IADR = Internal Address 
    */

    return (0);
}
        

int i2c_port_stop(struct i2c_driver_t *self_p)
{
    return (0);
}

ssize_t i2c_port_read(struct i2c_driver_t *self_p,
                      int address,
                      void *buf_p,
                      size_t size)
{
    /* After i2c Init */
    /* Master Mode: Transfer Direction Bit (Write ===> bit MREAD 1) */

    /* START */
        /* 1 Byte: TWI_CR = START | STOP */
        /* > 1  Byte: TWI_CR = START */

    /* Loop for N-1 bytes to read*/
        /* Wait for: RXRDY = 1 (Poll or ISR) */
        /* TWI_RHR = Byte to Read */

    /* For last 1 byte to read */
    /* TWI_CR = STOP */
        /* Wait for: RXRDY = 1 (Poll or ISR) */
        /* TWI_RHR = Byte to Read */

    /* Wait for: TXCOMP = 1 (Poll or ISR) */
    return (0);
}

ssize_t i2c_port_read_txn(struct i2c_driver_t *self_p,
                      int address,
                      int internalAddress,
                      int internalAddressSize,
                      void *buf_p,
                      size_t size)
{
    /* self_p->address = ((address << 1) | direction); */
    self_p->buf_p = (void *)buf_p;
    self_p->size = size;
    self_p->thrd_p = thrd_self();

    /* Master Mode: Transfer Direction Bit (Write ===> bit MREAD 1) */
    self_p->dev_p->regs_p->MMR = ((address << 16) & SAM_TWI_MMR_DADR_MASK) | SAM_TWI_MMR_MREAD | ((internalAddressSize & 0b11) << 8);
    self_p->dev_p->regs_p->IER = SAM_TWI_IER_TXCOMP | SAM_TWI_IER_RXRDY | SAM_TWI_IER_TXRDY | SAM_TWI_IER_NACK;

    /* START */
    /* This action is necessary when the TWI peripheral wants to read data from a slave. When configured in Master Mode with a
       write operation, a frame is sent as soon as the user writes a character in the Transmit Holding Register (TWI_THR). */
    if (size == 1) {
        /* 1 Byte: TWI_CR = START | STOP */
        self_p->dev_p->regs_p->CR = SAM_TWI_CR_START | SAM_TWI_CR_STOP;
    } else {
        /* > 1  Byte: TWI_CR = START */
        self_p->dev_p->regs_p->CR = SAM_TWI_CR_START;
    }

    std_printf(FSTR("i2c0 IMR: %ld\n"),  self_p->dev_p->regs_p->IMR);


    /* Loop for N-1 bytes to read*/
        /* Wait for: RXRDY = 1 (Poll or ISR) */
        /* TWI_RHR = Byte to Read */

    /* For last 1 byte to read */
    /* TWI_CR = STOP */
        /* Wait for: RXRDY = 1 (Poll or ISR) */
        /* TWI_RHR = Byte to Read */

    /* Wait for: TXCOMP = 1 (Poll or ISR) */



    /* Start the transfer by sending the START condition, and then
       wait for the transfer to complete. */
    sys_lock(); /* Take the system lock. Turns off interrupts. */
    std_printf(FSTR("i2c read txn sys Lock\n"));
    thrd_suspend_isr(NULL);
    std_printf(FSTR("i2c read txn sys unlocked\n"));
    sys_unlock(); /* Release the system lock. Turn on interrupts. */

    return (size - self_p->size); /* return delta between "original size" and "remaining size" */
}

ssize_t i2c_port_write(struct i2c_driver_t *self_p,
                       int address,
                       const void *buf_p,
                       size_t size)
{
    /* After i2c Init */
    /* Master Mode: Transfer Direction Bit (Write ===> bit MREAD 0) */
    /* Start sending data */
    /* Loop until no more bytes to send*/
        /* TWI_THR = Byte to Send */
        /* Wait for: TXRDY = 1 (Poll or ISR) */
    /* TWI_CR = STOP */
        /* Wait for: TXCOMP = 1 (Poll or ISR) */
    /* return (transfer(self_p, address, (void *)buf_p, size, I2C_WRITE)); */
    return (0);
}

int i2c_port_scan(struct i2c_driver_t *self_p,
                  int address)
{
    return (0);
}

/* Future Periperhal DMA Controller
    33.8.7 Using the Peripheral DMA Controller (PDC)
        The use of the PDC significantly reduces the CPU load.
        To assure correct implementation, respect the following programming sequences:
    33.8.7.1 Data Transmit with the PDC
        1. Initialize the transmit PDC (memory pointers, size, etc.).
        2. Configure the master mode (DADR, CKDIV, etc.).
        3. Start the transfer by setting the PDC TXTEN bit.
        4. Wait for the PDC end TX flag.
        5. Disable the PDC by setting the PDC TXDIS bit.
    33.8.7.2 Data Receive with the PDC
        1. Initialize the receive PDC (memory pointers, size - 1, etc.).
        2. Configure the master mode (DADR, CKDIV, etc.).
        3. Start the transfer by setting the PDC RXTEN bit.
        4. Wait for the PDC end RX flag.
        5. Disable the PDC by setting the PDC RXDIS bit.
*/

int i2c_port_slave_start(struct i2c_driver_t *self_p)
{
    return (0);
}

int i2c_port_slave_stop(struct i2c_driver_t *self_p)
{
    return (0);
}

ssize_t i2c_port_slave_read(struct i2c_driver_t *self_p,
                            void *buf_p,
                            size_t size)
{
    return (0);
}

ssize_t i2c_port_slave_write(struct i2c_driver_t *self_p,
                             const void *buf_p,
                             size_t size)
{
    return(0);
}

