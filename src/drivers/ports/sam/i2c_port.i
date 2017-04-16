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
#define SAM_TWI_0       0x4008C000
#define SAM_TWI_1       0x40090000

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
#define SAM_TWI_MMR_IADRSZ      (0b00000000 << 8)  /* Internal Device Address Size: None */
#define SAM_TWI_MMR_IADRSZ      (0b00000001 << 8)  /* Internal Device Address Size: 1 Byte */
#define SAM_TWI_MMR_IADRSZ      (0b00000010 << 8)  /* Internal Device Address Size: 2 Byte */
#define SAM_TWI_MMR_IADRSZ      (0b00000011 << 8)  /* Internal Device Address Size: 3 Byte */

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

/* TWI_IMR - Interrupt Disable Register */
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


/**
 * Transfer data to and from a slave.
 *
 * @return Number of bytes transferred or negative error code.
 */
static ssize_t transfer(struct i2c_driver_t *self_p,
                        int address,
                        void *buf_p,
                        size_t size,
                        int direction)
{
    self_p->address = ((address << 1) | direction);
    self_p->buf_p = (void *)buf_p;
    self_p->size = size;
    self_p->thrd_p = thrd_self();

    /* Start the transfer by sending the START condition, and then
       wait for the transfer to complete. */
    sys_lock();
    TWCR = (_BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE));
    thrd_suspend_isr(NULL);
    sys_unlock();

    return (size - self_p->size);
}

ISR(TWI_vect)
{
    struct i2c_device_t *dev_p = &i2c_device[0];
    struct i2c_driver_t *drv_p = dev_p->drv_p;
    uint8_t status;

    if (drv_p == NULL) {
        return;
    }

    status = (TWSR & 0xf8);

    switch (status) {

        /* Start. */
    case I2C_M_START:
    case I2C_M_REPEATED_START:
        TWDR = drv_p->address;
        TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
        break;

        /* Acknowledgement. */
    case I2C_M_TX_SLA_W_ACK:
    case I2C_M_TX_DATA_ACK:
        if (drv_p->size > 0) {
            TWDR = *drv_p->buf_p++;
            drv_p->size--;
            TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
        } else {
            TWCR = (_BV(TWINT) | _BV(TWSTO) | _BV(TWEN));
            thrd_resume_isr(drv_p->thrd_p, 0);
        }

        break;

    case I2C_M_RX_SLA_R_ACK:
        if (drv_p->size > 1) {
            TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        } else {
            /* Last data read. */
            TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
        }

        break;

    case I2C_M_RX_DATA_ACK:
        *drv_p->buf_p++ = TWDR;
        drv_p->size--;

        if (drv_p->size > 1) {
            TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        } else {
            /* Send NACK on last data read. */
            TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
        }

        break;

        /* Negative acknowledgement. */
    case I2C_M_RX_DATA_NACK:
        *drv_p->buf_p++ = TWDR;
        drv_p->size--;
    case I2C_M_TX_SLA_W_NACK:
    case I2C_M_TX_DATA_NACK:
    case I2C_M_RX_SLA_R_NACK:
        TWCR = (_BV(TWINT) | _BV(TWSTO) | _BV(TWEN));
        thrd_resume_isr(drv_p->thrd_p, -1);
        break;

        /* Slave transmit. */
    case I2C_S_TX_DATA_ACK:
        drv_p->size--;
    case I2C_S_TX_SLA_R_ACK:
    case I2C_S_TX_ARB_LOST_SLA_R_ACK:
        if (drv_p->thrd_p == NULL) {
            drv_p->size = -1;
            break;
        }

        TWDR = *drv_p->buf_p++;

        if (drv_p->size > 1) {
            TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        } else {
            /* Last data transmission. */
            TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
        }

        break;

    case I2C_S_TX_LAST_DATA_ACK:
    case I2C_S_TX_DATA_NACK:
        drv_p->size--;
        TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        thrd_resume_isr(drv_p->thrd_p, 0);
        drv_p->thrd_p = NULL;
        break;

        /* Slave receive. */
    case I2C_S_RX_SLA_W_ACK:
    case I2C_S_RX_ARB_LOST_SLA_W_ACK:
        if (drv_p->thrd_p == NULL) {
            drv_p->size = -1;
            break;
        }

        TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        break;

    case I2C_S_RX_DATA_ACK:
    case I2C_S_RX_DATA_NACK:
        *drv_p->buf_p++ = TWDR;
        drv_p->size--;

        if (drv_p->size > 0) {
            TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        } else {
            /* Last data transmission. */
            TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
        }

        break;

    case I2C_S_RX_STOP_OR_REPEATED_START:
        TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        thrd_resume_isr(drv_p->thrd_p, 0);
        drv_p->thrd_p = NULL;
        break;

    default:
        /* ToDo: Handle error cases. */
        break;
    }
}

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

    TWSR = 0;
    TWBR = self_p->twbr;

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
    return (transfer(self_p, address, buf_p, size, I2C_READ));
}

ssize_t i2c_port_write(struct i2c_driver_t *self_p,
                       int address,
                       const void *buf_p,
                       size_t size)
{
    return (transfer(self_p, address, (void *)buf_p, size, I2C_WRITE));
}

int i2c_port_scan(struct i2c_driver_t *self_p,
                  int address)
{
    int res;

    self_p->address = ((address << 1) | I2C_WRITE);
    self_p->size = 0;
    self_p->thrd_p = thrd_self();

    /* Start the transfer by sending the START condition, and then
       wait for the transfer to complete. */
    sys_lock();
    TWCR = (_BV(TWINT) | _BV(TWSTA) | _BV(TWEN) | _BV(TWIE));
    res = thrd_suspend_isr(NULL);
    sys_unlock();

    return (res == 0);
}

int i2c_port_slave_start(struct i2c_driver_t *self_p)
{
    self_p->dev_p->drv_p = self_p;
    self_p->thrd_p = NULL;

    TWSR = 0;
    TWAR = (self_p->address << 1);
    TWCR = (_BV(TWEA) | _BV(TWEN) | _BV(TWIE));

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
    sys_lock();

    /* Read immediately if already addressed by the master. */
    if (self_p->size == -1) {
        TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
    }

    self_p->buf_p = (void *)buf_p;
    self_p->size = size;
    self_p->thrd_p = thrd_self();
    thrd_suspend_isr(NULL);
    sys_unlock();

    return (size - self_p->size);
}

ssize_t i2c_port_slave_write(struct i2c_driver_t *self_p,
                             const void *buf_p,
                             size_t size)
{
    sys_lock();
    self_p->buf_p = (void *)buf_p;

    /* Write immediately if already addressed by the master. */
    if (self_p->size == -1) {
        TWDR = *self_p->buf_p++;

        if (size > 1) {
            TWCR = (_BV(TWINT) | _BV(TWEA) | _BV(TWEN) | _BV(TWIE));
        } else {
            /* Last data transmission. */
            TWCR = (_BV(TWINT) | _BV(TWEN) | _BV(TWIE));
        }
    }

    self_p->size = size;
    self_p->thrd_p = thrd_self();
    thrd_suspend_isr(NULL);
    sys_unlock();

    return (size - self_p->size);
}
