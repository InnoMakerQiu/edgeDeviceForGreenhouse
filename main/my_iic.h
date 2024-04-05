#ifndef MY_IIC_H
#define MY_IIC_H

#include "esp_err.h"
#define I2C_SCL_IO                  4    /* gpio number for I2C master clock */
#define I2C_SDA_IO                  5    /* gpio number for I2C master data */

#define I2C_MASTER_NUM          I2C_NUM_0 /* I2C port number for master dev */

#define WRITE_BIT                           I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                            I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                        0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                       0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                             0x0              /*!< I2C ack value */
#define NACK_VAL                            0x1              /*!< I2C nack value */
#define LAST_NACK_VAL                       0x2              /*!< I2C last_nack value */

esp_err_t i2c_master_init();

#endif