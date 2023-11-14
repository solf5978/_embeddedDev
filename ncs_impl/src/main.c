#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

// #include "../aw210xx/aw210xx.h"
// #include "../aw210xx/aw210xx_reg.h"

#define I2C0_NODE DT_NODELABEL(led_dev)


#define BAT_REMAIN_CAP          (0x0F)
#define BAT_FULL_CHARGE_CAP     (0x10)
#define BAT_RUNTIME2EMPTY       (0x11)
#define BAT_AVGTIME2EMPTY       (0x12)
#define BAT_BAT_STATUS          (0x16)
#define BAT_MANU_INFO           (0x81)

#define I2C_ADDR                127

#define LED_CHIP_ID             (0x12)


void main(void)

{
        uint8_t ret;
        static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C0_NODE);
        if (!device_is_ready(dev_i2c.bus)) {
                printk("I2C BUS %s IS NOT READY!\n\r", dev_i2c.bus->name);
                return;
                }
        printk("Device Ready : %s\n\r", device_is_ready(dev_i2c.bus) ? "Ready" : "IDLE");
        printk("Addr Located @ : %x\n\r", dev_i2c.addr);

        // ret = i2c_write_dt(&dev_i2c, BAT_REMAIN_CAP, sizeof(BAT_REMAIN_CAP));
        // if(ret != 0){
        //     printk("FAILED TO WRITE TO I2C TARGET @ %x ", dev_i2c.addr);
        //     return;
        //     }
        uint8_t send_and_get[2] = {LED_CHIP_ID, 0x00};
        ret = i2c_write_read_dt(&dev_i2c, &send_and_get[0] , 1, &send_and_get[1], 1);
        if(ret != 0){
                printk("FAILED TO READ FROM I2C TARGET @ %x \n\r", dev_i2c.addr);
                }
        printk("RW Status : %x\n\r", ret);
        printk("Return Value : %x\n\r", send_and_get[1]);

}
