#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define I2C0_NODE DT_NODELABEL(i2c0)


#define BAT_REMAIN_CAP          (0x0F)
#define BAT_FULL_CHARGE_CAP     (0x10)
#define BAT_RUNTIME2EMPTY       (0x11)
#define BAT_AVGTIME2EMPTY       (0x12)
#define BAT_BAT_STATUS          (0x16)
#define BAT_MANU_INFO           (0x81)

#define I2C_ADDR                127

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

        for (uint8_t i = 4; i <= 0x77; i++) {
		// struct i2c_msg msgs[1];
		// uint8_t dst = 1;

		// /* Send the address to read from */
		// msgs[0].buf = &dst;
		// msgs[0].len = 1U;
		// msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
		
		// error = i2c_transfer(dev_i2c, &msgs[0], 1, i);
        uint8_t batRC = 0;
        ret = i2c_write_read_dt(&dev_i2c, &BAT_FULL_CHARGE_CAP, 1, &batRC, 1);
		if (ret == 0) {
			printk("0x%2x FOUND\n", i);
		}
		else {
			printk("error %d \n", ret);
		}
		
		
	    }
	    printk("Scanning done\n");

        // // ret = i2c_write_dt(&dev_i2c, BAT_REMAIN_CAP, sizeof(BAT_REMAIN_CAP));
        // // if(ret != 0){
        // //     printk("FAILED TO WRITE TO I2C TARGET @ %x ", dev_i2c.addr);
        // //     return;
        // //     }
        // uint8_t batRC = 0;
        // ret = i2c_write_read_dt(&dev_i2c, BAT_FULL_CHARGE_CAP, 1, &batRC, 1);
        // if(ret != 0){
        //         printk("FAILED TO READ FROM I2C TARGET @ %x \n\r", dev_i2c.addr);
        //         }
        // printk("RW Status : %x\n\r", ret);
        // printk("Return Value : %x\n\r", batRC);

}
