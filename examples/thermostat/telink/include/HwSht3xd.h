#include <stdbool.h>
#include <stdint.h>

#include "AppEventCommon.h"

#include <lib/core/CHIPError.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

class HwSht3xd
{
public:
    unsigned int hw_sht30_init();

    unsigned int hw_sht30_sample_fetch();
    unsigned int hw_sht30_temp_get(struct sensor_value * temp);
    unsigned int hw_sht30_humi_get(struct sensor_value * humi);
};
