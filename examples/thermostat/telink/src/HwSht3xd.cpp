#include "HwSht3xd.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

unsigned int HwSht3xd::hw_sht30_init()
{
#if 0
    const struct device *const dev = DEVICE_DT_GET_ONE(sensirion_sht3xd);

    if (!device_is_ready(dev))
    {
        // LOG_INF("Device %s is not ready\n", dev->name);
        return 0;
    }
#endif

    return 1;
}

unsigned int HwSht3xd::hw_sht30_sample_fetch()
{
    const struct device * const dev = DEVICE_DT_GET_ONE(sensirion_sht3xd);

    if (!device_is_ready(dev))
    {
        LOG_INF("Device %s is not ready\n", dev->name);
        return 0;
    }

    return sensor_sample_fetch(dev);
}

unsigned int HwSht3xd::hw_sht30_temp_get(struct sensor_value * temp)
{
    const struct device * const dev = DEVICE_DT_GET_ONE(sensirion_sht3xd);

    if (!device_is_ready(dev))
    {
        LOG_INF("Device %s is not ready\n", dev->name);
        return 0;
    }

    return sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, temp);
}

unsigned int HwSht3xd::hw_sht30_humi_get(struct sensor_value * humi)
{
    const struct device * const dev = DEVICE_DT_GET_ONE(sensirion_sht3xd);

    if (!device_is_ready(dev))
    {
        LOG_INF("Device %s is not ready\n", dev->name);
        return 0;
    }

    return sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, humi);
}