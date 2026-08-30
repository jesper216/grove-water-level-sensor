#include "grove_water_level.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

// The following is in millimeters.
#define GROVE_WATER_LEVEL_SENSOR_LENGTH 100.0

namespace esphome::grove_water_level
{

    static const char *const TAG = "grove_water_level.sensor";

    void GroveWaterLevelI2CComponent::dump_config()
    {
        ESP_LOGCONFIG(TAG, "I2C devices:");
        ESP_LOGCONFIG(TAG, "  Low address: 0x%02X", this->low_device_.get_i2c_address());
        ESP_LOGCONFIG(TAG, "  High address: 0x%02X", this->high_device_.get_i2c_address());
        LOG_UPDATE_INTERVAL(this);

        LOG_SENSOR("  ", "Level", this->level_sensor_);
        LOG_SENSOR("  ", "Moisture", this->moisture_sensor_);
    }

    void GroveWaterLevelI2CComponent::update()
    {
        this->read_data_();
        if (this->read_status != i2c::NO_ERROR)
        {
            this->status_set_warning();
            if (this->level_sensor_ != nullptr)
            {
                this->level_sensor_->publish_state(NAN);
            }
            if (this->moisture_sensor_ != nullptr)
            {
                this->moisture_sensor_->publish_state(NAN);
            }
            return; // FIXME make sensor unknown or unavailable
        }

        this->status_clear_warning();

        if (this->level_sensor_ != nullptr)
        {
            float level = 0.0;
        
            // Only use the final 7 sensing pads.
            // Each pad represents 5 mm of water level.
            for (int i = 13; i <= 19; i++)
            {
                if (this->read_data[i] >= 240)
                {
                    level = (i - 12) * 5.0;
                }
            }
        
            ESP_LOGD(TAG, "Got level=%.1f mm", level);
            this->level_sensor_->publish_state(level);
        }

        if (this->moisture_sensor_ != nullptr)
        {
            float moisture = 0.0;
            for (int i = 0; i < sizeof(this->read_data); i++)
            {
                // Capacitors don't always peg the int value they resolve to.
                // Thus, the contribution of the capacitor to the level is
                // capped to the capacitor max value.
                moisture = moisture + (100.0 / sizeof(this->read_data)) / this->capacitor_max_value_ * std::min(this->read_data[i], this->capacitor_max_value_);
            }
            ESP_LOGD(TAG, "Got moisture=%.1f%%", moisture);
            this->moisture_sensor_->publish_state(moisture);
        }
    }

    void GroveWaterLevelI2CComponent::read_data_()
    {
        memset(this->read_data, 0, sizeof(this->read_data));

        i2c::ErrorCode ret = i2c::NO_ERROR;

        this->low_device_.read(this->read_data, 8);
        if (ret != i2c::NO_ERROR)
        {
            ESP_LOGE(TAG, "Failed to read low device data; I2C ErrorCode: %d", ret);
            this->read_status = ret;
            return;
        }

        ret = this->high_device_.read(&this->read_data[8], 12);
        if (ret != i2c::NO_ERROR)
        {
            ESP_LOGE(TAG, "Failed to read high device data; I2C ErrorCode: %d", ret);
            this->read_status = ret;
            return;
        }

        ESP_LOGD(TAG, "Successfully read bytes from devices: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d",
                 this->read_data[0], this->read_data[1], this->read_data[2], this->read_data[3],
                 this->read_data[4], this->read_data[5], this->read_data[6], this->read_data[7],
                 this->read_data[8], this->read_data[9], this->read_data[10], this->read_data[11],
                 this->read_data[12], this->read_data[13], this->read_data[14], this->read_data[15],
                 this->read_data[16], this->read_data[17], this->read_data[18], this->read_data[19]);

        this->read_status = ret;
    }
} // namespace esphome::grove_water_level
