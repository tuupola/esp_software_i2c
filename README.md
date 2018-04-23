#  Software I2C master for ESP-IDF (ESP32)

## What?

ESP32 has only two hardware I2C ports. Sometimes you are already using both ports for I2C slaves but still need to have an I2C master. Bitbanging to the rescue!

## Install

Install the library using by cloning it to the `components` folder of your ESP-IDF project.

``` bash
$ cd components
$ git clone git@github.com:tuupola/esp-software-i2c.git
```

## Usage

Software driver API mimics the original [ESP-IDF hardware master API](https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html#i2c-api-master-mode). Biggest difference is that there is no [command link](https://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/i2c.html#_CPPv219i2c_cmd_link_createv) and instead of queuing commands are sent directly to the I2C bus. Example below reads `SLAVE_DATA_LENGTH` bytes from `SLAVE_ADDRESS` and dumps it to console. See [tuupola/esp-examples](https://github.com/tuupola/esp-examples/tree/master/008-i2c-sw-master) for better example.

```c
static const char *TAG = "sofware_i2c";

uint8_t *data_read = (uint8_t *) malloc(SLAVE_DATA_LENGTH);

sw_i2c_master_start();
sw_i2c_master_write_byte((SLAVE_ADDRESS << 1) | I2C_MASTER_READ);
if (SLAVE_DATA_LENGTH > 1) {
    sw_i2c_master_read(data_read, SLAVE_DATA_LENGTH - 1, ACK);
}
sw_i2c_master_read_byte(data_read + SLAVE_DATA_LENGTH - 1, NAK)
sw_i2c_master_stop();

ESP_LOG_BUFFER_HEXDUMP(TAG, data_read, SLAVE_DATA_LENGTH, ESP_LOG_INFO);}
```

Same code with the original ESP-IDF hardware API would be.

```c
static const char *TAG = "hardware_i2c";

uint8_t *data_read = (uint8_t *) malloc(SLAVE_DATA_LENGTH);
i2c_cmd_handle_t cmd = i2c_cmd_link_create();

i2c_master_start(cmd);
i2c_master_write_byte(cmd, (SLAVE_ADDRESS << 1) | I2C_MASTER_READ, ACK_CHECK_ENABLE);
if (SLAVE_DATA_LENGTH > 1) {
    i2c_master_read(cmd, data_read, SLAVE_DATA_LENGTH - 1, ACK);
}
i2c_master_read_byte(cmd, data_read + SLAVE_DATA_LENGTH - 1, NAK)
i2c_master_stop(cmd);
i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
i2c_cmd_link_delete(cmd);

ESP_LOG_BUFFER_HEXDUMP(TAG, data_read, SLAVE_DATA_LENGTH, ESP_LOG_INFO);
```

## License

The MIT License (MIT). Please see [License File](LICENSE.txt) for more information.
