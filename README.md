# esp8266_easygpio
An easy way of setting up esp8266 GPIO pins.

I grew tired of juggling gpio pin numbers, gpio names and gpio functions. So i created this little helper library.

To setup a pin as a GPIO input you can now just do this:

```
#include "easygpio/easygpio.h"
...

uint8_t gpio_pin = 0;
EasyGPIO_PullStatus pullStatus = EASYGPIO_NOPULL;
EasyGPIO_PinMode pinMode = EASYGPIO_INPUT;
bool easygpio_pinMode(gpio_pin, pullStatus, pinMode);
```

Same thing with outputs:
```
uint8_t gpio_pin = 0;
EasyGPIO_PullStatus pullStatus = EASYGPIO_NOPULL;
EasyGPIO_PinMode pinMode = EASYGPIO_OUTPUT;
bool easygpio_pinMode(gpio_pin, pullStatus, pinMode);
```
pullStatus does not apply to output pins.

You might still need the gpio_name and func. No problem:
```
bool easygpio_getGPIONameFunc(uint8_t gpio_pin, uint32_t *gpio_name, uint8_t *gpio_func)
```

You can even setup an interrupt handler:
```
bool easygpio_attachInterrupt(uint8_t gpio_pin, EasyGPIO_PullStatus pullStatus, void (*interruptHandler)(void* arg), void *interruptArg)
```

But you will still have to do this little dance in your interrupt handler code:
```

static void interrupt_handler(void* arg) {
  uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  //clear interrupt status
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(my_interrupt_gpio_pin));
  ......
```

You can use the methods and macros defined in gpio.h (from the sdk) to access the 'standard' gpio pins (not GPIO16).
```
#include "gpio.h"
...
GPIO_OUTPUT_SET(gpio_no, bit_value) // GPIO_OUTPUT_SET(0,1) sets gpio0 to high
GPIO_DIS_OUTPUT(gpio_no) // GPIO_DIS_OUTPUT(2) turns off output on gpio2
GPIO_INPUT_GET(gpio_no) // GPIO_INPUT_GET(12) returns the input value of gpio12
```

To access *all* the pins in an uniform way you can use 
```
bool easygpio_inputGet(uint8_t gpio_pin);
void easygpio_outputSet(uint8_t gpio_pin, uint8_t value);
```
These methods does *not* change input/output status of a pin (for performance reasons). 

###Available pins

Pin number | Note
-----------|------
GPIO0 	   | this pin selects bootmode [(pull up for *normal* boot)](https://github.com/esp8266/esp8266-wiki/wiki/Boot-Process#esp-boot-modes)
GPIO1      | normally UART0 TX 
GPIO2 	   | this pin selects bootmode [(pull up for *normal* boot)](https://github.com/esp8266/esp8266-wiki/wiki/Boot-Process#esp-boot-modes)
GPIO3      | normally UART0 RX (you can use [stdout](https://github.com/eadf/esp8266_stdout) to use it as GPIO)
GPIO4      | sometimes mislabeled as GPIO5 (esp-12)
GPIO5      | sometimes mislabeled as GPIO4 (esp-12)
GPIO12     | 
GPIO13     |
GPIO14     |
GPIO15 	   | this pin selects bootmode [(pull down for *normal* boot)](https://github.com/esp8266/esp8266-wiki/wiki/Boot-Process#esp-boot-modes)
GPIO16      | requires easygpio_inputGet & easygpio_outputSet access methods (no interrupt on this pin)

All the GPIOs mentioned in ```easgle_soc.h``` are supported, but maybe we should not mess with the internal SPI pins for GPIO.

## Usage

The project has been designed for easy reuse, just create a folder with these files in it. Then point your ```MODULES``` variable in the ```Makefile``` to that folder (git subtree works great for that purpose)

See an example on how this library can be used [here](https://github.com/eadf/esp8266_digoleserial), [here](https://github.com/eadf/esp_mqtt_lcd), [here](https://github.com/eadf/esp8266_ping), [here](https://github.com/eadf/esp_mqtt_ports) - bha.. practically all of my esp projects uses it.


## Required:

esp_iot_sdk_v0.9.4_14_12_19 or higher.

I've successfully tested this with sdk v0.9.5 (linux & mac).
