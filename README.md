# esp8266_easygpio
An easy way of setting up esp8266 GPIO pins.

I grew tired of juggling gpio pin numbers, gpio names and gpio functions. So i created this little helper library.

To setup a pin as a GPIO input you can now just do this:

```
#include "easygpio/easygpio.h"
...

uint8_t gpio_pin = 0;
bool pullUp = false;
bool pullDown = false;
easygpio_setupAsInput(uint8_t gpio_pin, bool pullUp, bool pullDown);
```

Same thing with outputs:
```
bool easygpio_setupAsOutput(uint8_t gpio_pin)
```

You can set your pullup and pulldown registers on your input pins:

```
bool easygpio_setupPulls(uint32_t gpio_name, bool pullUp, bool pullDown)
```

To use the above method you need the gpio_name. No problem:
```
bool easygpio_getGpioNameFunc(uint8_t gpio_pin, uint32_t *gpio_name, uint8_t *gpio_func)
```

You can even setup an interrupt handler:
```
bool easygpio_setupInterrupt(uint8_t gpio_pin, bool pullUp, bool pullDown, void (*interruptHandler)(void))
```

But you will still have to do this little dance in your interrupt handler code:
```

static void interrupt_handler(void) {
  uint32_t gpio_status = GPIO_REG_READ(GPIO_STATUS_ADDRESS);
  //clear interrupt status
  GPIO_REG_WRITE(GPIO_STATUS_W1TC_ADDRESS, gpio_status & BIT(my_interrupt_gpio_pin));
  ......
```

See an example [here](https://github.com/eadf/esp8266_digoleserial)
