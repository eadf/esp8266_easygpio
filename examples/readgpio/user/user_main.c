/*
 *  Example read temperature and humidity from DHT22
 *  
 *  https://www.sparkfun.com/datasheets/Sensors/Temperature/DHT22.pdf
 *  https://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf
 * 
 *  For a single device, connect as follows:
 *  DHT22 1 (Vcc) to Vcc (3.3 Volts)
 *  DHT22 2 (DATA_OUT) to any of these ESP pins: GPIO0, GPIO2, GPIO4, GPIO5, GPIO12, GPIO13 or GPIO14
 *          This pin also needs a 5K pull-up resistor connected to Vcc.
 *  DHT22 3 (NC)
 *  DHT22 4 (GND) to GND
 * 
 * When the DHT22 is powered by 3.3 Volt the cables should be less than 30cm long.
 *
 */

#include <osapi.h>
#include <os_type.h>
#include "gpio.h"
#include "easygpio/easygpio.h"
#include "stdout/stdout.h"

#define SAMPLE_PERIOD 750
static os_timer_t dht22_timer;
uint8_t pinsToTest[] = {0,2,3,4,5,12,13,14,15,16};
uint8_t pinsToTestLen = 10;

static void ICACHE_FLASH_ATTR
loop(void) {
  uint8_t i=0;
  for (i=0; i<pinsToTestLen; i++) {
    os_printf("GPIO%d=>%d", pinsToTest[i], easygpio_inputGet(pinsToTest[i]));
    if(i<pinsToTestLen-1) {
      os_printf(", ");
    }
  }
  os_printf(" perireg=%x\n", READ_PERI_REG(PERIPHS_GPIO_BASEADDR));
}

void ICACHE_FLASH_ATTR
setup(void) {
  uint8_t i=0;
  for (i=0; i<pinsToTestLen; i++) {
    os_printf("Setting gpio%d as input\n", pinsToTest[i]);
    easygpio_pinMode(pinsToTest[i], EASYGPIO_PULLDOWN, EASYGPIO_INPUT);
  }
  os_printf("Starting to test the gpio pin values:\n");
  os_timer_disarm(&dht22_timer);
  os_timer_setfn(&dht22_timer, (os_timer_func_t *)loop, NULL);
  os_timer_arm(&dht22_timer, SAMPLE_PERIOD, true);
}

void ICACHE_FLASH_ATTR
user_init(void)
{
  // Make uart0 work with just the TX pin. Baud:115200,n,8,1
  // The RX pin is now free for GPIO use.
  stdout_init();

  // turn off WiFi for this console only demo
  wifi_station_set_auto_connect(false);
  wifi_station_disconnect();

  gpio_init();
  os_timer_disarm(&dht22_timer);
  os_timer_setfn(&dht22_timer, (os_timer_func_t *)setup, NULL);
  os_timer_arm(&dht22_timer, 2000, false);
}
