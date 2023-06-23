#include "nrf24/projdefs.h"	
#include "utils/utils.h"

void digitalWrite_B(int pin, int level) {
	if (level == HIGH) {
		asm("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PORTB)), "I" (pin));
	} else if (level == LOW) {
		asm("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PORTB)), "I" (pin));
	}
}

int digitalRead_D(int pin) {
	if(PIND & _BV(pin)) {
		return LOW; 
	}
	return HIGH;
}

void setSleep(int b) {
	WDTCSR |= b;    //Watchdog
	// Enable watchdog timer interrupts
	WDTCSR |= (1 << WDIE);

    cli(); // No interrupts; timed sequence
    sei(); // Enable global interrupts or we never wake

	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  }