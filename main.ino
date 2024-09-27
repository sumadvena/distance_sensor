#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "LCDI2C.h"

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif
#ifndef sbi
#define sbi(reg, bit) reg |= (_BV(bit))
#endif
#ifndef cbi
#define cbi(reg, bit) reg &= ~(_BV(bit))
#endif

#define FOSC 16000000  // Clock Speed
#define UART_BAUD 9600

float measurements_arr[20];  // array to store data samples from sensor
float distance;              // store the distance from sensor
unsigned long duration_us;
uint8_t threshold = 15;
uint8_t clicks = 0;
bool thresh_change = false;

LCD_I2C lcd = LCD_I2C(0x27);

const char* num_to_str(float value, uint8_t dec_places) {
  static char buffer[8];  // Static buffer to store the converted string
  if (dec_places == 0) {
    dtostrf(value, 2, dec_places, buffer);
  } else if (value < 10) {
    dtostrf(value, 4, dec_places, buffer);
  } else {
    dtostrf(value, 5, dec_places, buffer);
  }
  return buffer;
}

void serial_print(const char* str) {
  while (*str != '\0') {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)))
      ;

    // Put data into buffer, sends the data
    UDR0 = *str;
    str++;
  }
}

uint8_t serial_parse_int() {
  uint8_t value = 0;
  char c;

  while (1) {
    // Wait for input
    while (!(UCSR0A & (1 << RXC0)))
      ;

    c = UDR0;

    if (c >= '0' && c <= '9') {
      value = value * 10 + (c - '0');
    } else if (c == '\n' || c == '\r') {
      break;
    }
  }
  return value;
}

void print_threshold() {
  lcd.goTo(0, 0);
  lcd.writeText("Threshold: ");
  lcd.writeText(num_to_str(threshold, 0));
  if (threshold < 10) {
    lcd.writeText("  cm");
  } else {
    lcd.writeText(" cm");
  }
  serial_print("Threshold has been set to ");
  serial_print(num_to_str(threshold, 0));
  serial_print(" cm\n");
}

float ultrasonic_measure() {
  // generate 10-microsecond pulse to TRIG pin
  sbi(PORTD, PD5);
  _delay_us(10);
  cbi(PORTD, PD5);
  float distance_cm = duration_us * 0.017;
  return distance_cm;
}

// button interrupt
ISR(PCINT0_vect) {
  if (thresh_change) {
    clicks++;
    return;
  }
  clicks++;
  if (clicks > 2) {
    clicks = 0;
    if (threshold < 30) {
      threshold += 5;
    } else {
      threshold = 10;
    }
    clicks++;
    thresh_change = true;
  }
}

// counting pulse duration (ECHO pin interrupt)
ISR(PCINT2_vect) {
  if (bit_is_clear(PIND, 4)) {
    duration_us = TCNT1 * .0625 * 1024;
  } else {
    TCNT1 = 0;
  }
}

int main() {
  // serial monitor
  UBRR0H = 0;
  UBRR0L = (F_CPU / (UART_BAUD * 16UL)) - 1;
  UCSR0B = _BV(RXEN0) | _BV(TXEN0);
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

  // pin interrupt and timer for pulse counting
  TCCR1A = 0;
  TCCR1B = 0b00000101;  // Prescaler of 1024
  sbi(PCICR, PCIE2);    // enable pin interrupt for PD4
  sbi(PCMSK2, PCINT20);

  lcd.backLightOn();
  print_threshold();

  // pins in output mode
  sbi(DDRD, PD5);
  sbi(DDRD, PD6);

  // button interrupt setup
  sbi(PORTB, PB0);
  sbi(PCICR, PCIE0);
  sbi(PCMSK0, PCINT0);
  sei();
  while (1) {
    // creating measurements array
    for (uint8_t sample = 0; sample < 20; sample++) {
      measurements_arr[sample] = ultrasonic_measure();
      _delay_ms(30);  // to avoid untrasonic interfering
    }

    // sorting using insertion sort
    for (uint8_t i = 1; i < 19; ++i) {
      float key = measurements_arr[i];
      uint8_t j = i - 1;
      while (j > 0 && measurements_arr[j] > key) {
        measurements_arr[j + 1] = measurements_arr[j];
        --j;
      }
      measurements_arr[j + 1] = key;
    }

    // a mean of the ten middle elements
    float sum = 0;
    for (uint8_t sample = 5; sample < 15; sample++) {
      sum += measurements_arr[sample];
    }
    
    distance = sum / 10;

    // ALARM (beeping and visual clues)
    if (distance < threshold) {
    //if (1) {
      if (distance < threshold / 2) {
        lcd.goTo(0, 0);
        lcd.writeText("--- ACHTUNG! ---");
        sbi(PORTD, PD6);
        _delay_ms(100);
        cbi(PORTD, PD6);
        print_threshold();
      } else {
        lcd.goTo(0, 0);
        lcd.writeText("--- ACHTUNG! ---");
        sbi(PORTD, PD6);
        _delay_ms(350);
        cbi(PORTD, PD6);
        print_threshold();
      }
    }

    if (thresh_change) {
      print_threshold();
      thresh_change = false;
    }

    // print the value to serial monitor
    serial_print("distance: ");
    serial_print(num_to_str(distance, 2));
    serial_print(" cm\n");
    lcd.goTo(0, 1);
    lcd.writeText(num_to_str(distance, 2));
    lcd.writeText(" cm         ");

    // changing the value of the threshold variable
    if (UCSR0A & (1 << RXC0)) {
      uint8_t incoming_data = serial_parse_int();
      if (incoming_data != 0 && incoming_data <= 30 && incoming_data >= 10) {
        // Update the variable with the new value
        threshold = incoming_data;
        print_threshold();
      } else {
        serial_print("Threshold should be in a range from 10 to 30 cm\n");
      }
    }
  }
}
