//ATMEGA328 3M Ultrasonic Read/Write Pulse Sensor

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include "lcd.h"

#define F_CPU 1000000UL
#define TRIGGER_PIN PD0  // Ultrasonic trigger pin
#define ECHO_PIN PD2     // Ultrasonic echo pin (INT0)
#define PULSES_PER_CMx100 (F_CPU * 100 / 68600)
#define MAX_RANGE_CM 300
#define MAX_RANGE_COUNT ((MAX_RANGE_CM * PULSES_PER_CMx100) / 100)

volatile int pulse = 1;  // Pulse width storage
volatile int measuring = 0;  // Measurement status flag

void init_ultrasonic();
void start_measurement();

int main(void)
{
    lcd_init();
    init_ultrasonic();
    sei();  // Enable global interrupts

    while (1)
    {
        if (!measuring)  // Only trigger if not currently measuring
        {
            start_measurement();
        }
        else if (pulse != 0)  // New measurement available
        {
            int distance_cm = pulse * 100 / PULSES_PER_CMx100;

            lcd_clear();
            lcd_gotoxy(1, 1);
            lcd_print("Distance Sensor");
            lcd_gotoxy(1, 2);
            lcd_print("Distance=");
            
            char buffer[16];
            itoa(distance_cm, buffer, 10);
            lcd_print(buffer);
            lcd_print(" cm");

            measuring = 0;  // Ready for new measurement
        }

        // Handle timeout (if echo is too long)
        if (TCNT1 > MAX_RANGE_COUNT)
        {
            pulse = 1;  // Force a new pulse trigger
            measuring = 0;
        }
    }
}

// Initialize ultrasonic sensor
void init_ultrasonic()
{
    DDRD |= (1 << TRIGGER_PIN);  // Set trigger pin as output
    DDRD &= ~(1 << ECHO_PIN);    // Set echo pin as input

    GICR |= (1 << INT0);   // Enable INT0 interrupt
    MCUCR |= (1 << ISC00); // Trigger on any logic change

    TCCR1B = 0;  // Stop Timer1 initially
}

// Start a new ultrasonic measurement
void start_measurement()
{
    measuring = 1;
    pulse = 0;
    
    // Send a 10µs pulse
    PORTD |= (1 << TRIGGER_PIN);
    _delay_us(10);
    PORTD &= ~(1 << TRIGGER_PIN);

    // Reset timer
    TCNT1 = 0;
    
    // Enable interrupt and clear pending flag
    GIFR |= (1 << INTF0);
    GICR |= (1 << INT0);
}

// Interrupt Service Routine for echo measurement
ISR(INT0_vect)
{
    if (measuring)
    {
        if (pulse == 0)  // Start timing on rising edge
        {
            TCCR1B |= (1 << CS10);  // Start Timer1 (no prescaler)
        }
        else  // Stop timing on falling edge
        {
            TCCR1B = 0;  // Stop Timer1
            pulse = TCNT1;  // Store measured pulse width
            GICR &= ~(1 << INT0);  // Disable INT0 to prevent noise
        }
    }
}
