#### Design an Embedded System to perform following tasks 
### o Interface ATmega328P with TMP36 Temperature Sensor  
### o Read the sensor using inbuilt ADC every one second 
### o Print the data on LCD  
### o Use Embedded C for programming (Do Not use Arduino Programming 
Language)
![Screenshot (407)](https://github.com/user-attachments/assets/6637842b-97d2-46f8-b9a2-17befb472394)
#### OUTPUT

![Screenshot (408)](https://github.com/user-attachments/assets/56f85b3c-c9fe-40d2-9b29-3be5838c154a)
```

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// LCD Commands
#define LCD_CLR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRY 0x06
#define LCD_ON 0x0E
#define LCD_OFF 0x08
#define LCD_MOVE 0x10
#define LCD_FUNCTION 0x28
#define LCD_SET_DDRAM 0x80

// Function Declarations
void ADC_init();
uint16_t ADC_read(uint8_t channel);
void LCD_init();
void LCD_command(uint8_t cmnd);
void LCD_data(uint8_t data);
void LCD_string(const char *str);
void LCD_gotoxy(uint8_t x, uint8_t y);
void float_to_string(float value, char *buffer);

// Main Function
int main(void) {
    // Initialize ADC and LCD
    ADC_init();
    LCD_init();

    char buffer[10];
    float temperatureC;
    uint16_t adc_value;

    while (1) {
        adc_value = ADC_read(0); // Read ADC value from channel 0
        float voltage = adc_value * (5.0 / 1023.0);
        temperatureC = (voltage - 0.5) * 100.0;

        // Print temperature on LCD
        LCD_command(LCD_CLR);
        LCD_gotoxy(0, 0);
        LCD_string("Temp: ");
        float_to_string(temperatureC, buffer);
        LCD_string(buffer);
        LCD_string(" C");

        _delay_ms(1000); // Wait for 1 second
    }
}

// Initialize ADC
void ADC_init() {
    ADMUX = (1 << REFS0); // Reference voltage set to AVcc
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC and set prescaler to 64
}

// Read ADC value from a given channel
uint16_t ADC_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // Set the ADC channel
    ADCSRA |= (1 << ADSC); // Start conversion
    while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
    return ADC;
}

// Initialize LCD
void LCD_init() {
    DDRD = 0xFF; // Set PORTD as output
    _delay_ms(20); // Wait for LCD to power up

    LCD_command(LCD_FUNCTION);
    LCD_command(LCD_ON);
    LCD_command(LCD_CLR);
    LCD_command(LCD_ENTRY);
}

// Send command to LCD
void LCD_command(uint8_t cmnd) {
    PORTD = (PORTD & 0x0F) | (cmnd & 0xF0); // Send higher nibble
    PORTD &= ~ (1 << PD2); // RS = 0 for command
    PORTD |= (1 << PD3); // Enable = 1
    _delay_us(1);
    PORTD &= ~ (1 << PD3); // Enable = 0
    _delay_us(200);

    PORTD = (PORTD & 0x0F) | (cmnd << 4); // Send lower nibble
    PORTD |= (1 << PD3); // Enable = 1
    _delay_us(1);
    PORTD &= ~ (1 << PD3); // Enable = 0
    _delay_ms(2);
}

// Send data to LCD
void LCD_data(uint8_t data) {
    PORTD = (PORTD & 0x0F) | (data & 0xF0); // Send higher nibble
    PORTD |= (1 << PD2); // RS = 1 for data
    PORTD |= (1 << PD3); // Enable = 1
    _delay_us(1);
    PORTD &= ~ (1 << PD3); // Enable = 0
    _delay_us(200);

    PORTD = (PORTD & 0x0F) | (data << 4); // Send lower nibble
    PORTD |= (1 << PD3); // Enable = 1
    _delay_us(1);
    PORTD &= ~ (1 << PD3); // Enable = 0
    _delay_ms(2);
}

// Print string to LCD
void LCD_string(const char *str) {
    while (*str) {
        LCD_data(*str++);
    }
}

// Set cursor position
void LCD_gotoxy(uint8_t x, uint8_t y) {
    uint8_t address;
    switch (y) {
        case 0: address = 0x80 + x; break;
        case 1: address = 0xC0 + x; break;
        default: address = 0x80 + x; break;
    }
    LCD_command(address);
}

// Convert float to string
void float_to_string(float value, char *buffer) {
    sprintf(buffer, "%.2f", value);
}
```


####2.  Design an Embedded System to perform following tasks 
### Interface ATmega328P with DC Motor and a Potentiometer 
#### o Read Potentiometer using ADC 
#### o Generate PWM with duty cycle proportional to Potentiometer voltage  
#### o Control the speed of motor using PWM 
#### o Change the direction of motor using a button (use interrupt)
![Screenshot (537)](https://github.com/user-attachments/assets/f6aab15d-e2b4-47ae-9809-eca1d42cd907)
#### OUTPUT
##### Potentiometer at minimum position: The motor will run at the slowest speed or may stop entirely.
##### Potentiometer at maximum position: The motor will run at maximum speed.
##### Pressing the button: Each press will toggle the direction of the motor (forward/reverse).



![Screenshot (538)](https://github.com/user-attachments/assets/94c891fb-4ab4-431e-b657-ece6404b89bc)
```
#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t motorDirection = 0;  // 0 for forward, 1 for reverse

void ADC_init(void) {
    // Set reference voltage to AVcc and select ADC0 (PC0)
    ADMUX = (1 << REFS0); 
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Enable ADC, Prescaler = 64
}

uint16_t ADC_read(void) {
    ADCSRA |= (1 << ADSC);  // Start conversion
    while (ADCSRA & (1 << ADSC));  // Wait for conversion to complete
    return ADC;
}

void PWM_init(void) {
    // Set fast PWM mode, non-inverting, prescaler 64
    TCCR1A = (1 << WGM10) | (1 << COM1A1);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    DDRD |= (1 << PD6);  // Set PD6 (OC1A) as output
}

void set_PWM(uint8_t dutyCycle) {
    OCR1A = dutyCycle;  // Set the duty cycle
}

void MotorDirection_init(void) {
    DDRB |= (1 << PB0);  // Set PB0 as output for motor direction control
    PORTB &= ~(1 << PB0);  // Initially set motor to forward
}

void Button_init(void) {
    DDRD &= ~(1 << PD2);  // Set PD2 as input (INT0 pin)
    PORTD |= (1 << PD2);  // Enable pull-up resistor on PD2
    EICRA |= (1 << ISC01);  // Trigger INT0 on falling edge
    EIMSK |= (1 << INT0);   // Enable external interrupt INT0
}

ISR(INT0_vect) {
    motorDirection = !motorDirection;  // Toggle motor direction
    if (motorDirection) {
        PORTB |= (1 << PB0);  // Set motor to reverse
    } else {
        PORTB &= ~(1 << PB0);  // Set motor to forward
    }
}

int main(void) {
    ADC_init();
    PWM_init();
    MotorDirection_init();
    Button_init();
    
    sei();  // Enable global interrupts

    uint16_t adcValue;
    uint8_t pwmValue;

    while (1) {
        adcValue = ADC_read();  // Read potentiometer value
        pwmValue = adcValue / 4;  // Map 10-bit ADC value to 8-bit PWM value
        set_PWM(pwmValue);  // Set PWM to control motor speed
    }

    return 0;
}
```
