#### Design an Embedded System to perform following tasks 
### o Interface ATmega328P with TMP36 Temperature Sensor  
### o Read the sensor using inbuilt ADC every one second 
### o Print the data on LCD  
### o Use Embedded C for programming (Do Not use Arduino Programming 
Language)
![Screenshot (407)](https://github.com/user-attachments/assets/6637842b-97d2-46f8-b9a2-17befb472394)
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
