

/*
 *                       Arduino Uno
 *                                      +-------+
 *                                      |       |
 *           +-----+                    |       |
 *      +----|     |--------------------|       |-------+
 *      |    |     |                    +-------+       |
 *      |    +-----+                                    |
 *      |                                               |
 *      |                                               |
 *      |                                       SCL [ ] |
 *      |                                       SDA [ ] |
 *      |                                      AREF [ ] |
 *      |                                       GND [ ] |
 *      | [ ]                                    13 [ ] |
 *      | [ ] IOREF   +-----+                    12 [ ] |  P
 *      | [ ] RESET   |  A  |                   ~11 [ ] |  O
 *      | [ ] 3.3V    |  T  |                   ~10 [ ] |  R
 *      | [ ] 5V      |  M  |                    ~9 [ ] |  T
 *      | [ ] GND     |  E  |                     8 [ ] |  B
 *      | [ ] GND     |  G  |                           |
 *      | [ ] Vin     |  A  |                     7 [ ] |
 *      |             |     |                    ~6 [ ] |
 *      | [ ] A0      |  3  |                    ~5 [ ] |  P
 *   P  | [ ] A1      |  2  |                     4 [ ] |  O
 *   O  | [ ] A2      |  8  |                    ~3 [ ] |  R
 *   R  | [ ] A3      |  P  |                     2 [ ] |  T
 *   T  | [ ] A4      |     |               TX -> 1 [ ] |  D
 *   C  | [ ] A5      +-----+               RX <- 0 [ ] |
 *      +--\                              /-------------+
 *          \----------------------------/
 *            
 */


/*
 *  Clock Divisor Table
 * 0 | 0 | 0 No clock source
 * 0 | 0 | 1 clk/1
 * 0 | 1 | 0 clk/8
 * 0 | 1 | 1 clk/64
 * 1 | 0 | 0 clk/256
 * 1 | 0 | 1 clk/1024
 * 1 | 1 | 0 
 * 1 | 1 | 1 
 * 
 */


/*
 * Cyclical Buffer Operation
 * 
 * We define a buffer with n elements numbered from 0 to n-1.
 * The buffer elements contain readings taken sequentially.
 * We define an index to track which element is being written/overwritten.
 * We also define a buffer sum that keeps track of the sum of all elements within the buffer.
 * When a reading is taken, the value is placed in an element and the write index is incremented.
 * The removed element is subtracted from the buffer sum and the reading is added.
 * When the index reaches the end of the buffer, the index wraps around to zero.
 * 
 *      buffer, index = x          (n-4) (n-2)
 *  0  1  2  3  4  5  6     x   (n-5) (n-3) (n-1)
 * [#][#][#][#][#][#][#]...[#]...[#][#][#][#][#]
 * 
 * 
 * buffer_sum = buffer_sum + reading
 * buffer_sum = buffer_sum - buffer[x]
 * buffer[x] = reading;
 * 
 * Caveats:
 * 1. Alays add to the buffer sum before subtracting, especially if the buffer sum is an unsigned quantity.
 * 2. Avoid adding and subtracting quantities in the same line to avoid order of operations conflicts and variable wrap-around.
 * 
 */

// Length of the cyclical buffer used for the low-pass filter reading
// Higher values produce a "smoother" output but slow responsiveness to changes in touch
#define CYC_BUFF_LEN 128 

#define MHz_TO_MICROS 16 // Scaling clock ticks in MHz to microseconds

uint8_t idx = 0; // Index of the cyclical buffer

uint16_t cyc_buffer[CYC_BUFF_LEN][2] = { { 0, }, { 0, } }; // Definition of the cyclical buffer
uint32_t cyc_buffer_sum[2] = { 0, }; // Definition of the buffer sum

uint16_t tau_A = 0; // Signal A rise time
uint16_t tau_B = 0; // Signal B rise time

void setup() {
    
    TCCR1A = 0; // Clear initial settings (use 16 bit timing)
    TCCR1B = (1 << WGM12) | (1 << CS10); // Clear timer on Compare and set prescaler to 1

    // Output Compare Register reset ticks
    OCR1A = 8191;
    OCR1B = 5000;
    
    TCNT1 = 0; // Reset timer counter 1
    
    TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B); // (1 << TOIE1) | Enable TIMER1 overflow interrupt and output compare interrupt
        
    DDRD &= ~((1 << DDD2) | (1 << DDD3)); // Set input on pins 2 and 3
    DDRD |= (1 << DDD5) | (1 << DDD6);    // Set output on pins 5 and 6
    
    EICRA |= (1 << ISC10) | (1 << ISC00); // Fire interrupts on a change in external input
    EIMSK |= (1 << INT1) | (1 << INT0); // Enable external interrupts
    
    sei(); // Set enable interrupts
    
    Serial.begin(115200);
    
}

void loop() {

    // Print the average times to the serial port
    delay(10);
    Serial.print((float)cyc_buffer_sum[0]/(MHz_TO_MICROS*CYC_BUFF_LEN));
    Serial.print(" ");
    Serial.println((float)cyc_buffer_sum[1]/(MHz_TO_MICROS*CYC_BUFF_LEN));
    
}

/*
 * External Interrupt INT0 (Digital pin 2)
 */
ISR(INT0_vect) {
    tau_A = TCNT1; // Record the signal A rise time
}

/*
 * External Interrupt INT1 (Digital pin 3)
 */
ISR(INT1_vect) {
    tau_B = TCNT1; // Record the signal B rise time
}

/*
 * TIMER1 Compare channel A vector
 */
ISR(TIMER1_COMPA_vect) {

    // Invert digital pins 5 and 6
    PORTD ^= (1 << DDD5);
    PORTD ^= (1 << DDD6);

    // Put the tau_A and tau_B readings into the cyclical buffer
    cyc_buffer_sum[0] += tau_A;
    cyc_buffer_sum[0] -= cyc_buffer[idx][0];
    cyc_buffer_sum[1] += tau_B;
    cyc_buffer_sum[1] -= cyc_buffer[idx][1];
    cyc_buffer[idx][0] = tau_A;
    cyc_buffer[idx][1] = tau_B;
    idx++; // Increment the buffer index and reset to zero if it reaches the end
    if (idx >= CYC_BUFF_LEN)
        idx = 0;
}

ISR(TIMER1_COMPB_vect) {
    //tau_A = TCNT1; // Record the signal A rise time
    //tau_B = TCNT1; // Record the signal B rise time
}
