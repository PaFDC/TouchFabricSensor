/*
 * Compilation:
 * Arduino IDE 1.6.5-r5
 * Arduino SAM Boards 1.6.4
 */

/*
 * Notes:
 * PIOx_Handler is used in subsequent versions of the Arduino IDE. The code compiles without issue in Arduino IDE 1.6.5-r5.
 * Accessing the SysTick register allows for more precise time returns but alters the behavior of the delay function. delay() CANNOT be used in the program.
 */

/*
 * References:
 * (Arduino Due PIO/interrupts -> used in code) http://www.jaxcoder.com/Article/SinglePost?postID=1576867662
 * (Arduino Due Timer Counter) http://www.avrfreaks.net/forum/arduino-due-timer-question
 * (Arduino Due Timer Counter Tutorial) http://codetron.net/arduino-due-in-atmel-studio-using-c-led-blinking/
 * (Arduino Due Timer Counter Reference -> used in code) http://ko7m.blogspot.com/2015/01/arduino-due-timers-part-1.html
 * (Arduino Due pinout) http://www.robgray.com/temp/Due-pinout.svg
 * (Arduino Due ADC Tutorial) http://www.electrosmash.com/forum/software-pedalshield/16-configuring-adcs-and-dacs-in-arduino-due
 * 
 */

/*
 * SAM3X Functions:
 * core_cm3.h: static __INLINE uint32_t SysTick_Config(uint32_t ticks)
 */

/*
 * Sensing pin pairs (double-spacing):
 * Arduino  | SAM3X
 * ---------+----------
 * (26, 27) | (D1, D2)   \___ Pad 1
 * (30, 31) | (D9, A7)   /
 * 
 *                                                        DIGITAL PIN BLOCK
 *                                                        
 *                                                                                        |<-- Pad 1 -->|
 * 
 *          52     50     48     46     44     42     40     38     36     34     32     30     28     26     24     22
 *         PB21   PC13   PC15   PC17   PC19   PA19    PC8    PC6    PC4    PC2   PD10    PD9    PD3    PD1   PA15   PB26
 * [ GND] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [SIG1] [OUT1] [SIG0] [OUT0] [    ] [    ] [ 5V ]
 * [ GND] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [    ] [TRIG] [ IN1] [    ] [ IN0] [    ] [    ] [ 5V ]
 *         PB14   PC12   PC14   PC16   PC18   PA20    PC9    PC7    PC5    PC3    PC1    PA7    PD6    PD2    PD0   PA14
 *          53     51     49     47     45     43     41     39     37     35     33     31     29     27     25     23
 * 
 * Peripheral I/O Register A (PIOA):
 *     IN1
 * Peripheral I/O Register B (PIOB):
 *     
 * Peripheral I/O Register C (PIOC):
 *     TRIG
 * Peripheral I/O Register D (PIOD):
 *     OUT0, IN0, OUT1, SIG0, SIG1
 * 
 */

/*
 * Future Work:
 * Replace the hard-coded register names with the mappings from the g_APinDescription structure.
 * Allow compilation with later versions of the Arduino IDE (find workaround for PIOx_Handler).
 * Work on a better implementation of the SysTick handler (allow use of delay()).
 */

// PIO define statements
#define OUT0 PIO_PD1  // Digital pin 26
#define IN0 PIO_PD2   // Digital pin 27
#define SIG0 PIO_PD3  // Digital pin 28
#define OUT1 PIO_PD9  // Digital pin 30
#define IN1 PIO_PA7   // Digital pin 31
#define SIG1 PIO_PD10 // Digital pin 32
#define TRIG PIO_PC1  // Digital pin 33

// Buffer samples and number of pins
#define SAMPLES 100
#define BUFF_LEN 2

// Frame on which to send data
#define MAX_FRAME_COUNT 100

// 24 bit counter ticks
#define TICKS 0x00FFFFFF

// Clock cycles per microsecond conversion
#define MHZ 84

// Windowed moving average buffer for two pins
uint32_t buff[SAMPLES][BUFF_LEN] = { { 0, }, { 0, } };

// Sensor readings
volatile uint32_t readings[BUFF_LEN] = { 0, };

// Sum of all zeroth indexed elements within "buff"
uint32_t buff_sum[BUFF_LEN];

// Buffer sample index
uint16_t buff_sample_idx = 0;

// Frame index value
uint16_t frame_idx = 0;

// Flag that reports the program free-running mode (disabled when counter is active)
uint8_t freerun = 1;

// Counter that keeps track of the values that are sent from the program
uint32_t counter = 0;

// The state of the output square wave
volatile uint8_t state = 0;

// Separation character string between serial data values
String sep = " ";

// Serial port message
String msg = "";

/*
 * Initialize the Peripheral Input-Output (PIO) registers
 * (Arduino Due Timer Counter) http://www.avrfreaks.net/forum/arduino-due-timer-question
 * (Arduino Due Timer Counter Tutorial) http://codetron.net/arduino-due-in-atmel-studio-using-c-led-blinking/
 * (Arduino Due Timer Counter Reference -> used in code) http://ko7m.blogspot.com/2015/01/arduino-due-timers-part-1.html
 */
void InitPIO() {

    // Enable the pins in use
    PIOA->PIO_PER |= IN1;
    PIOC->PIO_PER |= TRIG;
    PIOD->PIO_PER |= OUT0 | IN0 | OUT1 | SIG0 | SIG1;

    // Set output pins, signal pins, and scope trigger as OUTPUT
    PIOC->PIO_OER |= TRIG;
    PIOD->PIO_OER |= OUT0 | OUT1 | SIG0 | SIG1;

    // Set input pins as INPUT
    PIOA->PIO_ODR |= IN1;
    PIOD->PIO_ODR |= IN0;

    // Disable pull-up resistors on all pins
    PIOA->PIO_PUDR |= IN1;
    PIOC->PIO_PUDR |= TRIG;
    PIOD->PIO_PUDR |= OUT0 | IN0 | OUT1 | SIG0 | SIG1;

    // Associate the clock with ports A and D
    PMC->PMC_PCER0 |= 1 << ID_PIOA;
    PMC->PMC_PCER0 |= 1 << ID_PIOD;

    // Enable interrupts on ports A and D
    NVIC_EnableIRQ(PIOA_IRQn);
    NVIC_EnableIRQ(PIOD_IRQn);

    // Enable the Additional Interrupt Modes Enable Register (AIMER) on all input pins
    PIOA->PIO_AIMER |= IN1;
    PIOD->PIO_AIMER |= IN0;

    // Set the interrupts to trigger on the Edge Select Register (ESR)
    PIOA->PIO_ESR |= IN1;
    PIOD->PIO_ESR |= IN0;

    // Set the interrupts to trigger on the Rising Edge/High-Level Select Register (REHLSR) 
    PIOA->PIO_REHLSR |= IN1;
    PIOD->PIO_REHLSR |= IN0;

    // Enable the interrupts
    PIOA->PIO_IER |= IN1;
    PIOD->PIO_IER |= IN0;
}

/*
 * Initialize the Timer Counter 0
 * (Arduino Due PIO/interrupts -> used in code) http://www.jaxcoder.com/Article/SinglePost?postID=1576867662
 */
void InitTC0() {

    // Associate the clock with Timer 0
    PMC->PMC_PCER0 = 1 << ID_TC0;
    
    // Disable TC clock
    TC0->TC_CHANNEL->TC_CCR = TC_CCR_CLKDIS;
    
    // Disable Interrupts
    TC0->TC_CHANNEL[0].TC_IDR = 0xfffffff;
    
    // Clear status register by reading it
    TC0->TC_CHANNEL->TC_SR;

    // Set the Clock Mode Register (CMR) to 
    TC0->TC_CHANNEL->TC_CMR = TC_CMR_CPCTRG | TC_CMR_TCCLKS_TIMER_CLOCK5;

    // Compare Value
    TC0->TC_CHANNEL[0].TC_RC = 24; // 24 = 1 ms; 59 = 2 ms
 
    // Configure and enable interrupt on RC compare
    NVIC_EnableIRQ(TC0_IRQn);
    TC0->TC_CHANNEL->TC_IER = TC_IER_CPCS;

    // Reset counter (SWTRG) and enable counter clock (CLKEN)
    TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}

/*
 * Timer Counter 0 Handler
 * Fires approximately every 1000 microseconds
 */
void TC0_Handler() {
    // Clear the Timer Counter Status Register (TC_SR) by reading it
    TC0->TC_CHANNEL[0].TC_SR;

    // For each sensing pin pair:
    for (uint8_t i = 0; i < BUFF_LEN; i++) {
        // Add the difference between the previous reading and the stored buffer reading to the buffer sum
        buff_sum[i] += readings[i] - buff[buff_sample_idx][i];
        // Replace the stored buffer reading with the previous reading
        buff[buff_sample_idx][i] = readings[i];
    }

    // Increment the buffer sample index by one and reset the index if it matches or exceeds the number of samples
    buff_sample_idx++;
    if (buff_sample_idx >= SAMPLES)
        buff_sample_idx = 0;

    // Increment the program frame and roll to 0 if it exceeds the maximum frame count
    // The frame counter effectively replaces the delay function as a way to track time
    frame_idx++;
    if (frame_idx >= MAX_FRAME_COUNT)
        frame_idx = 0;
    
    // Flip the state of the square wave
    state = !state;
    
    // Clear output on all signal pins
    PIOD->PIO_CODR |= SIG0 | SIG1;

    // Reload SysTick with the maximum value of the down counter
    SysTick_Config(TICKS);
    
    if (state) { // If the toggle state of the square wave is rising:
        // Set the interrupts to trigger on the Rising Edge/High-Level Select Register (REHLSR)
        PIOA->PIO_REHLSR |= IN1;
        PIOD->PIO_REHLSR |= IN0;

        // Pulse the trigger pin to sync the oscilloscope reading
        PIOB->PIO_SODR |= TRIG;
        PIOB->PIO_CODR |= TRIG;

        // Set the outputs to HIGH
        PIOD->PIO_SODR |= OUT0 | OUT1;        
        
    } else { // If the toggle state of the square wave is falling:
        // Set the interrupts to trigger on the Falling Edge/Low-Level Select Register (FELLSR)
        PIOA->PIO_FELLSR |= IN1;
        PIOD->PIO_FELLSR |= IN0;

        // Set the outputs to LOW
        PIOD->PIO_CODR |= OUT0 | OUT1;
    }
}

/*
 * PIOA Interrupt Handler
 * Pins:
 * 31 - PIO_PA7
 */
void PIOA_Handler() {
    // If the PIOA Interrupt Service Routine Register equals the IN1 register, save the time difference and turn on the SIG1 pin
    if ((PIOA->PIO_ISR & IN1) == IN1) {
         readings[1] = TICKS - SysTick->VAL;
         PIOD->PIO_SODR |= SIG1;
    }
}

/*
 * PIOD Interrupt Handler
 * Pins:
 * 27 - PIO_PD2
 */
void PIOD_Handler() {
    // If the PIOD Interrupt Service Routine Register equals the IN0 register, save the time difference and turn on the SIG0 pin
    if ((PIOD->PIO_ISR & IN0) == IN0) {
        readings[0] = TICKS - SysTick->VAL;
        PIOD->PIO_SODR |= SIG0;
    }
}

/*
 * Standard setup function
 */
void setup() {
    // Initialize the pins, the timer, and the serial port
    InitPIO();
    InitTC0();
    Serial.begin(115200);
}

/*
 * Standard loop function
 */
void loop() {
    // If there are bytes available in the serial buffer, parse the integer value (if valid) and return the appropriate number of samples
    if (Serial.available() > 0) {
        counter = Serial.parseInt();
        while (Serial.available()) {
            Serial.read();
        }
        Serial.println(counter);
        Serial.println(freerun);
        Serial.println(freerun || counter);
        Serial.println(Serial.available());

        // Toggle the freerun mode if the counter value equals zero
        if (counter == 0) {
            // Toggle the free run flag
            freerun = !freerun;
        } else if (counter > 0) {
            // Stop free running mode
            freerun = 0;
        }
    }

    // Return a reading on every rollover frame
    if ((frame_idx == 0) && (freerun || counter)) {
        for (uint8_t i = 0; i < BUFF_LEN-1; i++) {
            Serial.print((float)buff_sum[i] / (SAMPLES * MHZ));
            Serial.print(sep);
        }
        Serial.println((float)buff_sum[BUFF_LEN-1] / (SAMPLES * MHZ));

        // Decrement the counter on every rollover frame
        if ((freerun == 0) && (counter > 0)) {
            counter--;
        }
    }
}
