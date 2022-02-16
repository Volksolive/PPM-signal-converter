/**
 * \file            ppm_converter.c
 * \brief           Convert PWM signal or Analog signal to PPM signal for ESC or servo control
 */

/*
 * Copyright (c) 2022 Olivier GAILLARD
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of pwm2ppm.
 *
 * Author:          Olivier GAILLARD
 */

/***********************************************/
/*                 USER CONFIG                 */
/***********************************************/

// Pins mapping

// Mode Switch input pin
// Can be any pin except D9 (PPM signal pin) and D13 (built-in LED)
#define MODE_PIN 2

// Potentiometer/analog signal input pin
// Can be chosen between A0 to A7
#define POT_PIN A0

// PWM input pin
// can be chosen between D2 and D7
#define PWM_PIN 5

// uncomment this line to invert the mode selected by the physical switch
//#define INVERT_MODE

// Mode pin filtering to prevents glitches or parasites from randomly chnaging the mode 
// The filter count must be between 1 (no filtering) and 50 (0.5s filtering)
// default 20 (0.2s filtering)
#define MODE_PIN_FILTER_COUNT 20

// Change these values to change the PPM min and max range
// This values are represented in microsecond => 1000 represent 1 millisecond
// Classic PPM signal range from 1ms to 2ms
#define PPM_MIN_TIMING 1000
#define PPM_MAX_TIMING 2000

// Change these values to adjust the analog signal range (potentiometer)
// Scale factor must be a float (whole number with decimal point)
#define ADC_SCALE_FACTOR 1.0
// Offset must be an integer (number with no decimal point)
#define ADC_OFFSET 0

/***********************************************/
/*             APPLICATION CONSTANT            */
/***********************************************/

// PPM pin is on D9 because the timer 1 output compare alternative function is on that pin
#define PPM_PIN   9

#define PWM_PIN_MASK 1 << PWM_PIN

#define MODE_MANU false
#define MODE_PWM true

// PPM 1ms value = ClockI/O * 1ms / clock prescaler = (16Mhz * 0.001) / 8
#define PPM_1MS_VALUE 2000
// PPM 1ms factor is PPM 1ms value divided by 1000 to accomodate for PPM timing defined in microseconds
#define PPM_1MS_FACTOR 2
#define PPM_MIN_TIMING_VALUE PPM_1MS_FACTOR * PPM_MIN_TIMING
#define PPM_MAX_TIMING_VALUE PPM_1MS_FACTOR * PPM_MAX_TIMING
#define PPM_RANGE_VALUE PPM_MAX_TIMING_VALUE - PPM_MIN_TIMING_VALUE
#define PPM_RANGE_FACTOR ((float)(PPM_MAX_TIMING - PPM_MIN_TIMING) * 0.001)

// ADC to PPM factor is the cross product of the PPM range divided by the max ADC value and multiplied by a user defined scale factor
// the max ADC value is 10bits = 1024
// 0.512 = 1024 (10 bits) / 2000 (PPM 1ms value)
#define ADC_TO_PPM_FACTOR (PPM_RANGE_FACTOR / 0.512) * (float)ADC_SCALE_FACTOR

// PWM to PPM factor is the cross product of the PPM range divided by the max PWM value
// Max PWM value is ~= 10000 => ~100ms loop / 10us ISR reading
// 5.0 = 10000 (PWM max value / 2000 (PPM 1ms value)
#define PWM_TO_PPM_FACTOR PPM_RANGE_FACTOR/5.0

/***********************************************/
/*                GLOBAL VARIABLE              */
/***********************************************/

/**
 * \brief    States enumaration              
 * 
 * Possible states of the main loop
 */
enum {
  STATE_INIT = 0,                                  /*!< Initial state when program start */
  STATE_MODE_MANU,                                 /*!< Manual mode state = Analog signal to PPM signal conversion */
  STATE_MODE_PWM,                                  /*!< PWM mode state = PWM signal to PPM signal conversion */
  STATE_MODE_MANU_TO_PWM,                          /*!< Intermediary state from manual mode to PWM mode */
  STATE_MODE_PWM_TO_MANU,                          /*!< Intermediary state from PWM mode to manual mode */
} States;

/**
 * \brief    PWM Duty Cycle Counter variable              
 * \note     This variable is an integer (16bit) and is used in an ISR
 *           The interrupt mask must be disabled before this variable is read
 *           
 * This variable is incremented every 10us only if the PWM signal is high
 * The main loop read the value every 100ms and reset the variable
 * This variable is 0 when the duty cycle is 0% and 10000 (100ms/10us) when duty cycle is 100%
 */
volatile unsigned int PwmDutyCycleCounter = 0;

/***********************************************/
/*                LOCAL FUNCTIONS              */
/***********************************************/

/**
 * \brief           Interrupt Service Routine for the timer 2 Output Compare Match A
 * \note            This function must be executed as fast as possible since it is an ISR!
 */
ISR(TIMER2_COMPA_vect) {
  if(PIND & PWM_PIN_MASK)
    PwmDutyCycleCounter++;
}

/**
 * \brief        Enable the timer 2 Output Compare Match A Interrupt
 */
void timer2_interrupt_enable(void) {
  TIFR2 = 0x02;
  TIMSK2 = 0x02;
}

/**
 * \brief        Disable the timer 2 Output Compare Match A Interrupt
 */
void timer2_interrupt_disable(void) {
  TIFR2 = 0x02;
  TIMSK2 = 0x00;
}

/**
 * \brief        Get the state of the mode pin
 * \return       The state of the mode pin
 */
bool mode_get_pin_state(void) {
#ifdef REVERSE_MODE_PIN
  return !digitalRead(MODE_PIN);
#else
  return digitalRead(MODE_PIN);
#endif
}

/**
 * \brief        Initialize the mode (PWM or MANU)
 * \param[out]   Mode: Output variable used to save the mode
 * \return       True when initialized otherwise false
 */
bool mode_init(bool* const Mode) {
  static unsigned char CountManu, CountPWM;
  bool ModePinState;

  // Read the physical switch input pin state
  ModePinState = mode_get_pin_state();

  // Depending on the pin state one of two counter is incremented if the other one is 0
  // Otherwise both counters are reset
  // This prevents glitches or parasites to randomly define the mode when the program starts
  if(ModePinState && (CountManu == 0))
    CountPWM++;
  else if(!ModePinState && (CountPWM == 0))
    CountManu++;
  else {
    CountManu = CountPWM = 0;
  }

  // If one of the two counter reaches the 'filter' count the mode is defined and true is returned
  if(CountManu >= MODE_PIN_FILTER_COUNT) {
    *Mode = MODE_MANU;
    return true;
  } else if(CountPWM >= MODE_PIN_FILTER_COUNT) {
    *Mode = MODE_PWM;
    return true;
  }

  return false;
}

/**
 * \brief        check is the mode selected by the physical switch has changed
 * \param[out]   Mode: Output variable used to save the mode
 */
void mode_check(bool* const Mode) {
  static unsigned char Count = 0;
  bool ModePinState;

  // Read the physical switch input pin state
  ModePinState = mode_get_pin_state();

  // A counter is incremented if the pin state is different than the actual mode
  // Otherwise the counter is reset
  // This prevents glitches or parasites to unexpectedly change the mode
  if(ModePinState && (*Mode==MODE_MANU)) 
    Count++;
  else if(!ModePinState && (*Mode==MODE_PWM))
    Count++;
  else
    Count = 0;

  // If the counter reaches the "filter" count the mode is officialy updated
  if(Count >= MODE_PIN_FILTER_COUNT)
  {
      if(*Mode == MODE_MANU)
        *Mode = MODE_PWM;
      else
        *Mode = MODE_MANU;
      Count = 0;
  }
}

/**
 * \brief        Convert the ADC signal value to a PPM value
 * \param[in]    AdcValue: ADC signal value 10bits = 0 to 1024 
 * \return       Converted PPM value
 */
unsigned int adc_to_ppm(unsigned int AdcValue) {
  float Temp = AdcValue + ADC_OFFSET;

  if(Temp < 0.0)
    return 0;

  return Temp * ADC_TO_PPM_FACTOR;
}

/**
 * \brief        Convert the PWM signal duty cycle value to a PPM value
 * \param[in]    PwmDCValue: PWM signal duty cycle value 0 to 10000
 * \return       Converted PPM value
 */
unsigned int pwm_to_ppm(unsigned int PwmDCValue) {
  return (float)PwmDCValue * (float)PWM_TO_PPM_FACTOR;
}

/**
 * \brief        Set the PPM value 
 * \param[in]    PpmValue: PPM value
 */
void ppm_set_value(unsigned int PpmValue) {
  if(PpmValue > PPM_RANGE_VALUE)
    OCR1A = PPM_MAX_TIMING_VALUE;
  else
    OCR1A = PPM_MIN_TIMING_VALUE + PpmValue;
}

/**
 * \brief        Indicate when the main loop can be executed
 * \return       True if the delay time has elapsed, otherwise false
 */
bool main_loop_timer_delay(void) {
  // Check if the timer output compare match A flag is set = delay time has elapsed  
  if(TIFR0 & 0x02) {
    // Reset the timer output compare match A flag
    TIFR0 = 0x02;
    return true;
  }
  
  return false;
}

/**
 * \brief        Setup the necessary I/O and peripherals
 */
void setup(void) {

  // Setup pins direction
  pinMode(MODE_PIN, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);
  pinMode(PWM_PIN, INPUT);
  pinMode(PPM_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Setup Timer 0 for main loop timing
  // Timer set in CTC mode with an internal clock with 1024 prescaler
  // Period set to approx. 10ms => (16Mhz * 10ms) / 1024 ~= 156
  TCCR0A = 0x02;
  TCCR0B = 0x05;
  OCR0A = 156;
  TIMSK0 = 0x00;
  TIFR0 = 0x07;
  
  // Setup Timer 1 for PPM Generation (16 bits timer = more precision)
  // Timer set in Fast PWM mode with an internal clock with 8 prescaler
  // Period set to 20ms => (16Mhz * 20ms) / 8 = 40000
  // Duty Cycle set to the minimum PPM signal value
  TCCR1A = 0x82;
  TCCR1B = 0x1A;
  OCR1A = PPM_MIN_TIMING_VALUE;
  ICR1 = 40000;
  TIMSK1 = 0x00;

  // Setup timer 2 for PWM duty cycle measurement
  // Timer set in CTC mode with an internal clock with no prescaler
  // Period set to 10us => (16Mhz * 10us) = 160
  TCCR2A = 0x02;
  TCCR2B = 0x01;
  OCR2A = 160;
  TIMSK2 = 0x00;
  TIFR2 = 0x02;

  // Enable global interrupts
  interrupts();
}

/**
 * \brief        Main program loop
 * 
 * The main loop determine the selected mode, manual or PWM 
 * and convert the selected input signal (analog or PWM) to an output PPM signal
 */
void loop(void) {
  bool Mode = MODE_MANU;
  unsigned char State = STATE_INIT;
  unsigned char PwmUpdateCounter;

  // Initialization of the mode before the main loop is executed
  if(mode_init(&Mode)) { 
    while(true) {
      // Execute the main loop every 10ms
      if(main_loop_timer_delay()) {
        // Check if the mode has changed
        mode_check(&Mode);
        digitalWrite(LED_BUILTIN, Mode);

        // State Machine
        switch(State)
        {
          case STATE_INIT:
            if(Mode == MODE_MANU)
              State = STATE_MODE_MANU;
            else
              State = STATE_MODE_MANU_TO_PWM;
            break;
          
          case STATE_MODE_MANU:
            // Read the ADC value, convert it to PPM and set the value
            ppm_set_value(adc_to_ppm(analogRead(POT_PIN)));
            if(Mode == MODE_PWM)
              State = STATE_MODE_MANU_TO_PWM;
            break;
      
          case STATE_MODE_PWM:
            // Execute the PWM to PPM convertion only every 100ms
            if(PwmUpdateCounter == 9) {
              // Deactivate the timer 2 IT to avoid data corruption
              timer2_interrupt_disable();
              // Convert the PWM Duty Cycle to PPM and set the value
              ppm_set_value(pwm_to_ppm(PwmDutyCycleCounter));
              // Reset the PWM Duty Cycle value for the next loop
              PwmDutyCycleCounter = 0;
              // Reactivate the timer 2 IT
              timer2_interrupt_enable();
              PwmUpdateCounter = 0;
            }
            else
              PwmUpdateCounter++;
              
            if(Mode == MODE_MANU)
                State = STATE_MODE_PWM_TO_MANU;
            break;
      
          case STATE_MODE_MANU_TO_PWM:
            // Enable the timer 2 IT to read the PWM Duty Cycle value
            timer2_interrupt_enable();
            PwmUpdateCounter = 0;
            State = STATE_MODE_PWM;
            break;
      
          case STATE_MODE_PWM_TO_MANU:
            // Deactivate the timer 2 IT when mode is manual
            timer2_interrupt_disable();
            State = STATE_MODE_MANU;
            break;
    
          default:
            break;
        }
      }
    }
  }
}
