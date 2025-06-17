/**
 *******************************************************************************************************
 * @file            : main.c
 * @brief           : Main program body for a "Simon Says" game on an STM32 microcontroller.
 *******************************************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 *******************************************************************************************************
 */

/* Includes ------------------------------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

/* Global Variables ----------------------------------------------------------------------------------*/
static volatile uint32_t g_timing_delay;
static uint8_t game_size = 3;

/* Type Definitions ----------------------------------------------------------------------------------*/
typedef struct {
    GPIO_TypeDef *port;
    uint8_t pin;
} Pin_Info_t;

typedef struct {
    Pin_Info_t *pins; // A pointer to a list of pins
    size_t count;
} Pin_Collection_t;

typedef struct {
    GPIO_TypeDef **ports; // A pointer to a list of ports
    size_t count;
} Port_Collection_t;

/* Definitions ---------------------------------------------------------------------------------------*/
#define RED_LED_PORT        GPIOA
#define RED_LED_PIN         7
#define YELLOW_LED_PORT     GPIOA
#define YELLOW_LED_PIN      6
#define GREEN_LED_PORT      GPIOB
#define GREEN_LED_PIN       9
#define BLUE_LED_PORT       GPIOB
#define BLUE_LED_PIN        8

#define RED_BUTTON_PORT     GPIOA
#define RED_BUTTON_PIN      0
#define YELLOW_BUTTON_PORT  GPIOA
#define YELLOW_BUTTON_PIN   1
#define GREEN_BUTTON_PORT   GPIOC
#define GREEN_BUTTON_PIN    1
#define BLUE_BUTTON_PORT    GPIOC
#define BLUE_BUTTON_PIN     0

#define STATUS_LED_RED_PORT     GPIOB
#define STATUS_LED_RED_PIN      4
#define STATUS_LED_GREEN_PORT   GPIOB
#define STATUS_LED_GREEN_PIN    5

#define GPIO_MODE_O             0x1UL
#define GPIO_MODE_I             0x0UL
#define GPIO_OTYPE_PUSH_PULL    0x0UL
#define GPIO_OSPEED_LOW         0x0UL
#define GPIO_PUPDR_PULL_DOWN    0x2UL

#define MAX_PATTERN_SIZE 10
#define PLAYER_SUCCESS 0
#define PLAYER_FAILURE 1

#define START_SEQUENCE_COEF 350
#define SIMON_LED_HOLD_TIME 500
#define SIMON_LED_POST_DELAY 500
#define STATUS_LED_COEF 200
#define STATUS_LED_BLINK_COUNT 5

#define DEBOUNCING_DELAY 50 // milliseconds

/* Function Prototypes -------------------------------------------------------------------------------*/
static void usart2_init(void);
static void game_display_start_sequence(Pin_Collection_t *led_group);
static Pin_Collection_t game_generate_pattern(const Pin_Collection_t *led_group);
static int game_get_player_input(const Pin_Collection_t *led_group,
        const Pin_Collection_t *button_group, const Pin_Collection_t *pattern);
static void game_handle_status(const int status,
        const Pin_Collection_t *status_led_group);
static void led_pin_init(const Pin_Info_t led_info);
static void led_enable(const Pin_Info_t led_info);
static void led_disable(const Pin_Info_t led_info);
static void led_blink(Pin_Info_t led, int duration, int wait);
static void button_pin_init(const Pin_Info_t button_info);
static int button_is_active(const Pin_Info_t button_info);
static void mcu_clock_config(void);
static void gpio_init(const Port_Collection_t *port_group,
        const Pin_Collection_t *led_group, const Pin_Collection_t *button_group,
        const Pin_Collection_t *status_led_group);
static void gpio_enable_port_clock(const GPIO_TypeDef *port);
static void systick_init(void);
static void systick_delay_ms(uint32_t ms);
void SysTick_Handler(void);
static uint32_t get_random_seed_from_adc(void);
int _write(int file, char *ptr, int len);


/*======================================================================================================
 * Main
 ======================================================================================================*/

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* MCU Configuration--------------------------------------------------------*/

    // Configure Ports
    GPIO_TypeDef *ports[] = { GPIOA, GPIOB, GPIOC };
    Port_Collection_t port_group = { .ports = ports, .count = sizeof(ports) / sizeof(ports[0]) };

    /*** IMPORTANT: The index of an LED should be identical to the index of its associated button in their respective arrays ***/

    // Configure LEDSs
    Pin_Info_t red_led = { RED_LED_PORT, RED_LED_PIN };
    Pin_Info_t yellow_led = { YELLOW_LED_PORT, YELLOW_LED_PIN };
    Pin_Info_t green_led = { GREEN_LED_PORT, GREEN_LED_PIN };
    Pin_Info_t blue_led = { BLUE_LED_PORT, BLUE_LED_PIN };
    Pin_Info_t leds[] = { red_led, yellow_led, green_led, blue_led };
    Pin_Collection_t led_group = { .pins = leds, .count = sizeof(leds)
            / sizeof(leds[0]) };

    // Configure Buttons
    Pin_Info_t red_button = { RED_BUTTON_PORT, RED_BUTTON_PIN };
    Pin_Info_t yellow_button = { YELLOW_BUTTON_PORT, YELLOW_BUTTON_PIN };
    Pin_Info_t green_button = { GREEN_BUTTON_PORT, GREEN_BUTTON_PIN };
    Pin_Info_t blue_button = { BLUE_BUTTON_PORT, BLUE_BUTTON_PIN };
    Pin_Info_t buttons[] = { red_button, yellow_button, green_button,
            blue_button };
    Pin_Collection_t button_group = { .pins = buttons, .count = sizeof(buttons)
            / sizeof(buttons[0]) };

    // Configure Status LEDs
    Pin_Info_t status_red_led = { STATUS_LED_RED_PORT, STATUS_LED_RED_PIN };
    Pin_Info_t status_green_led =
            { STATUS_LED_GREEN_PORT, STATUS_LED_GREEN_PIN };
    Pin_Info_t status_leds[] = { status_red_led, status_green_led };
    Pin_Collection_t status_led_group = { .pins = status_leds, .count =
            sizeof(status_leds) / sizeof(status_leds[0]) };

    /* Configure the system clock */
    mcu_clock_config();

    /* Initialize our new SysTick timer */
    systick_init();

    /* Initialize all configured peripherals */
    gpio_init(&port_group, &led_group, &button_group, &status_led_group);

    /* Seed random number generator for Simon's randomization */
    srand(get_random_seed_from_adc());

    /* Initalize usart protocol for debug printing */
    usart2_init();

    /* The Game ---------------------------------------------------------------- */

    // Flash the lights before beginning the game
    game_display_start_sequence(&led_group);

    printf("++++++++++++++++++++++++ Begin ++++++++++++++++++++++++\r\n");

    /* Begin the game */
    while (1) {

        // Simon picks the pattern to be played
        Pin_Collection_t pattern = game_generate_pattern(&led_group);

        // Get and validate user input
        int status = game_get_player_input(&led_group, &button_group, &pattern);

        // Check if the user passed or failed
        game_handle_status(status, &status_led_group);
    }
}

/*======================================================================================================
 * Gameplay Functions
 ======================================================================================================*/

/**
 * @brief  Displays a flashing sequence on the LEDs to indicate the start of the game.
 * @param  led_group A pointer to the collection of game LEDs.
 * @retval None
 */
static void game_display_start_sequence(Pin_Collection_t *led_group) {
    for (int j = 0; j < 3; j++) {

        for (int i = 0; i < led_group->count; i++) {
            led_enable(led_group->pins[i]);
        }
        systick_delay_ms(START_SEQUENCE_COEF);

        for (int i = 0; i < led_group->count; i++) {
            led_disable(led_group->pins[i]);
        }
        systick_delay_ms(START_SEQUENCE_COEF);
    }
}

/**
 * @brief  Generates and displays the next random LED pattern for the player to follow.
 * @param  led_group A pointer to the collection of game LEDs from which to generate the pattern.
 * @retval Pin_Collection_t A struct containing the sequence of pins for the current round.
 * @note   The length of the pattern increases by one each time this function is called.
 */
static Pin_Collection_t game_generate_pattern(const Pin_Collection_t *led_group) {

    printf("---------------------- Round %d ----------------------\r\n",
            game_size - 2);

    // Declare variables to store simons pattern
    static Pin_Collection_t pattern;
    static Pin_Info_t pattern_pins[MAX_PATTERN_SIZE];

    // Randomly select LEDs to light up
    for (int i = 0; i < game_size; i++) {
        int index = rand() % led_group->count;
        pattern_pins[i] = led_group->pins[index];

        // Blink the led
        led_blink(pattern_pins[i], SIMON_LED_HOLD_TIME, SIMON_LED_POST_DELAY);
    }

    pattern.pins = pattern_pins;
    pattern.count = game_size;

    game_size++;   // game size increases every round until max is completed

    return pattern;
}

/**
 * @brief  Waits for and captures the player's button inputs for the current round.
 * @param  led_group A pointer to the collection of game LEDs.
 * @param  button_group A pointer to the collection of game buttons.
 * @param  pattern A pointer to the pattern generated by Simon that the player must match.
 * @return int Returns PLAYER_SUCCESS if the input matches the pattern, otherwise PLAYER_FAILURE.
 * @note   This function includes a debounce delay for button presses.
 */
static int game_get_player_input(const Pin_Collection_t *led_group,
        const Pin_Collection_t *button_group, const Pin_Collection_t *pattern) {

    int status = PLAYER_SUCCESS;
    int s = 0;   // Simon's pattern iterator

    // Retrieve input until player has finished their turn
    while (s < pattern->count) {

        int active_index = -1;// index of the active led and button in their corresponding collections

        // Wait for player to pick a button
        while (active_index < 0) {
            for (int i = 0; i < button_group->count; i++) {
                if (button_is_active(button_group->pins[i])) {
                    systick_delay_ms(DEBOUNCING_DELAY);   // we add this here to prevent accidental double reads when the button voltage bounces on initial press
                    if (button_is_active(button_group->pins[i])) {
                        active_index = i;
                        break;
                    }
                }
            }
        }

        // Enable the LED while the player presses it
        led_enable(led_group->pins[active_index]);
        while (button_is_active(button_group->pins[active_index])) {
            // wait until player releases button
        }
        led_disable(led_group->pins[active_index]);

        // Make sure the button press was correct
        Pin_Info_t simon_pin = pattern->pins[s];
        Pin_Info_t player_pin = led_group->pins[active_index];

        printf(
                "[Simon pin #: %d Player pin #: %d] [Simon port addr: %p Player port addr: %p\r\n]",
                simon_pin.pin, player_pin.pin, simon_pin.port, player_pin.port);

        if (simon_pin.port != player_pin.port
                || simon_pin.pin != player_pin.pin) {
            status = PLAYER_FAILURE;
            break;
        }

        s++;
    }

    return status;
}

/**
 * @brief  Handles the game status by blinking the appropriate status LED.
 * @param  status The player's status (PLAYER_SUCCESS or PLAYER_FAILURE).
 * @param  status_led_group A pointer to the collection of status LEDs.
 * @note   Resets the system if the player fails or completes the maximum pattern size.
 * @retval None
 */
static void game_handle_status(const int status,
        const Pin_Collection_t *status_led_group) {
    if (status == PLAYER_SUCCESS) {
        // Success blink
        for (int j = 0; j < STATUS_LED_BLINK_COUNT; j++) {
            led_blink(status_led_group->pins[1], STATUS_LED_COEF,
            STATUS_LED_COEF);   // pin 1 is the green led
        }
    } else {
        // Failure blink
        for (int j = 0; j < STATUS_LED_BLINK_COUNT; j++) {
            led_blink(status_led_group->pins[0], STATUS_LED_COEF,
            STATUS_LED_COEF);   // pin 0 is the red led
        }
    }

    if (status != PLAYER_SUCCESS || game_size == MAX_PATTERN_SIZE) {
        NVIC_SystemReset();
    }
}


/*======================================================================================================
 * MCU & Peripheral Configuration
 ======================================================================================================*/

/**
 * @brief  Configures the system clock. Uses the HSI16 to configure and enable PLL to 80MHz.
 * @retval None
 */
static void mcu_clock_config(void) {

    // Enable HSI for enabling the PLL
    RCC->CR |= RCC_CR_HSION;   // enable clock

    while (!(RCC->CR & RCC_CR_HSIRDY)) {
        // wait until clock is ready
    }

    // Set power voltage scaling to range 1 (high-performance mode)
    PWR->CR1 &= ~PWR_CR1_VOS_Msk;
    PWR->CR1 |= PWR_CR1_VOS_0;

    while (PWR->SR2 & PWR_SR2_VOSF) {
        // wait until target voltage is reached (cleared bits when ready)
    }

    // Increase latency to prevent cpu clock from outpacing flash memory access time
    FLASH->ACR &= ~FLASH_ACR_LATENCY_Msk;       // clear bits
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS;// set latency to 4 wait states (5 CPU cycles)

    // Turn PLL off before configurement
    RCC->CR &= ~RCC_CR_PLLON_Msk;

    while (RCC->CR & RCC_CR_PLLRDY) {
        // wait until PLL is shut off before continuing
    }

    // Configure the PLL (goal: HSI16 /M*N /R = 16MHz /1 *10 /2 = 80MHz)
    // PLLM = 1 (000b), PLLN = 10 (0001010b), PLLR = 2 (00b)
    // HSI16 is selected with value 0x2 for PLLSRC.
    RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC_Msk | RCC_PLLCFGR_PLLM_Msk
            | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLR_Msk);

    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI;   // set source as HSI16

    RCC->PLLCFGR |= (0x0UL << RCC_PLLCFGR_PLLM_Pos) |     // PLLM = 1
            (0xAUL << RCC_PLLCFGR_PLLN_Pos) |     // PLLN = 10
            (0x0UL << RCC_PLLCFGR_PLLR_Pos);      // PLLR = 2

    // Enable the PLL PLLCLK output (for SYSCLK)
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;

    while (!(RCC->CR & RCC_CR_PLLRDY)) {
        // wait until PLL is ready
    }

    // Set system clock to PLL
    RCC->CFGR &= ~RCC_CFGR_SW_Msk;
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL) {
        // wait until PLL is used as system clock source
    }

    // Configure AHB and APB prescalers
    RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk); // no division
}

/**
 * @brief  Initializes all GPIO ports and pins required for the game.
 * @param  port_group A pointer to the collection of GPIO ports to enable.
 * @param  led_group A pointer to the collection of game LEDs to initialize.
 * @param  button_group A pointer to the collection of game buttons to initialize.
 * @param  status_led_group A pointer to the collection of status LEDs to initialize.
 * @retval None
 */
static void gpio_init(const Port_Collection_t *port_group,
        const Pin_Collection_t *led_group, const Pin_Collection_t *button_group,
        const Pin_Collection_t *status_led_group) {
    // Enable GPIO ports
    for (int i = 0; i < port_group->count; i++) {
        gpio_enable_port_clock(port_group->ports[i]);
    }

    // Enable LED Pins
    for (int i = 0; i < led_group->count; i++) {
        led_pin_init(led_group->pins[i]);
    }

    // Enable Button Pins
    for (int i = d; i < button_group->count; i++) {
        button_pin_init(button_group->pins[i]);
    }

    // Enable status LED Pins
    for (int i = 0; i < status_led_group->count; i++) {
        led_pin_init(status_led_group->pins[i]);
    }
}

/**
 * @brief  Enables the clock for a specific GPIO port.
 * @param  port Pointer to the GPIO port definition (e.g., GPIOA, GPIOB).
 * @retval None
 */
static void gpio_enable_port_clock(const GPIO_TypeDef *port) {
    if (port == GPIOA) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    } else if (port == GPIOB) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    } else if (port == GPIOC) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    } else if (port == GPIOD) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
    } else if (port == GPIOE) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
    } else if (port == GPIOF) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN;
    } else if (port == GPIOG) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOGEN;
    } else if (port == GPIOH) {
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;
    }
    // Ignores invalid port value
}

/**
 * @brief Initializes the SysTick timer to generate an interrupt every 1ms.
 * @retval None
 */
static void systick_init(void) {
    // (80,000,000 cycles/sec) / (1000 ticks/sec) = 80,000 cycles/tick
    // The -1 is because the timer counts down to 0, which is one cycle.
    SysTick->LOAD = 80000 - 1;

    // Reset the current value
    SysTick->VAL = 0;

    // Enable the SysTick timer and its interrupt, using the main processor clock
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
            | SysTick_CTRL_ENABLE_Msk;
}


/*======================================================================================================
 * Driver Functions
 ======================================================================================================*/

/**
 * @brief  Initializes a led for the Simon Says game to an output push-pull low speed pin.
 * @param  led_info A struct containing the port and pin number for the LED.
 * @retval None
 */
static void led_pin_init(const Pin_Info_t led_info) {

    uint32_t mode_pos = led_info.pin * 2;         // mode is 2 bits long
    uint32_t mode_msk = (0x3UL << mode_pos);

    uint32_t otype_pos = led_info.pin;          // type is 1 bit long
    uint32_t otype_msk = (0x1UL << otype_pos);

    uint32_t ospeed_pos = led_info.pin * 2;   // speed is 2 bits long
    uint32_t ospeed_msk = (0x3UL << ospeed_pos);

    // Set mode to output
    led_info.port->MODER &= ~mode_msk;
    led_info.port->MODER |= (GPIO_MODE_O << mode_pos);

    // Set output type to push-pull
    led_info.port->OTYPER &= ~otype_msk;
    led_info.port->OTYPER |= (GPIO_OTYPE_PUSH_PULL << otype_pos);

    // Set speed to low
    led_info.port->OSPEEDR &= ~ospeed_msk;
    led_info.port->OSPEEDR |= (GPIO_OSPEED_LOW << ospeed_pos);
}

/**
 * @brief  Enables an LED output pin by setting the corresponding bit in the BSRR.
 * @param  led_info A struct containing the port and pin number for the LED.
 * @retval None
 */
static void led_enable(const Pin_Info_t led_info) {
    uint8_t pin_pos = led_info.pin;
    led_info.port->BSRR = (0x1UL << pin_pos);
}

/**
 * @brief  Disables an LED output pin by setting the corresponding bit in the BSRR.
 * @param  led_info A struct containing the port and pin number for the LED.
 * @retval None
 */
static void led_disable(const Pin_Info_t led_info) {
    uint8_t pin_pos = led_info.pin + 16;   // plus 16 to skip bit set registers
    led_info.port->BSRR = (0x1UL << pin_pos);
}

/**
 * @brief  Blinks an LED for a specified duration.
 * @param  led The Pin_Info_t struct for the LED to blink.
 * @param  duration The time in milliseconds to keep the LED on.
 * @param  wait The time in milliseconds to wait after turning the LED off.
 * @retval None
 */
static void led_blink(Pin_Info_t led, int duration, int wait) {
    led_enable(led);
    systick_delay_ms(duration);   // Hold light on for (duration) milliseconds
    led_disable(led);
    systick_delay_ms(wait);       // Sit (wait) milliseconds before continuing
}

/**
 * @brief  Initializes a button for the Simon Says game to an input pull-down pin.
 * @param  button_info A struct containing the port and pin number for the button.
 * @retval None
 */
static void button_pin_init(const Pin_Info_t button_info) {

    uint32_t mode_pos = button_info.pin * 2;         // each mode is 2 bits long
    uint32_t mode_msk = (0x3UL << mode_pos);

    uint32_t pull_resistor_pos = button_info.pin * 2;// each speed is 2 bits long
    uint32_t pull_resistor_msk = (0x3UL << pull_resistor_pos);

    // Set mode to input
    button_info.port->MODER &= ~mode_msk;
    button_info.port->MODER |= (GPIO_MODE_I << mode_pos);

    // Set resistor type to pull down
    button_info.port->PUPDR &= ~pull_resistor_msk;
    button_info.port->PUPDR |= (GPIO_PUPDR_PULL_DOWN << pull_resistor_pos);
}

/**
 * @brief  Checks if a button is currently being pressed by reading the Input Data Register (IDR).
 * @param  button_info A struct containing the port and pin number for the button.
 * @return int 1 if the button is pressed (high), 0 if not pressed (low).
 */
static int button_is_active(const Pin_Info_t button_info) {
    uint8_t position = button_info.pin;
    return (button_info.port->IDR & (0x1UL << position));
}


/*======================================================================================================
 * System & Debug Functions
 ======================================================================================================*/

/**
 * @brief Waits for a specified number of milliseconds using a global delay variable.
 * @param ms The number of milliseconds to wait.
 * @retval None
 */
static void systick_delay_ms(uint32_t ms) {
    g_timing_delay = ms;
    while (g_timing_delay > 0) {
        // Wait for g_timing_delay to hit 0, decremented by SysTick_Handler
    }
}

/**
 * @brief This function handles the SysTick interrupt, decrementing a global timing delay variable.
 * @note  This function is called every 1ms after systick_init().
 * @retval None
 */
void SysTick_Handler(void) {
    if (g_timing_delay > 0) {
        g_timing_delay--;
    }
}

/**
 * @brief  Initializes USART2 for debug printing.
 * @note   Configures PA2 as TX and PA3 as RX in alternate function mode.
 * Sets baud rate to 115200.
 * @retval None
 */
static void usart2_init(void) {
    // Enable clock access to USART2
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    // Set PA2 and PA3 to Alternate Function mode
    GPIOA->MODER &= ~(GPIO_MODER_MODE2_Msk | GPIO_MODER_MODE3_Msk);
    GPIOA->MODER |= (0x2UL << GPIO_MODER_MODE2_Pos);
    GPIOA->MODER |= (0x2UL << GPIO_MODER_MODE3_Pos);

    // Set the alternate function type to AF7 (USART2_TX/RX)
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3);
    GPIOA->AFR[0] |= (0x7UL << GPIO_AFRL_AFSEL2_Pos);
    GPIOA->AFR[0] |= (0x7UL << GPIO_AFRL_AFSEL3_Pos);

    // Configure USART2
    // Formula: SystemClock / BaudRate = 80,000,000 / 115200 = 694.44
    // Store integer part of that result into the BRR
    USART2->BRR = 694;

    // Enable the Transmitter (TE) and the USART itself (UE)
    USART2->CR1 |= (USART_CR1_TE | USART_CR1_UE);
}

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  file  File descriptor (not used).
 * @param  ptr   Pointer to the data to be written.
 * @param  len   Length of the data to be written.
 * @retval int   The number of characters written.
 */
int _write(int file, char *ptr, int len) {
    int i;
    for (i = 0; i < len; i++) {
        // Send each character to the USART2 peripheral
        while (!(USART2->ISR & USART_ISR_TXE)) {
            // Wait for transmit buffer to be empty
        }
        USART2->TDR = (uint8_t) ptr[i]; // Send the character
    }
    return len;
}

/**
 * @brief  Gets a pseudo-random seed by reading the noise from an unconnected ADC pin.
 * @note   This function temporarily enables ADC1 on pin PB1 to read a floating value.
 * The ADC and its clocks are disabled before returning to save power.
 * @retval A 32-bit integer based on the ADC reading to be used as a random seed.
 */
static uint32_t get_random_seed_from_adc(void) {
    uint32_t seed = 0;

    // Enable clocks for ADC and GPIOB
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN | RCC_AHB2ENR_GPIOBEN;

    RCC->CCIPR &= ~RCC_CCIPR_ADCSEL; // Clear the selection bits first
    RCC->CCIPR |= RCC_CCIPR_ADCSEL; // Select SYSCLK as ADC clock (sets bits to 11)

    // Configure PB1 as an analog pin
    GPIOB->MODER |= (0x3UL << GPIO_MODER_MODE1_Pos); // Set PB1 to Analog mode (11)
    GPIOB->ASCR |= GPIO_ASCR_ASC1;      // Connect analog switch to ADC for PB1

    // Wake up ADC and enable its voltage regulator
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |= ADC_CR_ADVREGEN;
    systick_delay_ms(1); // Wait for regulator to stabilize

    // Set ADC conversion sequence. Pin PB1 is on ADC Channel 16.
    ADC1->SQR1 = (16 << ADC_SQR1_SQ1_Pos);

    // Enable the ADC and wait for it to be ready
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
        ;

    // Start the conversion and wait for it to complete
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC))
        ;

    // Read the result
    seed = ADC1->DR;

    // Disable ADC to save power
    ADC1->CR |= ADC_CR_ADDIS;
    while (ADC1->CR & ADC_CR_ADDIS)
        ;
    ADC1->CR |= ADC_CR_DEEPPWD;
    RCC->AHB2ENR &= ~(RCC_AHB2ENR_ADCEN);

    return seed;
}


