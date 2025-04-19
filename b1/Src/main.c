#include <stdint.h>
#include <stm32f401re_gpio.h>
#include <stm32f401re_rcc.h>

#define HIGH 1
#define LOW  0
#define BTN_PRESS LOW // Nút kéo xuống GND khi nhấn

// GPIO Logic
#define GPIO_PIN_SET    1
#define GPIO_PIN_RESET  0

// LED: On-board PA5
#define LED_GPIO_PORT       GPIOA
#define LED_GPIO_PIN        GPIO_Pin_5
#define LED_PIN_NUM         5
#define LED_RCC_CLOCK       RCC_AHB1Periph_GPIOA

// Button: USER PC13
#define BUTTON_GPIO_PORT    GPIOC
#define BUTTON_GPIO_PIN     GPIO_Pin_13
#define BUTTON_PIN_NUM      13
#define BUTTON_RCC_CLOCK    RCC_AHB1Periph_GPIOC

void delay_ms(uint32_t time) {
    for (uint32_t i = 0; i < time * 1600; i++) __NOP();
}

static void LED_init(uint8_t outputType) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(LED_RCC_CLOCK, ENABLE);

    GPIO_InitStruct.GPIO_Pin = LED_GPIO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    if (outputType == GPIO_OType_PP) {
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    } else {
        GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    }

    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);
}

static void Button_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_AHB1PeriphClockCmd(BUTTON_RCC_CLOCK, ENABLE);

    GPIO_InitStruct.GPIO_Pin = BUTTON_GPIO_PIN;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // USER button dùng pull-up

    GPIO_Init(BUTTON_GPIO_PORT, &GPIO_InitStruct);
}

static void LED_Set(uint8_t state) {
    if (state == GPIO_PIN_SET)
        LED_GPIO_PORT->BSRRL = LED_GPIO_PIN;
    else
        LED_GPIO_PORT->BSRRH = LED_GPIO_PIN;
}

static uint8_t Button_Read(void) {
    return (BUTTON_GPIO_PORT->IDR & BUTTON_GPIO_PIN) ? HIGH : LOW;
}

int main(void) {
    uint8_t led_state = 0;
    uint8_t button_flag = 0;
    uint32_t delay_after_release = 2000; // Thời gian delay sau khi thả nút (2 giây)

    LED_init(GPIO_OType_PP);  // Hoặc GPIO_OType_OD nếu muốn Open-Drain
    Button_init();

    while (1) {
        // Kiểm tra nếu nút được nhấn (đã kéo xuống GND)
        if (Button_Read() == BTN_PRESS) {
            if (!button_flag) {
                button_flag = 1; // Đánh dấu là nút đã được nhấn
                // Không làm gì ngay lập tức khi nút nhấn
            }
        } else {
            if (button_flag) {
                // Nếu nút đã được nhấn và thả ra, bắt đầu đếm thời gian delay
                button_flag = 0; // Reset lại flag

                // Đợi một khoảng thời gian trước khi thay đổi trạng thái LED
                delay_ms(delay_after_release); // Đợi 2 giây

                // Sau khi delay, thay đổi trạng thái LED
                led_state = !led_state;
                LED_Set(led_state);
            }
        }
    }
}
