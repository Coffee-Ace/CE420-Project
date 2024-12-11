/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "Headers/RMC_parser.h"

const uart_port_t uart_num = UART_NUM_2;
//const int BLINK_GPIO = 2;
static uint8_t s_led_state = 0;
#define BLINK_GPIO 2
QueueHandle_t sentenceBuffer;

void initUART(){
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = UART_SCLK_APB
    };
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
}

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI("Uart Test Debug", "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, 2);
}


void rxTask(void *args) {
    uint8_t data[1024];
    char nmeaSentence[160] = {0};  // Buffer to hold the RMC sentence
    int length = 0;
    int sentenceIndex = 0;
    bool capturing = false;

    while (1) {
        // Read data from UART
        length = uart_read_bytes(uart_num, data, sizeof(data) - 1, 100 / portTICK_PERIOD_MS);
        if (length > 0) {
            data[length] = '\0';  // Null-terminate the buffer

            // Parse the data
            for (int i = 0; i < length; i++) {
                if (data[i] == '$') {
                    // Check if it's the start of an RMC sentence
                    if (strncmp((char *)&data[i], "$GPRMC", 6) == 0) {
                        capturing = true;
                        sentenceIndex = 0;  // Reset the sentence buffer
                    } else if (capturing) {
                        // End of RMC sentence, send to queue
                        nmeaSentence[sentenceIndex] = '\0';  // Null-terminate the sentence
                        xQueueSend(sentenceBuffer, &nmeaSentence, 0);
                        capturing = false;
                    }
                }

                // Capture characters into the sentence buffer
                if (capturing && sentenceIndex < sizeof(nmeaSentence) - 1) {
                    nmeaSentence[sentenceIndex++] = data[i];
                }
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);  // Adjust delay as needed
    }
}

void txTask(void *args){
    int i = 0;
    int* pI = &i;

    while (1){
        uart_write_bytes(uart_num, pI, i);
        ESP_LOGI("UART Test", "Xmit: %d \r\n", i++);
        vTaskDelay(2000 /  portTICK_PERIOD_MS);
    }
    
}

void blinkTask(void *args){
    while (1){    
        ESP_LOGI("Uart Test Debug", "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        gpio_set_level(BLINK_GPIO, s_led_state);
        s_led_state = !s_led_state;
        vTaskDelay(700 /  portTICK_PERIOD_MS);
    }
}

void parseTask(void *args){
    char nmeaSentence[160];
    RMC_Struct RMC_data;
    while(1){
        if (!xQueueIsQueueEmptyFromISR(sentenceBuffer))
        {
            xQueueReceive(sentenceBuffer,&nmeaSentence,portMUX_NO_TIMEOUT);
            parsedRMC = parseRMC(&nmeaSenctence);

            ESP_LOGI("Parsed\n", "\tLatitude: %d deg %d%d\r\n", parsedRMC.);
        }
        


    }
}


void app_main(void)
{
    sentenceBuffer = xQueueCreate(10,160);
    configure_led();
    initUART();
    xTaskCreate(blinkTask, "uart_tx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(rxTask, "uart_tx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);
}
