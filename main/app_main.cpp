/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_lvgl_port.h"
#include "driver/gpio.h"
#include "lvgl.h"
#include "i2c_oled.h"
#include "cbspI2C.h"
#include "cBMP280.h"
#include "cSMP3011.h"
//Biblioteca adicionada por mim. Serve para contar quanto tempo o esp está ligado
#include "esp_timer.h"

static const char *TAG = "example";

/*
        HARDWARE DEFINITIONS
*/
#define I2C_BUS_PORT                  0
#define EXAMPLE_PIN_NUM_SDA           GPIO_NUM_5
#define EXAMPLE_PIN_NUM_SCL           GPIO_NUM_4
#define EXAMPLE_PIN_LED               GPIO_NUM_16               

/*
        Components
*/
cbspI2C I2CChannel1;
cBMP280 BMP280;
cSMP3011 SMP3011;

/*
        TASKS
*/
// Prototypes
void TaskBlink(void *parameter); // Task para fazer o led do esp piscar
void TaskDisplay(void *parameter); // Task para adicionar os dados no displau
void TaskSensors(void *parameter); //  Task para atualizar os dados dos sensores
void TaskPressureConversion(void *parameter); // Task para converter os dados dos sensores para psi e atm

// Handlers
TaskHandle_t taskBlinkHandle   = nullptr;
TaskHandle_t taskDisplayHandle = nullptr;
TaskHandle_t taskSensorsHandle = nullptr;
TaskHandle_t taskPressureConversionHandle = nullptr;

// Variáveis globais para armazenar pressão e temperatura
float SMP3011Pressure = 0.0;    // Para pressão do SMP3011 em Pa
float BMP280Pressure = 0.0;     // Para pressão do BMP280 em Pa
float SMP3011Temperature = 0.0;  // Para temperatura do SMP3011
float BMP280Temperature = 0.0;   // Para temperatura do BMP280
float SMP3011PressurePsi = 0.0;  // Para pressão do SMP3011 em Psi
float SMP3011PressureBar = 0.0;  // Para pressão do SMP3011 em Bar
float BMP280pressureAtm = 0.0;   // Para pressão do BMP280 em Atm
unsigned long Seconds = 0; // Guarda o tempo que o esp está ligado em segundos

//Função Principal
extern "C"
void app_main()
{
    // Setup pin for LED
    gpio_set_direction(EXAMPLE_PIN_LED, GPIO_MODE_OUTPUT);

    // Setup I2C 0 for display
    ESP_LOGI(TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        }
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    // Setup I2C 1 for sensors
    I2CChannel1.init(I2C_NUM_1, GPIO_NUM_33, GPIO_NUM_32);
    I2CChannel1.openAsMaster(100000);

    // Initialize sensors
    BMP280.init(I2CChannel1);
    SMP3011.init(I2CChannel1);

    // Initialize display
    i2c_oled_start(i2c_bus);
    
    // Create tasks
    xTaskCreate(TaskBlink, "Blink", 1024, nullptr, tskIDLE_PRIORITY, &taskBlinkHandle);  
    xTaskCreate(TaskSensors, "Sensors", 2048, nullptr, tskIDLE_PRIORITY, &taskSensorsHandle);    
    xTaskCreate(TaskDisplay, "Display", 4096, nullptr, tskIDLE_PRIORITY, &taskDisplayHandle);
    xTaskCreate(TaskPressureConversion, "PressureConversion", 2048, nullptr, tskIDLE_PRIORITY, &taskPressureConversionHandle);
}

void TaskDisplay(void *parameter)
{
    lvgl_port_lock(0);
    lv_obj_t *scr = lv_disp_get_scr_act(nullptr);

    lv_obj_t *labelPressure = lv_label_create(scr); // Cria a label de pressão do smp3011
    lv_label_set_long_mode(labelPressure, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(labelPressure, "");
    lv_obj_set_width(labelPressure, 128);
    lv_obj_align(labelPressure, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *labelTemperature = lv_label_create(scr); // Cria a label de temperatura do smp3011
    lv_label_set_long_mode(labelTemperature, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(labelTemperature, "");
    lv_obj_set_width(labelTemperature, 128);
    lv_obj_align(labelTemperature, LV_ALIGN_TOP_MID, 0, 16);

    lv_obj_t *labelPressureAtm = lv_label_create(scr); // Cria a label de pressão do bmp280
    lv_label_set_long_mode(labelPressureAtm, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(labelPressureAtm, "");
    lv_obj_set_width(labelPressureAtm, 128);
    lv_obj_align(labelPressureAtm, LV_ALIGN_TOP_MID, 0, 0);

    lv_obj_t *labelTemperatureAtm = lv_label_create(scr); // Cria a label para a temperatura do bmp 280
    lv_label_set_long_mode(labelTemperatureAtm, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(labelTemperatureAtm, "");
    lv_obj_set_width(labelTemperatureAtm, 128);
    lv_obj_align(labelTemperatureAtm, LV_ALIGN_TOP_MID, 0, 40);    

    lv_obj_t *labelWarning = lv_label_create(scr); // Cria a label para avisos
    lv_label_set_long_mode(labelWarning, LV_LABEL_LONG_SCROLL_CIRCULAR); // Definindo o modo de rolagem
    lv_label_set_text(labelWarning, ""); // Inicializa com texto vazio
    lv_obj_set_width(labelWarning, 128); // Define a largura
    lv_obj_align(labelWarning, LV_ALIGN_CENTER, 0, -10); // Alinha na parte inferior da tela

    lvgl_port_unlock();

    while (1)
    {      
 
        Seconds = esp_timer_get_time() / 1e6; // Transforma o tempo que o esp tá ligado para segundos
        if (Seconds <= 6) // Mostra a temperatura e a pressão inicial do bmp 280 na tela por 6 segundos
        {
            lv_label_set_text_fmt(labelPressureAtm, "%6.2fAtm", BMP280pressureAtm); // Exibe pressão do BMP280 em atm
            lv_label_set_text_fmt(labelTemperatureAtm, " %6.1fC  ", BMP280Temperature); // Exibe temperatura do BMP280  
        }
        else // Lendo os valores de pressão do smp3011 e vendo se estão com valores ideias com base em valores genéricos de pneu
        {
            if (SMP3011PressurePsi < 30) // se estiver com menos de 30 psi (baixo) ele exibe o aviso de baixa pressão. Também apaga todas as labels, mostrando apenas a de aviso
            {
                lv_label_set_text_fmt(labelWarning, "!! LOW PRESSURE !!");
                lv_label_set_text(labelPressure, "");
                lv_label_set_text(labelTemperature, "");
                lv_label_set_text(labelPressureAtm, "");
                lv_label_set_text(labelTemperatureAtm, "");
            }
            else if (SMP3011PressurePsi> 35) // se estiver com mais de 35 psi (alta) ele exibe o aviso de alta pressão. Também apaga todas as labels, mostrando apenas a de aviso
            {
                lv_label_set_text_fmt(labelWarning, "!! HIGH PRESSURE !!");
                lv_label_set_text(labelPressure, "");
                lv_label_set_text(labelTemperature, "");
                lv_label_set_text(labelPressureAtm, "");
                lv_label_set_text(labelTemperatureAtm, "");
            }
            else // se estiver dentro dos valores ideias, ele mostra a leitura do sensor com duas casas após a virgula
            {
                lv_obj_set_style_text_font(labelPressure, &lv_font_montserrat_14,0); // Aplica um estilo na label (diminui a fonte)
                lv_label_set_text_fmt(labelPressure, "        %6.2f Psi", SMP3011PressurePsi); // Exibe pressão em psi
                lv_label_set_text(labelWarning, "");
                lv_label_set_text(labelTemperature, "");
                lv_label_set_text(labelPressureAtm, "");
                lv_label_set_text(labelTemperatureAtm, "");
            }

        }
        
        lvgl_port_unlock();

        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }    
}

void TaskBlink(void *parameter) // Pisca o led do esp 
{
    while (1)
    {
        gpio_set_level(EXAMPLE_PIN_LED, 1); 
        vTaskDelay(500 / portTICK_PERIOD_MS);
        gpio_set_level(EXAMPLE_PIN_LED, 0); 
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void TaskSensors(void *parameter) // Lê os sensores
{
    while (1)
    {
        BMP280.poll();
        SMP3011.poll();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void TaskPressureConversion(void *parameter) // Converte os valores dos sensores para psi e atm
 { 
    while (1) {
        // Acesse os dados dos sensores
        SMP3011Pressure =  SMP3011.getPressure(); //Obter a pressão do SMP3011 em Pa. Para fins de teste, 240000 =  pressão ideal, 200000 = pressão baixa, 250000 = pressão alta   
        BMP280Pressure = BMP280.getPressure(); // Obter a pressão do BMP280 em Pa
        SMP3011Temperature = SMP3011.getTemperature(); // Obter a temperatura do SMP3011
        BMP280Temperature = BMP280.getTemperature(); // Obter a temperatura do BMP280

        // Converte as pressões para as unidades desejadas
        SMP3011PressurePsi = SMP3011Pressure * 0.0001450377; // Converte Pa para psi
        SMP3011PressureBar = SMP3011Pressure / 100000.0;;  // Para pressão do SMP3011 em bar
        BMP280pressureAtm = BMP280Pressure / 101325.0;   // Converte Pa para atm

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Aguarda 1 segundo antes de repetir
    }
}
