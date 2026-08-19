/***************************************************************************************************************************
 * FileName:     main.c
 * Processor:    Tensilica Xtensa LX6 160 MHz
 * Board:        ESP32-C3 SuperMini
 * Company:      TecNM /IT Chihuahua
 * Description:  Celerino line follower with PID controller in a state machine
 * Authors:      Ana Cardona, Emiliano Pérez, Luis Anchondo
 * Updated:      05/2026
 * Created on:   2 mar. 2026
 * updated:      18/08/2026
 **************************************************************************************************************************/
/**************************************************************************************************************************
 * * Copyright (C) 2026 by Ana Cardona, Emiliano Pérez, Luis Anchondo - TecNM /IT Chihuahua
 *
 * Se permite la redistribucion, modificacion o uso de este software en formato fuente o binario
 * siempre que los archivos mantengan estos derechos de autor.
 * Los usuarios pueden modificar esto y usarlo para aprender sobre el campo de software embebido.
 * Ana Cardona, Emiliano Pérez, Luis Anchondo y el TecNM /IT Chihuahua no son responsables del mal uso de este material.
 **************************************************************************************************************************/

/*
Hardware: CELERINO CON ESP32C3 SUPER MINI 
BARRA DE SENSORES: 16 sensores IR conectados a un MUX 16 canales (74HC4067) que a su vez se conecta al pin ADC del ESP32.
    PINES:
        *S0 - GPIO 7
        *S1 - GPIO 10
        *S2 - GPIO 2
        *S3 - GPIO 3
        *ADC - GPIO 4
MOTORES: 2 motores DC controlados por un medios puentes h
    PINES:
        *Motor Izquierdo IN1 - GPIO 20
        *Motor Izquierdo IN2 - GPIO 21
        *Motor Derecho IN1 - GPIO 6
        *Motor Derecho IN2 - GPIO 5
VENTILADOR:
    PINES:
        *PWM Ventilador - GPIO 8
LED_CARRO:
    PINES:
        *LED BLANCO - GPIO 9
RECEPTOR_IR:
    PINES:
        *Salida Receptor IR - GPIO 0
*/


// =====================================================
// Probablemente puede que sea o no sea esta la versión 
// con el ajuste de PID para el doble sentido de giro
// de los motores, así que de antemano pido disculpas si 
// no es la correcta.
// =====================================================


#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_adc/adc_oneshot.h"
#include "hal/adc_types.h"

#include "esp_timer.h"
#include "esp_rom_sys.h"

// =====================================================
// DEFINICIONES
// =====================================================

#define SENSOR_DELAY_US  10
#define NUM_SENSORS      16
#define SETPOINT         7500.0f

// ================= MUX =================

#define MUX_S0  3
#define MUX_S1  2
#define MUX_S2 10
#define MUX_S3  7

// ================ MOTORES ===============

#define MOTOR_L_IN1 20
#define MOTOR_L_IN2 21
#define MOTOR_R_IN1  6
#define MOTOR_R_IN2  5

// ================= EXTRAS ===============

#define FAN_PWM_PIN   8
#define LED_WHITE_PIN 9
#define IR_RX_PIN     1

// =============== VELOCIDADES ============

#define BASE_SPEED          340
#define SEARCH_SPEED        430
#define CAL_SPEED           160
#define MAX_SPEED           1050

// =================== IR =================

#define IR_CMD_CALIBRATE 0x45
#define IR_CMD_START     0x46
#define IR_CMD_STOP      0x47

// ================== ESC =================

#define FAN_ESC_MIN_US    900
#define FAN_ESC_IDLE_US  1100
#define FAN_ESC_RUN_US   1400
#define FAN_ARM_TIME_MS  5000

// =====================================================
// VARIABLES
// =====================================================

adc_oneshot_unit_handle_t adc_handle;

int sensors[NUM_SENSORS];
int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];

float last_pos = SETPOINT;

float P,I,D;
float previous_error = 0;

float Kp = 0.128f;
float Ki = 0.0f;
float Kd = 6.5f;

volatile uint32_t fan_pulse_us = FAN_ESC_MIN_US;

// =====================================================
// ESTADOS
// =====================================================

typedef enum{
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_RUNNING,
    STATE_STOPPED
}robot_state_t;

volatile robot_state_t robot_state = STATE_IDLE;

// Contador para los pasos de calibración (0 = negro, 1 = blanco)
volatile uint8_t calibration_step = 0; 

volatile bool start_sequence_pending = false;
volatile int64_t start_time_us = 0;

// =====================================================
// IR
// =====================================================

volatile uint32_t ir_data = 0;
volatile uint8_t ir_bits = 0;
volatile bool ir_ready = false;
volatile int64_t ir_last_us = 0;

// =====================================================
// PROTOTIPOS
// =====================================================

void init_peripherals(void);
void init_motors(void);

void set_mux(uint8_t ch);
void read_sensors(int *readings);

bool get_line_pos(int *readings,float *out_position);

void set_motor_speeds(int left,int right);

float calculate_pid(float error);

void calibrate(void);

void process_ir(void);

void fan_task(void *arg);
void fan_set_speed_us(uint32_t us);
void fan_test_startup(void);
void fan_enable(bool enable);

// =====================================================
// ISR IR
// =====================================================

static void IRAM_ATTR ir_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time();
    int64_t duration = now - ir_last_us;

    ir_last_us = now;

    if(duration > 13000 && duration < 14000)
    {
        ir_data = 0;
        ir_bits = 0;
        ir_ready = false;
    }
    else if(duration > 1000 && duration < 1300)
    {
        if(ir_bits < 32)
            ir_bits++;
    }
    else if(duration > 2000 && duration < 2500)
    {
        if(ir_bits < 32)
        {
            ir_data |= (1UL << ir_bits);
            ir_bits++;
        }
    }
    else
    {
        ir_bits = 0;
        ir_data = 0;
    }

    if(ir_bits == 32)
    {
        ir_ready = true;
        ir_bits = 0;
    }
}

// =====================================================
// APP MAIN
// =====================================================

void app_main(void)
{
    init_peripherals();

    xTaskCreatePinnedToCore(
        fan_task,
        "fan_task",
        2048,
        NULL,
        4,
        NULL,
        0
    );

    vTaskDelay(pdMS_TO_TICKS(500));

    while(1)
    {
        process_ir();

        // =========================================
        // ARRANQUE NO BLOQUEANTE
        // =========================================

        if(start_sequence_pending)
        {
            if((esp_timer_get_time() - start_time_us) >= 2000000)
            {
                start_sequence_pending = false;
                robot_state = STATE_RUNNING;
            }
        }

        switch(robot_state)
        {
            case STATE_IDLE:
            case STATE_STOPPED:

                set_motor_speeds(0,0);

                vTaskDelay(pdMS_TO_TICKS(10));

                break;

            // =====================================

            case STATE_CALIBRATING:

                calibrate();

                robot_state = STATE_IDLE;
                
                // Apagar el LED para indicar que terminaron los 5 segundos de este paso
                gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

                break;

            // =====================================

            case STATE_RUNNING:
            {
                read_sensors(sensors);

                float position;

                bool line_found =
                    get_line_pos(sensors,&position);

                if(line_found)
                {
                    float error = position - SETPOINT;

                    float pid =
                        calculate_pid(error);

                    int left =
                        BASE_SPEED + (int)pid;

                    int right =
                        BASE_SPEED - (int)pid;

                    if(left > MAX_SPEED) left = MAX_SPEED;
                    if(right > MAX_SPEED) right = MAX_SPEED;

                    if(left < -MAX_SPEED) left = -MAX_SPEED;
                    if(right < -MAX_SPEED) right = -MAX_SPEED;

                    set_motor_speeds(left,right);
                }
                else
                {
                    I = 0;

                    if(previous_error > 0)
                        set_motor_speeds(SEARCH_SPEED,0);
                    else
                        set_motor_speeds(0,SEARCH_SPEED);
                }

                esp_rom_delay_us(SENSOR_DELAY_US);

                vTaskDelay(1);

                break;
            }
        }
    }
}

// =====================================================
// PROCESS IR
// =====================================================

void process_ir(void)
{
    if(!ir_ready)
        return;

    uint8_t command = (ir_data >> 16) & 0xFF;

    ir_ready = false;

    switch(command)
    {
        // =====================================
        // CALIBRAR
        // =====================================

        case IR_CMD_CALIBRATE:

            robot_state = STATE_CALIBRATING;

            gpio_set_level(
                (gpio_num_t)LED_WHITE_PIN,
                1
            );

            break;

        // =====================================
        // START
        // =====================================

        case IR_CMD_START:
            
            calibration_step = 0; // Reiniciar estado del contador por seguridad
            
            previous_error = 0;
            I = 0;

            gpio_set_level(
                (gpio_num_t)LED_WHITE_PIN,
                0
            );

            fan_enable(true);

            fan_set_speed_us(FAN_ESC_RUN_US);

            start_sequence_pending = true;

            start_time_us = esp_timer_get_time();

            break;

        // =====================================
        // STOP
        // =====================================

        case IR_CMD_STOP:

            calibration_step = 0; // Reiniciar estado del contador
            
            start_sequence_pending = false;

            robot_state = STATE_STOPPED;

            set_motor_speeds(0,0);

            fan_enable(false);

            gpio_set_level(
                (gpio_num_t)LED_WHITE_PIN,
                0
            );

            break;

        default:
            break;
    }
}

// =====================================================
// INIT
// =====================================================

void init_peripherals(void)
{
    int mux_pins[4] = {
        MUX_S0,
        MUX_S1,
        MUX_S2,
        MUX_S3
    };

    for(int i=0;i<4;i++)
    {
        gpio_reset_pin((gpio_num_t)mux_pins[i]);

        gpio_set_direction(
            (gpio_num_t)mux_pins[i],
            GPIO_MODE_OUTPUT
        );

        gpio_set_level(
            (gpio_num_t)mux_pins[i],
            0
        );
    }

    // ================= ADC =================

    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(
        &adc_cfg,
        &adc_handle
    );

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };

    adc_oneshot_config_channel(
        adc_handle,
        ADC_CHANNEL_4,
        &chan_cfg
    );

    // ================= LED =================

    gpio_reset_pin((gpio_num_t)LED_WHITE_PIN);

    gpio_set_direction(
        (gpio_num_t)LED_WHITE_PIN,
        GPIO_MODE_OUTPUT
    );

    gpio_set_level(
        (gpio_num_t)LED_WHITE_PIN,
        0
    );

    // ================= IR =================

    gpio_reset_pin((gpio_num_t)IR_RX_PIN);

    gpio_set_direction(
        (gpio_num_t)IR_RX_PIN,
        GPIO_MODE_INPUT
    );

    gpio_set_pull_mode(
        (gpio_num_t)IR_RX_PIN,
        GPIO_PULLUP_ONLY
    );

    gpio_set_intr_type(
        (gpio_num_t)IR_RX_PIN,
        GPIO_INTR_NEGEDGE
    );

    gpio_install_isr_service(0);

    gpio_isr_handler_add(
        (gpio_num_t)IR_RX_PIN,
        ir_isr_handler,
        NULL
    );

    init_motors();
}

// =====================================================
// INIT MOTORS
// =====================================================

void init_motors(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    int motor_pins[4] = {
        MOTOR_L_IN1,
        MOTOR_L_IN2,
        MOTOR_R_IN1,
        MOTOR_R_IN2
    };

    for(int i=0;i<4;i++)
    {
        ledc_channel_config_t ch = {
            .gpio_num = motor_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = i,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };

        ledc_channel_config(&ch);
    }

    // ================= ESC =================

    ledc_timer_config_t esc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&esc_timer);

    ledc_channel_config_t esc = {
        .gpio_num = FAN_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&esc);
}

// =====================================================
// FAN TASK
// =====================================================

void fan_task(void *arg)
{
    const uint32_t max_duty = (1 << 14) - 1;

    fan_set_speed_us(FAN_ESC_MIN_US);

    vTaskDelay(pdMS_TO_TICKS(FAN_ARM_TIME_MS));

    fan_set_speed_us(FAN_ESC_IDLE_US);

    while(1)
    {
        uint32_t duty =
            (fan_pulse_us * max_duty) / 20000;

        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_4,
            duty
        );

        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_4
        );

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================
// FAN CONTROL
// =====================================================

void fan_set_speed_us(uint32_t us)
{
    if(us < 900) us = 900;
    if(us > 2000) us = 2000;

    fan_pulse_us = us;
}

void fan_enable(bool enable)
{
    if(enable)
        fan_set_speed_us(FAN_ESC_RUN_US);
    else
        fan_set_speed_us(FAN_ESC_MIN_US);
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void set_motor_speeds(int left,int right)
{
    int duty_l = abs(left);
    int duty_r = abs(right);

    if(duty_l > MAX_SPEED) duty_l = MAX_SPEED;
    if(duty_r > MAX_SPEED) duty_r = MAX_SPEED;

    // LEFT

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_0,
        (left > 0) ? duty_l : 0
    );

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_1,
        (left < 0) ? duty_l : 0
    );

    // RIGHT

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_2,
        (right > 0) ? duty_r : 0
    );

    ledc_set_duty(
        LEDC_LOW_SPEED_MODE,
        LEDC_CHANNEL_3,
        (right < 0) ? duty_r : 0
    );

    for(int i=0;i<4;i++)
    {
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            i
        );
    }
}

// =====================================================
// SENSORES
// =====================================================

void set_mux(uint8_t ch)
{
    gpio_set_level((gpio_num_t)MUX_S0,(ch>>0)&1);
    gpio_set_level((gpio_num_t)MUX_S1,(ch>>1)&1);
    gpio_set_level((gpio_num_t)MUX_S2,(ch>>2)&1);
    gpio_set_level((gpio_num_t)MUX_S3,(ch>>3)&1);
}

void read_sensors(int *readings)
{
    for(int i=0;i<NUM_SENSORS;i++)
    {
        set_mux(i);

        esp_rom_delay_us(SENSOR_DELAY_US);

        adc_oneshot_read(
            adc_handle,
            ADC_CHANNEL_4,
            &readings[i]
        );
    }
}

// =====================================================
// CALIBRACIÓN
// =====================================================

void calibrate(void)
{
    int buf[NUM_SENSORS];
    int64_t t;

    // Si es la primera vez que se presiona (paso 0), reiniciamos los mínimos y máximos
    if (calibration_step == 0)
    {
        for(int i=0;i<NUM_SENSORS;i++)
        {
            minValues[i] = 4095;
            maxValues[i] = 0;
        }
    }

    // Asegurarnos de que el robot NO se mueva durante la captura
    set_motor_speeds(0,0);

    // Capturar datos durante 5 segundos
    t = esp_timer_get_time();

    while(esp_timer_get_time() - t < 5000000)
    {
        process_ir();

        // Permitir abortar la calibración con el comando STOP
        if(robot_state == STATE_STOPPED)
        {
            calibration_step = 0; // Reiniciamos el contador si se cancela
            return;
        }

        read_sensors(buf);

        // Actualizar min y max dinámicamente
        for(int i=0;i<NUM_SENSORS;i++)
        {
            if(buf[i] < minValues[i])
                minValues[i] = buf[i];

            if(buf[i] > maxValues[i])
                maxValues[i] = buf[i];
        }

        vTaskDelay(1);
    }

    // Incrementar el contador de pasos de calibración
    calibration_step++;

    // Si ya completamos ambas capturas (negro y blanco), reiniciamos el contador a 0
    if (calibration_step >= 2)
    {
        calibration_step = 0;
        
        // Ejecutar prueba de ventilador al finalizar ambas fases
        fan_test_startup(); 
    }
}

// =====================================================
// FAN TEST
// =====================================================

void fan_test_startup(void)
{
    fan_set_speed_us(1200);
    vTaskDelay(pdMS_TO_TICKS(1000));

    fan_set_speed_us(1350);
    vTaskDelay(pdMS_TO_TICKS(1000));

    fan_set_speed_us(FAN_ESC_RUN_US);
    vTaskDelay(pdMS_TO_TICKS(2000));

    fan_set_speed_us(FAN_ESC_IDLE_US);
}

// =====================================================
// POSICIÓN
// =====================================================

bool get_line_pos(int *readings,float *out_position)
{
    long weighted_sum = 0;
    long total_sum = 0;

    bool line_found = false;

    for(int i=0;i<NUM_SENSORS;i++)
    {
        int threshold =
            (minValues[i] + maxValues[i]) / 2;

        if(readings[i] > threshold)
        {
            line_found = true;

            int weight =
                readings[i] - minValues[i];

            if(weight < 0)
                weight = 0;

            weighted_sum +=
                (long)weight * (i * 1000);

            total_sum += weight;
        }
    }

    if(!line_found || total_sum == 0)
    {
        *out_position = last_pos;
        return false;
    }

    last_pos =
        (float)weighted_sum / total_sum;

    *out_position = last_pos;

    return true;
}

// =====================================================
// PID
// =====================================================

float calculate_pid(float error)
{
    P = error;

    I += error;

    if(I > 40000) I = 40000;
    if(I < -40000) I = -40000;

    D = error - previous_error;

    previous_error = error;

    return
        (Kp * P) +
        (Ki * I) +
        (Kd * D);
}