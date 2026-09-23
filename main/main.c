#if VERSION == 3
/***************************************************************************************************************************
 * FileName:     test_sensores.c  (usar como main.c)
 * Board:        ESP32-C3 SuperMini - Celerino
 * Description:  Prueba de la barra de 16 sensores: lectura ADC cruda, normalizada y error lateral en mm.
 *
 * Uso:
 *   1. Al arrancar, el LED blanco se enciende 5 s: pasa la barra varias veces sobre la línea y el fondo.
 *   2. Al apagarse el LED, imprime min/max de calibración y luego, a 10 Hz:
 *        RAW  : 16 lecturas ADC (0..4095)
 *        NORM : 16 lecturas normalizadas con piso de ruido (0..920)
 *        ERR  : posición de la línea en mm (0 = centro, - = lado sensor 1, + = lado sensor 16)
 *
 * No inicializa motores ni ESC.
 **************************************************************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// ================= PINES =================

#define MUX_S0   3
#define MUX_S1   2
#define MUX_S2  10
#define MUX_S3   7

#define ADC_LINE_CHANNEL ADC_CHANNEL_4   // GPIO 4
#define LED_WHITE_PIN    9

// ============== PARÁMETROS ===============

#define NUM_SENSORS       16
#define SENSOR_DELAY_US   20
#define ADC_SAMPLES       2

#define SENSOR_PITCH_MM   7.0f
#define MIN_CAL_SPAN      100
#define LINE_MIN_PEAK     350
#define NOISE_FLOOR       80
#define BLOB_HALF         3

#define CAL_TIME_US       5000000
#define PRINT_PERIOD_MS   100

// ================ VARIABLES ==============

static adc_oneshot_unit_handle_t adc_handle;

static int raw[NUM_SENSORS];
static int norm[NUM_SENSORS];
static int minValues[NUM_SENSORS];
static int maxValues[NUM_SENSORS];

static inline float sensor_x(int i) { return (i - 7.5f) * SENSOR_PITCH_MM; }

// =========================================

static void set_mux(uint8_t ch)
{
    gpio_set_level((gpio_num_t)MUX_S0, (ch >> 0) & 1);
    gpio_set_level((gpio_num_t)MUX_S1, (ch >> 1) & 1);
    gpio_set_level((gpio_num_t)MUX_S2, (ch >> 2) & 1);
    gpio_set_level((gpio_num_t)MUX_S3, (ch >> 3) & 1);
}

static void read_sensors(int *r)
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        set_mux(i);
        esp_rom_delay_us(SENSOR_DELAY_US);

        int acc = 0;
        for (int s = 0; s < ADC_SAMPLES; s++)
        {
            int v = 0;
            adc_oneshot_read(adc_handle, ADC_LINE_CHANNEL, &v);
            acc += v;
        }
        r[i] = acc / ADC_SAMPLES;
    }
}

static void init_hw(void)
{
    const int mux_pins[4] = { MUX_S0, MUX_S1, MUX_S2, MUX_S3 };

    for (int i = 0; i < 4; i++)
    {
        gpio_reset_pin((gpio_num_t)mux_pins[i]);
        gpio_set_direction((gpio_num_t)mux_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)mux_pins[i], 0);
    }

    gpio_reset_pin((gpio_num_t)LED_WHITE_PIN);
    gpio_set_direction((gpio_num_t)LED_WHITE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

    adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&adc_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(adc_handle, ADC_LINE_CHANNEL, &chan_cfg);
}

static void calibrate(void)
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        minValues[i] = 4095;
        maxValues[i] = 0;
    }

    printf("\nCALIBRANDO 5 s: pasa la barra sobre la linea y el fondo...\n");
    gpio_set_level((gpio_num_t)LED_WHITE_PIN, 1);

    int64_t t = esp_timer_get_time();

    while (esp_timer_get_time() - t < CAL_TIME_US)
    {
        read_sensors(raw);

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (raw[i] < minValues[i]) minValues[i] = raw[i];
            if (raw[i] > maxValues[i]) maxValues[i] = raw[i];
        }
        vTaskDelay(1);
    }

    gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

    printf("\n S#  :");
    for (int i = 0; i < NUM_SENSORS; i++) printf("%5d", i + 1);
    printf("\n MIN :");
    for (int i = 0; i < NUM_SENSORS; i++) printf("%5d", minValues[i]);
    printf("\n MAX :");
    for (int i = 0; i < NUM_SENSORS; i++) printf("%5d", maxValues[i]);
    printf("\n SPAN:");
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int span = maxValues[i] - minValues[i];
        printf("%4d%c", span, (span < MIN_CAL_SPAN) ? '!' : ' ');   // '!' = sensor sin contraste
    }
    printf("\n\n");
}

// Mismo estimador que el firmware (sin filtro IIR ni límite de salto, para ver la medición pura)
static bool get_line_pos_mm(const int *r, float *out_mm)
{
    int peak_i = -1;
    int peak_v = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int span = maxValues[i] - minValues[i];
        int v = 0;

        if (span >= MIN_CAL_SPAN)
        {
            v = ((r[i] - minValues[i]) * 1000) / span;
            if (v < 0)    v = 0;
            if (v > 1000) v = 1000;
        }

        v -= NOISE_FLOOR;
        if (v < 0) v = 0;

        norm[i] = v;

        if (v > peak_v) { peak_v = v; peak_i = i; }
    }

    if (peak_i < 0 || peak_v < (LINE_MIN_PEAK - NOISE_FLOOR))
        return false;

    int lo = peak_i, hi = peak_i;
    while (lo > 0 && lo > peak_i - BLOB_HALF && norm[lo - 1] > 0) lo--;
    while (hi < NUM_SENSORS - 1 && hi < peak_i + BLOB_HALF && norm[hi + 1] > 0) hi++;

    float ws = 0.0f, w = 0.0f;
    for (int j = lo; j <= hi; j++)
    {
        ws += (float)norm[j] * sensor_x(j);
        w  += (float)norm[j];
    }

    *out_mm = ws / w;
    return true;
}

void app_main(void)
{
    init_hw();
    vTaskDelay(pdMS_TO_TICKS(1000));   // tiempo para abrir el monitor

    calibrate();

    while (1)
    {
        int64_t t0 = esp_timer_get_time();
        read_sensors(raw);
        int64_t t_read = esp_timer_get_time() - t0;

        float err_mm;
        bool found = get_line_pos_mm(raw, &err_mm);

        printf("RAW :");
        for (int i = 0; i < NUM_SENSORS; i++) printf("%5d", raw[i]);

        printf("\nNORM:");
        for (int i = 0; i < NUM_SENSORS; i++) printf("%5d", norm[i]);

        if (found)
            printf("\nERR : %+7.2f mm   (lectura barra: %lld us)\n\n", err_mm, t_read);
        else
            printf("\nERR : LINEA PERDIDA   (lectura barra: %lld us)\n\n", t_read);

        vTaskDelay(pdMS_TO_TICKS(PRINT_PERIOD_MS));
    }
}
#endif
#if VERSION == 4
/***************************************************************************************************************************
 * FileName:     main.c
 * Processor:    ESP32-C3 (RISC-V RV32IMC) 160 MHz
 * Board:        ESP32-C3 SuperMini
 * Company:      TecNM /IT Chihuahua
 * Description:  Celerino line follower con PID en mm y máquina de estados
 * Authors:      Ana Cardona, Emiliano Pérez, Luis Anchondo
 * Created on:   2 mar. 2026
 * Updated:      09/2026
 **************************************************************************************************************************/
/**************************************************************************************************************************
 * Copyright (C) 2026 by Ana Cardona, Emiliano Pérez, Luis Anchondo - TecNM /IT Chihuahua
 *
 * Se permite la redistribucion, modificacion o uso de este software en formato fuente o binario
 * siempre que los archivos mantengan estos derechos de autor.
 * Los usuarios pueden modificar esto y usarlo para aprender sobre el campo de software embebido.
 * Ana Cardona, Emiliano Pérez, Luis Anchondo y el TecNM /IT Chihuahua no son responsables del mal uso de este material.
 **************************************************************************************************************************/

/*
Hardware: CELERINO CON ESP32-C3 SUPER MINI
BARRA DE SENSORES: 16 sensores IR sobre un arco de R = 120 mm, paso horizontal 7 mm,
                   conectados a un MUX 74HC4067 -> ADC1_CH4.
    Posición lateral: x_i = (i - 7.5) * 7 mm  (i = 0..15, sensor 1 = -52.5 mm)
    Retraso sobre Y:  y_i = 120 - sqrt(120^2 - x_i^2)  (0.05 mm centro, 12.09 mm extremos)
    PINES:
        *S0  - GPIO 3
        *S1  - GPIO 2
        *S2  - GPIO 10
        *S3  - GPIO 7
        *ADC - GPIO 4 (ADC1_CH4)
MOTORES: 2 motores DC con medios puentes H
    PINES:
        *Motor Izquierdo IN1 - GPIO 21
        *Motor Izquierdo IN2 - GPIO 20
        *Motor Derecho IN1   - GPIO 5
        *Motor Derecho IN2   - GPIO 6
VENTILADOR (ESC):
        *PWM Ventilador      - GPIO 8
LED_CARRO:
        *LED BLANCO          - GPIO 9
RECEPTOR_IR (NEC):
        *Salida Receptor IR  - GPIO 1
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

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

#define NUM_SENSORS         16
#define SENSOR_DELAY_US     20      // asentamiento del MUX antes de convertir
#define ADC_SAMPLES         2       // lecturas promediadas por canal

// ================= MUX =================

#define MUX_S0   3
#define MUX_S1   2
#define MUX_S2  10
#define MUX_S3   7

#define ADC_LINE_CHANNEL ADC_CHANNEL_4

// ================ MOTORES ===============

#define MOTOR_L_IN1 21
#define MOTOR_L_IN2 20
#define MOTOR_R_IN1  5
#define MOTOR_R_IN2  6

// ================= EXTRAS ===============

#define FAN_PWM_PIN   8
#define LED_WHITE_PIN 9
#define IR_RX_PIN     1

// =============== VELOCIDADES ============

#define BASE_SPEED          500   //600
#define SEARCH_SPEED        500
#define CAL_SPEED           165
#define MAX_SPEED           900   //1850

// =================== IR =================

#define IR_CMD_CALIBRATE 0x45
#define IR_CMD_START     0x46
#define IR_CMD_STOP      0x47

// ================== ESC =================

#define FAN_ESC_MIN_US    900   // mínimo / desarmado
#define FAN_ESC_IDLE_US  1100   // ralentí
#define FAN_ESC_MAX_US   1350   // tope del ESC
#define FAN_ARM_TIME_MS  5000

#define FAN_FRAME_MS       20   // 1 trama = 20 ms (50 Hz)
#define FAN_RAMP_US_PER_S 400   // pendiente de la rampa: µs de pulso por segundo

#define FAN_DEFAULT_RUN_US 1500 // se recorta a FAN_ESC_MAX_US en fan_set_speed_us()

// ============ GEOMETRÍA / ESTIMADOR ============

#define SENSOR_PITCH_MM   7.0f    // separación entre centros de sensores horizontalmente
#define MIN_CAL_SPAN      100     // span min-max mínimo para usar un sensor
#define LINE_MIN_PEAK     350     // pico normalizado (0..1000) para considerar línea
#define NOISE_FLOOR       80      // se resta a cada lectura normalizada
#define BLOB_HALF         3       // sensores máximos a cada lado del pico
#define POS_ALPHA         0.6f    // IIR de posición (1 = sin filtro)
#define MAX_JUMP_MM       25.0f   // salto máximo aceptado entre muestras

// ================== CONTROL =================

#define CONTROL_PERIOD_MS 2
#define LOOP_TICKS ((pdMS_TO_TICKS(CONTROL_PERIOD_MS) > 0) ? pdMS_TO_TICKS(CONTROL_PERIOD_MS) : 1)

#if CONFIG_FREERTOS_HZ < 1000
#warning "CONFIG_FREERTOS_HZ < 1000: el lazo de control no podrá correr a CONTROL_PERIOD_MS"
#endif

#define D_ALPHA           0.3f    // filtro pasa-bajas de la derivada
#define I_LIMIT           50.0f   // límite del integrador [mm*s]

// =====================================================
// VARIABLES
// =====================================================

adc_oneshot_unit_handle_t adc_handle;

int sensors[NUM_SENSORS];
int minValues[NUM_SENSORS];
int maxValues[NUM_SENSORS];
static int norm[NUM_SENSORS];

static float last_pos_mm  = 0.0f;
static bool  line_tracked = false;

float P, I, D;
float previous_error = 0.0f;
static float   d_filt    = 0.0f;
static int64_t last_t_us = 0;

/*
 * Ganancias en unidades físicas (error en mm, derivada en mm/s, integral en mm*s).
 * Equivalencia con las anteriores (1000 unidades por sensor de 7 mm, lazo ~10.5 ms):
 *   Kp = 0.1 * 1000/7          ~= 14.3
 *   Kd = 0.6 * 1000/7 * 0.0105 ~= 0.9
 */
float Kp = 14.3f;
float Ki = 0.0f;
float Kd = 0.9f;

// ================ TURBINA ===============

volatile uint32_t fan_pulse_us  = FAN_ESC_MIN_US;      // pulso actual
volatile uint32_t fan_target_us = FAN_ESC_MIN_US;      // pulso objetivo
volatile uint32_t fan_run_us    = FAN_DEFAULT_RUN_US;  // velocidad de trabajo

// =====================================================
// ESTADOS
// =====================================================

typedef enum {
    STATE_IDLE,
    STATE_CALIBRATING,
    STATE_RUNNING,
    STATE_STOPPED
} robot_state_t;

volatile robot_state_t robot_state = STATE_IDLE;

// 0 = listo para primera captura, 1 = listo para segunda captura
volatile uint8_t calibration_step = 0;

volatile bool    start_sequence_pending = false;
volatile int64_t start_time_us = 0;

// =====================================================
// IR
// =====================================================

volatile uint32_t ir_data  = 0;
volatile uint8_t  ir_bits  = 0;
volatile bool     ir_ready = false;
volatile int64_t  ir_last_us = 0;

// =====================================================
// PROTOTIPOS
// =====================================================

void init_peripherals(void);
void init_motors(void);

void set_mux(uint8_t ch);
void read_sensors(int *readings);

bool get_line_pos_mm(const int *raw, float *out_mm);

void set_motor_speeds(int left, int right);

void  pid_reset(void);
float calculate_pid(float error_mm);

void calibrate(void);

void process_ir(void);

void     fan_task(void *arg);
void     fan_set_speed_us(uint32_t us);
void     fan_kill(void);
void     fan_enable(bool enable);
uint32_t fan_percent_to_us(uint8_t pct);

static inline float sensor_x(int i) { return (i - 7.5f) * SENSOR_PITCH_MM; }

// =====================================================
// ISR IR (NEC)
// =====================================================

static void IRAM_ATTR ir_isr_handler(void *arg)
{
    int64_t now = esp_timer_get_time();
    int64_t duration = now - ir_last_us;

    ir_last_us = now;

    if (duration > 13000 && duration < 14000)
    {
        ir_data  = 0;
        ir_bits  = 0;
        ir_ready = false;
    }
    else if (duration > 1000 && duration < 1300)
    {
        if (ir_bits < 32)
            ir_bits++;
    }
    else if (duration > 2000 && duration < 2500)
    {
        if (ir_bits < 32)
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

    if (ir_bits == 32)
    {
        ir_ready = true;
        ir_bits  = 0;
    }
}

// =====================================================
// APP MAIN
// =====================================================

void app_main(void)
{
    init_peripherals();

    xTaskCreatePinnedToCore(fan_task, "fan_task", 2048, NULL, 4, NULL, 0);

    vTaskDelay(pdMS_TO_TICKS(500));

    TickType_t last_wake = xTaskGetTickCount();
    bool was_lost = true;

    while (1)
    {
        process_ir();

        // ============ ARRANQUE NO BLOQUEANTE ============

        if (start_sequence_pending)
        {
            if ((esp_timer_get_time() - start_time_us) >= 1000000)
            {
                start_sequence_pending = false;
                pid_reset();
                was_lost = true;
                robot_state = STATE_RUNNING;
            }
        }

        switch (robot_state)
        {
            case STATE_IDLE:
            case STATE_STOPPED:

                set_motor_speeds(0, 0);
                break;

            case STATE_CALIBRATING:

                calibrate();

                if (robot_state == STATE_CALIBRATING)
                    robot_state = STATE_IDLE;

                gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

                last_wake = xTaskGetTickCount();   // evita ráfaga de ciclos tras 5 s bloqueado
                break;

            case STATE_RUNNING:
            {
                read_sensors(sensors);

                float position_mm;

                if (get_line_pos_mm(sensors, &position_mm))
                {
                    float error = position_mm;       // setpoint = 0 mm (centro de la barra)

                    if (was_lost)
                    {
                        // Reenganche sin patada derivativa
                        previous_error = error;
                        d_filt    = 0.0f;
                        last_t_us = 0;
                        was_lost  = false;
                    }

                    float pid = calculate_pid(error);

                    int left  = BASE_SPEED + (int)pid;
                    int right = BASE_SPEED - (int)pid;

                    if (left  >  MAX_SPEED) left  =  MAX_SPEED;
                    if (right >  MAX_SPEED) right =  MAX_SPEED;
                    if (left  < -MAX_SPEED) left  = -MAX_SPEED;
                    if (right < -MAX_SPEED) right = -MAX_SPEED;

                    set_motor_speeds(left, right);
                }
                else
                {
                    I = 0.0f;
                    was_lost = true;

                    if (last_pos_mm > 0.0f)
                        set_motor_speeds(SEARCH_SPEED, 0);
                    else
                        set_motor_speeds(0, SEARCH_SPEED);
                }
                break;
            }
        }

        vTaskDelayUntil(&last_wake, LOOP_TICKS);
    }
}

// =====================================================
// PROCESS IR
// =====================================================

void process_ir(void)
{
    if (!ir_ready)
        return;

    uint8_t command = (ir_data >> 16) & 0xFF;

    ir_ready = false;

    switch (command)
    {
        case IR_CMD_CALIBRATE:

            robot_state = STATE_CALIBRATING;
            gpio_set_level((gpio_num_t)LED_WHITE_PIN, 1);
            break;

        case IR_CMD_START:

            pid_reset();
            gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

            fan_enable(true);

            start_sequence_pending = true;
            start_time_us = esp_timer_get_time();
            break;

        case IR_CMD_STOP:

            start_sequence_pending = false;
            robot_state = STATE_STOPPED;

            set_motor_speeds(0, 0);
            fan_enable(false);

            gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);
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
    const int mux_pins[4] = { MUX_S0, MUX_S1, MUX_S2, MUX_S3 };

    for (int i = 0; i < 4; i++)
    {
        gpio_reset_pin((gpio_num_t)mux_pins[i]);
        gpio_set_direction((gpio_num_t)mux_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)mux_pins[i], 0);
    }

    // ================= ADC =================

    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id = ADC_UNIT_1
    };
    adc_oneshot_new_unit(&adc_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(adc_handle, ADC_LINE_CHANNEL, &chan_cfg);

    // Sin calibración: span 0 -> ningún sensor se usa hasta calibrar
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        minValues[i] = 0;
        maxValues[i] = 0;
    }

    // ================= LED =================

    gpio_reset_pin((gpio_num_t)LED_WHITE_PIN);
    gpio_set_direction((gpio_num_t)LED_WHITE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

    // ================= IR =================

    gpio_reset_pin((gpio_num_t)IR_RX_PIN);
    gpio_set_direction((gpio_num_t)IR_RX_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)IR_RX_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type((gpio_num_t)IR_RX_PIN, GPIO_INTR_NEGEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)IR_RX_PIN, ir_isr_handler, NULL);

    init_motors();
}

// =====================================================
// INIT MOTORS
// =====================================================

void init_motors(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 20000,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    const int motor_pins[4] = { MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_R_IN1, MOTOR_R_IN2 };

    for (int i = 0; i < 4; i++)
    {
        ledc_channel_config_t ch = {
            .gpio_num   = motor_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = LEDC_TIMER_0,
            .duty       = 0,
            .hpoint     = 0
        };
        ledc_channel_config(&ch);
    }

    // ================= ESC =================

    ledc_timer_config_t esc_timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = 50,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ledc_timer_config(&esc_timer);

    ledc_channel_config_t esc = {
        .gpio_num   = FAN_PWM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_4,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&esc);
}

// =====================================================
// TURBINA (ESC)
// =====================================================

static void fan_write_us(uint32_t us)
{
    const uint32_t max_duty = (1 << 14) - 1;          // 16383
    uint32_t duty = (us * max_duty) / 20000;           // periodo 20000 µs @ 50 Hz

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
}

void fan_set_speed_us(uint32_t us)
{
    if (us < FAN_ESC_MIN_US) us = FAN_ESC_MIN_US;
    if (us > FAN_ESC_MAX_US) us = FAN_ESC_MAX_US;

    fan_target_us = us;
}

void fan_kill(void)
{
    fan_target_us = FAN_ESC_MIN_US;
    fan_pulse_us  = FAN_ESC_MIN_US;
}

void fan_enable(bool enable)
{
    if (enable)
        fan_set_speed_us(fan_run_us);
    else
        fan_kill();
}

uint32_t fan_percent_to_us(uint8_t pct)
{
    if (pct > 100) pct = 100;

    return FAN_ESC_IDLE_US + ((FAN_ESC_MAX_US - FAN_ESC_IDLE_US) * pct) / 100;
}

void fan_task(void *arg)
{
    const uint32_t step = (FAN_RAMP_US_PER_S * FAN_FRAME_MS) / 1000;

    // ---- Armado ----
    fan_pulse_us  = FAN_ESC_MIN_US;
    fan_target_us = FAN_ESC_MIN_US;
    fan_write_us(FAN_ESC_MIN_US);

    vTaskDelay(pdMS_TO_TICKS(FAN_ARM_TIME_MS));

    // ---- Rampa hasta ralentí ----
    fan_target_us = FAN_ESC_IDLE_US;

    while (1)
    {
        uint32_t cur = fan_pulse_us;
        uint32_t tgt = fan_target_us;

        if (cur < tgt)
            cur = (tgt - cur > step) ? cur + step : tgt;
        else if (cur > tgt)
            cur = (cur - tgt > step) ? cur - step : tgt;

        fan_pulse_us = cur;
        fan_write_us(cur);

        vTaskDelay(pdMS_TO_TICKS(FAN_FRAME_MS));
    }
}

// =====================================================
// MOTOR CONTROL
// =====================================================

void set_motor_speeds(int left, int right)
{
    int duty_l = abs(left);
    int duty_r = abs(right);

    if (duty_l > MAX_SPEED) duty_l = MAX_SPEED;
    if (duty_r > MAX_SPEED) duty_r = MAX_SPEED;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (left  > 0) ? duty_l : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (left  < 0) ? duty_l : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (right > 0) ? duty_r : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, (right < 0) ? duty_r : 0);

    for (int i = 0; i < 4; i++)
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
}

// =====================================================
// SENSORES
// =====================================================

void set_mux(uint8_t ch)
{
    gpio_set_level((gpio_num_t)MUX_S0, (ch >> 0) & 1);
    gpio_set_level((gpio_num_t)MUX_S1, (ch >> 1) & 1);
    gpio_set_level((gpio_num_t)MUX_S2, (ch >> 2) & 1);
    gpio_set_level((gpio_num_t)MUX_S3, (ch >> 3) & 1);
}

void read_sensors(int *readings)
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        set_mux(i);
        esp_rom_delay_us(SENSOR_DELAY_US);

        int acc = 0;
        for (int s = 0; s < ADC_SAMPLES; s++)
        {
            int v = 0;
            adc_oneshot_read(adc_handle, ADC_LINE_CHANNEL, &v);
            acc += v;
        }
        readings[i] = acc / ADC_SAMPLES;
    }
}

// =====================================================
// CALIBRACIÓN (2 pasos de 5 s: negro y blanco)
// =====================================================

void calibrate(void)
{
    int buf[NUM_SENSORS];

    if (calibration_step == 0)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            minValues[i] = 4095;
            maxValues[i] = 0;
        }
    }

    set_motor_speeds(0, 0);

    int64_t t = esp_timer_get_time();

    while (esp_timer_get_time() - t < 5000000)
    {
        process_ir();

        if (robot_state == STATE_STOPPED)
        {
            calibration_step = 0;
            return;
        }

        read_sensors(buf);

        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (buf[i] < minValues[i]) minValues[i] = buf[i];
            if (buf[i] > maxValues[i]) maxValues[i] = buf[i];
        }

        vTaskDelay(1);
    }

    calibration_step++;

    if (calibration_step >= 2)
        calibration_step = 0;

    line_tracked = false;
}

// =====================================================
// POSICIÓN EN mm (centroide local del blob alrededor del pico)
// =====================================================
//
// 1. Normaliza cada sensor a 0..1000 con su calibración.
// 2. Resta NOISE_FLOOR: el fondo aporta peso 0 y los sensores entran/salen
//    del cálculo de forma continua (sin saltos ni puntos ciegos).
// 3. Toma el pico y solo los sensores contiguos con señal (±BLOB_HALF);
//    reflejos o ruido en otra parte de la barra no contaminan la medición.
// 4. Centroide con las posiciones físicas x_i en mm.
// 5. Limita saltos imposibles y aplica IIR.
//
bool get_line_pos_mm(const int *raw, float *out_mm)
{
    int peak_i = -1;
    int peak_v = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        int span = maxValues[i] - minValues[i];
        int v = 0;

        if (span >= MIN_CAL_SPAN)
        {
            v = ((raw[i] - minValues[i]) * 1000) / span;
            if (v < 0)    v = 0;
            if (v > 1000) v = 1000;
        }

        v -= NOISE_FLOOR;
        if (v < 0) v = 0;

        norm[i] = v;

        if (v > peak_v)
        {
            peak_v = v;
            peak_i = i;
        }
    }

    if (peak_i < 0 || peak_v < (LINE_MIN_PEAK - NOISE_FLOOR))
    {
        line_tracked = false;
        *out_mm = last_pos_mm;
        return false;
    }

    int lo = peak_i;
    int hi = peak_i;

    while (lo > 0 && lo > peak_i - BLOB_HALF && norm[lo - 1] > 0) lo--;
    while (hi < NUM_SENSORS - 1 && hi < peak_i + BLOB_HALF && norm[hi + 1] > 0) hi++;

    float ws = 0.0f;
    float w  = 0.0f;

    for (int j = lo; j <= hi; j++)
    {
        ws += (float)norm[j] * sensor_x(j);
        w  += (float)norm[j];
    }

    float pos = ws / w;   // w > 0 garantizado por el pico

    if (line_tracked)
    {
        float jump = pos - last_pos_mm;

        if (fabsf(jump) > MAX_JUMP_MM)
            pos = last_pos_mm + copysignf(MAX_JUMP_MM, jump);

        last_pos_mm = POS_ALPHA * pos + (1.0f - POS_ALPHA) * last_pos_mm;
    }
    else
    {
        last_pos_mm  = pos;          // reenganche: sin filtro ni límite de salto
        line_tracked = true;
    }

    *out_mm = last_pos_mm;
    return true;
}

// =====================================================
// PID (error en mm, dt real)
// =====================================================

void pid_reset(void)
{
    P = I = D = 0.0f;
    previous_error = 0.0f;
    d_filt    = 0.0f;
    last_t_us = 0;
    line_tracked = false;
}

float calculate_pid(float error_mm)
{
    int64_t now = esp_timer_get_time();
    float dt = (last_t_us == 0) ? (CONTROL_PERIOD_MS * 1e-3f) : (float)(now - last_t_us) * 1e-6f;

    if (dt < 1e-4f) dt = 1e-4f;
    last_t_us = now;

    P = error_mm;

    I += error_mm * dt;
    if (I >  I_LIMIT) I =  I_LIMIT;
    if (I < -I_LIMIT) I = -I_LIMIT;

    float d_raw = (error_mm - previous_error) / dt;
    d_filt = D_ALPHA * d_raw + (1.0f - D_ALPHA) * d_filt;
    D = d_filt;

    previous_error = error_mm;

    return (Kp * P) + (Ki * I) + (Kd * D);
}
#endif