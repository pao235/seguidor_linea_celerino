/***************************************************************************************************************************
 * FileName:     main.c
 * Processor:    Tensilica Xtensa LX6 160 MHz
 * Board:        ESP32-C3 SuperMini
 * Company:      TecNM /IT Chihuahua
 * Description:  Celerino line follower with PID controller in a state machine
 * Authors:      Ana Cardona, Emiliano Pérez, Luis Anchondo
 * Updated:      05/2026
 * Created on:   20 feb. 2026
 * updated:      25/05/2026
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
// LIBRERIAS
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

#define SENSOR_DELAY_US 10 // Tiempo de espera entre seleccionar canal del MUX y leer el ADC (us)
#define NUM_SENSORS 16 // Número de sensores IR en la barra
#define SETPOINT 7500.0f // Posición central de la línea (rango 0 a 15000)

// ================= MUX =================
// Pines de selección del multiplexor 74HC4067
#define MUX_S0  3
#define MUX_S1  2
#define MUX_S2 10
#define MUX_S3  7

// ================ MOTORES ===============
// Pines de control de los medio-puentes H (BTN9960LV)
// Adelante: IN1 = PWM, IN2 = 0
// Atrás:    IN1 = 0,   IN2 = PWM
// Freno:    IN1 = 0,   IN2 = 0
#define MOTOR_L_IN1 21
#define MOTOR_L_IN2 20
#define MOTOR_R_IN1  6
#define MOTOR_R_IN2  5

// ================= EXTRAS ===============
#define FAN_PWM_PIN   8   // PWM del ESC del ventilador
#define LED_WHITE_PIN 9   // LED indicador de estado
#define IR_RX_PIN     1   // Receptor IR (protocolo NEC)

// =============== VELOCIDADES ============
// Valoresdel PWM 
#define BASE_SPEED    140   // Velocidad base en línea recta
#define SEARCH_SPEED  175   // Velocidad al buscar la línea cuando se pierde
#define CAL_SPEED     175   // Velocidad durante la calibración
#define MAX_SPEED     440   // Límite superior de velocidad

// =================== IR =================
// Códigos de comando del control remoto IR (protocolo NEC)
#define IR_CMD_CALIBRATE 0x45   // Botón 1: iniciar calibración
#define IR_CMD_START     0x46   // Botón 2: arrancar el robot
#define IR_CMD_STOP      0x47   // Botón 3: detener el robot

// ================== ESC =================
// Parámetros del controlador ESC del ventilador 
#define FAN_ESC_MIN_US    900    // Pulso mínimo: motor detenido / armado
#define FAN_ESC_IDLE_US  1100    // Pulso en reposo: velocidad baja
#define FAN_ESC_RUN_US   1500    // Pulso en operación: velocidad de carrera
#define FAN_ARM_TIME_MS  5000    // Tiempo de armado del ESC al iniciar (ms)

// =====================================================
// VARIABLES GLOBALES
// =====================================================

adc_oneshot_unit_handle_t adc_handle;  // Handle del ADC en modo oneshot

int sensors[NUM_SENSORS]; // Lecturas de los 16 sensores
int minValues[NUM_SENSORS]; // Mínimos registrados durante calibración
int maxValues[NUM_SENSORS];  // Máximos registrados durante calibración

float last_pos = SETPOINT; // Última posición válida de la línea

float P, I, D; // Términos proporcional, integral y derivativo del PID
float previous_error = 0; // Error del ciclo anterior (para calcular D)

// Ganancias del controlador PID
float Kp = 0.4f;
float Ki = 0.0f;
float Kd = 0.08f;

volatile uint32_t fan_pulse_us = FAN_ESC_MIN_US;  // Ancho de pulso actual del ESC (us)
int64_t last_time = 0; // Timestamp del último ciclo PID (us)

// Contador de pasos de calibración: 0 = listo para capturar negros, 1 = listo para capturar blancos
volatile uint8_t calibration_step = 0;

// =====================================================
// Máquina de estados 
// =====================================================
typedef enum{
    STATE_IDLE, // Esperando comando IR
    STATE_CALIBRATING, // Ejecutando calibración
    STATE_RUNNING, // Siguiendo la línea
    STATE_STOPPED // Detenido por comando IR
} robot_state_t;

volatile robot_state_t robot_state = STATE_IDLE;

// Bandera y timestamp para el arranque no bloqueante del ESC
volatile bool  start_sequence_pending = false;
volatile int64_t start_time_us  = 0;

// =====================================================
// VARIABLES IR — Decodificación protocolo NEC por ISR
// =====================================================

volatile uint32_t ir_data = 0; // Trama de 32 bits recibida
volatile uint8_t  ir_bits = 0; // Número de bits recibidos hasta ahora
volatile bool ir_ready= false; // true cuando se recibieron los 32 bits
volatile int64_t ir_last_us = 0; // Timestamp del último flanco (us)

// =====================================================
// PROTOTIPOS
// =====================================================

void  init_peripherals(void);
void  init_motors(void);
void  set_mux(uint8_t ch);
void  read_sensors(int *readings);
bool  get_line_pos(int *readings, float *out_position);
void  set_motor_speeds(int left, int right);
float calculate_pid(float error);
void  calibrate(void);
void  process_ir(void);
void  fan_task(void *arg);
void  fan_set_speed_us(uint32_t us);
void  fan_test_startup(void);
void  fan_enable(bool enable);

// =====================================================
// ISR — RECEPTOR IR (protocolo NEC)
// =====================================================
// Se ejecuta en cada flanco de bajada del pin IR_RX_PIN
// Mide la duración entre flancos para decodificar bits:
//   ~13.5 ms -> cabecera de inicio (reset)
//   ~1.1 ms -> bit 0
//   ~2.2 ms -> bit 1
// Al completar 32 bits, activa ir_ready
static void IRAM_ATTR ir_isr_handler(void *arg)
{
    int64_t now  = esp_timer_get_time();
    int64_t duration = now - ir_last_us;

    ir_last_us = now;

    if(duration > 13000 && duration < 14000)
    {
        // Cabecera NEC detectada: reiniciar recepción
        ir_data  = 0;
        ir_bits  = 0;
        ir_ready = false;
    }
    else if(duration > 1000 && duration < 1300)
    {
        // Bit 0: solo incrementar contador
        if(ir_bits < 32)
            ir_bits++;
    }
    else if(duration > 2000 && duration < 2500)
    {
        // Bit 1: guardar el bit en la posición correspondiente
        if(ir_bits < 32)
        {
            ir_data |= (1UL << ir_bits);
            ir_bits++;
        }
    }
    else
    {
        // Pulso no reconocido: reiniciar
        ir_bits = 0;
        ir_data = 0;
    }

    if(ir_bits == 32)
    {
        // Trama completa recibida
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

    // Crear tarea del ventilador en el núcleo 0, prioridad 4
    xTaskCreatePinnedToCore(fan_task,"fan_task",2048,NULL,4,NULL,0);

    // Esperar a que el ESC termine de armarse
    vTaskDelay(pdMS_TO_TICKS(500));

    while(1)
    {
        // Atender comandos IR antes de ejecutar la lógica de estado
        process_ir();

        // =========================================
        // Después de recibir START, espera 2 s
        // para que el ESC alcance velocidad de operación
        // antes de pasar a STATE_RUNNING
        // =========================================
        if(start_sequence_pending)
        {
            if((esp_timer_get_time() - start_time_us) >= 2000000)
            {
                start_sequence_pending = false;
                robot_state = STATE_RUNNING;
            }
        }

        // Máquina de estados principal
        switch(robot_state)
        {
            // =====================================
            case STATE_IDLE:
            case STATE_STOPPED:
                // Robot detenido, esperar comando
                set_motor_speeds(0, 0);
                vTaskDelay(pdMS_TO_TICKS(10));
                break;

            // =====================================
            case STATE_CALIBRATING:
                // Ejecutar calibración y regresar a IDLE
                calibrate();
                robot_state = STATE_IDLE;
                
                // Apagar el LED para indicar que terminaron los 5 segundos
                gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0); 
                break;

            // =====================================
            case STATE_RUNNING:
            {
                read_sensors(sensors);

                float position;
                bool line_found = get_line_pos(sensors, &position);

                if(line_found)
                {
                    float error = position - SETPOINT;
                    float pid   = calculate_pid(error);

                    // reduce velocidad proporcionalmente al error
                    // así el robot frena en curvas y acelera en rectas
                    int dynamic_base = BASE_SPEED - (abs((int)error) * 0.035f);

                    if(dynamic_base < 0)
                        dynamic_base = 0;

                    // Control diferencial: PID ajusta la diferencia entre ruedas
                    int left  = dynamic_base + (int)pid;
                    int right = dynamic_base - (int)pid;

                    // Limitar al máximo permitido
                    if(left  > MAX_SPEED) left  = MAX_SPEED;
                    if(right > MAX_SPEED) right = MAX_SPEED;

                    // No se permite marcha atrás en esta versión
                    if(left  < 0) left  = 0;
                    if(right < 0) right = 0;

                    set_motor_speeds(left, right);
                }
                else
                {
                    // Línea perdida: reiniciar integral y pivotar
                    // hacia el lado donde se vio la línea por última vez
                    I = 0;

                    if(previous_error > 0)
                        set_motor_speeds(SEARCH_SPEED, 0);
                    else
                        set_motor_speeds(0, SEARCH_SPEED);
                }

                esp_rom_delay_us(SENSOR_DELAY_US);
                esp_rom_delay_us(1000);  // Pequeño delay adicional entre ciclos

                break;
            }
        }
    }
}

// =====================================================
// PROCESS IR — Decodifica y ejecuta comandos IR
// =====================================================
void process_ir(void)
{
    if(!ir_ready)
        return;

    // El comando está en los bits [23:16] de la trama NEC
    uint8_t command = (ir_data >> 16) & 0xFF;
    ir_ready = false;

    switch(command)
    {
        // =====================================
        // CALIBRAR
        // =====================================
        case IR_CMD_CALIBRATE:
            robot_state = STATE_CALIBRATING;
            gpio_set_level((gpio_num_t)LED_WHITE_PIN, 1);  // LED encendido durante calibración
            break;

        // =====================================
        // START
        // =====================================
        case IR_CMD_START:
            // Reiniciar estado del PID
            calibration_step = 0; // Reiniciar estado del contador por seguridad
            previous_error = 0;
            I              = 0;
            last_time      = esp_timer_get_time();

            gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

            // Activar ventilador y armar ESC
            fan_enable(false);  // Parámetro en false: usar velocidad FAN_ESC_RUN_US directamente
            fan_set_speed_us(FAN_ESC_RUN_US);

            // Iniciar cuenta regresiva de 2 s antes de comenzar a seguir la línea
            start_sequence_pending = true;
            start_time_us          = esp_timer_get_time();
            break;

        // =====================================
        // STOP
        // =====================================
        case IR_CMD_STOP:
            calibration_step = 0; // Reiniciar estado del contador por seguridad
            start_sequence_pending = false;
            robot_state            = STATE_STOPPED;

            set_motor_speeds(0, 0);
            fan_enable(false);
            gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);
            break;

        default:
            break;
    }
}

// =====================================================
// INIT — Inicialización de periféricos
// =====================================================
void init_peripherals(void)
{
    // Configurar los 4 pines de selección del MUX como salidas
    int mux_pins[4] = { MUX_S0, MUX_S1, MUX_S2, MUX_S3 };

    for(int i = 0; i < 4; i++)
    {
        gpio_reset_pin((gpio_num_t)mux_pins[i]);
        gpio_set_direction((gpio_num_t)mux_pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)mux_pins[i], 0);
    }

    // ================= ADC =================
    // Configurar ADC1 canal 4 (GPIO4) a 12 dB de atenuación (~0-3.3 V)
    adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&adc_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12
    };
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_4, &chan_cfg);

    // ================= LED =================
    gpio_reset_pin((gpio_num_t)LED_WHITE_PIN);
    gpio_set_direction((gpio_num_t)LED_WHITE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LED_WHITE_PIN, 0);

    // ================= IR =================
    // Configurar receptor IR con interrupción en flanco de bajada
    gpio_reset_pin((gpio_num_t)IR_RX_PIN);
    gpio_set_direction((gpio_num_t)IR_RX_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)IR_RX_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type((gpio_num_t)IR_RX_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)IR_RX_PIN, ir_isr_handler, NULL);

    init_motors();
}

// =====================================================
// INIT MOTORS — Configura PWM de motores y ESC
// =====================================================
void init_motors(void)
{
    // Timer 0: motores DC (10-bit, 20 kHz — por encima del rango audible)
    ledc_timer_config_t timer = {
        .speed_mode  = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num  = LEDC_TIMER_0,
        .freq_hz  = 20000,
        .clk_cfg  = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    // Canales 0-3: un canal por pin de motor (L_IN1, L_IN2, R_IN1, R_IN2)
    int motor_pins[4] = { MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_R_IN1, MOTOR_R_IN2 };

    for(int i = 0; i < 4; i++)
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
    // Timer 1: ESC del ventilador (14-bit, 50 Hz — señal tipo servo, periodo 20 ms)
    ledc_timer_config_t esc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&esc_timer);

    // Canal 4: ESC del ventilador
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
// FAN TASK — Tarea de FreeRTOS para el ventilador
// =====================================================
// Corre en el núcleo 0. Arma el ESC con pulso mínimo,
// espera FAN_ARM_TIME_MS, luego entra en loop actualizando
// el duty del canal LEDC según fan_pulse_us cada 20 ms.
void fan_task(void *arg)
{
    const uint32_t max_duty = (1 << 14) - 1;  // 16383 para resolución de 14 bits

    // Enviar pulso mínimo para armar el ESC
    fan_set_speed_us(FAN_ESC_MIN_US);
    vTaskDelay(pdMS_TO_TICKS(FAN_ARM_TIME_MS));

    // Pasar a velocidad de reposo una vez armado
    fan_set_speed_us(FAN_ESC_IDLE_US);

    while(1)
    {
        // Convertir microsegundos de pulso a valor de duty (periodo = 20 000 us)
        uint32_t duty = (fan_pulse_us * max_duty) / 20000;

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

        vTaskDelay(pdMS_TO_TICKS(20));  // Actualizar cada 20 ms (igual al periodo del ESC)
    }
}

// =====================================================
// FAN CONTROL
// =====================================================

// Ajusta el ancho de pulso del ESC, limitando al rango válido [900, 2000] us
void fan_set_speed_us(uint32_t us)
{
    if(us <  900) us =  900;
    if(us > 2000) us = 2000;

    fan_pulse_us = us;
}

// Activa o desactiva el ventilador usando los pulsos predefinidos
void fan_enable(bool enable)
{
    if(enable)
        fan_set_speed_us(FAN_ESC_RUN_US);
    else
        fan_set_speed_us(FAN_ESC_MIN_US);
}

// =====================================================
// MOTOR CONTROL — Controla velocidad y dirección
// =====================================================
// left / right: rango [-MAX_SPEED, +MAX_SPEED]
//   Positivo -> adelante (IN1 = duty, IN2 = 0)
//   Negativo -> atrás    (IN1 = 0,   IN2 = duty)
//   Cero -> freno    (IN1 = 0,   IN2 = 0)
void set_motor_speeds(int left, int right)
{
    int duty_l = abs(left);
    int duty_r = abs(right);

    if(duty_l > MAX_SPEED) duty_l = MAX_SPEED;
    if(duty_r > MAX_SPEED) duty_r = MAX_SPEED;

    // Motor izquierdo
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (left  > 0) ? duty_l : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (left  < 0) ? duty_l : 0);

    // Motor derecho
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, (right > 0) ? duty_r : 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, (right < 0) ? duty_r : 0);

    // Aplicar cambios en todos los canales
    for(int i = 0; i < 4; i++)
        ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
}

// =====================================================
// SENSORES — MUX y lectura ADC
// =====================================================

// Selecciona el canal ch del MUX poniendo sus 4 bits en los pines S0-S3
void set_mux(uint8_t ch)
{
    gpio_set_level((gpio_num_t)MUX_S0, (ch >> 0) & 1);
    gpio_set_level((gpio_num_t)MUX_S1, (ch >> 1) & 1);
    gpio_set_level((gpio_num_t)MUX_S2, (ch >> 2) & 1);
    gpio_set_level((gpio_num_t)MUX_S3, (ch >> 3) & 1);
}

// Lee los 16 sensores secuencialmente a través del MUX
void read_sensors(int *readings)
{
    for(int i = 0; i < NUM_SENSORS; i++)
    {
        set_mux(i);
        esp_rom_delay_us(SENSOR_DELAY_US);  // Esperar a que el MUX estabilice
        adc_oneshot_read(adc_handle, ADC_CHANNEL_4, &readings[i]);
    }
}

// =====================================================
// CALIBRACIÓN
// =====================================================
// Hace girar el robot 5 s a cada lado para que la barra
// de sensores pase por línea blanca y negra
// Registra el mínimo y máximo de cada sensor para
// normalizar las lecturas después.
/* void calibrate(void)
{
    // Inicializar min al máximo posible y max al mínimo posible
    for(int i = 0; i < NUM_SENSORS; i++)
    {
        minValues[i] = 4095;
        maxValues[i] = 0;
    }

    int buf[NUM_SENSORS];
    int64_t t;

    // --- Giro a la izquierda durante 5 s ---
    t = esp_timer_get_time();
    while(esp_timer_get_time() - t < 5000000)
    {
        process_ir();

        // Permitir abortar calibración con STOP
        if(robot_state == STATE_STOPPED)
            return;

        set_motor_speeds(-CAL_SPEED, CAL_SPEED);
        read_sensors(buf);

        for(int i = 0; i < NUM_SENSORS; i++)
        {
            if(buf[i] < minValues[i]) minValues[i] = buf[i];
            if(buf[i] > maxValues[i]) maxValues[i] = buf[i];
        }

        vTaskDelay(1);
    }

    // --- Giro a la derecha durante 5 s ---
    t = esp_timer_get_time();
    while(esp_timer_get_time() - t < 5000000)
    {
        process_ir();

        if(robot_state == STATE_STOPPED)
            return;

        set_motor_speeds(CAL_SPEED, -CAL_SPEED);
        read_sensors(buf);

        for(int i = 0; i < NUM_SENSORS; i++)
        {
            if(buf[i] < minValues[i]) minValues[i] = buf[i];
            if(buf[i] > maxValues[i]) maxValues[i] = buf[i];
        }

        vTaskDelay(1);
    }

    set_motor_speeds(0, 0);

    //fan_test_startup();  // Descomentar para probar el ventilador al finalizar calibración
}*/

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
        for(int i = 0; i < NUM_SENSORS; i++)
        {
            minValues[i] = 4095;
            maxValues[i] = 0;
        }
    }

    // Asegurarnos de que el robot NO se mueva durante la captura
    set_motor_speeds(0, 0);

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

        // Actualizar min y max dinámicamente con las lecturas de la superficie
        for(int i = 0; i < NUM_SENSORS; i++)
        {
            if(buf[i] < minValues[i]) minValues[i] = buf[i];
            if(buf[i] > maxValues[i]) maxValues[i] = buf[i];
        }

        vTaskDelay(1);
    }

    // Incrementar el contador de pasos de calibración
    calibration_step++;

    // Si ya completamos ambas capturas (negro y blanco), reiniciamos el contador a 0
    if (calibration_step >= 2)
    {
        calibration_step = 0;
        
        // fan_test_startup(); 
    }
}

// =====================================================
// FAN TEST — Secuencia de prueba del ventilador
// =====================================================
// Sube la velocidad gradualmente para verificar el ESC
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
// POSICIÓN — Centroide ponderado de la línea
// =====================================================
// Calcula la posición de la línea en el rango [0, 15000],
// donde 0 = extremo izquierdo y 15000 = extremo derecho.
// Usa el umbral individual por sensor (promedio min/max).
// Retorna true si la línea es visible, false si no.
bool get_line_pos(int *readings, float *out_position)
{
    long weighted_sum = 0;
    long total_sum    = 0;
    bool line_found   = false;

    for(int i = 0; i < NUM_SENSORS; i++)
    {
        // Umbral individual: punto medio entre min y max calibrados
        int threshold = (minValues[i] + maxValues[i]) / 2;

        if(readings[i] > threshold)
        {
            line_found = true;

            // Peso: qué tan por encima del mínimo está la lectura
            int weight = readings[i] - minValues[i];
            if(weight < 0) weight = 0;

            weighted_sum += (long)weight * (i * 1000);
            total_sum    += weight;
        }
    }

    if(!line_found || total_sum == 0)
    {
        // Línea no detectada: regresar última posición conocida
        *out_position = last_pos;
        return false;
    }

    last_pos      = (float)weighted_sum / total_sum;
    *out_position = last_pos;
    return true;
}

// =====================================================
// PID — Controlador proporcional-integral-derivativo
// =====================================================
// Calcula el ajuste de velocidad basado en el error actual.
// Usa dt real (en segundos) para términos I y D correctos.
float calculate_pid(float error)
{
    int64_t current_time = esp_timer_get_time();
    float dt = (current_time - last_time) / 1000000.0f;

    // Evitar dt de cero o negativo en el primer ciclo
    if(dt <= 0.0f) dt = 0.001f;

    last_time = current_time;

    P  = error;
    I += error * dt;

    // Limitar la integral para evitar wind-up
    if(I >  40000) I =  40000;
    if(I < -40000) I = -40000;

    D = (error - previous_error) / dt;

    previous_error = error;

    return (Kp * P) + (Ki * I) + (Kd * D);
}