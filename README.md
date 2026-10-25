# CELERINO · Robot Seguidor de Línea de Alta Velocidad

**Robot Seguidor de Línea con Control PID, Mux IR de 16 Canales, Turbina Succión y ESP32-C3**  
Proyecto de Robótica / Software Embebido — Instituto Tecnológico de Chihuahua (ITCH) · 2026

## Descripción

**CELERINO** es un robot seguidor de línea de alto rendimiento diseñado para competencias de robótica de velocidad. Su arquitectura combina un microcontrolador ESP32-C3 SuperMini, un sistema de adquisición de datos multiplexado de 16 sensores infrarrojos analógicos, control de velocidad de motores mediante puentes H de alta corriente BTN9960 con protección térmica y sobrecorriente, y un sistema de succión aerodinámica mediante turbina accionada por ESC para maximizar el agarre en curvas.

El firmware está programado en C sobre el framework **ESP-IDF**, utilizando una máquina de estados finitos gestionada mediante interrupciones de un receptor infrarrojo para arranque/paro, cálculo de posición por centroide local (promedio ponderado) y control de lazo cerrado PID en tiempo real.

---

## Características Principales

* **Control de Procesamiento:** ESP32-C3 SuperMini ejecutando firmware nativo en ESP-IDF con temporización por hardware (`esp_timer`) y lectura ADC de 12 bits.
* **Barra de Sensores Modular:** 16 fototransistores/LEDs IR conmutados por un multiplexor analógico 74HC4067 (16:1), muestreados en microsegundos y procesados mediante estimación de centroide local.
* **Etapa de Potencia:** 4 medios puentes H BTN9960 que forman puentes H completos para ambos motores, con soporte de sobrevoltaje, respuesta lineal PWM (5 kHz) y disipación de sobrecorriente al plano de tierra.
* **Sistema de Succión (Efecto Suelo):** Turbina impulsada por motor brushless y controlada vía ESC (PWM 50 Hz) con rampa de aceleración automática conforme a reglamento de competencia.
* **Control Remoto Infrarrojo:** Decodificación de protocolo NEC en tiempo real vía interrupciones GPIO para comandos de arranque, calibración y paro de emergencia.
* **Transmisión Mecánica Personalizada:** Engranajes de reducción 5:1 impresos en PETG para alta durabilidad e intercambiabilidad.

---

## Arquitectura del Sistema

```
                      ┌───────────────────────────┐
                      │ Control Remoto IR (NEC)  │
                      └─────────────┬─────────────┘
                                    │ Interrupción GPIO0
                                    ▼
┌──────────────────┐  16:1 MUX   ┌───────────────────┐  PWM (5 kHz)  ┌──────────────────┐
│ Barra de 16      │────────────>│ ESP32-C3          │──────────────>│ Puentes H        │───> Motores DC
│ Sensores IR      │  ADC GPIO4  │ SuperMini         │               │ BTN9960          │     (Tracción)
└──────────────────┘             │                   │               └──────────────────┘
                                 │ (ESP-IDF / C)     │  PWM (50 Hz)  ┌──────────────────┐
                                 │ Lazo PID 2 ms     │──────────────>│ ESC / Turbina    │───> Succión
                                 └───────────────────┘               │ (Fan)            │     (Efecto Suelo)
                                                                     └──────────────────┘
```

---

## Hardware y Componentes

| Subsistema | Componente | Notas |
| --- | --- | --- |
| **Controlador** | ESP32-C3 SuperMini | RISC-V 32-bit, ADC de alta velocidad |
| **Sensores Lineales** | Arreglo de 16 Sensores IR + MUX 74HC4067 | Muestreo multiplexado de 16 canales a 12 bits |
| **Drivers de Motor** | 4x BTN9960 (Medios Puentes H) | Control bidireccional, protección térmica y sobrecorriente |
| **Turbina / Succión** | Motor Brushless + ESC | Genera presión negativa para adherencia en curvas |
| **Receptor IR** | Receptor Infrarrojo 38 kHz (GPIO 1) | Lectura de comandos ASCII vía protocolo NEC |
| **Transmisión** | Engranajes PETG (Reducción 5:1) | Impresión 3D, relación de engranes intercambiable |
| **Chasis** | Diseño en SolidWorks / PCB Modular | Placa principal y barra de sensores separadas |

---

## Mapeo de Pines (ESP32-C3 SuperMini)

| Periférico | Pin GPIO | Función |
| --- | --- | --- |
| **MUX S0** | GPIO 3 | Selector bit 0 del multiplexor 74HC4067 |
| **MUX S1** | GPIO 2 | Selector bit 1 del multiplexor 74HC4067 |
| **MUX S2** | GPIO 10 | Selector bit 2 del multiplexor 74HC4067 |
| **MUX S3** | GPIO 7 | Selector bit 3 del multiplexor 74HC4067 |
| **ADC MUX** | GPIO 4 (ADC1_CH4) | Salida analógica del multiplexor |
| **Motor Izq IN1 / IN2** | GPIO 21 / GPIO 20 | PWM LEDC Canal 0 / 1 (5 kHz) |
| **Motor Der IN1 / IN2** | GPIO 5 / GPIO 6 | PWM LEDC Canal 2 / 3 (5 kHz) |
| **ESC Turbina** | GPIO 8 | PWM LEDC Canal 4 (50 Hz / Servo Protocol) |
| **LED Indicador** | GPIO 9 | Salida Digital (Indicador blanco) |
| **Receptor IR** | GPIO 1 | Entrada Digital con Interrupción Negedge |

---

## Firmware y Algoritmo de Control

El software se ejecuta en un lazo de control periódico de **2 ms** estructurado en una máquina de estados finitos:

```
                  ┌──────────────┐
                  │  STATE_IDLE  │
                  └──────┬───────┘
                         │
        ┌────────────────┴────────────────┐
        ▼                                 ▼
┌───────────────────────┐       ┌───────────────────┐
│   STATE_CALIBRATING   │       │   STATE_RUNNING   │
│ Autocalibración 5 s   │       │ Lazo PID + Motors │
└───────────────────────┘       └─────────┬─────────┘
                                          │ IR_CMD_STOP
                                          ▼
                                ┌───────────────────┐
                                │   STATE_STOPPED   │
                                └───────────────────┘
```

### Algoritmo del Estimador de Posición
1. **Normalización por Sensor:** Scale individual de cada sensor ($0 \dots 1000$) utilizando los valores mínimo y máximo capturados durante la fase de calibración.
2. **Filtrado de Ruido:** Sustracción de piso de ruido (`NOISE_FLOOR`) para eliminar lecturas parásitas fuera de la línea.
3. **Centroide Local (Blobing):** Identificación del sensor pico e inclusión única de los sensores contiguos activos ($\pm \text{BLOB\_HALF}$), previniendo saltos por reflejos en la pista.
4. **Filtro IIR y Límite de Salto:** Suavizado de posición para evitar discontinuidades abruptas.

---

## Comandos Infrarrojos (Control Remoto)

| Comando | Código Hex | Acción |
| --- | --- | --- |
| **CALIBRAR** | `0x45` | Entra a modo calibración (5 segundos de muestreo min/máx por paso) |
| **ARRANCAR** | `0x46` | Inicia secuencia de seguridad (5 s delay + rampa de turbina + arranque de lazo PID) |
| **PARAR** | `0x47` | Paro de emergencia de motores y turbina |

---

## Estructura del Repositorio

```
Celerino-LineFollower/
├── main/
│   ├── main.c              # Código principal en C (ESP-IDF)
│   └── CMakeLists.txt      # Configuración de compilación del componente principal
├── CMakeLists.txt          # Proyecto CMake
└── README.md               # Documentación general del repositorio
```

---

## Requisitos de Software y Compilación

| Herramienta | Versión Recomendada |
| --- | --- |
| **ESP-IDF** | v5.x |
| **Toolchain RISC-V** | Incluido en ESP-IDF |
| **EasyEDA** | Para diseño de PCB / Gerber |
| **SolidWorks** | Para visualización y modificación de partes 3D |

### Compilación y Carga vía ESP-IDF

```bash
# Configurar el objetivo para el ESP32-C3
idf.py set-target esp32c3

# Compilar el proyecto
idf.py build

# Flashear el microcontrolador y abrir monitor serie
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## Equipo de Desarrollo

**Desarrollo de Firmware, Electrónica y Mecánica:**

* Ana Paola Cardona Valenzuela
* Emiliano Pérez Dyck
* Luis Adrián Anchondo Carreón

**Institución:** Instituto Tecnológico de Chihuahua (TecNM / ITCH)  
**Departamento:** Eléctrica y Electrónica  
**Especialidad:** IA en Sistemas Embebidos
**Año:** 2026

---

<p align="center">
  Instituto Tecnológico de Chihuahua · Chihuahua, México · 2026
</p>