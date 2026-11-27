# asm_2026_s2_PII — Proyecto Grupal 2 (CE 1110 Análisis de Señales Mixtas, ITCR)

Sistema de control digital de frenado tipo **ABS didáctico** con **ESP32 clásica**, motor DC **JGB37-520** + driver **XY-MOS**, actuador de freno por servomotor **MG996**, sensor óptico **IR AD33838** (`P = 32` pulsos/rev) y pipeline modular **Python + MATLAB** para caracterización, diseño PID en Laplace ($s$) y plano $Z$, ajuste experimental y evaluación final.

---

## 1. Estructura modular del repositorio

```text
├── README.md
├── tarea2/                                       # Código base de la Tarea 2 (lazo abierto)
└── proyecto_2/
    ├── firmware/
    │   └── freno_pid_esp32/
    │       └── freno_pid_esp32.ino               # Firmware ESP32: estimador por ventana, step, PID (Ts = 20 ms) y hooks extras
    └── matlab/
        ├── +frenado/                             # Paquete compartido MATLAB (validación, identificación FOPDT, rampas)
        │   ├── cargarEnsayoCSV.m
        │   ├── evaluarCoastdown.m
        │   ├── evaluarDeltaW.m
        │   ├── identificar.m
        │   ├── leerCSV.m
        │   ├── referenciaRampa.m
        │   ├── respuestaEscalon.m
        │   ├── tiemposCaptura.m
        │   ├── validarComandoTabla.m
        │   └── validarTabla.m
        ├── captura_serial.py                     # CLI base de captura ESP32 -> CSV validado (pyserial)
        ├── caracterizacion_parametros/           # Etapa 3: caracterización de planta con promediado de ensamble
        │   ├── caracterizacion.py
        │   ├── caracterizar_planta.m
        │   ├── modelo_planta_identificado.mat
        │   ├── csv/
        │   └── graficas/
        ├── diseno-controlador-digital/           # Etapa 4: diseño SIMC en Laplace (s) y plano Z + simulación
        │   ├── diseno.py
        │   ├── disenar_pid.m
        │   ├── condiciones_diseno.json
        │   ├── diseno_pid_resultado.mat
        │   └── graficas/
        ├── ajuste-controlador/                   # Etapa 5: estudio experimental P / PI / PID (N repeticiones)
        │   ├── ajuste.py
        │   ├── analizar_ajuste.m
        │   ├── ajuste_resultado.mat
        │   ├── resumen_ajuste.csv
        │   ├── tabla_ajuste.tex
        │   ├── csv/
        │   └── graficas/
        └── experimentos-finales/                 # Etapa 6: comparativa final OPEN vs PID en V1 y V2
            ├── experimentos.py
            ├── analizar_finales.m
            ├── finales_resultado.mat
            ├── resumen_finales.csv
            ├── tabla_finales.tex
            ├── csv/
            └── graficas/
```

---

## 2. Hardware y pines (ESP32 clásica)

| Subsistema | Componente | Pin ESP32 | Detalle clave |
| :--- | :--- | :--- | :--- |
| **Motor DC + Driver** | JGB37-520 ($12\text{ V}$, $600\text{ RPM}$) + XY-MOS + 1N4006 | `GPIO25` | PWM $20\text{ kHz}$, 8 bits (`0..255`), fuente $12\text{ V}$ |
| **Freno (actuador)** | Servomotor MG996 ($180^\circ$) | `GPIO26` | PWM $50\text{ Hz}$, 14 bits; calibración `47° / 52° / 64°`, fuente $6\text{ V}$ |
| **Sensor de velocidad** | Sensor óptico reflectivo IR `AD33838` | `GPIO27` | Alimentado a `3V3`; disco $\varnothing 130\text{ mm}$ ($P = 32$ pulsos/rev) |
| **Botón de inicio** | Pulsador de arranque manual | `GPIO32` | `INPUT_PULLDOWN` (activo en `HIGH`) |
| **Botón de parada** | Pulsador de parada de emergencia | `GPIO33` | `INPUT_PULLDOWN` (corta `PWM = 0` y libera freno) |
| **Referencia** | Tierra común (`GND`) | `GND` | Unión común ESP32, driver XY-MOS, servo MG996 y sensor IR |

---

## 3. Flujo de trabajo por etapas

> **Requisitos:** Python 3.10+ (`pip install pyserial`) y MATLAB con **Control System Toolbox** en el `PATH`. Cierra el Monitor Serial de Arduino IDE antes de ejecutar los scripts.

### Etapa 3 — Caracterización de la planta (`caracterizacion_parametros/`)
Captura $N$ tríos (`coast-down`, escalón ID `u=0.50` y validación cruzada `u=0.85`), aplica promediado de ensamble y actualiza `modelo_planta_identificado.mat` + 4 gráficas PNG a 300 dpi:
```powershell
cd proyecto_2/matlab/caracterizacion_parametros
python -B caracterizacion.py 3 --port COM3
# Solo recalcular MATLAB con los CSV actuales:
python -B caracterizacion.py --solo-matlab --sin-pid
```

### Etapa 4 — Diseño del controlador digital (`diseno-controlador-digital/`)
Calcula ganancias iniciales SIMC, polos/ceros/ROC en **Laplace ($s$)** con Padé 1, verificación discreta en **plano $Z$** ($T_s = 20\;\text{ms}$, retardo exacto $z^{-d}$), región realizable de rampas y simulación no lineal con *anti-windup*:
```powershell
cd proyecto_2/matlab/diseno-controlador-digital
python -B diseno.py
```

### Etapa 5 — Ajuste experimental P / PI / PID (`ajuste-controlador/`)
Lee automáticamente las ganancias y rampa desde `diseno-controlador-digital/condiciones_diseno.json`, ejecuta $N$ repeticiones por condición en la ESP32, calcula métricas (`RMSE`, `IAE`, `|e|_max`, `TV_u`, saturación y $t_f$) y genera 6 gráficas (comparativa + vista limpia del ganador + Laplace del ganador) junto a `tabla_ajuste.tex` y `resumen_ajuste.csv`:
```powershell
cd proyecto_2/matlab/ajuste-controlador
python -B ajuste.py 3 --port COM3
# Probar una variante adicional sin borrar P / PI / PID previos:
python -B ajuste.py 3 --solo PID_v3 --kp 0.0354 --ki 0.025 --kd 0.0006 --tf 0.10
# Solo recalcular en MATLAB:
python -B ajuste.py --solo-matlab
```

### Etapa 6 — Experimentos finales OPEN vs PID (`experimentos-finales/`)
Lee automáticamente el controlador ganador de `ajuste-controlador/resumen_ajuste.csv`, ejecuta $N$ repeticiones por condición (`V1_OPEN`, `V1_PID`, `V2_OPEN`, `V2_PID`) y exporta `tabla_finales.tex`, `resumen_finales.csv` y 4 gráficas comparativas a 300 dpi:
```powershell
cd proyecto_2/matlab/experimentos-finales
python -B experimentos.py 3 --port COM3
# Solo recalcular en MATLAB:
python -B experimentos.py --solo-matlab
```

---

## 4. Protocolo serial y formatos CSV (ESP32 ↔ PC)

* **Comandos ESP32 (`115200` baudios, fin de línea `\n`):**
  * `step <pwm_ini> <u_freno> <spinup_ms> <pre_ms> <dur_ms>` — ensayo a lazo abierto (9 columnas).
  * `runpid <pwm_ini> <spinup_ms> <pre_ms> <dur_ms>` — ensayo a lazo cerrado con PID en `rad/s` (15 columnas).
  * `pid <kp> <ki> <kd> <tf>` y `ref <rate_rad_s2> <w_stop_rad_s>` — configuración en caliente de ganancias y rampa.
  * `cfg p <n> | mp <us> | win <ms>` — configuración del sensor IR en caliente.
  * `x` — parada de emergencia inmediata (`motor_pwm = 0`, freno liberado).
* **Contratos de columnas CSV:**
  * **`step` (9 cols):** `t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses`
  * **`runpid` (15 cols):** `t_ms,phase,ctrl_mode,motor_pwm,w_ref,w_meas,w_raw,error,u_brake,p_term,i_term,d_term,servo_deg,pulses,flags`
  * **`flags` (máscara de bits):** bit `0` (`1`) `u = 0`; bit `1` (`2`) `u = 1`; bit `2` (`4`) ABS activo; bit `3` (`8`) gain scheduling; bit `4` (`16`) *anti-windup* por *clamping*.
