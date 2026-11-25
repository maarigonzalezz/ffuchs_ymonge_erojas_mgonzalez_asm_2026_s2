# Contexto para Evaluación de Experimentos y Extras (Mariana)

Este documento resume exactamente lo que ya quedó disponible en `develop` dentro de `proyecto_2/matlab/` para implementar:
* `proyecto_2/matlab/evaluacion_experimentos.m` (Etapas 5 y 6)
* `proyecto_2/matlab/evaluacion_extras.m` (Extras E2, E3, E4 y E5)

**Regla de reparto:** trabajar únicamente en esos dos scripts y exportar figuras en `proyecto_2/paper/figuras/` (300 dpi). No modificar `+frenado/*`, `generar_datos_ejemplo.m` ni `caracterizacion_planta.m` sin coordinar.

---

## 1. Lector y validador dual ya listo en `+frenado/`

Todas las utilidades de carga y validación ya soportan **ambos formatos CSV** automáticamente (se distinguen por la presencia de la columna `ctrl_mode`):

1. **Formato `step` (9 columnas — lazo abierto):**
   ```csv
   t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses
   ```
   * Fases: `PRE` / `BRAKE` (compatibilidad con `STEP`).
2. **Formato `runpid` (15 columnas — lazo cerrado PID o lazo abierto con referencia):**
   ```csv
   t_ms,phase,ctrl_mode,motor_pwm,w_ref,w_meas,w_raw,error,u_brake,p_term,i_term,d_term,servo_deg,pulses,flags
   ```
   * Fases: `PRE` / `PID` (o `BRAKE`).
   * `ctrl_mode`: `"OPEN"` o `"PID"`.
   * Todo en **rad/s**, convención de error `error = w_meas - w_ref`, `u_brake` en `[0, 1]`.
   * `flags` (entero sin signo, máscara de bits):
     * Bit 0 (`1`): `u = 0` saturado
     * Bit 1 (`2`): `u = 1` saturado
     * Bit 2 (`4`): antibloqueo simplificado activo (**E3**)
     * Bit 3 (`8`): región baja de *gain scheduling* (**E4**)
     * Bit 4 (`16`): *anti-windup* actuando

### Cómo cargar cualquier CSV desde MATLAB

```matlab
% Recomendado para evaluacion de control: useFiltered = true (usa w_meas en runpid / rad_s en step)
ensayo = frenado.cargarEnsayoCSV(rutaArchivo, true);
```

Campos devueltos en `ensayo`:
* **Comunes (`step` y `runpid`):**
  * `archivo`, `formato` (`"step"` o `"runpid"`), `ctrl_mode` (`"OPEN"` o `"PID"`)
  * `t_full`, `w_full`, `u_full`, `servo_full`, `pulses_full` (tiempo en **s**, centrado en `t = 0` justo en el corte de motor)
  * `t_val`, `w_val`, `u_val` (región válida sobre `omegaMin`, por defecto `4 rad/s`)
  * `w0` ($\text{rad/s}$ iniciales promedio en `PRE`), `u0`, `Ts_prom`, `jitter_max_s`, `hueco_max_s`, `variacion_inicial`
* **Adicionales cuando `ensayo.formato == "runpid"`:**
  * `w_ref_full`, `w_meas_full`, `w_raw_full`, `error_full`, `p_full`, `i_full`, `d_full`, `flags_full`
  * `w_ref_val`, `w_meas_val`, `w_raw_val`, `error_val`, `p_val`, `i_val`, `d_val`, `servo_val`, `pulses_val`, `flags_val`
  * `banderas` (booleanos ya decodificados sobre la región válida):
    * `ensayo.banderas.sat_u0`
    * `ensayo.banderas.sat_u1`
    * `ensayo.banderas.abs_activo`
    * `ensayo.banderas.gs_baja`
    * `ensayo.banderas.antiwindup`

---

## 2. Acuerdo oficial para `w_ref`, IAE y RMSE en lazo abierto (`step` 9 cols vs. `runpid`)

Cuando una corrida de lazo abierto venga en formato **`step` (9 columnas, sin columna `w_ref`)**, su referencia justa de comparación es la **misma rampa decreciente del PID para esa condición (`v1` o `v2`)**, iniciando en el `w0` real de la corrida:

$$\omega_{\text{ref}}(t) = \max\!\big(\omega_{\text{stop}},\ \omega_0 - r_{\text{ref}}\cdot t\big), \quad t \ge 0$$

Para evitar cualquier discrepancia entre scripts, ya existe el helper oficial en `+frenado/`:

```matlab
function [w_ref, err] = obtenerReferenciaYError(ensayo, r_ref, w_stop)
    if ensayo.formato == "runpid"
        w_ref = ensayo.w_ref_val;
        err   = ensayo.error_val; % ya coincide con w_meas_val - w_ref_val
    else
        % Formato step (9 columnas): reconstruye la rampa oficial de esa condicion
        w_ref = frenado.referenciaRampa(ensayo.t_val, ensayo.w0, r_ref, w_stop);
        err   = ensayo.w_val - w_ref;
    end
end
```

* En **datos de ejemplo**, los pares `(r_ref, w_stop)` de `v1` y `v2` están guardados en:
  `proyecto_2/matlab/datos_ejemplo/finales/condiciones_ejemplo.json`
* En **datos reales**, esos mismos pares `(r_ref, w_stop)` saldrán de `diseno_pid.m` (`diseno_pid_resultado.mat` / mismo JSON de condiciones en `datos/finales/`).

---

## 3. CSV de ejemplo listos para desarrollar sin esperar la maqueta

Ubicación: `proyecto_2/matlab/datos_ejemplo/` (generados determinísticamente con `generar_datos_ejemplo.m`, todos validados por `frenado.validarTabla`):

* **Lazo abierto de identificación (formato `step`, 9 cols):**
  * `coastdown_1.csv`, `step_u04_1.csv`, `step_u07_1.csv`
* **Etapa 5 — Ajuste de ganancias (`datos_ejemplo/ajuste/`, formato `runpid`, 15 cols):**
  * `ajuste_P.csv` (solo proporcional)
  * `ajuste_PI.csv` (proporcional + integral)
  * `ajuste_PID.csv` (PID completo con derivada filtrada)
* **Etapa 6 — Experimentos finales (`datos_ejemplo/finales/`, formato `runpid`, 12 CSV + JSON):**
  * `condiciones_ejemplo.json` (`v1`: `r_ref = 15.5 rad/s^2`, `w_stop = 4 rad/s`, `rpm0 = 320`; `v2`: `r_ref = 20.5 rad/s^2`, `w_stop = 4 rad/s`, `rpm0 = 450`)
  * Velocidad 1 (`v1`): `v1_open_rep1.csv`, `v1_open_rep2.csv`, `v1_open_rep3.csv`, `v1_pid_rep1.csv`, `v1_pid_rep2.csv`, `v1_pid_rep3.csv`
  * Velocidad 2 (`v2`): `v2_open_rep1.csv`, `v2_open_rep2.csv`, `v2_open_rep3.csv`, `v2_pid_rep1.csv`, `v2_pid_rep2.csv`, `v2_pid_rep3.csv`
* **Extras (`datos_ejemplo/extras/`, formato `runpid`, 9 CSV):**
  * **E2 (muestreo / discretización):** `e2_ts20_euler.csv`, `e2_ts50_euler.csv`, `e2_ts20_tustin.csv`
  * **E3 (antibloqueo simplificado):** `e3_abs_off.csv`, `e3_abs_on.csv` *(tiene `banderas.abs_activo` encendido durante la modulación)*
  * **E4 (*gain scheduling*):** `e4_gs_off.csv`, `e4_gs_on.csv` *(tiene `banderas.gs_baja` encendido en la región baja)*
  * **E5 (rechazo de perturbaciones — aumento de inercia $\Delta J$):** `e5_pert_open.csv`, `e5_pert_pid.csv`

---

## 4. Métricas y salidas esperadas en `evaluacion_experimentos.m` y `evaluacion_extras.m`

1. **`evaluacion_experimentos.m`:**
   * Opción booleana `UsarDatosEjemplo` (`true` lee `datos_ejemplo/`, `false` lee `datos/`).
   * Soportar tanto ensayos abiertos en formato `runpid` (`ctrl_mode = OPEN`) como en formato `step` (9 cols, usando `frenado.referenciaRampa`).
   * Calcular por corrida y resumir como **media ± desviación estándar** por condición (`v1_open`, `v1_pid`, `v2_open`, `v2_pid`):
     * **IAE:** `trapz(t, abs(err))` ($\text{rad}$)
     * **RMSE:** `sqrt(mean(err.^2))` ($\text{rad/s}$ y equivalente en RPM)
     * **Tiempo de frenado:** `t_val(end)` ($\text{s}$, hasta cruzar `omegaMin`)
     * **Oscilación de control:** variación total `sum(abs(diff(u_val)))`
     * **% de tiempo en saturación:** porcentaje de muestras con `u <= 1e-6` o `u >= 1 - 1e-6` (o `sat_u0 | sat_u1`)
     * **Variabilidad entre repeticiones:** desviación estándar del tiempo de frenado y del RMSE entre `rep1..rep3`
   * Procesar `ajuste/` (`P`, `PI`, `PID`) y `finales/` (12 corridas).
   * Exportar figuras en **300 dpi** a `proyecto_2/paper/figuras/` e imprimir tabla resumen en consola y en formato **LaTeX** (`\begin{tabular}...\end{tabular}`).
2. **`evaluacion_extras.m`:**
   * **E2:** comparación `Ts = 20 ms` vs `50 ms` y `Euler` vs `Tustin` (RMSE, IAE, oscilación + polos en plano $Z$ usando `modelo_planta_identificado.mat`).
   * **E3:** `abs off` vs `abs on` comparando picos de desaceleración $|\dot{\omega}|$ y señalizando los instantes donde `banderas.abs_activo == true` *(denominar siempre "antibloqueo simplificado", nunca "medición de deslizamiento")*.
   * **E4:** `gs off` vs `gs on` (RMSE, IAE y transición de región `banderas.gs_baja`).
   * **E5:** `pert open` vs `pert pid` (RMSE, IAE y recuperación ante perturbación).

> **Tip visual para MATLAB R2025a+/R2026b en modo oscuro:** al crear figuras para el paper, aplicar `try theme(fig, "light"); catch; end` con `"Color", "w"` para garantizar fondo blanco y texto negro al exportar PNG en 300 dpi.
