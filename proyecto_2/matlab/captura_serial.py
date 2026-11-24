"""Captura ESP32 a CSV validado. La dependencia pyserial solo se carga al conectar.

Ejemplo: python3 captura_serial.py --port /dev/cu.usbserial-0001 \
  --cmd "step 180 0.4 3000 400 6000" --out step_u04_1.csv
Lazo cerrado: --setup "pid 0.02 0 0 0.05" --setup "ref 40 5" --cmd "runpid 200 3000 1000 8000" \
  --out runpid_1.csv (CSV de 15 columnas). Al abrir el puerto la ESP32 se reinicia y las ganancias
y la rampa vuelven a sus valores por defecto (kp=ki=kd=0: no frena), por eso --setup las reenvia
justo antes del comando del ensayo.
Un CSV solo se publica tras END_CSV, RESULT satisfactorio y validacion de filas.
En errores se conserva *.partial.csv y siempre se intenta enviar x.
"""
import argparse
import csv
from datetime import datetime
import json
import math
import os
from pathlib import Path
import time

HEADER = "t_ms,phase,motor_pwm,u_brake,servo_deg,rpm_raw,rpm_filt,rad_s,pulses"
HEADER_PID = ("t_ms,phase,ctrl_mode,motor_pwm,w_ref,w_meas,w_raw,error,u_brake,"
              "p_term,i_term,d_term,servo_deg,pulses,flags")
SUCCESS_RESULTS = {"parada_detectada", "duracion_maxima"}
SUCCESS_RESULTS_PID = SUCCESS_RESULTS | {"w_final_alcanzada"}
FLAG_U_MIN, FLAG_U_MAX, FLAG_ABS = 1, 2, 4   # bits de la columna flags (ver firmware)


class CaptureError(RuntimeError):
    """Captura incompleta, rechazada o inconsistente con el firmware."""


SETUP_COMMANDS = {"pid", "ref", "cfg"}


def check_setup(setup):
    """Comandos de configuracion permitidos antes del ensayo; sin letra x (el firmware la toma como parada)."""
    cleaned = []
    for command in setup:
        command = command.strip()
        parts = command.split()
        if not parts or parts[0].lower() not in SETUP_COMMANDS:
            raise ValueError("--setup solo admite comandos pid, ref o cfg.")
        if not command.isascii() or not command.isprintable() or "x" in command.lower():
            raise ValueError("Comando de configuracion invalido (ASCII imprimible y sin la letra x).")
        cleaned.append(command)
    return cleaned


def is_pid_command(command):
    parts = command.split()
    return bool(parts) and parts[0].lower() == "runpid"


def capture_timeouts(command, minimum=20.0):
    if not math.isfinite(minimum) or minimum <= 0:
        raise ValueError("El timeout debe ser finito y positivo.")
    if not command.strip():
        return minimum, minimum
    parts = command.split()
    if parts[0].lower() == "runpid":
        if len(parts) != 5:
            raise ValueError("Se esperaba: runpid <pwm> <spinup_ms> <pre_ms> <dur_ms>.")
        if any(not parts[i].isascii() or not parts[i].isdigit() for i in range(1, 5)):
            raise ValueError("PWM y tiempos deben ser enteros sin signo.")
        pwm, spin, pre, duration = (int(parts[i]) for i in range(1, 5))
        if (not 1 <= pwm <= 255 or not 20 <= spin <= 600000 or not 0 <= pre <= 600000
                or not 20 <= duration <= 600000):
            raise ValueError("Parametros fuera de los limites del firmware.")
        return max(minimum, spin / 1000 + 5), max(minimum, (pre + duration) / 1000 + 5)
    if len(parts) != 6 or parts[0].lower() != "step":
        raise ValueError("Se esperaba: step <pwm> <u> <spinup_ms> <pre_ms> <dur_ms>.")
    if any(not parts[i].isascii() or not parts[i].isdigit() for i in (1, 3, 4, 5)):
        raise ValueError("PWM y tiempos deben ser enteros sin signo.")
    pwm, spin, pre, duration = (int(parts[i]) for i in (1, 3, 4, 5))
    brake = float(parts[2])
    if (not math.isfinite(brake) or not 0 <= brake <= 1 or not 1 <= pwm <= 255
            or not 20 <= spin <= 600000 or not 0 <= pre <= 600000
            or not 20 <= duration <= 600000):
        raise ValueError("Parametros fuera de los limites del firmware.")
    return max(minimum, spin / 1000 + 5), max(minimum, (pre + duration) / 1000 + 5)


def validate_csv(path, command=""):
    """Valida sin rellenar silenciosamente columnas, valores o muestras perdidas."""
    expected = command.split() if command.strip() else None
    with Path(path).open(newline="", encoding="ascii") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames == HEADER_PID.split(","):
            return validate_pid_rows(reader, command)
        if reader.fieldnames != HEADER.split(","):
            raise CaptureError("Encabezado CSV incompatible.")
        if is_pid_command(command):
            raise CaptureError("El CSV no coincide con el comando solicitado.")
        previous_time = -math.inf
        previous_pulses = None
        braking = False
        brake_level = None
        rows = post_rows = 0
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise CaptureError("Fila con numero incorrecto de columnas.")
            try:
                values = {key: float(value) for key, value in row.items() if key != "phase"}
            except ValueError as exc:
                raise CaptureError("Columna numerica invalida.") from exc
            if not all(math.isfinite(value) for value in values.values()):
                raise CaptureError("CSV con NaN o infinito.")
            t, pwm, u, servo = (values[k] for k in ("t_ms", "motor_pwm", "u_brake", "servo_deg"))
            phase = row["phase"].upper()
            if t <= previous_time:
                raise CaptureError("Tiempo duplicado o no creciente.")
            if pwm != int(pwm) or not 0 <= pwm <= 255 or not 0 <= u <= 1 or not 0 <= servo <= 180:
                raise CaptureError("Actuador fuera de rango.")
            if expected and ((phase == "PRE" and pwm != int(expected[1]))
                             or (phase in {"BRAKE", "STEP"} and abs(u-float(expected[2])) > 1e-6)):
                raise CaptureError("El CSV no coincide con el comando solicitado.")
            if phase == "PRE":
                if braking or t >= 0 or pwm <= 0 or u != 0:
                    raise CaptureError("Fase PRE inconsistente.")
            elif phase in {"BRAKE", "STEP"}:
                if t < 0 or pwm != 0:
                    raise CaptureError("Fase de frenado inconsistente.")
                if brake_level is not None and abs(u-brake_level) > 1e-6:
                    raise CaptureError("La accion de frenado no es constante.")
                brake_level = u
                braking = True
                post_rows += 1
            else:
                raise CaptureError("Fase desconocida.")
            if any(values[k] < 0 for k in ("rpm_raw", "rpm_filt", "rad_s")):
                raise CaptureError("Velocidad negativa.")
            if abs(values["rad_s"] - values["rpm_filt"] * math.pi / 30) > .02:
                raise CaptureError("rad_s y rpm_filt no son coherentes.")
            pulses = values["pulses"]
            if pulses != int(pulses) or not 0 <= pulses <= 4294967295:
                raise CaptureError("Conteo de pulsos invalido.")
            if previous_pulses is not None and pulses < previous_pulses:
                if previous_pulses - pulses < 2147483648:
                    raise CaptureError("Conteo de pulsos retrocede.")
            previous_time, previous_pulses = t, pulses
            rows += 1
        if post_rows < 2:
            raise CaptureError("Se requieren al menos dos filas de frenado.")
        return rows


def validate_pid_rows(reader, command=""):
    """CSV de runpid: PRE en lazo abierto (OPEN) y BRAKE con el PID (PID), sin rellenar nada."""
    expected = command.split() if command.strip() else None
    if expected and expected[0].lower() != "runpid":
        raise CaptureError("El CSV no coincide con el comando solicitado.")
    previous_time = -math.inf
    previous_pulses = None
    previous_ref = None
    braking = False
    rows = post_rows = 0
    for row in reader:
        if None in row or any(value is None for value in row.values()):
            raise CaptureError("Fila con numero incorrecto de columnas.")
        try:
            values = {k: float(v) for k, v in row.items() if k not in ("phase", "ctrl_mode")}
        except ValueError as exc:
            raise CaptureError("Columna numerica invalida.") from exc
        if not all(math.isfinite(v) for v in values.values()):
            raise CaptureError("CSV con NaN o infinito.")
        t, pwm, u, servo = (values[k] for k in ("t_ms", "motor_pwm", "u_brake", "servo_deg"))
        wref, wmeas, wraw, err = (values[k] for k in ("w_ref", "w_meas", "w_raw", "error"))
        p, i, d, flags = (values[k] for k in ("p_term", "i_term", "d_term", "flags"))
        phase, mode = row["phase"].upper(), row["ctrl_mode"].upper()
        if t <= previous_time:
            raise CaptureError("Tiempo duplicado o no creciente.")
        if pwm != int(pwm) or not 0 <= pwm <= 255 or not 0 <= u <= 1 or not 0 <= servo <= 180:
            raise CaptureError("Actuador fuera de rango.")
        if flags != int(flags) or not 0 <= flags <= 31:
            raise CaptureError("Banderas invalidas.")
        if wmeas < 0 or wraw < 0:
            raise CaptureError("Velocidad negativa.")
        if phase == "PRE":
            if expected and pwm != int(expected[1]):
                raise CaptureError("El CSV no coincide con el comando solicitado.")
            if (braking or mode != "OPEN" or t >= 0 or pwm <= 0 or u != 0
                    or any(v != 0 for v in (wref, err, p, i, d, flags))):
                raise CaptureError("Fase PRE inconsistente.")
        elif phase == "BRAKE":
            if mode != "PID" or t < 0 or pwm != 0 or wref < 0:
                raise CaptureError("Fase de frenado inconsistente.")
            if previous_ref is not None and wref > previous_ref + 1e-3:
                raise CaptureError("La referencia no es decreciente.")
            if post_rows == 0 and abs(wref - wmeas) > 0.01:
                raise CaptureError("La referencia no arranca en la velocidad medida.")
            if abs(err - (wmeas - wref)) > 2e-3:
                raise CaptureError("error no coincide con w_meas - w_ref.")
            if not int(flags) & FLAG_ABS and abs(min(1.0, max(0.0, p + i + d)) - u) > 2e-3:
                raise CaptureError("u_brake no coincide con P + I + D saturado.")
            if int(flags) & FLAG_U_MIN and u > 1e-4:
                raise CaptureError("Bandera u=0 con freno aplicado.")
            if int(flags) & FLAG_U_MAX and u < 1 - 1e-4:
                raise CaptureError("Bandera u=1 sin freno maximo.")
            previous_ref = wref
            braking = True
            post_rows += 1
        else:
            raise CaptureError("Fase desconocida.")
        pulses = values["pulses"]
        if pulses != int(pulses) or not 0 <= pulses <= 4294967295:
            raise CaptureError("Conteo de pulsos invalido.")
        if previous_pulses is not None and pulses < previous_pulses:
            if previous_pulses - pulses < 2147483648:
                raise CaptureError("Conteo de pulsos retrocede.")
        previous_time, previous_pulses = t, pulses
        rows += 1
    if post_rows < 2:
        raise CaptureError("Se requieren al menos dos filas de frenado.")
    return rows


class CaptureProtocol:
    """Maquina de estados independiente del puerto serial para poder probarla."""
    def __init__(self, handle, header=HEADER, success=SUCCESS_RESULTS):
        self.handle = handle
        self.header = header
        self.success = success
        self.state = "WAIT_START"
        self.result = None
        self.header_seen = False
        self.rows = 0
        self.warnings = []

    def accept(self, line):
        if line == "#START_CSV":
            if self.state != "WAIT_START":
                raise CaptureError("Inicio CSV duplicado.")
            self.state = "CAPTURE"
        elif line == "#END_CSV":
            if self.state != "CAPTURE" or not self.header_seen or self.rows < 2:
                raise CaptureError("Fin CSV sin captura completa.")
            self.state = "WAIT_RESULT"
        elif line.startswith("#RESULT "):
            self.result = line[8:].strip()
            if self.state != "WAIT_RESULT" or self.result not in self.success:
                raise CaptureError(f"Ensayo no valido: {self.result}")
            self.state = "DONE"
        elif line.startswith("#ERR"):
            raise CaptureError(line)
        elif line.startswith("#WARN"):
            if self.state != "WAIT_RESULT":
                raise CaptureError("Advertencia dentro del CSV.")
            self.warnings.append(line)
        elif self.state == "CAPTURE":
            if not self.header_seen:
                if line != self.header:
                    raise CaptureError("Encabezado CSV incompatible.")
                self.header_seen = True
            elif line.startswith("#"):
                raise CaptureError("Texto de estado dentro del CSV.")
            else:
                self.rows += 1
            self.handle.write(line + "\n")
        elif self.state == "WAIT_RESULT":
            raise CaptureError("Datos inesperados despues de END_CSV.")


def capture(ser, destination, command="", timeout=20.0, clock=time.monotonic, setup=()):
    """Captura un puerto ya abierto; conserva parcial y metadatos ante cualquier salida."""
    destination = Path(destination)
    partial = destination.with_suffix(".partial.csv")
    metadata_path = destination.with_suffix(".capture.json")
    for path in (destination, partial, metadata_path):
        if path.exists():
            raise FileExistsError(f"No se sobrescribe: {path}")
    wait_timeout, data_timeout = capture_timeouts(command, timeout)
    setup = check_setup(setup)
    destination.parent.mkdir(parents=True, exist_ok=True)
    metadata = {"command": command, "setup": setup, "status": "incomplete", "result": None}
    protocol = None
    try:
        with partial.open("x", encoding="ascii", newline="") as handle:
            protocol = (CaptureProtocol(handle, HEADER_PID, SUCCESS_RESULTS_PID)
                        if is_pid_command(command) else CaptureProtocol(handle))
            for line in setup:
                ser.write((line + "\n").encode("ascii"))
            if command.strip():
                ser.write((command.strip() + "\n").encode("ascii"))
            deadline = clock() + wait_timeout
            pending = b""
            while protocol.state != "DONE":
                if clock() >= deadline:
                    raise CaptureError(f"Timeout en estado {protocol.state}.")
                fragment = ser.readline(513-len(pending))
                if not fragment:
                    continue
                pending += fragment
                if len(pending) > 512:
                    raise CaptureError("Linea serial demasiado larga.")
                if not pending.endswith(b"\n"):
                    continue
                try:
                    line = pending.decode("ascii").strip()
                except UnicodeDecodeError as exc:
                    raise CaptureError("Datos binarios en el puerto serial.") from exc
                pending = b""
                if any(ord(char) < 32 or ord(char) > 126 for char in line):
                    raise CaptureError("Caracteres de control en el puerto serial.")
                if not line:
                    continue
                old_state = protocol.state
                protocol.accept(line)
                if protocol.state != old_state:
                    deadline = clock() + (data_timeout if protocol.state == "CAPTURE" else 3.0)
        metadata.update(result=protocol.result, warnings=protocol.warnings)
        metadata["rows"] = validate_csv(partial, command)
        # link crea el destino de forma exclusiva; nunca reemplaza un ensayo existente.
        os.link(partial, destination)
        partial.unlink()
        metadata["status"] = "complete"
        return destination
    except BaseException as exc:
        metadata["error"] = str(exc)
        if protocol is not None:
            metadata["result"] = protocol.result
        raise
    finally:
        try:
            ser.write(b"x\n")
        except Exception:
            pass
        # Se conserva la causa original si tambien falla la escritura de metadatos.
        try:
            with metadata_path.open("x", encoding="utf-8") as handle:
                json.dump(metadata, handle, indent=2)
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--cmd", default="")
    parser.add_argument("--setup", action="append", default=[],
                        help='comando previo (pid, ref o cfg); se puede repetir, p. ej. --setup "pid 0.02 0 0 0.05"')
    parser.add_argument("--out", default="")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        parser.error("Instala pyserial: python3 -m pip install pyserial")
    if args.list or not args.port:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description}")
        return
    name = args.out or f"ensayo_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
    if not name.lower().endswith(".csv"):
        name += ".csv"
    destination = Path(__file__).resolve().parent / "datos" / name
    with serial.Serial(args.port, args.baud, timeout=.2) as ser:
        time.sleep(1.8)
        ser.reset_input_buffer()
        result = capture(ser, destination, args.cmd, args.timeout, setup=args.setup)
    print(f"Captura verificada: {result}")


if __name__ == "__main__":
    main()
