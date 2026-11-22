"""
Captura serial automatizada desde la ESP32 hacia archivos CSV.

Alternativa ligera en Python (requiere: pip install pyserial) por si prefieres
capturar desde terminal mientras analizas en MATLAB.

Ejemplos de uso:
    python captura_serial.py --port COM5 --cmd "step 180 0.0 3000 400 6000" --out coastdown_1.csv
    python captura_serial.py --port COM5 --cmd "step 180 0.4 3000 400 6000" --out step_u04_1.csv
    python captura_serial.py --port COM5 --cmd "step 180 0.7 3000 400 6000" --out step_u07_1.csv
"""

import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Error: instala pyserial primero con 'pip install pyserial'.")
    sys.exit(1)


def listar_puertos() -> None:
    puertos = list(list_ports.comports())
    if not puertos:
        print("No se detectaron puertos seriales disponibles.")
        return
    print("Puertos seriales disponibles:")
    for p in puertos:
        print(f"  - {p.device}: {p.description}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Captura CSV de ensayos desde ESP32.")
    parser.add_argument("--port", help="Puerto serial (ej. COM5)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--cmd", default="", help='Comando a enviar, ej. "step 180 0.4 3000 400 6000"')
    parser.add_argument("--out", default="", help="Nombre del archivo CSV de salida")
    parser.add_argument("--timeout", type=float, default=20.0, help="Timeout total en segundos")
    parser.add_argument("--list", action="store_true", help="Listar puertos disponibles y salir")
    args = parser.parse_args()

    if args.list or not args.port:
        listar_puertos()
        if not args.port:
            sys.exit(0)

    carpeta = Path(__file__).resolve().parent / "datos"
    carpeta.mkdir(parents=True, exist_ok=True)

    nombre = args.out.strip()
    if not nombre:
        nombre = f"ensayo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    elif not nombre.lower().endswith(".csv"):
        nombre += ".csv"
    ruta_salida = carpeta / nombre

    print(f"Conectando a {args.port} a {args.baud} baudios...")
    capturando = False
    lineas = 0

    with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
        time.sleep(1.8)
        ser.reset_input_buffer()

        if args.cmd.strip():
            print(f"Enviando comando: {args.cmd}")
            ser.write((args.cmd.strip() + "\n").encode("utf-8"))
        else:
            print("Modo solo escucha. Esperando #START_CSV...")

        t_inicio = time.monotonic()
        try:
            with ruta_salida.open("w", encoding="utf-8", newline="\n") as f:
                while (time.monotonic() - t_inicio) < args.timeout:
                    raw = ser.readline()
                    if not raw:
                        continue
                    linea = raw.decode("utf-8", errors="replace").strip()
                    if not linea:
                        continue

                    if linea == "#START_CSV":
                        capturando = True
                        t_inicio = time.monotonic()
                        print(">> Inicio de captura CSV (#START_CSV).")
                        continue

                    if linea == "#END_CSV":
                        print(f">> Fin de captura CSV (#END_CSV). Lineas guardadas: {lineas}")
                        break

                    if capturando:
                        f.write(linea + "\n")
                        lineas += 1
                        if lineas == 1 or lineas % 25 == 0:
                            print(f"   [{lineas:4d}] {linea}")
                    else:
                        print(f"[ESP32] {linea}")
        except KeyboardInterrupt:
            print("\nInterrumpido por el usuario. Enviando parada de emergencia 'x'...")
            ser.write(b"x\n")

    if not capturando or lineas < 2:
        print("Advertencia: no se capturo un CSV completo.")
    else:
        print(f"Archivo guardado en: {ruta_salida}")


if __name__ == "__main__":
    main()
