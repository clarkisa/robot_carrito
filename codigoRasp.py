# rpi_robot_uart_dist.py
# Control de 2 motores con L298N + lectura de distancia desde Tiva por UART (DIST,<cm>)
# Comandos: w/s/a/d (mover), l (stop), f <0-100> (velocidad),
#           b|buzz (enviar 'b' a Tiva), dist, auto <cm>, auto off, status, q

import threading
import serial
import time
import sys
import RPi.GPIO as GPIO

# ===== UART hacia Tiva (USB-ICDI) =====
SERDEVS = ["/dev/ttyACM0", "/dev/ttyACM1"]
BAUD    = 115200

ser = None
for dev in SERDEVS:
    try:
        ser = serial.Serial(dev, BAUD, timeout=0.2)
        print(f"[OK] UART abierto en {dev} @ {BAUD}")
        break
    except Exception as e:
        print(f"[i] No se pudo abrir {dev}: {e}")
if not ser:
    print("⚠️  No se pudo abrir ningún /dev/ttyACM*. Continúo sin UART (buzz/lectura deshabilitados).")

# ===== Pines L298N (BCM) =====
# Motor A (izquierdo)
ENA, IN1, IN2 = 18, 23, 24
# Motor B (derecho)
ENB, IN3, IN4 = 19, 25, 26

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
for pin in (IN1, IN2, IN3, IN4):
    GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENA, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ENB, GPIO.OUT, initial=GPIO.LOW)

pwmA = GPIO.PWM(ENA, 1000)
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(50)
pwmB.start(50)

speed = 50

def set_speed(dc):
    global speed
    dc = max(0, min(100, int(dc)))
    speed = dc
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    print(f"✅ Velocidad: {speed}%")

def forward():
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def backward():
    GPIO.output(IN1, 0); GPIO.output(IN2, 1)
    GPIO.output(IN3, 0); GPIO.output(IN4, 1)

def left():
    GPIO.output(IN1, 0); GPIO.output(IN2, 0)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def right():
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 0)

def stop():
    GPIO.output(IN1, 0); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 0)

def send_buzz():
    if ser and ser.writable():
        ser.write(b"b\n")
        ser.flush()
        print("📡 Enviado: b")
    else:
        print("⚠️  UART no disponible")

# ===== Lectura de distancia desde la Tiva (DIST,<cm>) =====
last_dist_cm = None
autostop_enabled = True      # si quieres que siempre actúe, déjalo en True por defecto
autostop_thresh  = 3         # cm
hyst             = 1         # histéresis 1 cm para liberar “zona de peligro”
stop_due_to_autostop = False

# ===== Secuencia evasiva =====
sequence_active = False
sequence_lock   = threading.Lock()
last_avoid_t    = 0.0
avoid_cooldown_s = 3.0       # no re-ejecutar dentro de este tiempo

def auto_avoid_sequence():
    """Frena 1.5 s, retrocede 2 s y detiene. Protegido contra reentradas."""
    global sequence_active, last_avoid_t
    with sequence_lock:
        if sequence_active:
            return
        sequence_active = True
    try:
        print("⚠️  DIST ≤ umbral → Secuencia evasiva: stop 1.5 s, atrás 2 s...")
        stop()
        time.sleep(1.5)

        # Si quieres retroceder con velocidad fija más suave, descomenta:
        # prev = speed
        # set_speed(40)

        backward()
        time.sleep(2.0)
        stop()

        # Si usaste velocidad fija:
        # set_speed(prev)

        last_avoid_t = time.monotonic()
        print("🧭 Maniobra evasiva completada. Esperando comandos...")
    finally:
        sequence_active = False

# Hilo lector
stop_reader_evt = threading.Event()

def parse_dist_line(line: str):
    # Acepta "DIST,<int>" y devuelve int o None
    if not line.startswith("DIST,"):
        return None
    try:
        return int(line.split(",", 1)[1])
    except ValueError:
        return None

def serial_reader():
    global last_dist_cm, stop_due_to_autostop
    while not stop_reader_evt.is_set() and ser:
        try:
            chunk = ser.readline()
            if not chunk:
                continue
            line = chunk.decode("utf-8", errors="ignore").strip()
            val = parse_dist_line(line)
            if val is None:
                continue

            # Filtrar lecturas inválidas
            if val <= 0 or val > 400:
                last_dist_cm = -1
                continue

            last_dist_cm = val
            print(f"📥 DIST = {last_dist_cm} cm")

            now = time.monotonic()

            # Lógica de zona de peligro con histéresis
            if autostop_enabled:
                # Disparar secuencia evasiva al entrar en zona de peligro y respetar cooldown
                if (last_dist_cm <= autostop_thresh) and (now - last_avoid_t >= avoid_cooldown_s):
                    threading.Thread(target=auto_avoid_sequence, daemon=True).start()
                    stop_due_to_autostop = True

                # Liberar estado cuando se aleje
                elif stop_due_to_autostop and last_dist_cm >= autostop_thresh + hyst:
                    stop_due_to_autostop = False
                    print(f"✅ Fuera de peligro (≥ {autostop_thresh + hyst} cm)")
        except Exception as e:
            print(f"[UART RX] Error: {e}")
            break

rx_thread = None
if ser:
    rx_thread = threading.Thread(target=serial_reader, daemon=True)
    rx_thread.start()

# ===== CLI =====
def print_help():
    print(
        "\nComandos:\n"
        "  w/s/a/d         → mover (adelante/atrás/izq/der)\n"
        "  l               → stop (frena)\n"
        "  f <0-100>       → velocidad PWM ENA/ENB\n"
        "  b | buzz        → envía 'b' a la Tiva (buzzer 2s)\n"
        "  dist            → muestra la última distancia recibida\n"
        "  auto <cm>       → activa autofreno a ese umbral (ej.: auto 3)\n"
        "  auto off        → desactiva autofreno\n"
        "  status          → muestra estado actual\n"
        "  q               → salir\n"
    )

def show_status():
    print(f"Velocidad: {speed}%")
    print(f"Última distancia: {last_dist_cm} cm" if last_dist_cm is not None else "Última distancia: --")
    print(f"Autofreno: {'ON' if autostop_enabled else 'OFF'} (umbral={autostop_thresh} cm, hist={hyst} cm)")
    print(f"Cooldown evasiva: {avoid_cooldown_s} s; activa: {sequence_active}")

print_help()
try:
    while True:
        try:
            cmd = input(">>> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nSaliendo...")
            break

        if not cmd:
            continue

        # Bloquear comandos mientras corre la maniobra evasiva
        if sequence_active and cmd in ("w", "s", "a", "d", "l", "f"):
            print("⏳ Maniobra evasiva en curso; espera a que termine…")
            continue

        if cmd == "q":
            break
        elif cmd == "w":
            forward()
        elif cmd == "s":
            backward()
        elif cmd == "a":
            right()
        elif cmd == "d":
            left()
        elif cmd == "l":
            stop()
        elif cmd.startswith("f"):
            parts = cmd.split()
            if len(parts) == 2 and parts[1].isdigit():
                set_speed(int(parts[1]))
            else:
                print("⚠️  Usa: f <0-100>")
        elif cmd in ("b", "buzz"):
            send_buzz()
        elif cmd == "dist":
            if last_dist_cm is None:
                print("📟 Distancia: -- (aún no recibida)")
            else:
                print(f"📟 Distancia: {last_dist_cm} cm")
        elif cmd.startswith("auto "):
            arg = cmd.split(" ", 1)[1]
            if arg == "off":
                autostop_enabled = False
                print("🟢 Autofreno: OFF")
            else:
                try:
                    autostop_thresh = max(0, int(arg))
                    autostop_enabled = True
                    print(f"🟠 Autofreno: ON (umbral={autostop_thresh} cm)")
                except ValueError:
                    print("⚠️  Usa: auto <cm>  o  auto off")
        elif cmd == "status":
            show_status()
        else:
            print("Comando inválido. Escribe 'status' o 'q' para salir.")
except KeyboardInterrupt:
    pass
finally:
    try:
        stop()
        pwmA.stop()
        pwmB.stop()
    except Exception:
        pass
    GPIO.cleanup()
    stop_reader_evt.set()
    if ser:
        try:
            ser.close()
        except Exception:
            pass
    print("Bye!")
