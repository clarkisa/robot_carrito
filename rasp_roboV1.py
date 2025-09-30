# rpi_motores_uart.py
import serial, time
import RPi.GPIO as GPIO
import sys, select

SERDEV = "/dev/ttyACM0"     # puerto USB de la Tiva
BAUD   = 115200

ENA, IN1, IN2 = 18, 23, 24  # Motor 1 (canal A)
ENB, IN3, IN4 = 19, 25, 26  # Motor 2 (canal B)

GPIO.setmode(GPIO.BCM)
for pin in (IN1, IN2, IN3, IN4):
    GPIO.setup(pin, GPIO.OUT); GPIO.output(pin, 0)

GPIO.setup(ENA, GPIO.OUT); GPIO.setup(ENB, GPIO.OUT)
pwmA = GPIO.PWM(ENA, 1000)  # 1 kHz
pwmB = GPIO.PWM(ENB, 1000)
pwmA.start(0)
pwmB.start(0)

# ===== Estado de motores =====
m1_running = False
m2_running = False
last_duty  = 50  # duty por defecto

def apply_duty_to_running():
    """Aplica el duty actual (last_duty) a los motores que estén encendidos, sin detenerlos."""
    if m1_running:
        pwmA.ChangeDutyCycle(last_duty)
    if m2_running:
        pwmB.ChangeDutyCycle(last_duty)

def motor1_on(duty=None):
    """Enciende motor 1 hacia adelante. Si duty es None, usa last_duty."""
    global m1_running
    if duty is None:
        duty = last_duty
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    pwmA.ChangeDutyCycle(duty)
    m1_running = True

def motor2_on(duty=None):
    """Enciende motor 2 hacia adelante. Si duty es None, usa last_duty."""
    global m2_running
    if duty is None:
        duty = last_duty
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)
    pwmB.ChangeDutyCycle(duty)
    m2_running = True

def motors_off():
    """Apaga ambos motores (PWM=0 y pines IN en 0)."""
    global m1_running, m2_running
    pwmA.ChangeDutyCycle(0); pwmB.ChangeDutyCycle(0)
    GPIO.output(IN1,0); GPIO.output(IN2,0); GPIO.output(IN3,0); GPIO.output(IN4,0)
    m1_running = False; m2_running = False

ser = serial.Serial(SERDEV, BAUD, timeout=0.1)

print("Listo. Comandos Tiva esperados: motor1 / motor2.")
print("Comandos desde SSH: duty <0..100>, buzzer, stop")

try:
    while True:
        # ===== RX desde Tiva por UART =====
        line = ser.readline().decode('utf-8', errors='ignore').strip().lower()
        if line:
            print("RX Tiva:", line)
            if line == "motor1":
                motor1_on()            # usa last_duty actual
            elif line == "motor2":
                motor2_on()            # usa last_duty actual
            elif line.startswith("duty "):
                # (Opcional) si quieres aceptar "duty X" también desde la Tiva:
                parts = line.split()
                if len(parts) == 2 and parts[1].isdigit():
                    v = max(0, min(100, int(parts[1])))
                    last_duty = v
                    apply_duty_to_running()
                    print(f"Nuevo duty (UART) = {last_duty}% aplicado en caliente.")

        # ===== Entrada manual desde SSH (no bloqueante) =====
        if select.select([sys.stdin], [], [], 0.01)[0]:
            cmd = sys.stdin.readline().strip().lower()
            if cmd.startswith("duty"):
                try:
                    _, v = cmd.split()
                    v = max(0, min(100, int(v)))
                    last_duty = v
                    apply_duty_to_running()  # <— aplica en tiempo real
                    print(f"Nuevo duty = {last_duty}% (aplicado en motores activos)")
                except Exception:
                    print("Uso: duty <0..100>")
            elif cmd == "buzzer":
                ser.write(b"buzzer\n")
                print("TX a Tiva: buzzer")
            elif cmd == "stop":
                motors_off()
                print("Motores en stop")

        time.sleep(0.01)

except KeyboardInterrupt:
    pass
finally:
    motors_off()
    pwmA.stop(); pwmB.stop()
    GPIO.cleanup()
    ser.close()