import serial
import keyboard
import time

# ==============================
# SERIAL CONFIGURATION
# ==============================

PORT = "/dev/ttyACM0"      # Change if required
BAUD_RATE = 115200

ser = serial.Serial(
    PORT,
    BAUD_RATE,
    timeout=1
)

time.sleep(2)


# ==============================
# SPEED CONFIGURATION
# ==============================

speed = 500

MIN_SPEED = 100
MAX_SPEED = 1000
SPEED_STEP = 100

# Current movement state
# Possible values:
# stop, forward, backward, left, right

movement = "stop"


# ==============================
# SEND COMMAND
# ==============================

def send_motor(left, right):

    # Safety clamp
    left = max(-1000, min(1000, left))
    right = max(-1000, min(1000, right))

    command = f"{left},{right}\n"

    ser.write(command.encode())

    print(
        f"\rMovement: {movement:<8} "
        f"Speed: {speed:<4} "
        f"Command: {left:>5},{right:<5}",
        end="",
        flush=True
    )


# ==============================
# APPLY CURRENT MOVEMENT
# ==============================

def apply_movement():

    if movement == "forward":
        send_motor(speed, speed)

    elif movement == "backward":
        send_motor(-speed, -speed)

    elif movement == "left":
        send_motor(-speed, speed)

    elif movement == "right":
        send_motor(speed, -speed)

    else:
        send_motor(0, 0)


# ==============================
# KEY FUNCTIONS
# ==============================

def forward():
    global movement

    movement = "forward"
    apply_movement()


def backward():
    global movement

    movement = "backward"
    apply_movement()


def left():
    global movement

    movement = "left"
    apply_movement()


def right():
    global movement

    movement = "right"
    apply_movement()


def stop():
    global movement

    movement = "stop"
    apply_movement()


def increase_speed():
    global speed

    speed += SPEED_STEP

    if speed > MAX_SPEED:
        speed = MAX_SPEED

    apply_movement()


def decrease_speed():
    global speed

    speed -= SPEED_STEP

    if speed < MIN_SPEED:
        speed = MIN_SPEED

    apply_movement()


# ==============================
# KEYBOARD CONTROL
# ==============================

keyboard.on_press_key("w", lambda _: forward())
keyboard.on_press_key("s", lambda _: backward())
keyboard.on_press_key("a", lambda _: left())
keyboard.on_press_key("d", lambda _: right())

keyboard.on_press_key(
    "up",
    lambda _: increase_speed()
)

keyboard.on_press_key(
    "down",
    lambda _: decrease_speed()
)

keyboard.on_press_key(
    "space",
    lambda _: stop()
)


# ==============================
# MAIN
# ==============================

print("\n6-Wheel Rover Controller")
print("------------------------")
print("W       : Forward")
print("S       : Backward")
print("A       : Left")
print("D       : Right")
print("UP      : Increase speed")
print("DOWN    : Decrease speed")
print("SPACE   : STOP")
print("Q       : Quit")
print()

# Start safely
stop()

try:

    keyboard.wait("q")

finally:

    movement = "stop"

    send_motor(0, 0)

    ser.close()

    print("\nRover stopped.")
