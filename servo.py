import serial
import time

PORT = "/dev/ttyACM0"
BAUD = 1000000

ser = serial.Serial(PORT, BAUD, timeout=0.1)


def checksum(data):
    return (~sum(data) & 0xFF)


def move_servo(servo_id, position, speed=200):

    pos_l = position & 0xFF
    pos_h = (position >> 8) & 0xFF

    spd_l = speed & 0xFF
    spd_h = (speed >> 8) & 0xFF

    packet = [
        0xFF, 0xFF,
        servo_id,
        7,
        0x03,
        0x2A,
        pos_l,
        pos_h,
        spd_l,
        spd_h
    ]

    packet.append(checksum(packet[2:]))

    ser.write(bytearray(packet))


# -------- SET ZERO POSITION --------

CENTER = 512   # physical center of servo

print("Setting both servos to center (software zero)...")

move_servo(1, CENTER)
move_servo(2, CENTER)

time.sleep(3)

print("Zero reference established.")

# conversion
DEG_TO_POS = 1023 / 300


def angle_to_pos(angle):
    return int(CENTER + angle * DEG_TO_POS)


while True:

    # +90°
    move_servo(1, angle_to_pos(90))
    move_servo(2, angle_to_pos(90))
    print("Moved to +90°")
    time.sleep(2)

    # back to 0
    move_servo(1, angle_to_pos(0))
    move_servo(2, angle_to_pos(0))
    print("Moved back to 0°")
    time.sleep(2)

    # -90°
    move_servo(1, angle_to_pos(-90))
    move_servo(2, angle_to_pos(-90))
    print("Moved to -90°")
    time.sleep(2)

    # back to 0
    move_servo(1, angle_to_pos(0))
    move_servo(2, angle_to_pos(0))
    time.sleep(2)