import serial

PORT = "/dev/ttyACM1"
BAUD = 9600
N = 9

ser = serial.Serial(PORT, BAUD, timeout=1)

print("Teleop started")
print("m <idx 0-4> <val>     : single motor")
print("s <d1> <d2> <a1> <a2>: servo block")
print("q                     : quit")

def send(u):
    packet = ",".join(str(x) for x in u) + "\n"
    ser.write(packet.encode())
    print("Sent:", u)

while True:
    try:
        line = input("> ").strip().lower()
        if line == "q":
            break

        tokens = line.split()

        u = [0] * N

        if tokens[0] == "m":
            idx = int(tokens[1])
            val = int(tokens[2])

            if not (0 <= idx <= 4):
                print("Motor index must be 0–4")
                continue

            u[idx] = val

        elif tokens[0] == "s":
            if len(tokens) != 5:
                print("Usage: s d1 d2 a1 a2")
                continue

            u[5] = int(tokens[1])
            u[6] = int(tokens[2])
            u[7] = int(tokens[3])
            u[8] = int(tokens[4])

        else:
            print("Unknown command")
            continue

        send(u)

    except Exception as e:
        print("Error:", e)

ser.close()
print("Teleop stopped")
