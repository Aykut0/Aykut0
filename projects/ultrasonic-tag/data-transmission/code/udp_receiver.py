import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

UDP_IP = "0.0.0.0"
UDP_PORT = 12345
FRAME_SIZE = 700  # number of samples per ADC
PACKET_SIZE = FRAME_SIZE * 4  # 2 ADC channels * 2 bytes

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
print(f"Listening on {UDP_IP}:{UDP_PORT}")

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot(np.zeros(FRAME_SIZE * 2))
ax.set_xlabel("Sample Index")
ax.set_ylabel("ADC Value")
ax.set_title("Real-time ADC data")

first = True
while True:
    data, addr = sock.recvfrom(PACKET_SIZE)
    if len(data) != PACKET_SIZE:
        print(f"Unexpected packet size {len(data)}")
        continue
    values = np.frombuffer(data, dtype='<u2')  # little-endian unsigned short
    if first:
        print("First 10 ADC pairs:")
        for i in range(0, 20, 2):
            print(f"{i//2}: ADC1={values[i]}, ADC2={values[i+1]}")
        first = False
    if np.all(values == 0):
        print("Warning: received all zeros")
    line.set_ydata(values)
    line.set_xdata(np.arange(values.size))
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()

