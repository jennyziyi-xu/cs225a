import matplotlib.pyplot as plt
import math

########### Q 1 b)

# file = open("data_files/q1.txt")

# time = []
# q1 = []
# q3 = []
# q4 = []


# for line in file:
#     text_splitted = line.split('\t')
#     if len(text_splitted) > 3:
#         time.append(float(text_splitted[0]))
#         q1.append(float(text_splitted[1]))
#         q3.append(float(text_splitted[3]))
#         q4.append(float(text_splitted[4]))

# plt.figure()
# plt.xlabel("Time elapsed")
# plt.ylabel("joint positions")
# plt.plot(time, q1, label='joint 1')

# plt.plot(time, q3, label='joint 3')

# plt.plot(time, q4, label='joint 4')


# plt.legend(['joint 1', 'joint 3', 'joint 4'])
# plt.axhline(y = -(125 * math.pi/180))
# plt.axhline(y = 0)
# plt.axhline(y = math.pi / 2)
# plt.title("Joint trajectory for joints 1,3,4")
# plt.savefig("q1b.png")

file = open("data_files/q1.txt")

time = []
q1 = []
q3 = []
q4 = []

for line in file:
    text_splitted = line.split('\t')
    if len(text_splitted) > 3:
        time.append(float(text_splitted[0]))
        q1.append(float(text_splitted[1]) * 180 / math.pi)
        q3.append(float(text_splitted[3]) * 180 / math.pi)
        q4.append(float(text_splitted[4]) * 180 / math.pi)

plt.figure()
plt.xlabel("Time elapsed (seconds)")
plt.ylabel("Joint 1 position (deg)")
plt.plot(time, q1,'r', label='joint 1')
plt.axhline(y = 90, label="joint 1 target position = 90 degrees")
plt.legend(['joint 1', 'joint 1 target position = 90 degrees'])
plt.title("Joint trajectory for joint 1")
plt.savefig("q1_joint1.png")

plt.figure()
plt.xlabel("Time elapsed (seconds)")
plt.ylabel("Joint 3 position (deg)")
plt.plot(time, q3, 'r', label='joint 3')
plt.axhline(y = 0, label="joint 3 target position = 0 degrees")
plt.legend(['joint 3', 'joint 3 target position = 0 degrees'])
plt.title("Joint trajectory for joint 3")
plt.savefig("q1_joint3.png")

plt.figure()
plt.xlabel("Time elapsed (seconds)")
plt.ylabel("Joint 4 position (deg)")
plt.plot(time, q4, 'r', label='joint 4')
plt.axhline(y = -125 , label="joint 4 target position = -125 degrees")
plt.legend(['joint 4', 'joint 4 target position = -125 degrees'])
plt.title("Joint trajectory for joint 4")
plt.savefig("q1_joint4.png")


########### Q 2 b)

# file = open("data_files/q2.txt")

# time = []
# q1 = []
# q3 = []
# q4 = []


# for line in file:
#     text_splitted = line.split('\t')
#     if len(text_splitted) > 3:
#         time.append(float(text_splitted[0]))
#         q1.append(float(text_splitted[1]))
#         q3.append(float(text_splitted[3]))
#         q4.append(float(text_splitted[4]))

# plt.figure()
# plt.xlabel("Time elapsed")
# plt.ylabel("joint positions")
# plt.plot(time, q1, label='joint 1')
# plt.plot(time, q3, label='joint 3')
# plt.plot(time, q4, label='joint 4')

# plt.axhline(y = math.pi / 2)
# plt.axhline(y = 0)
# plt.axhline(y = -(125 * math.pi/180))

# plt.legend(['joint 1', 'joint 3', 'joint 4'])
# plt.title("Joint trajectory for joints 1,3,4")
# plt.savefig("q2b.png")

# ########### Q 3 b)

# file = open("data_files/q3.txt")

# time = []
# q1 = []
# q3 = []
# q4 = []


# for line in file:
#     text_splitted = line.split('\t')
#     if len(text_splitted) > 3:
#         time.append(float(text_splitted[0]))
#         q1.append(float(text_splitted[1]))
#         q3.append(float(text_splitted[3]))
#         q4.append(float(text_splitted[4]))

# plt.figure()
# plt.xlabel("Time elapsed")
# plt.ylabel("joint positions")
# plt.plot(time, q1, label='joint 1')
# plt.axhline(y = math.pi / 2)

# plt.plot(time, q3, label='joint 3')
# plt.axhline(y = 0)

# plt.plot(time, q3, label='joint 4')
# plt.axhline(y = -(125 * math.pi/180))

# plt.legend(['joint 1', 'joint 3', 'joint 4'])
# plt.title("Joint trajectory for joints 1,3,4")
# plt.savefig("q3b.png")

########### Q 4 b)

# file = open("data_files/q4.txt")

# time = []
# q1 = []
# q3 = []
# q4 = []


# for line in file:
#     text_splitted = line.split('\t')
#     if len(text_splitted) > 3:
#         time.append(float(text_splitted[0]))
#         q1.append(float(text_splitted[1]))
#         q3.append(float(text_splitted[3]))
#         q4.append(float(text_splitted[4]))

# plt.figure()
# plt.xlabel("Time elapsed")
# plt.ylabel("joint positions")
# plt.plot(time, q1, label='joint 1')
# plt.axhline(y = math.pi / 2)

# plt.plot(time, q3, label='joint 3')
# plt.axhline(y = 0)

# plt.plot(time, q3, label='joint 4')
# plt.axhline(y = -(125 * math.pi/180))

# plt.legend(['joint 1', 'joint 3', 'joint 4'])
# plt.title("Joint trajectory for joints 1,3,4")
# plt.savefig("q4b.png")


########### Q 5 b)

# file = open("data_files/q5.txt")

# time = []
# q1 = []
# q3 = []
# q4 = []


# for line in file:
#     text_splitted = line.split('\t')
#     if len(text_splitted) > 3:
#         time.append(float(text_splitted[0]))
#         q1.append(float(text_splitted[1]))
#         q3.append(float(text_splitted[3]))
#         q4.append(float(text_splitted[4]))

# plt.figure()
# plt.xlabel("Time elapsed")
# plt.ylabel("joint positions")
# plt.plot(time, q1, label='joint 1')
# plt.plot(time, q3, label='joint 3')
# plt.plot(time, q4, label='joint 4')

# plt.axhline(y = math.pi / 2)
# plt.axhline(y = 0)
# plt.axhline(y = -(125 * math.pi/180))

# plt.legend(['joint 1', 'joint 3', 'joint 4'])
# plt.title("Joint trajectory for joints 1,3,4")
# plt.savefig("q5b.png")

########### Q 6 b)

# file = open("data_files/q6.txt")

# time = []
# q1 = []
# q3 = []
# q4 = []


# for line in file:
#     text_splitted = line.split('\t')
#     if len(text_splitted) > 3:
#         time.append(float(text_splitted[0]))
#         q1.append(float(text_splitted[1]))
#         q3.append(float(text_splitted[3]))
#         q4.append(float(text_splitted[4]))

# plt.figure()
# plt.xlabel("Time elapsed")
# plt.ylabel("joint positions")
# plt.plot(time, q1, label='joint 1')
# plt.plot(time, q3, label='joint 3')
# plt.plot(time, q4, label='joint 4')

# plt.axhline(y = math.pi / 2)
# plt.axhline(y = 0)
# plt.axhline(y = -(125 * math.pi/180))

# plt.legend(['joint 1', 'joint 3', 'joint 4'])
# plt.title("Joint trajectory for joints 1,3,4")
# plt.savefig("q6b.png")