import matplotlib.pyplot as plt
import math

question = 5

file = open("data_files/q{}.txt".format(question))

time = []
q1 = []
q3 = []
q4 = []

for line in file:
    text_splitted = line.split('\t')
    if len(text_splitted) > 3:
        time.append(float(text_splitted[0]))
        q1.append(round(float(text_splitted[1]) * 180 / math.pi, 3))
        q3.append(round(float(text_splitted[3]) * 180 / math.pi, 3))
        q4.append(round(float(text_splitted[4]) * 180 / math.pi, 3))

plt.figure()
plt.xlabel("Time elapsed (seconds)")
plt.ylabel("Joint 1 position (deg)")
plt.plot(time, q1,'r', label='joint 1')
plt.axhline(y = 90, label="joint 1 target position = 90 degrees")
plt.legend(['joint 1', 'joint 1 target position = 90 degrees'])
plt.title("Joint trajectory for joint 1 - Q{}".format(question))
plt.savefig("q{}_joint1.png".format(question))

plt.figure(constrained_layout=True)
plt.xlabel("Time elapsed (seconds)")
plt.ylabel("Joint 3 position (deg)")
plt.plot(time, q3, 'r', label='joint 3')
plt.axhline(y = 0, label="joint 3 target position = 0 degrees")
plt.legend(['joint 3', 'joint 3 target position = 0 degrees'])
plt.title("Joint trajectory for joint 3 - Q{}".format(question))
plt.savefig("q{}_joint3.png".format(question))

plt.figure(constrained_layout=True)
plt.xlabel("Time elapsed (seconds)")
plt.ylabel("Joint 4 position (deg)")
plt.plot(time, q4, 'r', label='joint 4')
plt.axhline(y = -125 , label="joint 4 target position = -125 degrees")
plt.legend(['joint 4', 'joint 4 target position = -125 degrees'])
plt.title("Joint trajectory for joint 4 - Q{}".format(question))
plt.savefig("q{}_joint4.png".format(question))