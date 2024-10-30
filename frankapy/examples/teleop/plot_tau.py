import matplotlib.pyplot as plt
import numpy as np

taus = []
with open("examples/teleop/log.txt") as log_file:
    for line in log_file.readlines():
        tau = [float(i) for i in line.split(' ')]
        taus.append(tau)

try:
    taus = np.array(taus)
except:
    taus.pop()
    taus = np.array(taus)

fig1, axs1 = plt.subplots(7, 1, figsize=(10,8))
fig2, axs2 = plt.subplots(6, 1, figsize=(10,8))

for i, ax in enumerate(axs1):
    ax.plot(taus[:,i])

for i, ax in enumerate(axs2):
    ax.plot(taus[:,i+7])
plt.show()
        