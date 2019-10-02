#!/usr/bin/python3
import numpy as np
import matplotlib.pyplot as plt

mine = np.genfromtxt("bldc_controller.csv", skip_header=1, delimiter=',')
theirs = np.genfromtxt("20a_esc.csv", skip_header=1, delimiter=',')

thrust1 = mine[:, 9]
current1 = mine[:, 11]
power1 = mine[:, 14]

thrust2 = theirs[:, 9]
current2 = theirs[:, 11]
power2 = theirs[:, 14]

plt.plot(power1, thrust1, label="mine")
plt.plot(power2, thrust2, label="theirs")
plt.xlabel("Power (W)")
plt.ylabel("Thrust (kg)")
plt.legend()
plt.show()
