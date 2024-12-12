import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import numpy as np

F_N = askopenfilename()
df = pd.read_csv(F_N)
# df.plot(x = 'epoch', y = 'yaw_unwrapped', label="Response")

x = df["epoch"]
y = df["yaw_unwrapped"] - df["yaw_unwrapped"][0]

plt.plot(x, y*180/np.pi, label="Response")

# df.plot(x = 'epoch', y = 'yaw', label="Setpoint")
plt.xlabel('Epoch time (s)')
plt.ylabel('Yaw (rad)')
plt.legend()
plt.show()
