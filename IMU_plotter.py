import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import numpy as np

F_N = askopenfilename()
df = pd.read_csv(F_N)
df["yaw (deg)"] = np.rad2deg(df["yaw"])
avg = sum(df["yaw (deg)"])/len(df["yaw (deg)"])
ax = df.plot(x = 'epoch', y = 'yaw (deg)')
ax.set_xlabel('Epoch time (s)')
ax.set_ylabel('Yaw (deg)')
ax.axhline(avg, linestyle="--")

# df_angular = pd.read_csv("angular_data.csv")
# df.plot(x = 'epoch', y = 'angular_speed')

# ax.plot()
plt.show()
