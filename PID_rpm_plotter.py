import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import numpy as np

F_N = askopenfilename()
df = pd.read_csv(F_N)
plt.figure()
ax = df.plot(x = 'epoch', y = 'yaw', label="Yaw Response")
df.plot(x = 'epoch', y = 'yaw_unwrapped', label="YAW", ax=ax)
df.plot(x = 'epoch', y = 'rpm_left', label="Left RPM", ax=ax, secondary_y=True)
df.plot(x = 'epoch', y = 'rpm_right', label="Right RPM", ax=ax, secondary_y=True)

ax.axhline(df["yaw"][0], c="k", linestyle="--")
plt.xlabel('Epoch time (s)')
plt.ylabel('Yaw (rad)')
plt.legend()
plt.show()
