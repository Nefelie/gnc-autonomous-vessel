import pandas as pd
import matplotlib.pyplot as plt
from tkinter import Tk
from tkinter.filedialog import askopenfilename
import numpy as np

F_N = askopenfilename()
df = pd.read_csv(F_N)
ax = df.plot(x = 'x_MM', y = 'y_MM')
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.show()
