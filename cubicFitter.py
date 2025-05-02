import numpy as np
import matplotlib.pyplot as plt

# Input data
r_values = np.array([115, 125, 140, 220, 235, 245, 255])
agtron_values = np.array([25, 35, 45, 65, 75, 85, 95])

# Fit a cubic polynomial
coeffs = np.polyfit(r_values, agtron_values, 3)
poly = np.poly1d(coeffs)

print("Cubic coefficients:")
print(f"{coeffs[0]:.6f} * x^3 + {coeffs[1]:.6f} * x^2 + {coeffs[2]:.6f} * x + {coeffs[3]:.6f}")

# Plot for visualization
x_plot = np.linspace(min(r_values), max(r_values), 200)
y_plot = poly(x_plot)

plt.scatter(r_values, agtron_values, color='red', label='Data')
plt.plot(x_plot, y_plot, label='Cubic Fit')
plt.xlabel("Red Channel Value")
plt.ylabel("Agtron Number")
plt.legend()
plt.grid(True)
plt.show()
