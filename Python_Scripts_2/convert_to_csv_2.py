import pandas as pd
import matplotlib.pyplot as plt

# File paths
mq135_file = "mq135_data.csv"
bme680_file = "bme680_data.csv"

# Load data from CSV files
mq135_data = pd.read_csv(mq135_file)
bme680_data = pd.read_csv(bme680_file)

# Create a figure with 3 subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
fig.tight_layout(pad=5.0)

# Plot MQ-135 data on ax1
ax1.plot(mq135_data.index, mq135_data["ADC Value"], label="ADC Value", color="blue", marker="o")
ax1.plot(mq135_data.index, mq135_data["Rs (kOhms)"], label="Rs (kOhms)", color="orange", marker="o")
ax1.plot(mq135_data.index, mq135_data["Concentration (ppm)"], label="Concentration (ppm)", color="green", marker="o")
ax1.set_title("MQ-135 Sensor Data")
ax1.set_xlabel("Sample Index")
ax1.set_ylabel("Values")
ax1.legend()
ax1.grid(True)

# Plot BME680 Temperature and Humidity on ax2
ax2.plot(bme680_data.index, bme680_data["Temperature (\u00B0C)"], label="Temperature (°C)", color="red", marker="o")
ax2.plot(bme680_data.index, bme680_data["Humidity (%)"], label="Humidity (%)", color="green", marker="o")
ax2.set_title("BME680 Sensor Data - Temperature and Humidity")
ax2.set_xlabel("Sample Index")
ax2.set_ylabel("Values")
ax2.legend()
ax2.grid(True)

# Plot BME680 Other Parameters on ax3
ax3.plot(bme680_data.index, bme680_data["Pressure (Pa)"], label="Pressure (Pa)", color="blue", marker="o")
ax3.plot(bme680_data.index, bme680_data["Gas Resistance (Ohms)"], label="Gas Resistance (Ohms)", color="purple", marker="o")
ax3.plot(bme680_data.index, bme680_data["IAQ Score"], label="IAQ Score", color="orange", marker="o")
ax3.set_title("BME680 Sensor Data - Other Parameters")
ax3.set_xlabel("Sample Index")
ax3.set_ylabel("Values")
ax3.legend()
ax3.grid(True)

# Show the plot
plt.show()
