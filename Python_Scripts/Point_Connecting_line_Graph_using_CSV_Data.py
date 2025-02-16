import pandas as pd
import matplotlib.pyplot as plt

# File paths
mq135_file = "mq135_data.csv"
bme680_file = "bme680_data.csv"

# Load data from CSV files
mq135_data = pd.read_csv(mq135_file)
bme680_data = pd.read_csv(bme680_file)

# Create a figure with 2 subplots
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
fig.tight_layout(pad=5.0)

# Plot MQ-135 data on ax1
ax1.plot(mq135_data.index, mq135_data["PPM"], label="PPM", color="blue", marker="o")
ax1.plot(mq135_data.index, mq135_data["ADC Value"], label="ADC Value", color="green", marker="o")
ax1.set_title("MQ-135 Sensor Data")
ax1.set_xlabel("Sample Index")
ax1.set_ylabel("Values")
ax1.legend()
ax1.grid(True)

# Plot BME680 Temperature and Humidity on ax2
ax2.plot(bme680_data.index, bme680_data["Temperature (°C)"], label="Temperature (°C)", color="red", marker="o")
ax2.plot(bme680_data.index, bme680_data["Humidity (%)"], label="Humidity (%)", color="blue", marker="o")
ax2.set_title("BME680 Sensor Data - Temperature and Humidity")
ax2.set_xlabel("Sample Index")
ax2.set_ylabel("Values")
ax2.legend()
ax2.grid(True)

# Show the plot
plt.show()
