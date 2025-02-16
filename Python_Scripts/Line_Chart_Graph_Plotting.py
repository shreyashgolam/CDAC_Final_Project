import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import re

# Input file for real-time updates
input_file = "formatted_output.txt"

# Define the file change handler class
class FileChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path.endswith(input_file):
            parse_and_save_data()

# Function to parse the minicom data and save to CSV files
def parse_and_save_data():
    mq135_data = []
    bme680_data = []

    with open(input_file, "r", encoding="utf-8") as infile:
        lines = infile.readlines()

    for line in lines:
        line = line.strip()
        match = re.search(r'MQ-135 PPM:\s*(\d+)\s*\|\s*ADC:\s*(\d+)\s*\|\s*Temp:\s*([\d.]+)°C\s*\|\s*Humidity:\s*([\d.]+)%', line)
        
        if match:
            ppm, adc, temp, humidity = match.groups()
            mq135_data.append({"PPM": int(ppm), "ADC Value": int(adc)})
            bme680_data.append({"Temperature (°C)": float(temp), "Humidity (%)": float(humidity)})
    
    # Save data to CSV
    pd.DataFrame(mq135_data).to_csv("mq135_data.csv", index=False)
    pd.DataFrame(bme680_data).to_csv("bme680_data.csv", index=False)

# Start monitoring the input file
observer = Observer()
event_handler = FileChangeHandler()
observer.schedule(event_handler, ".", recursive=False)
observer.start()

# Create a figure with 2 subplots
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
fig.tight_layout(pad=5.0)

# Update the plots in real-time using line charts and limiting to last 30 data points
def update_plot(frame):
    # Load data from CSV files
    mq135_data = pd.read_csv("mq135_data.csv")
    bme680_data = pd.read_csv("bme680_data.csv")

    # Keep only the last 30 entries
    mq135_data = mq135_data.tail(30)
    bme680_data = bme680_data.tail(30)

    # Clear the axes
    ax1.clear()
    ax2.clear()

    # Plot MQ-135 data as line chart
    if not mq135_data.empty:
        ax1.plot(mq135_data.index, mq135_data["PPM"], label="PPM", color="blue", marker="o")
        ax1.plot(mq135_data.index, mq135_data["ADC Value"], label="ADC Value", color="orange", marker="o")
        ax1.set_title("MQ-135 Sensor Data (Last 30 Readings)")
        ax1.set_xlabel("Sample Index")
        ax1.set_ylabel("Values")
        ax1.legend()
        ax1.grid(True)
    
    # Plot BME680 Temperature and Humidity as line chart
    if not bme680_data.empty:
        ax2.plot(bme680_data.index, bme680_data["Temperature (°C)"], label="Temperature (°C)", color="red", marker="o")
        ax2.plot(bme680_data.index, bme680_data["Humidity (%)"], label="Humidity (%)", color="green", marker="o")
        ax2.set_title("BME680 Sensor Data - Temperature & Humidity (Last 30 Readings)")
        ax2.set_xlabel("Sample Index")
        ax2.set_ylabel("Values")
        ax2.legend()
        ax2.grid(True)

# Animate the plots
ani = FuncAnimation(fig, update_plot, interval=1000)

# Show the plot
plt.show()

# Stop observer on script exit
observer.stop()
observer.join()
