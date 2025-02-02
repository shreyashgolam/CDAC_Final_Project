import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Input file for real-time updates
input_file = "minicom_output.txt"

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
        if line.startswith("MQ-135 ADC Value:"):
            parts = line.split(", ")
            mq135_row = {}
            for part in parts:
                if "ADC Value" in part:
                    mq135_row["ADC Value"] = float(part.split(": ")[1])
                elif "Rs" in part:
                    mq135_row["Rs (kOhms)"] = float(part.split(": ")[1].split()[0])
                elif "Concentration" in part:
                    mq135_row["Concentration (ppm)"] = float(part.split(": ")[1].split()[0])
            mq135_data.append(mq135_row)

        elif line.startswith("Temperature (°C):"):
            parts = line.split(", ")
            bme680_row = {}
            for part in parts:
                if "Temperature" in part:
                    bme680_row["Temperature (°C)"] = float(part.split(": ")[1])
                elif "Pressure" in part:
                    bme680_row["Pressure (Pa)"] = float(part.split(": ")[1])
                elif "Humidity" in part:
                    bme680_row["Humidity (%)"] = float(part.split(": ")[1])
                elif "Gas Resistance" in part:
                    bme680_row["Gas Resistance (Ohms)"] = float(part.split(": ")[1])
                elif "IAQ Score" in part:
                    bme680_row["IAQ Score"] = float(part.split(": ")[1])
            bme680_data.append(bme680_row)

    # Save data to CSV
    pd.DataFrame(mq135_data).to_csv("mq135_data.csv", index=False)
    pd.DataFrame(bme680_data).to_csv("bme680_data.csv", index=False)

# Start monitoring the input file
observer = Observer()
event_handler = FileChangeHandler()
observer.schedule(event_handler, ".", recursive=False)
observer.start()

# Create a figure with 3 subplots
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 15))
fig.tight_layout(pad=5.0)

# Update the plots in real-time
def update_plot(frame):
    # Load data from CSV files
    mq135_data = pd.read_csv("mq135_data.csv")
    bme680_data = pd.read_csv("bme680_data.csv")

    # Clear the axes
    ax1.clear()
    ax2.clear()
    ax3.clear()

    # Plot MQ-135 data on ax1
    if not mq135_data.empty:
        ax1.plot(mq135_data.index, mq135_data["ADC Value"], label="ADC Value", color="blue", marker="o")
        ax1.plot(mq135_data.index, mq135_data["Rs (kOhms)"], label="Rs (kOhms)", color="orange", marker="o")
        ax1.plot(mq135_data.index, mq135_data["Concentration (ppm)"], label="Concentration (ppm)", color="green", marker="o")
        ax1.set_title("MQ-135 Sensor Data")
        ax1.set_xlabel("Sample Index")
        ax1.set_ylabel("Values")
        ax1.legend()
        ax1.grid(True)

    # Plot BME680 Temperature and Humidity on ax2
    if not bme680_data.empty:
        ax2.plot(bme680_data.index, bme680_data["Temperature (°C)"], label="Temperature (°C)", color="red", marker="o")
        ax2.plot(bme680_data.index, bme680_data["Humidity (%)"], label="Humidity (%)", color="green", marker="o")
        ax2.set_title("BME680 Sensor Data - Temperature and Humidity")
        ax2.set_xlabel("Sample Index")
        ax2.set_ylabel("Values")
        ax2.legend()
        ax2.grid(True)

    # Plot BME680 Other Parameters on ax3
    if not bme680_data.empty:
        ax3.plot(bme680_data.index, bme680_data["Pressure (Pa)"], label="Pressure (Pa)", color="blue", marker="o")
        ax3.plot(bme680_data.index, bme680_data["Gas Resistance (Ohms)"], label="Gas Resistance (Ohms)", color="purple", marker="o")
        ax3.plot(bme680_data.index, bme680_data["IAQ Score"], label="IAQ Score", color="orange", marker="o")
        ax3.set_title("BME680 Sensor Data - Other Parameters")
        ax3.set_xlabel("Sample Index")
        ax3.set_ylabel("Values")
        ax3.legend()
        ax3.grid(True)

# Animate the plots
ani = FuncAnimation(fig, update_plot, interval=1000)

# Show the plot
plt.show()

# Stop observer on script exit
observer.stop()
observer.join()
