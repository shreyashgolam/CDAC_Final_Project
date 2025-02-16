import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

# Input file (data coming from minicom)
input_file = "formatted_output.txt"

# Initialize data storage for MQ-135 and BME680
mq135_ppm = []
mq135_adc = []
bme680_temperature = []
bme680_humidity = []

def parse_data():
    """Parse data from the input file."""
    try:
        with open(input_file, "r", encoding="utf-8") as infile:
            lines = infile.readlines()
            for line in lines:
                line = line.strip()
                print(f"Parsing line: {line}")  # Debug: Print each line
                
                match = re.search(r'MQ-135 PPM:\s*(\d+)\s*\|\s*ADC:\s*(\d+)\s*\|\s*Temp:\s*([\d.]+)°C\s*\|\s*Humidity:\s*([\d.]+)%', line)
                
                if match:
                    ppm, adc, temp, humidity = match.groups()
                    mq135_ppm.append(int(ppm))
                    mq135_adc.append(int(adc))
                    bme680_temperature.append(float(temp))
                    bme680_humidity.append(float(humidity))
    except Exception as e:
        print(f"Error parsing data: {e}")

def update_plot(frame):
    """Update the plots with new data."""
    parse_data()
    
    # Clear the axes
    ax1.clear()
    ax2.clear()
    
    # Plot MQ-135 Data
    if mq135_ppm:
        ax1.bar(
            ["PPM", "ADC Value"],
            [mq135_ppm[-1], mq135_adc[-1]],
            color=["blue", "orange"],
        )
        ax1.set_title("MQ-135 Latest Data")
        ax1.set_ylabel("Values")
    
    # Plot BME680 Temperature and Humidity
    if bme680_temperature:
        ax2.bar(
            ["Temp (°C)", "Humidity (%)"],
            [bme680_temperature[-1], bme680_humidity[-1]],
            color=["red", "green"],
        )
        ax2.set_title("BME680 Latest Data - Temp & Humidity")
        ax2.set_ylabel("Values")

# Initialize the figure and axes
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
fig.tight_layout(pad=5.0)

# Animate the plots
ani = FuncAnimation(fig, update_plot, interval=1000)

# Show the plot
plt.show()
