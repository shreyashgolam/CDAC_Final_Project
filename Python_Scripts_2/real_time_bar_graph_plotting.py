import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Input file (data coming from minicom)
input_file = "minicom_output.txt"

# Initialize data storage for MQ-135 and BME680
mq135_adc = []
mq135_rs = []
mq135_concentration = []

bme680_temperature = []
bme680_pressure = []
bme680_humidity = []
bme680_gas_resistance = []
bme680_iaq_score = []

def parse_data():
    """Parse data from the input file."""
    mq135_data = {"ADC Value": None, "Rs (kOhms)": None, "Concentration (ppm)": None}
    bme680_data = {
        "Temperature (°C)": None,
        "Pressure (Pa)": None,
        "Humidity (%)": None,
        "Gas Resistance (Ohms)": None,
        "IAQ Score": None,
    }

    try:
        with open(input_file, "r", encoding="ISO-8859-1") as infile:
            lines = infile.readlines()
            for line in lines:
                line = line.strip()
                print(f"Parsing line: {line}")  # Debug: Print each line
                
                # Parse MQ-135 data
                if line.startswith("MQ-135 ADC Value:"):
                    parts = line.split(", ")
                    for part in parts:
                        if "ADC Value" in part:
                            mq135_data["ADC Value"] = float(part.split(": ")[1])
                        elif "Rs" in part:
                            mq135_data["Rs (kOhms)"] = float(part.split(": ")[1].split()[0])
                        elif "Concentration" in part:
                            mq135_data["Concentration (ppm)"] = float(part.split(": ")[1].split()[0])
                
                # Parse BME680 data
                elif line.startswith("Temperature"):
                    parts = line.split(", ")
                    for part in parts:
                        if "Temperature" in part:
                            bme680_data["Temperature (°C)"] = float(part.split(": ")[1])
                        elif "Pressure" in part:
                            bme680_data["Pressure (Pa)"] = float(part.split(": ")[1])
                        elif "Humidity" in part:
                            bme680_data["Humidity (%)"] = float(part.split(": ")[1])
                        elif "Gas Resistance" in part:
                            bme680_data["Gas Resistance (Ohms)"] = float(part.split(": ")[1])
                        elif "IAQ Score" in part:
                            bme680_data["IAQ Score"] = float(part.split(": ")[1])
    except Exception as e:
        print(f"Error parsing data: {e}")

    # Debug parsed data
    print(f"Parsed MQ-135 Data: {mq135_data}")
    print(f"Parsed BME680 Data: {bme680_data}")
    return mq135_data, bme680_data


def update_plot(frame):
    """Update the plots with new data."""
    # Parse new data
    mq135_data, bme680_data = parse_data()

    # Append MQ-135 data
    if mq135_data["ADC Value"] is not None:
        mq135_adc.append(mq135_data["ADC Value"])
        mq135_rs.append(mq135_data["Rs (kOhms)"])
        mq135_concentration.append(mq135_data["Concentration (ppm)"])

    # Append BME680 data
    if bme680_data["Temperature (°C)"] is not None:
        bme680_temperature.append(bme680_data["Temperature (°C)"])
        bme680_pressure.append(bme680_data["Pressure (Pa)"])
        bme680_humidity.append(bme680_data["Humidity (%)"])
        bme680_gas_resistance.append(bme680_data["Gas Resistance (Ohms)"])
        bme680_iaq_score.append(bme680_data["IAQ Score"])

    # Debug statements to check data flow
    print(f"MQ-135 Data: {mq135_data}")
    print(f"BME680 Data: {bme680_data}")

    # Clear the axes
    ax1.clear()
    ax2.clear()
    ax3.clear()
    ax4.clear()

    # Plot MQ-135 as a bar chart
    if mq135_adc:
        ax1.bar(
            ["ADC Value", "Rs (kOhms)", "Concentration (ppm)"],
            [mq135_adc[-1], mq135_rs[-1], mq135_concentration[-1]],
            color=["blue", "orange", "green"],
        )
        ax1.set_title("MQ-135 Latest Data")
        ax1.set_ylabel("Values")

    # Plot BME680 Temperature and Humidity as a bar chart
    if bme680_temperature:
        ax2.bar(
            ["Temp (°C)", "Humidity (%)"],
            [bme680_temperature[-1], bme680_humidity[-1]],
            color=["red", "green"],
        )
        ax2.set_title("BME680 Latest Data - Temp & Humidity")
        ax2.set_ylabel("Values")

    # Plot BME680 IAQ Score as a bar chart
    if bme680_iaq_score:
        ax3.bar(
            ["IAQ Score"],
            [bme680_iaq_score[-1]],
            color=["orange"],
        )
        ax3.set_title("BME680 Latest Data - IAQ Score")
        ax3.set_ylabel("Values")

    # Plot BME680 Other Parameters as a bar chart
    if bme680_pressure:
        ax4.bar(
            ["Press (Pa)", "Gas Res (Ohms)"],
            [
                bme680_pressure[-1],
                bme680_gas_resistance[-1],
            ],
            color=["blue", "purple"],
        )
        ax4.set_title("BME680 Latest Data - Other Parameters")
        ax4.set_ylabel("Values")

# Initialize the figure and axes
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 15))
fig.tight_layout(pad=5.0)

# Animate the plots
ani = FuncAnimation(fig, update_plot, interval=1000)

# Show the plot
plt.show()
