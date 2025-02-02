import csv

# Input file name
input_file = "cleaned_output.txt"

# Output file names
mq135_file = "mq135_data.csv"
bme680_file = "bme680_data.csv"

# Open the input file and parse data
with open(input_file, "r", encoding="utf-8") as infile:
    lines = infile.readlines()

# Prepare headers for both CSVs
mq135_headers = ["ADC Value", "Rs (kOhms)", "Concentration (ppm)"]
bme680_headers = ["Temperature (°C)", "Pressure (Pa)", "Humidity (%)", "Gas Resistance (Ohms)", "IAQ Score"]

# Prepare data rows for both sensors
mq135_rows = []
bme680_rows = []

for line in lines:
    line = line.strip()
    if line.startswith("MQ-135 ADC Value:"):
        # Parse MQ-135 data
        parts = line.split(", ")
        mq135_row = {}
        for part in parts:
            if "ADC Value" in part:
                mq135_row["ADC Value"] = part.split(": ")[1]
            elif "Rs" in part:
                mq135_row["Rs (kOhms)"] = part.split(": ")[1].split()[0]
            elif "Concentration" in part:
                mq135_row["Concentration (ppm)"] = part.split(": ")[1].split()[0]
        mq135_rows.append(mq135_row)

    elif line.startswith("Temperature (°C):"):
        # Parse BME680 data
        parts = line.split(", ")
        bme680_row = {}
        for part in parts:
            if "Temperature" in part:
                bme680_row["Temperature (°C)"] = part.split(": ")[1]
            elif "Pressure" in part:
                bme680_row["Pressure (Pa)"] = part.split(": ")[1]
            elif "Humidity" in part:
                bme680_row["Humidity (%)"] = part.split(": ")[1]
            elif "Gas Resistance" in part:
                bme680_row["Gas Resistance (Ohms)"] = part.split(": ")[1]
            elif "IAQ Score" in part:
                bme680_row["IAQ Score"] = part.split(": ")[1]
        bme680_rows.append(bme680_row)

# Write MQ-135 data to CSV
with open(mq135_file, "w", newline="", encoding="utf-8") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=mq135_headers)
    writer.writeheader()
    writer.writerows(mq135_rows)

# Write BME680 data to CSV
with open(bme680_file, "w", newline="", encoding="utf-8") as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=bme680_headers)
    writer.writeheader()
    writer.writerows(bme680_rows)

print(f"MQ-135 data successfully written to {mq135_file}")
print(f"BME680 data successfully written to {bme680_file}")
