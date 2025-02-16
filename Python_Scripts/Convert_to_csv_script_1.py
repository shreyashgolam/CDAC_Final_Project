import csv
import re

# Input file name
input_file = "formatted_output.txt"

# Output file names
mq135_file = "mq135_data.csv"
bme680_file = "bme680_data.csv"

# Open the input file and parse data
with open(input_file, "r", encoding="utf-8") as infile:
    lines = infile.readlines()

# Prepare headers for both CSVs
mq135_headers = ["PPM", "ADC Value"]
bme680_headers = ["Temperature (°C)", "Humidity (%)"]

# Prepare data rows for both sensors
mq135_rows = []
bme680_rows = []

for line in lines:
    line = line.strip()
    match = re.search(r'MQ-135 PPM:\s*(\d+)\s*\|\s*ADC:\s*(\d+)\s*\|\s*Temp:\s*([\d.]+)°C\s*\|\s*Humidity:\s*([\d.]+)%', line)
    
    if match:
        ppm, adc, temp, humidity = match.groups()
        mq135_rows.append({"PPM": ppm, "ADC Value": adc})
        bme680_rows.append({"Temperature (°C)": temp, "Humidity (%)": humidity})

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
