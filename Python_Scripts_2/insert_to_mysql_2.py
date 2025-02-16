import mysql.connector
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Database configuration
db_config = {
    "host": "localhost",
    "port": 3306,
    "user": "root",
    "password": "root",
    "database": "iot_db"
}

# Input file
input_file = "cleaned_output.txt"

# Track the last processed line
last_position = 0

def insert_data_to_mysql(lines):
    try:
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        for line in lines:
            line = line.strip()
            if line.startswith("MQ-135 ADC Value:"):
                parts = line.split(", ")
                adc_value = rs_kohms = concentration_ppm = None
                for part in parts:
                    if "ADC Value" in part:
                        adc_value = float(part.split(": ")[1])
                    elif "Rs" in part:
                        rs_kohms = float(part.split(": ")[1].split()[0])
                    elif "Concentration" in part:
                        concentration_ppm = float(part.split(": ")[1].split()[0])

                cursor.execute("""
                INSERT INTO mq135_data (adc_value, rs_kohms, concentration_ppm)
                VALUES (%s, %s, %s);
                """, (adc_value, rs_kohms, concentration_ppm))

            elif line.startswith("Temperature (°C):"):
                parts = line.split(", ")
                temperature_c = pressure_pa = humidity_percent = gas_resistance_ohms = iaq_score = None
                for part in parts:
                    if "Temperature" in part:
                        temperature_c = float(part.split(": ")[1])
                    elif "Pressure" in part:
                        pressure_pa = float(part.split(": ")[1])
                    elif "Humidity" in part:
                        humidity_percent = float(part.split(": ")[1])
                    elif "Gas Resistance" in part:
                        gas_resistance_ohms = float(part.split(": ")[1])
                    elif "IAQ Score" in part:
                        iaq_score = float(part.split(": ")[1])

                cursor.execute("""
                INSERT INTO bme680_data (temperature_c, pressure_pa, humidity_percent, gas_resistance_ohms, iaq_score)
                VALUES (%s, %s, %s, %s, %s);
                """, (temperature_c, pressure_pa, humidity_percent, gas_resistance_ohms, iaq_score))

        connection.commit()
        print("Data successfully inserted into MySQL.")

    except mysql.connector.Error as e:
        print(f"MySQL Error: {e}")
    finally:
        if cursor:
            cursor.close()
        if connection:
            connection.close()

class FileChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        global last_position
        if event.src_path.endswith(input_file):
            print(f"Detected change in {input_file}, processing new lines...")
            with open(input_file, "r", encoding="utf-8") as infile:
                infile.seek(last_position)
                new_lines = infile.readlines()
                last_position = infile.tell()

            if new_lines:
                insert_data_to_mysql(new_lines)

if __name__ == "__main__":
    event_handler = FileChangeHandler()
    observer = Observer()
    observer.schedule(event_handler, path=".", recursive=False)

    try:
        print(f"Monitoring {input_file} for changes...")
        observer.start()
        while True:
            pass
    except KeyboardInterrupt:
        print("\nStopping observer...")
        observer.stop()

    observer.join()
