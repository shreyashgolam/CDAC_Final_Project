import mysql.connector
import re
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Database configuration
db_config = {
    "host": "localhost",
    "port": 3306,
    "user": "root",
    "password": "root",
    "database": "iot_db",
}

# Input file
input_file = "formatted_output.txt"

# Define the file change handler class
class FileChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path.endswith(input_file):
            insert_latest_data_to_mysql()

# Function to insert only the latest data entry into MySQL
def insert_latest_data_to_mysql():
    try:
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Ensure tables exist
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS mq135_data (
                id INT AUTO_INCREMENT PRIMARY KEY,
                ppm INT,
                adc_value INT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            """
        )
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS bme680_data (
                id INT AUTO_INCREMENT PRIMARY KEY,
                temperature_c FLOAT,
                humidity_percent FLOAT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            );
            """
        )

        # Read only the last line from the cleaned data
        with open(input_file, "r", encoding="utf-8") as infile:
            lines = infile.readlines()
            if not lines:
                return  # No new data to process
            last_line = lines[-1].strip()

        match = re.search(
            r"MQ-135 PPM:\s*(\d+)\s*\|\s*ADC:\s*(\d+)\s*\|\s*Temp:\s*([\d.]+)°C\s*\|\s*Humidity:\s*([\d.]+)%",
            last_line,
        )

        if match:
            ppm, adc, temp, humidity = match.groups()

            # Insert into MQ-135 table
            cursor.execute(
                """
                INSERT INTO mq135_data (ppm, adc_value)
                VALUES (%s, %s);
                """,
                (ppm, adc),
            )

            # Insert into BME680 table
            cursor.execute(
                """
                INSERT INTO bme680_data (temperature_c, humidity_percent)
                VALUES (%s, %s);
                """,
                (temp, humidity),
            )

        # Commit the transaction
        connection.commit()
    except mysql.connector.Error as e:
        print(f"Error: {e}")
    finally:
        if cursor:
            cursor.close()
        if connection:
            connection.close()

# Start monitoring the input file
observer = Observer()
event_handler = FileChangeHandler()
observer.schedule(event_handler, ".", recursive=False)
observer.start()

print("Monitoring formatted_output.txt for changes... Press Ctrl+C to stop.")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Stopping observer...")
    observer.stop()
observer.join()
