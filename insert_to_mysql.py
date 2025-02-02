import mysql.connector

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

# Parse data and insert into MySQL
def insert_data_to_mysql():
    try:
        # Connect to the MySQL database
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Create tables if not exist
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS mq135_data (
            id INT AUTO_INCREMENT PRIMARY KEY,
            adc_value FLOAT,
            rs_kohms FLOAT,
            concentration_ppm FLOAT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        );
        """)
        cursor.execute("""
        CREATE TABLE IF NOT EXISTS bme680_data (
            id INT AUTO_INCREMENT PRIMARY KEY,
            temperature_c FLOAT,
            pressure_pa FLOAT,
            humidity_percent FLOAT,
            gas_resistance_ohms FLOAT,
            iaq_score FLOAT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        );
        """)

        # Read the cleaned data
        with open(input_file, "r", encoding="utf-8") as infile:
            lines = infile.readlines()

        for line in lines:
            line = line.strip()

            if line.startswith("MQ-135 ADC Value:"):
                # Extract MQ-135 data
                parts = line.split(", ")
                adc_value = rs_kohms = concentration_ppm = None
                for part in parts:
                    if "ADC Value" in part:
                        adc_value = float(part.split(": ")[1])
                    elif "Rs" in part:
                        rs_kohms = float(part.split(": ")[1].split()[0])
                    elif "Concentration" in part:
                        concentration_ppm = float(part.split(": ")[1].split()[0])

                # Insert into MQ-135 table
                cursor.execute("""
                INSERT INTO mq135_data (adc_value, rs_kohms, concentration_ppm)
                VALUES (%s, %s, %s);
                """, (adc_value, rs_kohms, concentration_ppm))

            elif line.startswith("Temperature (°C):"):
                # Extract BME680 data
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

                # Insert into BME680 table
                cursor.execute("""
                INSERT INTO bme680_data (temperature_c, pressure_pa, humidity_percent, gas_resistance_ohms, iaq_score)
                VALUES (%s, %s, %s, %s, %s);
                """, (temperature_c, pressure_pa, humidity_percent, gas_resistance_ohms, iaq_score))

        # Commit the transactions
        connection.commit()

    except mysql.connector.Error as e:
        print(f"Error: {e}")
    finally:
        # Close the cursor and connection
        if cursor:
            cursor.close()
        if connection:
            connection.close()

if __name__ == "__main__":
    insert_data_to_mysql()
    print("Data insertion completed.")
