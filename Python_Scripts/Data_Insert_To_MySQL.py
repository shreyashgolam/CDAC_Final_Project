import mysql.connector
import re

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


# Parse data and insert into MySQL
def insert_data_to_mysql():
    try:
        # Connect to the MySQL database
        connection = mysql.connector.connect(**db_config)
        cursor = connection.cursor()

        # Create tables if they don't exist
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

        # Read the cleaned data
        with open(input_file, "r", encoding="utf-8") as infile:
            lines = infile.readlines()

        for line in lines:
            line = line.strip()
            match = re.search(
                r"MQ-135 PPM:\s*(\d+)\s*\|\s*ADC:\s*(\d+)\s*\|\s*Temp:\s*([\d.]+)°C\s*\|\s*Humidity:\s*([\d.]+)%",
                line,
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
