import time
import re
import chardet
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Define the input and output files
input_file = "minicom_to_output.txt"
output_file = "formatted_output.txt"

class FileChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        # Check if the modified file is the input file
        if event.src_path.endswith(input_file):
            print(f"Detected change in {input_file}, processing...")
            self.clean_and_save()

    def detect_encoding(self, file_path):
        """Detect file encoding using chardet."""
        try:
            with open(file_path, "rb") as f:
                raw_data = f.read()
                result = chardet.detect(raw_data)
                return result["encoding"]  # Return detected encoding
        except Exception as e:
            print(f"Encoding detection failed: {e}")
            return "utf-8"  # Default to UTF-8 if detection fails

    def clean_and_save(self):
        try:
            # Detect encoding dynamically
            detected_encoding = self.detect_encoding(input_file)
            print(f"Detected encoding: {detected_encoding}")

            with open(input_file, "r", encoding=detected_encoding, errors="replace") as infile:
                lines = infile.readlines()

            formatted_lines = []
            for line in lines:
                print(f"RAW LINE: {repr(line)}")  # Debugging: Show raw line content
                
                line = line.replace("Â", "").strip()  # Remove unwanted characters
                
                # More flexible regex to handle variations
                match = re.search(r'MQ-135\s*PPM:\s*(\d+)\s*\|\s*ADC:\s*(\d+)\s*\|\s*Temp:\s*([\d.]+)[^\d]*\|\s*Humidity:\s*([\d.]+)', line)

                if match:
                    ppm, adc, temp, humidity = match.groups()
                    formatted_line = f"MQ-135 PPM: {ppm} | ADC: {adc} | Temp: {temp}°C | Humidity: {humidity}%\n"
                    formatted_lines.append(formatted_line)
                else:
                    print(f"DEBUG: No match for -> {repr(line)}")  # Debugging output

            # Write cleaned data to the output file
            with open(output_file, "w", encoding="utf-8") as outfile:
                outfile.writelines(formatted_lines)

            print(f"File cleaned and saved as '{output_file}'.")
        except Exception as e:
            print(f"Error processing file: {e}")

# Set up the observer and handler
event_handler = FileChangeHandler()
observer = Observer()
observer.schedule(event_handler, ".", recursive=False)

try:
    print(f"Monitoring {input_file} for changes...")
    observer.start()
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nStopping observer...")
    observer.stop()

observer.join()
