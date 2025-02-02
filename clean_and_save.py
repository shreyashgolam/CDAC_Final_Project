import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# Define the input and output files
input_file = "minicom_output.txt"
output_file = "cleaned_output.txt"

class FileChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        # Check if the modified file is the input file
        if event.src_path.endswith(input_file):
            print(f"Detected change in {input_file}, processing...")
            self.clean_and_save()

    def clean_and_save(self):
        try:
            # Open the input file using ISO-8859-1 encoding
            with open(input_file, "r", encoding="ISO-8859-1") as infile:
                # Read the content and replace unwanted characters
                content = infile.read().replace("Â", "")

            # Write the cleaned content to a new file in UTF-8 encoding
            with open(output_file, "w", encoding="utf-8") as outfile:
                outfile.write(content)

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
