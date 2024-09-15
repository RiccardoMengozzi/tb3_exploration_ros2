import os
file_path = "src/sanification/files/goal_sequence.txt"

# Get the absolute path by joining with the current working directory
file_path = os.path.join(os.getcwd(), file_path)
print(file_path)
try:
    with open(file_path, 'r') as file:
        print("diocane")
        first_line = file.readline().strip()  # Read and strip any trailing newline characters
    print(first_line)
except FileNotFoundError:
    print("The file was not found.")
except Exception as e:
    print(f"An error occurred: {e}")
