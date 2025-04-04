import csv

# Input file (your txt file)
input_file = '../Map/two_walls_points_w.txt'

# Output file (where you want to save the csv)
output_file = '../Map/two_walls_points_w.csv'

with open(input_file, 'r') as txt_file, open(output_file, 'w', newline='') as csv_file:
    # Create a CSV writer object
    csv_writer = csv.writer(csv_file)
    
    # Read each line of the txt file
    for line in txt_file:
        # Skip lines that start with '#'
        if line.startswith('#'):
            continue
        
        # Split the line by whitespace and convert to floats
        values = line.split()
        try:
            # Write to CSV
            csv_writer.writerow([float(value) for value in values])
        except ValueError:
            print(f"Skipping line due to conversion error: {line}")

print(f"Converted {input_file} to {output_file}")