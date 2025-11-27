import serial
from collections import Counter

def parse_value(val):
    """Try to parse as int, then float, otherwise keep as string"""
    try:
        return int(val)
    except ValueError:
        try:
            return float(val)
        except ValueError:
            return val

def find_stable_length(samples, min_occurrences=5):
    """Find the most common data length after seeing it enough times"""
    length_counts = Counter(samples)
    for length, count in length_counts.most_common():
        if count >= min_occurrences:
            return length
    return None

port_serial = serial.Serial('/dev/ttyACM0', 115200)  # Fixed baud rate typo
data = []
initial_samples = []
expected_length = None
startup_jitter_detected = False

print("Starting data collection...")

while len(data) < 10000:
    try:
        line = port_serial.readline().decode('utf-8').strip()
        
        if not line:  # Skip empty lines
            continue
            
        points = line.split(',')
        current_length = len(points)
        
        # Detect expected length from first stable samples
        if expected_length is None:
            initial_samples.append(current_length)
            if len(initial_samples) >= 20:  # Check after 20 samples
                expected_length = find_stable_length(initial_samples)
                if expected_length:
                    print(f"Detected stable format: {expected_length} values per line")
                    startup_jitter_detected = True
                else:
                    initial_samples = initial_samples[-10:]  # Keep sliding window
            continue
        
        # Filter out lines with wrong length
        if current_length != expected_length:
            print(f"Warning: Skipping line with {current_length} values (expected {expected_length})")
            continue
        
        # Parse values with type detection
        parsed_points = [parse_value(point) for point in points]
        data.append(parsed_points)
        
        # Progress indicator
        if len(data) % 1000 == 0:
            print(f"Collected {len(data)} valid lines...")
            
    except (UnicodeDecodeError, ValueError) as e:
        print(f"Error parsing line: {e}")
        continue
    except KeyboardInterrupt:
        print("\nCollection interrupted by user")
        break

port_serial.close()

# Save data
with open('raw-data.txt', 'w') as f:
    for row in data:
        f.write(','.join(map(str, row)) + '\n')

print(f"\nData collection complete. {len(data)} valid lines saved to raw-data.txt")
print(f"Expected format: {expected_length} values per line")

