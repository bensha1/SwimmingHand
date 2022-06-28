import sys
import re
import pickle

FLOAT_RE = b"-?\d\.\d\d|-?\d\d\.\d\d"

def parse_log(log_file):
    with open(log_file, "rb") as f:
        lines = f.readlines()

    full_output = []
    merged_input = []
    for line in lines:
        if line.startswith(b"mpu 1 ypr"):
            ypr = [float(x) for x in re.findall(FLOAT_RE, line)]
            merged_input = [1]
            merged_input.extend(ypr)
        elif line.startswith(b"mpu 2 ypr"):
            ypr = [float(x) for x in re.findall(FLOAT_RE, line)]
            merged_input.extend(ypr)
            full_output.append(tuple(merged_input))
        elif line.startswith(b"mpu 3 ypr"):
            ypr = [float(x) for x in re.findall(FLOAT_RE, line)]
            output = [3]
            output.extend(ypr)
            full_output.append((tuple(merged_input), tuple(output)))
    
    # validations
    # for i, line in enumerate(full_output):
        # if (len(line) != 2 or len(line[0]) != 7 or len(line[1]) != 4
                # or line[0][0] != 1 or line[1][0] != 3):
            # print("dropping line: ", full_output.pop(i))

    return full_output

def main():
    if len(sys.argv) != 2:
        print("Usage: python log_parser.py <log_file>")
        return

    output = parse_log(sys.argv[1])
    with open(sys.argv[1] + ".pkl", "wb") as f:
        pickle.dump(output, f)

if __name__ == "__main__":
    main()
