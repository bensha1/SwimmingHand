import sys
import pickle

RANGES = [(-180, 14.999, 0.0), # (lower, upper, target)
          (15.000, 44.999, 30.0),
          (45.000, 74.999, 60.0),
          (75.000, 180, 90.0)]
INPUT_RANGES = [(-180.0, -165.0, -180.0),
                (-164.999, -135.0, -150.0),
                (-134.999, -105.0, -120.0),
                (-104.999, -75.0, -90.0),
                (-74.999, -45.0, -60.0),
                (-44.999, -15.0, -30.0),
                (-14.999, 15.0, 0.0),
                (14.999, 45.0, 30.0),
                (44.999, 75.0, 60.0),
                (74.999, 105.0, 90.0),
                (104.999, 135.0, 120.0),
                (134.999, 165.0, 150.0),
                (164.999, 180.0, 180.0)]

def fix_value(v, ranges):
    for r in ranges:
        if r[0] <= v <= r[1]:
            return r[2]
    raise ValueError()

def fix_line(line, fix_input):
    fixed_output = fix_value(line[1][2], RANGES)
    fixed_input = list(line[0])
    if fix_input == "t":
        for i in range(1, 7):
           fixed_input[i] = fix_value(line[0][i], INPUT_RANGES)
    new_line = (tuple(fixed_input), (line[1][0], line[1][1], fixed_output, line[1][2]))
    return new_line

def quantize(log_pkl, fix_input):
    with open(log_pkl, "rb") as f:
        source = pickle.load(f)
    
    out = []
    for line in source:
        try:
            new_line = fix_line(line, fix_input)
        except ValueError:
            print("WARNING: line", line, "not quantized correctly")
        out.append(new_line[0])
        out.append(new_line[1])
        
    return out


def main():
    if len(sys.argv) != 3:
        print("Usage: python quantizer.py <log.pkl> <t|f>")
        return

    output = quantize(sys.argv[1], sys.argv[2])
    with open(sys.argv[1] + ".quant.pkl", "wb") as f:
        pickle.dump(output, f)


if __name__ == "__main__":
    main()
