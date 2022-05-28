import sys
import pickle

RANGES = [(-180, 14.999, 0), # (lower, upper, target)
          (15.000, 44.999, 30),
          (45.000, 74.999, 60),
          (75.000, 180, 90)]

def fix_line(line):
    new_line = line
    for r in RANGES:
        if r[0] <= line[1][2] <= r[1]:
            # changing only the pitch value of mpu 3
            new_line = (line[0], (line[1][0], line[1][1], r[2], line[1][2]))
            break
    return new_line

def quantize(log_pkl):
    with open(log_pkl, "rb") as f:
        source = pickle.load(f)
    
    out = []
    for line in source:
        new_line = fix_line(line)
        if new_line[1][2] not in [x[2] for x in RANGES]:
            print("WARNING: line", new_line, "not quantized correctly")
        out.append(new_line)
        
    
    return out


def main():
    if len(sys.argv) != 2:
        print("Usage: python quantizer.py <log.pkl>")
        return

    output = quantize(sys.argv[1])
    with open(sys.argv[1] + ".quant.pkl", "wb") as f:
        pickle.dump(output, f)


if __name__ == "__main__":
    main()
