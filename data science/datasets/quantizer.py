import sys
import pickle
import numpy
import argparse

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
INTERP_SRC = [-45, 45]
INTERP_DST = [-15, 75]

def quantize_value(v, ranges):
    for r in ranges:
        if r[0] <= v <= r[1]:
            return r[2]
    raise ValueError()

def fix_line(line, fix_input, calibrate_pitch, interp):
    pitch_value = line[1][2]
    if calibrate_pitch:
        # pitch     = mpu3 pitch  - mpu1 pitch ( line = ((1, y, p, r, y, p, r), (3, y, p, r)) )
        pitch_value = pitch_value - line[0][2]

    if interp:
        pitch_value = numpy.interp(pitch_value, INTERP_SRC, INTERP_DST)

    fixed_output = quantize_value(pitch_value, RANGES)
    
    fixed_input = list(line[0])
    if fix_input:
        for i in range(1, 7):
           fixed_input[i] = quantize_value(line[0][i], INPUT_RANGES)

    new_line = (tuple(fixed_input), (line[1][0], line[1][1], fixed_output, line[1][2]))
    return new_line

def quantize(log_pkl, fix_input, calibrate_pitch, interp):
    with open(log_pkl, "rb") as f:
        source = pickle.load(f)
    
    out = []
    for line in source:
        try:
            new_line = fix_line(line, fix_input, calibrate_pitch, interp)
        except ValueError:
            print("WARNING: line", line, "not quantized correctly")

        out.append(new_line[0])
        out.append(new_line[1])
        
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=str, help="name of source file. should be a pickle generated by log_parser.py")
    parser.add_argument("-i", "--input", help="quantize input values (of mpu1 and mpu2)", action="store_true")
    parser.add_argument("-p", "--pitch", help="calibrate mpu3 pitch values based on mpu1", action="store_true")
    parser.add_argument("-m", "--map", help="map the values of mpu3 pitch to a different range", action="store_true")
    parser.add_argument("-o", "--output", help="name of output file") 
    args = parser.parse_args()

    output = quantize(args.source, args.input, args.pitch, args.map)

    if args.output is not None:
        out_file = args.output
    else:
        out_file = args.source + ".quant.pkl"
        
    with open(out_file, "wb") as f:
        pickle.dump(output, f)


if __name__ == "__main__":
    main()
