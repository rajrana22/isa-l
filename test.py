import os

max_iter = 5

def measure_throughput():
    throughputs = []
    for i in range(max_iter):
        os.system("./erasure_code/erasure_code_perf_mlec 16 2 8 2 128 > throughput.log")
        with open("throughput.log") as tlog:
            lines = tlog.readlines()
            for line in lines:
                if "Overall" in line:
                    desired_line = line.split("Throughput: ")[1].split(" MB/s")[0]
                    throughputs.append(float(desired_line))   
    avg_throughput = sum(throughputs) / max_iter
    return avg_throughput

def main():
    print("\nBeginning Tests...\n")
    print("\nCompiling...\n")
    os.system("make perfs")
    print("\nMeasuring Throughput Performance...\n")
    throughput = measure_throughput()
    os.system("rm -rf throughput.log")
    with open("avg-throughputs.log", 'a') as avgtlog:
        avgtlog.write(str(throughput) + "\n")

    print("\nAverage Throughput: " + str(throughput))
    
    with open("avg-throughputs.log") as avgtlog:
        for line in avgtlog:
            pass
        prev_throughput = float(line)

    delta = throughput - prev_throughput

    if (delta >= 0):
        print("\nChange in Throughput: +" + str(delta))
    else:
        print("\nChange in Throughput: -" + str(delta))

    print("\nMeasuring Cache Performance...\n")

if __name__ == "__main__":
    main()