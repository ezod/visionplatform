from sys import argv

if __name__ == '__main__':
    with open(argv[1], 'r') as f:
        for line in f.readlines():
            values = [float(c) * float(argv[2]) for c in line.rstrip().split(' ')[:9]]
            for p in range(3):
                for i in range(3):
                    values[p * 3 + i] += float(argv[3 + i])
            print ' '.join([str(v) for v in values])
