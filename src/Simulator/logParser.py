#!/usr/bin/env python3

def parse():
    f = open("log.txt", "r")
    lines = f.readlines()
    result = []
    for x in lines:
        data = x.split(' ')
        point = (float(data[1][:-1]), float(data[2][:-1]))
        result.append(point)

    f.close()
    return result

if __name__ == "__main__":
    parse()