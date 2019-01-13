import sys

file_ = open('raw_in.csv', 'r')

for line, i in zip(file_, range(0,3)):
    line = line.split(',')
    print(line)

print("\n\n\n\n\n")

for line  in file_:
    line = line.split(",")
    #print(line)
    milisec = line[0][-3:]
    xacc = line[1]
    print(int(milisec)/1000.0)
    yacc = line[2]
    print(xacc)
    print(yacc)
    xgyro = line[7]
    print(xgyro)
    ygryo = line[8]
    print(ygryo)
    #print(milisec + " " + xacc + " " + yacc + " " + xgyro + " " + ygryo+ "\n")
    

