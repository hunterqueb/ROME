# replace last occurrence of a string old in a total string s
def rreplace(s, old, new, occurrence):
    li = s.rsplit(old, occurrence)
    return new.join(li)


def main():
    # open file and read its contents into a string variable
    f = open('B.txt', 'r')
    content = f.read()
    f.close()

    # replace operations in jacobion
    for num in range(6):
        content = content.replace("cos(theta"+str(num+1)+")","cth"+str(num+1))
        content = content.replace("sin(alpha"+str(num+1)+")","sal"+str(num+1))
        content = content.replace("sin(theta"+str(num+1)+")","sth"+str(num+1))
        content = content.replace("cos(alpha"+str(num+1)+")","cal"+str(num+1))


    # fix formatting
    content = content.replace("], ","];\n")
    content = content.replace("matrix(","")
    content = rreplace(content,')',';\n',1)
    # remember to remove to last close parenthesis on the matrix

    Jac = "function matrix = B_AR2(theta0)\n"

    Jac = Jac + "d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;\nd2 = 0;         a2 = 305;   alpha2 = 0;\nd3 = 0;         a3 = 0;     alpha3 = 90*pi/180;\nd4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;\nd5 = 0;         a5 = 0;     alpha5 = 90*pi/180;\nd6 = -36.25;    a6 = 0;     alpha6 = 0;\n"

    Jac = Jac + "\nm1 = 0.88065;   m2 = 0.57738;    m3 = 0.1787;    m4 = 0.34936;    m5 = 0.11562;    m6 = 0.013863;\ng=1000*9.81; %==should this be in mm/s^2?\n"
    Jac = Jac + "\npc1 = [0 0 0];   pc2 = 1000*[0.064818 -0.11189 -0.038671];    pc3 = 1000*[-0.00029765 -0.023661 -0.0019125];    pc4 = 1000*[-0.0016798 -0.00057319 -0.074404];    pc5 = 1000*[2.9287E-10 -1.6472E-09 0.0091432];    pc6 = 1000*[-0.000294 0 0.02117];\n"
    Jac = Jac + "\nIc1 = 1000*1000*[0.0034 0.00042296 -0.00089231;\n0.00042296 0.00042296 0.0010848;\n-0.00089231 0.0010848 0.0027077];\nIc2 = 1000*1000*[0.0047312 0.0022624 0.00032144;\n0.0022624 0.0020836 -0.00056569;\n0.00032144 -0.00056569 0.0056129];\n"
    Jac = Jac + "Ic3 = 1000*1000*[0.0001685 -2.7713E-05 5.6885E-06;\n-2.7713E-05 0.00012865 2.9256E-05;\n5.6885E-06 2.9256E-05 0.00020744];\nIc4 =1000*1000*[0.0030532 -1.8615E-05 -7.0047E-05;\n-1.8615E-05 0.0031033 -2.3301E-05;\n-7.0047E-05 -2.3301E-05 0.00022264];\n"
    Jac = Jac + "Ic5 = 1000*1000*[5.5035E-05 -1.019E-08 -2.6243E-06;\n-1.019E-08 8.2921E-05 1.4437E-08;\n-2.6243E-06 1.4437E-08 5.2518E-05];\nIc6 =1000*1000*[1.3596E-06 3.0585E-13 5.7102E-14;\n3.0585E-13 1.7157E-06 6.3369E-09;\n5.7102E-14 6.3369E-09 2.4332E-06];\n"

    Jac = Jac + "pc1_1 = pc1(1); pc1_2 = pc1(2); pc1_3 = pc1(3);\n"
    Jac = Jac + "pc2_1 = pc2(1); pc2_2 = pc2(2); pc2_3 = pc2(3);\n"
    Jac = Jac + "pc3_1 = pc3(1); pc3_2 = pc3(2); pc3_3 = pc3(3);\n"
    Jac = Jac + "pc4_1 = pc4(1); pc4_2 = pc4(2); pc4_3 = pc4(3);\n"
    Jac = Jac + "pc5_1 = pc5(1); pc5_2 = pc5(2); pc5_3 = pc5(3);\n"
    Jac = Jac + "pc6_1 = pc6(1); pc6_2 = pc6(2); pc6_3 = pc6(3);\n"

    Jac = Jac + "Ic1_1_1 = Ic1(1,1); Ic1_1_2 = Ic1(1,2); Ic1_1_3 = Ic1(1,3);\n"
    Jac = Jac + "Ic1_2_1 = Ic1(2,1); Ic1_2_2 = Ic1(2,2); Ic1_2_3 = Ic1(2,3);\n"
    Jac = Jac + "Ic1_3_1 = Ic1(3,1); Ic1_3_2 = Ic1(3,2); Ic1_3_3 = Ic1(3,3);\n"
    
    Jac = Jac + "Ic2_1_1 = Ic2(1,1); Ic2_1_2 = Ic2(1,2); Ic2_1_3 = Ic2(1,3);\n"
    Jac = Jac + "Ic2_2_1 = Ic2(2,1); Ic2_2_2 = Ic2(2,2); Ic2_2_3 = Ic2(2,3);\n"
    Jac = Jac + "Ic2_3_1 = Ic2(3,1); Ic2_3_2 = Ic2(3,2); Ic2_3_3 = Ic2(3,3);\n"
    
    Jac = Jac + "Ic3_1_1 = Ic3(1,1); Ic3_1_2 = Ic3(1,2); Ic3_1_3 = Ic3(1,3);\n"
    Jac = Jac + "Ic3_2_1 = Ic3(2,1); Ic3_2_2 = Ic3(2,2); Ic3_2_3 = Ic3(2,3);\n"
    Jac = Jac + "Ic3_3_1 = Ic3(3,1); Ic3_3_2 = Ic3(3,2); Ic3_3_3 = Ic3(3,3);\n"

    Jac = Jac + "Ic4_1_1 = Ic4(1,1); Ic4_1_2 = Ic4(1,2); Ic4_1_3 = Ic4(1,3);\n"
    Jac = Jac + "Ic4_2_1 = Ic4(2,1); Ic4_2_2 = Ic4(2,2); Ic4_2_3 = Ic4(2,3);\n"
    Jac = Jac + "Ic4_3_1 = Ic4(3,1); Ic4_3_2 = Ic4(3,2); Ic4_3_3 = Ic4(3,3);\n"

    Jac = Jac + "Ic5_1_1 = Ic5(1,1); Ic5_1_2 = Ic5(1,2); Ic5_1_3 = Ic5(1,3);\n"
    Jac = Jac + "Ic5_2_1 = Ic5(2,1); Ic5_2_2 = Ic5(2,2); Ic5_2_3 = Ic5(2,3);\n"
    Jac = Jac + "Ic5_3_1 = Ic5(3,1); Ic5_3_2 = Ic5(3,2); Ic5_3_3 = Ic5(3,3);\n"

    Jac = Jac + "Ic6_1_1 = Ic6(1,1); Ic6_1_2 = Ic6(1,2); Ic6_1_3 = Ic6(1,3);\n"
    Jac = Jac + "Ic6_2_1 = Ic6(2,1); Ic6_2_2 = Ic6(2,2); Ic6_2_3 = Ic6(2,3);\n"
    Jac = Jac + "Ic6_3_1 = Ic6(3,1); Ic6_3_2 = Ic6(3,2); Ic6_3_3 = Ic6(3,3);\n"


    for num in range(6):
        Jac = Jac + "theta"+str(num+1)+" = theta0("+str(num+1)+");\n"
        Jac = Jac + "cth"+str(num+1)+ " = cos(theta"+str(num+1)+");\n"
        Jac = Jac + "sal"+str(num+1)+ " = sin(alpha"+str(num+1)+");\n"
        Jac = Jac + "sth"+str(num+1)+ " = sin(theta"+str(num+1)+");\n"
        Jac = Jac + "cal"+str(num+1)+ " = cos(alpha"+str(num+1)+");\n\n"


    Jac = Jac + "matrix = " + content
    Jac = Jac + "end"

    f = open('B_AR2.m', 'w')

    f.write(Jac)

    f.close()

if __name__ == "__main__":
    main()
