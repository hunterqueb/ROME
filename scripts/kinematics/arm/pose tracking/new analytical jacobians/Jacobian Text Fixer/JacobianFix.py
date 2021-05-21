# replace last occurrence of a string old in a total string s
def rreplace(s, old, new, occurrence):
    li = s.rsplit(old, occurrence)
    return new.join(li)

# open file and read its contents into a string variable
f = open('Jacobian0.txt', 'r')
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

Jac = "function J = Jacobian0(theta0)\n"

Jac = Jac + "d1 = 169.77;    a1 = 64.2;  alpha1 = -90*pi/180;\nd2 = 0;         a2 = 305;   alpha2 = 0;\nd3 = 0;         a3 = 0;     alpha3 = 90*pi/180;\nd4 = -222.63;   a4 = 0;     alpha4 = -90*pi/180;\nd5 = 0;         a5 = 0;     alpha5 = 90*pi/180;\nd6 = -36.25;    a6 = 0;     alpha6 = 0;\n"

for num in range(6):
    Jac = Jac + "theta"+str(num+1)+" = theta0("+str(num+1)+");\n"
    Jac = Jac + "cth"+str(num+1)+ " = cos(theta"+str(num+1)+");\n"
    Jac = Jac + "sal"+str(num+1)+ " = sin(alpha"+str(num+1)+");\n"
    Jac = Jac + "sth"+str(num+1)+ " = sin(theta"+str(num+1)+");\n"
    Jac = Jac + "cal"+str(num+1)+ " = cos(alpha"+str(num+1)+");\n\n"


Jac = Jac + "J = " + content
Jac = Jac + "end"

f = open('Jacobian0.m', 'w')

f.write(Jac)

f.close()
