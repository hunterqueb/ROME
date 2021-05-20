initialStates = [10,10,10,10,10,10,10,10,0]';
velocity = [5,5,5,5,5,5,5,5,5]';

JROME = J_ROME(initialStates)

JAR2 = Jacobian0_analytical(initialStates)

ROMEvel = JROME * velocity
AR2vel = JAR2 * velocity(1:6)