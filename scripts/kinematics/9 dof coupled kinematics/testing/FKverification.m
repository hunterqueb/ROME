intitialStates = [10,10,10,10,10,10,10,10,0];

[position1,orientation1] = ROMEFK(intitialStates);

[position2,orientation2] = AR2FKZYZ(intitialStates);

pose1 = [position1;orientation1]

pose2 = [position2;orientation2]