% The symbols are
syms M m1 m2 l1 l2 g;
% State space representation of the linearised system
A=[0 1 0 0 0 0;
    0 0 -(m1*g)/M 0 -(m2*g)/M 0;
    0 0 0 1 0 0;
    0 0 -((M+m1)*g)/(M*l1) 0 -(m2*g)/(M*l1) 0;
    0 0 0 0 0 1;
    0 0 -(m1*g)/(M*l2) 0 -((M+m2)*g)/(M*l2) 0];

B=[0; 1/M; 0; 1/(M*(l1)); 0; 1/(M*l2)];

% Controllability matrix C
C= [B A*B A*A*B A*A*A*B A*A*A*A*B A*A*A*A*A*B];
disp("The Controllablity matrix C ="); disp(C);
disp("The Determinant of C is "); disp(simplify(det(C)));
disp("The Rank of C is "); disp(rank(C));

% Check Controllablity
if rank(C) == 6
    disp('System is controllable')
else
    disp('System is not controllable')
end

% Substituting l1=l2
disp("When l1 = l2, the controllability matrix is")

% Subs function makes l1 = l2
C1 = subs(C,l1,l2); 
disp(C1);
disp("The Determinant of C1 is "); disp(simplify(det(C1)));
disp("The Rank of C1 is "); disp(rank(C1));

% Check Controllablity
if rank(C1) == 6
    disp('System is controllable')
else
    disp('System is not controllable')
end