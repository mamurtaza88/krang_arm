This Repository computes tracking using Force based method and then augmenting it to solve for the controller.


The details are as follow:

M * ddq + h = Gamma       --------------- Eq (a)

xddot = Jdot * qdot + J * ddq ------------ eq (b)

Putting value of ddq from Eq (a) into Eq (b)
xddot = Jdot * qdot + J * M_inverse * ( Gamma - h)
=>  xddot = Jdot * qdot - J * M_inverse * h + J * M_inverse * Gamma

Since Gamma = J_transpose*F     and    F = M_e*u (To distinguish it from M already present. Its M related to task space and u can be desired  acceleration in task space)
we can do
 xddot = Jdot * qdot - J * M_inverse * h + J * M_inverse * J_transpose * M_e*u       ------------------ eq (c)


Now let us suppose M_e = (J*M_inverse*J_transpose)^{-1}        (Whole inverse of the full)
           and     u   = ddx_desired + J * M_inverse*h - J_dot * qdot

This will yield 
xddot = ddx_desired


So actual control Gamma = J_transpose * M_e*u 
                => Gamma = J_transpose * (J*M_inverse*J_transpose)^{-1} * u


Results were not good enough. This was taken from studywolf blogspot. Control in operational space https://studywolf.wordpress.com/2013/09/17/robot-control-4-operation-space-control/

However, if we apply
Gamma = J_transpose * M_e*u + h ...   and u = ddx_desired ( or u = ddx_desired - J_dot * qdot). Then it works



To do multiple task space, we do the following.

We know

xddot_1 = f_1 + g_1*Gamma_1

where, Gamma_1 =  J_transpose * M_e1*u_1 (for one task will yield the desired result)   
           f_1 = Jdot * qdot - J * M_inverse * h 
           g_1 = J * M_inverse 

So for multiple task, we do the following

xddot = F + G*Gamma

where F = [f_1;f_2]
      G = diag(g_1,g_2)

Now if we set  Gamma = J_aug_transpose*M_aug_inverse*u_aug_desired + h   ( Note +h is anamoly but it works)

where
J_aug = [J_position;J_orientation]
M_aug = diag(M_e1,M_e2)
u_aug_desired = [ddxref - dJv*dq;dwref -dJw*dq];

RESCLF has not been implemented


