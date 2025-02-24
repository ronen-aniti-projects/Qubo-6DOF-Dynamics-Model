clear 
clc

% Constants
syms m g

% Gravity force
Fg = m*g;

% Inertia
syms Ixx Ixy Ixz 
syms Iyx Iyy Iyz 
syms Izx Izy Izz 

% State variables
syms x xdot xddot 
syms y ydot yddot 
syms z zdot zddot 
syms phi phid 
syms theta thetad 
syms psi psid 
syms p pdot 
syms q qdot 
syms r rdot 

% Center of volume
syms xrv yrv zrv

% Center of volume as vector
rv = [xrv; yrv; zrv];

% Center of mass 
syms xrm yrm zrm 

% Center of mass as vector
rm = [xrm; yrm; zrm];

% Thrust forces
syms u1 u2 u3 u4 u5 u6 u7 u8 

% Thrust forces as an array
u = [u1, u2, u3, u4, u5, u6, u7, u8];

% Thrust application points
syms xru1 yru1 zru1 
syms xru2 yru2 zru2 
syms xru3 yru3 zru3 
syms xru4 yru4 zru4 
syms xru5 yru5 zru5 
syms xru6 yru6 zru6 
syms xru7 yru7 zru7 
syms xru8 yru8 zru8

% Vectorized thrust application points
ru1 = [xru1; yru1; zru1];
ru2 = [xru2; yru2; zru2];
ru3 = [xru3; yru3; zru3];
ru4 = [xru4; yru4; zru4];
ru5 = [xru5; yru5; zru5];
ru6 = [xru6; yru6; zru6];
ru7 = [xru7; yru7; zru7];
ru8 = [xru8; yru8; zru8];

% Thrust application points as an array of vectors
ru = [ru1, ru2, ru3, ru4, ru5, ru6, ru7, ru8];

% Thrust body-frame orientation vector (normalized)
syms xehatu1 yehatu1 zehatu1
syms xehatu2 yehatu2 zehatu2
syms xehatu3 yehatu3 zehatu3
syms xehatu4 yehatu4 zehatu4
syms xehatu5 yehatu5 zehatu5
syms xehatu6 yehatu6 zehatu6
syms xehatu7 yehatu7 zehatu7
syms xehatu8 yehatu8 zehatu8

% Thrust body-frame orientation vectors (normalized)
ehatu1 = [xehatu1; yehatu1; zehatu1];
ehatu2 = [xehatu2; yehatu2; zehatu2];
ehatu3 = [xehatu3; yehatu3; zehatu3];
ehatu4 = [xehatu4; yehatu4; zehatu4];
ehatu5 = [xehatu5; yehatu5; zehatu5];
ehatu6 = [xehatu6; yehatu6; zehatu6];
ehatu7 = [xehatu7; yehatu7; zehatu7];
ehatu8 = [xehatu8; yehatu8; zehatu8];

% Thrust body-frame orientation vectors as an array of vectors
ehatu = [ehatu1, ehatu2, ehatu3, ehatu4, ehatu5, ehatu6, ehatu7, ehatu8];

% Torques
syms taux tauy tauz 

% Torques as vector
tau = [taux; tauy; tauz];

% Angular damping constants
syms Cphi Ctheta Cpsi 

% Angular damping constants as array
C_angular = [Cphi; Ctheta; Cpsi];

% Linear damping constants 
syms Cx Cy Cz

% Linear damping constants as array 
C_linear = [Cx; Cy; Cz];

% Buoyancy force magnitude
syms Fb 

% Buoyancy force vector (world frame)
Fb_vector = [0; 0; Fb];


% Inertia tensor
I = [Ixx, Ixy, Ixz;
     Iyx, Iyy, Iyz;
     Izx, Izy, Izz 
];

% Rotation matrices: body->world: body_vector = Rx Ry Rz world_vector
Rx = [1, 0, 0;
      0, cos(phi), -sin(phi);
      0, sin(phi), cos(phi)
];

Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
      -sin(theta), 0, cos(theta) 
];

Rz = [cos(psi), -sin(psi), 0;
      sin(psi), cos(psi), 0;
      0, 0, 1
];


R = Rx * Ry * Rz; % Composite rotation

% Angular acceleration vector
omegadot = [pdot; qdot; rdot];

% Angular rate vector
omega = [p; q; r];

% Velocity
velocity = [xdot; ydot; zdot];

% Sum of thruster torques
thruster_torques = [0; 0; 0];
for i = 1:8
    thruster_torques = thruster_torques + cross(ru(:,i),u(i)*ehatu(:,i));
end

% Sum of thruster forces
thruster_forces = [0; 0; 0];
for i=1:8
    thruster_forces = thruster_forces + u(i) * ehatu(:, i);
end

% Right-hand side expression for angular dynamics
tauRHS = I * omegadot + cross(omega, I*omega);

% Left-hand side expressions for angular dynamics
tauLHS = thruster_torques + cross(rv, transpose(R) * Fb_vector) - C_angular .* omega;

% Isolate for omega dot but don't plug in tau yet
omega_dot_expression = inv(I) * (tau - cross(omega, I*omega));

% Rotational equations for angular dynamics with omega dot isolated
rotational_equations = subs(omega_dot_expression, [taux tauy tauz],[tauLHS(1) tauLHS(2) tauLHS(3)]);

% Linear dynamics 
linear_equations = 1/m * (Fb * [0;0;1] + Fg * [0;0;-1] + R*thruster_forces - C_linear .* velocity);

% Euler rates 
T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
eulerRates = T * [p;q;r]; % Transform body rates to Euler rates

% State space model
state_vector_expressions = [velocity; eulerRates; linear_equations; rotational_equations];

simplify(state_vector_expressions)

