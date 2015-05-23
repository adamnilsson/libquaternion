% syms Rx Ry Rz
% Qx = quaternion.rotateAxis(Rx,[1 0 0])
% Qy = quaternion.rotateAxis(Ry,[0 1 0])
% Qz = quaternion.rotateAxis(Rz,[0 0 1])
% Q = (Qx*Qy*Qz + Qx*Qz*Qy + Qy*Qx*Qz + Qy*Qz*Qx + Qz*Qx*Qy + Qz*Qy*Qx)*(1/6)

% Rx = 5*pi/180
% Ry = 5*pi/180
% Rz = 5*pi/180
% 
% Q = quaternion([cos(Rx/2)*cos(Ry/2)*cos(Rz/2), cos(Ry/2)*cos(Rz/2)*sin(Rx/2), cos(Rx/2)*cos(Rz/2)*sin(Ry/2), cos(Rx/2)*cos(Ry/2)*sin(Rz/2)])
% 
% Q.draw()

be = beam()

be.position = dualquaternion(quaternion(cos(pi/8), sin(pi/8)*[0 1 0]), [-1 0 0])
be.L = 2
be.U(1:3) = [1 0 0]'
be.U(4:6) = [0 10 0]*pi/180
be.U(7:9) = [0 0 0]'
be.U(10:12) = [0 10 20]*pi/180
DQs = be.deformed_start()
DQe = be.deformed_end()
clf
xlabel('x')
ylabel('y')
zlabel('z')
xlim([-2 2])
ylim([-2 2])
zlim([-2 2])
grid on
view(3)

dQz = dualquaternion([1, 0, 0, 0], [0, 0, 0, .5])
dDQs = DQs.S*quaternion([0 0 1]*be.L)*DQs.S'
dDQe = DQe.S*quaternion([0 0 1]*be.L)*DQe.S'
Y = [DQs.S.vector4() zeros(4,1) DQe.S.vector4() zeros(4,1);
     DQs.translation().vector3() dDQs.vector3() DQe.translation().vector3() dDQe.vector3()]


P = Y/[1 -3 0 0; 0 3 0 0; 0 0 0 -3; 0 0 1 3]
x = linspace(0,1, 25)
X = [(1-x).^3; 3*x.*(1-x).^2;  3*x.^2.*(1-x);  x.^3]
BEZ = P*X
for i=1:length(x)
    S = quaternion(BEZ(1:4,i)/norm(BEZ(1:4,i)))
    Qi = dualquaternion(S, quaternion(BEZ(5:7,i)/2)*S)
    Qi.draw('AxisSize', .3)
end

% T = dualquaternion(quaternion(), [1 0 0])
% be.position.draw('AxisSize', .5)
% DQs.draw()
% be.deformed_end().draw('AxisSize', .5)
% be.undeformed_end().draw('AxisSize', .5)
% DQt = T*DQs
% DQt.draw()
