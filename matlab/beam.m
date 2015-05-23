classdef beam
    properties
        stiff_params
        K
        U
        position
        L
    end
    methods
        function self = beam()
            % 
            % 
            self.position = dualquaternion();
            self.L = 1;
            self.U = zeros(12,1);
            % syms EIx EIy EIxy EA GJ L
            self.stiff_params = [3 4 1 8 2 ];
            EIx = self.stiff_params(1);
            EIy = self.stiff_params(2);
            EIxy = self.stiff_params(3);
            EA = self.stiff_params(4);
            GJ = self.stiff_params(5);
            L = 1;
            self.K = [[12/L^3*[EIy EIxy; EIxy EIx] [0; 0] 6/L^2*[EIxy EIy; EIx EIxy] [0; 0] -12/L^3*[EIy EIxy; EIxy EIx] [0; 0] -6/L^2*[EIxy EIy; EIx EIxy] [0; 0]];
                      [0 0 EA 0 0 0 0 0 -EA 0 0 0];
                      [6/L^2*[EIxy EIx; EIy EIxy] [0; 0] 4/L*[EIx EIxy; EIxy EIy] [0; 0] -6/L^2*[EIxy EIx; EIy EIxy] [0; 0] 2/L*[EIx EIxy; EIxy EIy] [0; 0]];
                      [0 0 0 0 0 GJ 0 0 0 0 0 -GJ];
                      [-12/L^3*[EIy EIxy; EIxy EIx] [0; 0] -6/L^2*[EIxy EIy; EIx EIxy] [0; 0] 12/L^3*[EIy EIxy; EIxy EIx] [0; 0] -6/L^2*[EIxy EIy; EIx EIxy] [0; 0]];
                      [0 0 -EA 0 0 0 0 0 EA 0 0 0];
                      [-6/L^2*[EIxy EIx; EIy EIxy] [0; 0] 2/L*[EIx EIxy; EIxy EIy] [0; 0] -6/L^2*[EIxy EIx; EIy EIxy] [0; 0] 4/L*[EIx EIxy; EIxy EIy] [0; 0]];
                      [0 0 0 0 0 -GJ 0 0 0 0 0 GJ];];
        end
        function DQ = deformed_start(self)
            Rx = self.U(4);
            Ry = self.U(5);
            Rz = self.U(6);
            Q = quaternion([cos(Rx/2)*cos(Ry/2)*cos(Rz/2), cos(Ry/2)*cos(Rz/2)*sin(Rx/2), cos(Rx/2)*cos(Rz/2)*sin(Ry/2), cos(Rx/2)*cos(Ry/2)*sin(Rz/2)]);
%             T = quaternion();
            DQ = self.position*dualquaternion(Q, self.U(1:3));
        end
        function DQ = deformed_end(self)
            Rx = self.U(10);
            Ry = self.U(11);
            Rz = self.U(12);
            Q = quaternion([cos(Rx/2)*cos(Ry/2)*cos(Rz/2), cos(Ry/2)*cos(Rz/2)*sin(Rx/2), cos(Rx/2)*cos(Rz/2)*sin(Ry/2), cos(Rx/2)*cos(Ry/2)*sin(Rz/2)]);
%             T = quaternion();
%             TL = dualquaternion(quaternion(), [0, 0, self.L]);
            DQ = self.undeformed_end()*dualquaternion(Q, self.U(7:9));
        end
        function DQ = undeformed_end(self)
            DQ = self.position*dualquaternion(quaternion(), [0, 0, self.L]);
        end
    end
end

 
 