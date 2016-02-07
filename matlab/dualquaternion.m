% Class: dualquaternion
% Author: Adam Nilsson
% Last modified 29-Nov-2015
%   Added substitute of symbolic variables
% Modified 10-Oct-2015
%   Added addition operator
% 
% dualquaternion()
% 
% dualquaternion(q)
%     q = q0 + q1*i + q2*j + q3*k + \eps*(q4 + q5*i + q6*j + q7*k)
% 
% dualquaternion(q0, q1)
%     q = q0 + \eps*q1, q0, q1 \in quaternion
% 
% dualquaternion(q0, q1)
%     q = q0 + \eps*(q1_0 + q1_1*i + q1_2*j + q1_3*k)
% 
% dualquaternion(q0, q1)
%     q0 is rotation quaternion
%     q1 is componets of dual part
% 
% dualquaternion(q0, q1, q2)
%     q0 is scalar angle
%     q1 is unit axis of rotation
%     q2 is translation vector
% 
classdef dualquaternion
    % dualquaternion is used to represent transformations
    % The transformations are applied after each other by multiplications.
    % Such multiplication operator is implemented to the dualquaternion
    % class and is used by the regular * multiplication sign.
    % E.g.    A*B yields the product between A and B
    properties
        S
        D
    end
    methods
        function self = dualquaternion(q0, q1, q2, q3, q4, q5, q6, q7)
            if nargin == 0
                self.S = quaternion();
                self.D = quaternion(0, 0, 0, 0);
            elseif nargin == 1
                self.S = quaternion(q0(1:4));
                self.D = quaternion(q0(5:8));
            elseif nargin == 2
                if isa(q0, 'quaternion')
                    self.S = q0;
                else
                    self.S = quaternion(q0);
                end
                
                if isa(q1, 'quaternion')
                    self.D = q1;
                elseif length(q1) == 4
                    self.D = quaternion(q1);
                elseif length(q1) == 3
                    T = quaternion(q1/2);
                    self.D = T*self.S;
                else
                    error('Input not understod')
                end
            elseif nargin == 3
                % Angle, axis, translation
                self.S = quaternion(cos(q0/2), sin(q0/2)*q1);
                T = quaternion(q2/2);
                self.D = T*self.S;
            end
        end
        
        %% Operators
        function DQ = mtimes(A, B)
            if isa(A,'dualquaternion') && isa(B,'dualquaternion')
                DQ = A;
                DQ.S = A.S*B.S;
                DQ.D = A.S*B.D + A.D*B.S;
            elseif isa(A,'dualquaternion')
                DQ = A;
                DQ.S = A.S*B;
                DQ.S = A.D*B;
            elseif isa(B,'dualquaternion')
                DQ = B;
                DQ.S = B.S*A;
                DQ.D = B.D*A;
            end
        end
        function DQ = plus(A, B)
            % Sum of dual quaternions
            DQ = dualquaternion();
            DQ.S = A.S + B.S;
            DQ.D = A.D + B.D;
        end
        function K = ctranspose(Q)
            % Konjugate of quaternion
            % Added 8-Aug-2015 --Adam
            K = dualquaternion(Q.S', Q.D')
        end
        %% Other functions
        function V = vector8(self)
            V = [self.S.Q; self.D.Q];
        end
        function T = translation(self)
            S = self.S*(1/norm(self.S.Q));
            T = self.D*S'*2;
        end
        function draw(self, varargin)
            p = inputParser;
            p.addOptional('AxisSize', 1)
            parse(p, varargin{:})
            
            T = self.translation().vector3();
            self.S.draw('Origin', T, 'AxisSize', p.Results.AxisSize);
        end
        function display(self)
            disp(['       ' self.S.strrep()])
            disp(['\eps*( ' self.D.strrep() ' )'])
        end
        function M = getT44(self)
            % Return the 4x4 transformation atrix for the same quaternion
            Q = self.S;
            Ax = Q*quaternion([0,1,0,0])*Q';
            Ay = Q*quaternion([0,0,1,0])*Q';
            Az = Q*quaternion([0,0,0,1])*Q';
            p = self.translation();
            M = eye(4);
            M(1:3,1) = Ax.vector3()/norm(Ax.vector3());
            M(1:3,2) = Ay.vector3()/norm(Ay.vector3());
            M(1:3,3) = Az.vector3()/norm(Az.vector3());
            M(1:3,4) = p.vector3();
        end
        function Q = subs(self, params, values)
            Q = dualquaternion();
            Q.S = self.S.subs(params, values);
            Q.D = self.D.subs(params, values);
        end
    end
end