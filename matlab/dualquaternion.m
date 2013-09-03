classdef dualquaternion
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
                self.S = quaternion(q0, q1);
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
            if length(self) == 1
                disp(['       ' self.S.strrep()])
                disp(['\eps*( ' self.D.strrep() ' )'])
            else
                disp(['dualquaternion array of length ' num2str(length(self))])
            end
        end
    end
%     methods(Static)
%         function DQ = translation(vect)
%             DQ = 0
%         end
%     end
end