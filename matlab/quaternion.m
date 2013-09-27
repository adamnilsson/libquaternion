classdef quaternion
    properties
        Q
        DisplayFormat
    end
    methods
        % quaternion()                      = Unit quaternion
        % quaternion([q0, q1, q2, q3])      = Quaternion
        % quaternion([q1, q2, q3])          = Vector hypercomplex space
        % quaternion(q1, [q2, q3, q4])      = Quaternion
        % quaternion(q1, q2, q3)            = Vector hypercomplex space
        % quaternion(q0, q1, q2, q3)        = Quaternion
        function self = quaternion(q0, q1, q2, q3)
            if nargin == 0
                self.Q = [1; 0; 0; 0];
            elseif nargin == 1
                if length(q0) == 4
                    self.Q = [q0(1); q0(2); q0(3); q0(4)];
                elseif length(q0) == 3
                    self.Q = [0; q0(1); q0(2); q0(3)];
                else
                    error('Input not understood');
                end
            elseif nargin == 2
                self.Q = [q0; q1(1); q1(2); q1(3)];
            elseif nargin == 3
                self.Q = [0; q0; q1; q2];
            elseif nargin == 4
                self.Q = [q0; q1; q2; q3];
            else
                error('Input not understood');
            end
            self.DisplayFormat = 'scalar';
        end
        function S = plus(A, B)
            % Addition operator
            S = A;
            S.Q = A.Q + B.Q;
        end
        function S = minus(A, B)
            % Subtraction operator
            S = A;
            S.Q = A.Q - B.Q;
        end
        function product = mtimes(A, B)
            if isa(A,'quaternion') && isa(B,'quaternion')
                M = A.left_multiply_matrix();
                product = A;
                product.Q = M*B.Q;
            elseif isa(A,'quaternion')
                product = A;
                product.Q = A.Q*B;
            elseif isa(B,'quaternion')
                product = B;
                product.Q = B.Q*A;
            end
        end
        function M = left_multiply_matrix(Q)
            % Matrix representing the actions for multiplying to the left
            M = [Q.Q(1) -Q.Q(2) -Q.Q(3) -Q.Q(4);
                 Q.Q(2) Q.Q(1) -Q.Q(4) Q.Q(3);
                 Q.Q(3) Q.Q(4) Q.Q(1) -Q.Q(2);
                 Q.Q(4) -Q.Q(3) Q.Q(2) Q.Q(1)];
        end
        
        function quat = subs(self, params, values)
            quat = self;
            quat.Q = subs(self.Q, params, values);
        end
        
        function K = ctranspose(Q)
            % Konjugate of quaternion
            K = Q;
            K.Q(2:4) = -Q.Q(2:4);
        end
        function display(self)
            disp(self.strrep())
        end
        function text = strrep(self)
            
            function str = n2str(t)
                if strcmp(class(t),'sym')
                    str = char(t);
                else
                    str = num2str(t);
                end
            end
            
            if strcmp(self.DisplayFormat, 'vector')
                if isa(self.Q, 'sym')
                    Qchar = char(self.Q.');
                    disp(Qchar(9:length(Qchar)-2))
                else
                    disp([' [ ' num2str(self.Q') ']'])
                end
            end
            if strcmp(self.DisplayFormat, 'scalar')
                prev = 0;
                text = [];
                if self.Q(1) ~= 0
                    text = [text n2str(self.Q(1))];
                    prev = 1;
                end
                if self.Q(2) ~= 0
                    if isa(self.Q(2), 'sym')
                        sign = ' + ';
                    elseif (self.Q(2) > 0 && prev == 1)
                        sign = ' + ';
                    else
                        sign = ' ';
                    end
                    text = [text sign n2str(self.Q(2)) ' i '];
                    prev = 1;
                end
                if self.Q(3) ~= 0
                    if isa(self.Q(3), 'sym')
                        sign = ' + ';
                    elseif (self.Q(3) > 0 && prev == 1)
                        sign = ' + ';
                    else
                        sign = ' ';
                    end
                    text = [text sign n2str(self.Q(3)) ' j '];
                    prev = 1;
                end
                if self.Q(4) ~= 0
                    if isa(self.Q(4), 'sym')
                        sign = ' + ';
                    elseif (self.Q(4) > 0 && prev == 1)
                        sign = ' + ';
                    else
                        sign = ' ';
                    end
                    text = [text sign n2str(self.Q(4)) ' k '];
                end
                if length(text) == 0
                    text = ['0'];
                end
%                 disp(text);
            end
        end
        
        function V = vector3(self)
            V = self.Q(2:4);
        end
        function V = vector4(self)
            V = self.Q(1:4);
        end
        function draw(self, varargin)
            p = inputParser;
            p.addOptional('Origin', [0; 0; 0])
            p.addOptional('AxisSize', 1)
            p.addOptional('AxisWeight', 1)
            parse(p, varargin{:})
            
            C = p.Results.Origin;
            
            Q = self;
            normerror = abs(double(norm(self.Q) - 1));
            if isa(normerror, 'sym')
                normerror
            end
            if normerror > 1e-14
                Q.Q = self.Q/norm(self.Q);
                warning(['Quaternion not normalized, tol = ' num2str(normerror)])
            end
            washold = ishold;
            hold on;
%             plot3([C(2) B(2)],[C(3) B(3)],[C(4) B(4)],'--m')
            ex = Q*quaternion([1 0 0])*Q'; ex = ex.vector3()*p.Results.AxisSize;
            ey = Q*quaternion([0 1 0])*Q'; ey = ey.vector3()*p.Results.AxisSize;
            ez = Q*quaternion([0 0 1])*Q'; ez = ez.vector3()*p.Results.AxisSize;
            plot3([0 ex(1)]+C(1),[0 ex(2)]+C(2),[0 ex(3)]+C(3),'-r', 'LineWidth', p.Results.AxisWeight)
            plot3([0 ey(1)]+C(1),[0 ey(2)]+C(2),[0 ey(3)]+C(3),'-g', 'LineWidth', p.Results.AxisWeight)
            plot3([0 ez(1)]+C(1),[0 ez(2)]+C(2),[0 ez(3)]+C(3),'-b', 'LineWidth', p.Results.AxisWeight)
            if washold == 0
                hold off;
            end
        end
    end
    methods(Static)
        function self = rotateAxis(angle, axis)
            self = quaternion(cos(angle/2), sin(angle/2)*axis);
        end
    end
end