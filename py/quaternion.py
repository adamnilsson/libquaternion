# libquaternions

from sympy import Matrix
import numpy as np
import pdb
from sympy import sin, cos
import sympy as sp

class Quaternion(object):
    """Class to represent a quaternion object with common operations.
    """
    
    def __init__(self, Q=None, axis=None, angle=0.0):
        if isinstance(Q, Quaternion):
            self = Q
        elif not (Q is None):
            self.q = np.array(Q)
        elif not (axis is None):
            self.quaternion_from_axis_angle(axis, angle)
            
    def quaternion_from_axis_angle(self, axis, angle):
        self.q = np.array([cos(angle/2), 
                           axis[0]*sin(angle/2), 
                           axis[1]*sin(angle/2), 
                           axis[2]*sin(angle/2)])
        
    def __add__(self, B):
        """Addition operator for quaternion
        """
        return self.__class__(self.q+B.q)
    
    def __neg__(self):
        """Invert sign to negative
        """
        return self.__class__(-self.q)
    
    def __sub__(self,B):
        """Subtraction operator for quaternion
        """
        return self.__class__(self.q-B.q)
        
    def __mul__(self, B):
        """Multiplication for quaternions
        """
        qM = (self._left_multiply_matrix()*Matrix(B.q)).transpose()
        qM.simplify()
        q = np.array(qM.tolist())[0,:]
        return self.__class__(q)

    def _left_multiply_matrix(self):
        """ Matrix representing the actions for multiplying to the left
        """
        Q = self
        
        M = Matrix([[Q.q[0], -Q.q[1], -Q.q[2], -Q.q[3]],
                    [Q.q[1], Q.q[0], -Q.q[3], Q.q[2]],
                    [Q.q[2], Q.q[3], Q.q[0], -Q.q[1]],
                    [Q.q[3], -Q.q[2], Q.q[1], Q.q[0]]])
        return M
#        function K = ctranspose(Q)
#            % Konjugate of quaternion
#            K = Q;
#            K.Q(2:4) = -Q.Q(2:4);
#        end

    def __str__(self):
        """String representation
        """
        return str(self.q)
        
if __name__ == '__main__':
    print 'checking quaternion'
    A = Quaternion([1.,0.,1.,0.])
    B = Quaternion([1.,3.,0.,0.])
    print 'A = ' + str(A)
    print 'B = ' + str(B)
    print '-A = ' + str(-A)
    print '-B = ' + str(-B)
    print 'A+B = ' + str(A+B)
    print 'A-B = ' + str((A-B))
    print 'A-A = ' + str(A-A)
    print 'A = ' + str(A)
    print 'B = ' + str(B)
    C = Quaternion(axis=[1,0,0], angle=1.2)
    print 'C(1.2) = ' + str(C)
    x = sp.symbols('x')
    C = Quaternion(axis=[1,0,0], angle=x)
    print 'C(x) = ' + str(C)
    print 'C(x)*C(x) = ' + str(C*C)
    pdb.set_trace()
