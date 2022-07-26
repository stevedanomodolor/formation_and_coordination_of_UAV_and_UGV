import numpy as np
class SimplePI:
    """ 
    Simple PI implementation with basic anti-windup. The _saturation - variable sets the saturation of the anti-windup.
    """
    
    _saturation = 0.1
    
    def __init__(self,kp,ki,dt):
        """ Initialize the controller """

        self.e = np.zeros(3)
        self.dt = dt # Sample rate
        self.kp = kp # P-part gain
        self.ki = ki # I-part gain
    
    def update_control(self,e):
        self.e += e*self.dt # Update cumulative error
        
        # Saturate internal error for anti-windup
        self.e = np.sign(self.e)*np.array([min(ei,self._saturation) for ei in abs(self.e)])
        
        # Return control signal
        return self.kp*e + self.ki*self.e
