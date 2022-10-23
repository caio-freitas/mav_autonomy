import rospy
import numpy as np
from simple_pid import PID

# TODO do superclass 


class Controller():
    def __init__(self, setpoint, Z):
        self.setpoint = setpoint
        self.Z = 0
        self.vb = 0
        self.input_limits = [1, 1, 0.3, 0.2]

    def set_target(self, setpoint):
        self.setpoint = setpoint

    def update_measurements(self, m, Z=None):
        self.m = [m.x, m.y, m.z, m.w]
        self.Z = Z

    def calc_input(self):
        return self.vb

    def update_height(self, Z):
        self.Z = Z
    
    def saturate_output(self, v):
        v[0] = min(v[0], self.input_limits[0])
        v[0] = max(v[0], -self.input_limits[0])

        v[1] = min(v[1], self.input_limits[1])
        v[1] = max(v[1], -self.input_limits[1])
        
        v[2] = min(v[2], self.input_limits[2])
        v[2] = max(v[2], -self.input_limits[2])

        v[3] = min(v[3], self.input_limits[3])
        v[3] = max(v[3], -self.input_limits[3])

        return v

class ImageBasedController(Controller):
    def __init__(self, init_setpoint = [400, 480, 0.2, 0], Z=2):
        super().__init__(init_setpoint, Z)
        """
        s: set of visual features
        s = (x, y, A)
        e = s - setpoint: current error
        m = (u, v): image measurements
        a = (u0, v0, px, py): camera intrinsic parameters
        px, py atio between focal lengtv[0] = min(v[0], self.input_limits[0])h and pixel size
        Vc = (vc, wc): spacial velocity of camera
        lambda: controller gain
        TODO adaptive gain as in https://visp-doc.inria.fr/doxygen/visp-daily/classvpAdaptiveGain.html
        
        * TODO add yaw control
        
        J: jacobian beten Vc and q, such as v = J q_dot

        q_dot = (vx, vy, vz, wz)
        x_dot = Lx * Vc

        vc = - lambda * Lx+ * e
        """
        self.f = 692.978391 
        self.lamb = 0.7
        self.setpoint = init_setpoint
        self.b_V_c = np.array([[0, -1, 0, 0],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, -1]])
        
        # J = np.zeros(4, 4)
        self.e = np.zeros(3)
        self.m = np.zeros(3)

    def set_target(self, setpoint):
        self.setpoint = setpoint
    
    def update_measurements(self, m, Z=None):
        self.m = [m.x, m.y, m.z, m.w]
        self.Z = Z

    def calc_input(self):
        e = (np.array(self.setpoint) - np.array(self.m)).T
        # Lx = np.array([[0.003/self.Z, 0, 0, 0],
        #                 [0, 0.003/self.Z, 0, 0],
        #                 [0, 0, 0.4, 0],
        #                 [0, 0, 0, 0.05]])
        Lx_i = np.array([[self.Z/self.f, 0, 0, 0],
                        [0, self.Z/self.f, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 0.05]])
        
        ur = np.matmul(Lx_i, e)
        
        vr = -self.lamb*np.matmul(self.b_V_c,ur)
        vr = self.saturate_output(vr)

        return vr


class IBVS(Controller):
    def __init__(self, init_setpoint = [], Z=2):
        super().__init__(init_setpoint, Z)
        """
        s: set of visual features
        s = (x_c, y_c, A)
        e = s - setpoint: current error
        m = (u, v): image measurements
        a = (u0, v0, px, py): camera intrinsic parameters
        px, py atio between focal length and pixel size
        Vc = (vc, wc): spacial velocity of camera
        lambda: controller gain
        TODO adaptive gain as in https://visp-doc.inria.fr/doxygen/visp-daily/classvpAdaptiveGain.html
        
        * TODO add yaw control
        
        J: jacobian beten Vc and q, such as v = J q_dot

        q_dot = (vx, vy, vz, wz)
        x_dot = Lx * Vc

        vc = - lambda * Lx+ * e
        """
        self.setpoint = init_setpoint
        self.Z = 2
        self.f = 692.978391 #3.67e-3
        x = y = 0
        self.lamb = 10

        Ls = self.calc_Ls(x, y, self.Z)
        
        self.c_V_n = np.array([[0, 1, 0, 0],
                            [1, 0, 0, 0],
                            [0, 0, 1, 0],
                            [0, 0, 0, -1]])
        
        # J = np.zeros(4, 4)
        self.e = np.zeros(4)
        self.m = np.zeros(4)
        self.vr = np.zeros(4)

    def calc_Ls(self, x, y, Z):
        return np.array([[-1/Z,        0, x/Z, x*y   , -(1+x**2),  y],
                        [0,         -1/Z, y/Z,  1+y**2, -x*y,     -x]])
    
    def calc_input(self):
        x = (self.m[0] - 400)/self.f
        y = (self.m[1] - 400)/self.f
        est_X = x*self.Z
        est_Y = y*self.Z
        rospy.loginfo("estimated relative position: {}, {}".format(est_X, est_Y))


        e = (np.array(self.setpoint)[:2] - np.array([x,y])).T
        rospy.loginfo("e: {}".format(e))

        Ls = self.calc_Ls(x, y, self.Z)
        rospy.loginfo("Ls: {}".format(Ls))
        try:
            Ls_inv = np.linalg.pinv(Ls)
        except Exception as ex:
            rospy.logerr(ex)
            return [0, 0, 0, 0]
        rospy.loginfo("Ls_inv: {}".format(Ls_inv))
        self.vr = -self.lamb*Ls_inv @ e
        self.vr = self.c_V_n[:2,:2]@self.vr[:2]
        vr = self.saturate_output(vr)
        rospy.loginfo("vr: {}".format(self.vr))
        rospy.loginfo(self.vr)
        return self.vr