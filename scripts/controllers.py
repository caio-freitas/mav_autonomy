import rospy
import numpy as np


class Controller():
    def __init__(self, setpoint, Z):
        self.setpoint = setpoint
        self.Z = 0
        self.vb = 0
        self.input_limits = [rospy.get_param("/limit_v_x"),
                             rospy.get_param("/limit_v_y"),
                             rospy.get_param("/limit_v_z"),
                             rospy.get_param("/limit_v_w")]

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
        self.f = rospy.get_param("/controller_f")
        self.lamb = rospy.get_param("/controller_lambda")
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
        A0l = 6*0.24
        e = (np.array(self.setpoint) - np.array(self.m)).T
        # rospy.loginfo("e: {}".format(e))
        Lx_i = np.array([[self.Z/self.f, 0, 0, 0],
                        [0, self.Z/self.f, 0, 0],
                        [0, 0, (self.Z**3)/A0l, 0],
                        [0, 0, 0, 1]])
        
        ur = np.matmul(Lx_i, e)
        
        vr = -self.lamb*np.matmul(self.b_V_c,ur)
        # rospy.loginfo("vr: {}".format(vr))
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
        self.f = rospy.get_param("/controller_f")
        x = y = 0
        self.lamb = rospy.get_param("/controller_lambda")

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
        A0l = 3*0.24
        Z = max(0.4, Z) # saturate Z influence on interaction matrix
        # return np.array([[-1/Z,        0, x/Z, x*y   , -(1+x**2),  y],
        #                 [0,         -1/Z, y/Z,  1+y**2, -x*y,     -x],
        #                 [0, 0, 2*A0l/(Z**3), -y*2*A0l/(Z**3), x*2*A0l/(Z**3), 0],
        #                 [0, 0, 0, 0, 0, 1]])
        return np.array([[-1/Z,        0, x/Z,  y],
                        [0,         -1/Z, y/Z,     -x],
                        [0, 0, A0l/(Z**3), 0],
                        [0, 0,  0,          1]])
    
    def calc_input(self):
        x = (self.m[0] - 400)/self.f
        y = (self.m[1] - 480)/self.f
        est_X = x*self.Z
        est_Y = y*self.Z

        # e = (np.array(self.setpoint)[:2] - np.array([x,y])).T
        e = (np.array(self.setpoint) - np.array([x,y, self.m[2], self.m[3]])).T

        Ls = self.calc_Ls(x, y, self.Z)
        
        try:
            Ls_inv = np.linalg.pinv(Ls)
        except Exception as ex:
            rospy.logerr(ex)
            return [0, 0, 0, 0]
        
        self.vr = -self.lamb*Ls_inv @ e
        
        self.vr[:4] = self.c_V_n@self.vr[:4]
        
        self.vr = self.saturate_output(self.vr)
        
        return self.vr