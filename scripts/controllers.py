import rospy
import numpy as np
from simple_pid import PID


class ImageBasedController():
    def __init__(self, s_d0 = [400, 480, 0.2, 0]):
        """
        s: set of visual features
        s = (x, y, A)
        e = s - s_d: current error
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
        self.sx = self.sy = 1.042
        self.f = 692.978391
        self.lamb = 0.7
        self.s_d = s_d0
        self.x_PID = PID(0.002, 0, 0, setpoint=s_d0[0])
        self.y_PID = PID(0.002, 0, 0, setpoint=s_d0[1])
        self.z_PID = PID(0.4, 0.001, 0, setpoint=s_d0[2])
        self.w_PID = PID(0.1, 0, 0, setpoint=s_d0[3])
        self.x_PID.output_limits = (-1, 1)
        self.y_PID.output_limits = (-1, 1)
        self.z_PID.output_limits = (-0.5, 0.5)
        self.w_PID.output_limits = (-0.3, 0.3)
        
        # J = np.zeros(4, 4)
        self.e = np.zeros(3)
        self.m = np.zeros(3)

    def set_target(self, s_d):
        self.s_d = s_d
        self.x_PID.setpoint = s_d[0]
        self.y_PID.setpoint = s_d[1]
        self.z_PID.setpoint = s_d[2]
        self.w_PID.setpoint = s_d[3]
    
    def update_measurements(self, m, Z=None):
        self.m = [m.x, m.y, m.z, m.w]
        self.Z = Z
        # print("measurements: {}".format(self.m))

    def calc_input(self):
        e = (np.array(self.s_d) - np.array(self.m)).T
        # Lx = np.array([[0.003/self.Z, 0, 0, 0],
        #                 [0, 0.003/self.Z, 0, 0],
        #                 [0, 0, 0.4, 0],
        #                 [0, 0, 0, 0.05]])
        Lx = np.array([[self.sx*self.Z/self.f, 0, 0, 0],
                        [0, self.sy*self.Z/self.f, 0, 0],
                        [0, 0, 0.4, 0],
                        [0, 0, 0, 0.05]])
        ur = np.matmul(Lx, e)

        #### use PIDs ################
        # ur = np.array([[self.x_PID (self.m[0])],
        #                 [self.y_PID(self.m[1])], 
        #                 [self.z_PID(self.m[2])],
        #                 [self.w_PID(self.m[3])]])
        
        b_V_c = np.array([[0, -1, 0, 0],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, -1]])
        vr = -self.lamb*np.matmul(b_V_c,ur)
        # print(vr)
        return vr


class IBVS():
    def __init__(self, s_d0 = []):
        """
        s: set of visual features
        s = (x_c, y_c, A)
        e = s - s_d: current error
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
        self.s_d = s_d0
        self.sx = self.sy = 1.042
        self.Z = 2
        self.d_real = 0.2
        self.f = 692.978391
        x = y = 400
        self.lamb = 1

        Ls = np.array([[ -self.f/self.Z,  0, x*self.f/self.Z,     y],
                        [0, -self.f/self.Z, y*self.f/self.Z, -x],
                        [0, 0, 1/0.4, 0],
                        [0, 0, 0, -1]])
        self.c_V_n = np.array([[0, -1, 0, 0],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, -1]])
        Js = np.linalg.inv(np.matmul(Ls, self.c_V_n))
        # J = np.zeros(4, 4)
        self.e = np.zeros(4)
        self.m = np.zeros(4)
        self.vr = np.zeros(4)

    def set_target(self, s_d):
        self.s_d = s_d
    
    def update_height(self, z):
        self.Z = z

    def update_measurements(self, m, Z):
        self.Z = Z
        self.m = [m.x, m.y, m.z, m.w]
        e = (np.array(self.s_d) - np.array(self.m)).T
        
        # Ls = np.array([[ -self.f/self.Z,  0, m.x*self.f/self.Z,     m.y],
        #                 [0, -self.f/self.Z, m.y*self.f/self.Z, -m.x],
        #                 [0, 0, 1/0.4, 0],
        #                 [0, 0, 0, -1]])
        Ls = np.array([[self.f/self.sx*self.Z, 0, 0, 0],
                        [0, self.f/self.sy*self.Z, 0, 0],
                        [0, 0, 1/0.4, 0],
                        [0, 0, 0, 1/0.05]])
        self.c_V_n = np.array([[0, -1, 0, 0],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, -1]])
        
        rospy.loginfo("Ls: {}".format(Ls))
        Js = np.matmul(Ls, self.c_V_n)
        rospy.loginfo("Js: {}".format(Ls))
        Js_i = np.linalg.inv(Js)
        rospy.loginfo("Js_inverse: {}".format(Js_i))
        rospy.loginfo("e: {}".format(e))
        self.vr = -self.lamb*np.matmul(Js,e)
        rospy.loginfo(self.vr)
    
    def calc_input(self):
        
        return self.vr