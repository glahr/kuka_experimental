import numpy as np
from numpy import pi
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import scipy
import spatialmath as smath
from enum import Enum, auto
from copy import deepcopy


class KinematicsKR16(rtb.DHRobot):
    def __init__(self, tool=None):
        if tool is None:
            super().__init__(
                [
                    rtb.RevoluteDH(d= 0.675, a= 0.26, alpha= -pi / 2),
                    rtb.RevoluteDH(a=0.68),
                    rtb.RevoluteDH(a=-0.035, alpha=pi / 2, offset= -pi / 2),
                    rtb.RevoluteDH(d=-0.67, alpha=-pi / 2),
                    rtb.RevoluteDH(alpha=pi / 2),
                    rtb.RevoluteDH(d=-0.158, alpha=pi, offset=pi)
                ], name="KukaKR16"
            )
        else:
            super().__init__(
                [
                    rtb.RevoluteDH(d= 0.675, a= 0.26, alpha= -pi / 2),
                    rtb.RevoluteDH(a=0.68),
                    rtb.RevoluteDH(a=-0.035, alpha=pi / 2, offset= -pi / 2),
                    rtb.RevoluteDH(d=-0.67, alpha=-pi / 2),
                    rtb.RevoluteDH(alpha=pi / 2),
                    rtb.RevoluteDH(d=tool[0], a=tool[1], alpha=tool[2], offset=tool[3])
                    # double_tool: rtb.RevoluteDH(d=-0.512, a=0.036, alpha=pi, offset=pi)
                    # no_tool: rtb.RevoluteDH(d=-0.158, alpha=pi, offset=pi)
                ], name="KukaKR16"
            )
        self.k = 0
        # self.q = None
        # self.qvel = None
        # self.qacc = None
        # self.trajx = None
        self.q0 = None
        self.qf = None
        self.T0 = None
        self.Tf = None
        self.t_delta = None
        self.n_step = 500
        self.v = np.zeros(3)
        # self.a = np.zeros(3)
        self.w = np.zeros(3)
        # self.alpha = np.zeros(3)

    def ik_kr16(self, t, R, q0=np.zeros(7)):
        Td = smath.SE3(t) * smath.SE3.OA(R[:, 1], R[:, 2])
        qd = self.ikine_LM(Td, q0=q0.reshape(1, 6), ilimit=200)
        return qd[0]

    def fk_kr16(self, qd):
        fk = self.fkine(qd)
        return fk.t, fk.R

    def cartesian_change(self, q0, tvar):
        [t, R] = self.fk_kr16(q0)
        t = t + tvar
        qr = self.ik_kr16(t, R, q0)
        return qr, t

    def create_T(self, t, R):
        T = smath.SE3(t) * smath.SE3.OA(R[:, 1], R[:, 2])
        return T

    def traj_cart_generate(self, td, Rd, t0, R0, dt=0.2, tmax=2):
        self.T0 = self.create_T(t0, R0)
        self.Tf = self.create_T(td, Rd)
        self.n_step = int(tmax / dt)
        self.t_delta = (td - t0) * (dt / tmax)
        # print(self.T0)
        self.k = -1  # restart k for interpolation

    def traj_cart_get_point(self, dt=0.002):
        if self.k < self.n_step-1:
            self.k += 1
        # T = self.Tf.interp(s=self.k/self.n_step, start=self.T0)
        T = self.T0
        # print("HEEEEEEEEEEEEEEEEE")
        # print(self.t_delta[0])
        # print(self.k)
        # print(T.t[0])
        T.t[0] = T.t[0] + self.k * self.t_delta[0]
        T.t[1] = T.t[1] + self.k * self.t_delta[1]
        T.t[2] = T.t[2] + self.k * self.t_delta[2]
        # print(T.t[0])
        # print("Test", T.t[0], T.t[1], T.t[2])
        # T = self.create_T()
        # T = self.T0.interp(self.Tf, [self.k / self.n_step])
        quat_d = smath.quaternion.UnitQuaternion(T.R)
        t = T.t
        R = T.R
        return t, self.v, self.a, quat_d, self.w, self.alpha, R

    # def traj_cart_generate(self, td, Rd, t0, R0, dt=0.2, tmax=2):
    #     self.T0 = self.create_T(t0, R0)
    #     self.Tf = self.create_T(td, Rd)
    #     self.n_step = int(tmax / dt)
    #     self.k = -1  # restart k for interpolation
    #
    # def traj_cart_get_point(self, dt=0.002):
    #     if self.k < self.n-1:
    #         self.k += 1
    #     # T = self.Tf.interp(s=self.k/self.n_step, start=self.T0)
    #     T = self.T0.interp(self.Tf, [self.k / self.n_step])
    #     quat_d = smath.quaternion.UnitQuaternion(T.R)
    #     t = T.t
    #     R = T.R
    #     return t, self.v, self.a, quat_d, self.w, self.alpha, R



if __name__ == '__main__':
    kuka = KinematicsKR16()
    qi = np.array([0, -3.14159 / 4, 3.14159 / 4, 0, 3.14159 / 2, -3.14159 / 2])
    qr = np.zeros([8, 6])
    tr = np.zeros([8, 3])
    print(kuka)
    # [ti, Ri] = kuka.fk_kr16(qi)
    # print(ti)
    # print("HERE!!!!")
    # t2 = ti + [-0.5, 0, 0]
    # kuka.traj_cart_generate(t2, Ri, ti, Ri)
    # [t, _, _, quat_d, _, _, R] = kuka.traj_cart_get_point()
    # [t, _, _, quat_d, _, _, R] = kuka.traj_cart_get_point()
    # print(t)
    # print("QUATD!!!")
    # print(quat_d)
    # [t, _, _, quat_d, _, _, R] = kuka.traj_cart_get_point()
    # print(t)
    # print("QUATD!!!")
    # print(quat_d)
    # [qr[0], tr[0]] = kuka.cartesian_change(qi, [-0.5, 0, 0])
    # [qr[1], tr[1]] = kuka.cartesian_change(qr[0], [0, 0, -0.2])
    # [qr[2], tr[2]] = kuka.cartesian_change(qr[1], [0, 0.2, 0])
    # [qr[3], tr[3]] = kuka.cartesian_change(qr[2], [0.4, 0, 0])
    # [qr[4], tr[4]] = kuka.cartesian_change(qr[3], [0, -0.4, 0])
    # [qr[5], tr[5]] = kuka.cartesian_change(qr[4], [-0.4, 0, 0])
    # [qr[6], tr[6]] = kuka.cartesian_change(qr[5], [0, 0.2, 0])
    # [qr[7], tr[7]] = kuka.cartesian_change(qr[6], [0, 0, 0.2])
    # print(qr)

    # qz = np.zeros([1, 6])
    qz = np.array([0, 0, 0, 0, -pi/2, -pi/2])
    fk = kuka.fkine(qz)
    [t, R] = kuka.fk_kr16(qz)
    print(t)
    print(R)
    # t[0] = t[0] - 0.2
    # print(t)
    # q1 = kuka.ik_kr16(t, R, qz)
    # print(q1)
