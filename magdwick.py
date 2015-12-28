##Julien Leimer
##Python implementation of the Magdwick Attitude estimator


#!/usr/bin/env python

import rospy
import time
import math

class MadgwickAHRS:
    def __init__(self):

        #System constants
        self.deltat = 0.01 #sampling period in seconds
        self.gyroMeasError = 3.14159265358979 * (5/180) #gyroscope measurement error in rad/s
        self.gyroMeasDrift = 3.14159265358979 * (0.2/180) #gyroscope measurement error in rad/s/s
        self.beta = sqrt(3/4)*gyroMeasError #compute beta
        self.zeta = sqrt(3/4)*gyroMeasDrift #compute zeta

    def __filterUpdate(self, w, a, m, SEq, b, wb):
        halfSEq[0] = 0.5*SEq[0]
        halfSEq[1] = 0.5*SEq[1]
        halfSEq[2] = 0.5*SEq[2]
        halfSEq[3] = 0.5*SEq[3]
        twoSEq[0] = 2*SEq[0]
        twoSEq[1] = 2*SEq[1]
        twoSEq[2] = 2*SEq[2]
        twoSEq[3] = 2*SEq[3]
        twob[0] = 2*b[0]
        twob[2] = 2*b[2]
        twobxSEq[0]=2*b[0]*SEq[0]
        twobxSEq[1]=2*b[0]*SEq[1]
        twobxSEq[2]=2*b[0]*SEq[2]
        twobxSEq[3]=2*b[0]*SEq[3]
        twobzSEq[0]=2*b[2]*SEq[0]
        twobzSEq[1]=2*b[2]*SEq[1]
        twobzSEq[2]=2*b[2]*SEq[2]
        twobzSEq[3]=2*b[2]*SEq[3]
        SEq1SEq2 = 0
        SEq1SEq3 = SEq[0]*SEq[2]
        SEq1SEq4 = 0
        SEq2SEq3 = 0
        SEq2SEq4 = SEq[1]*SEq[3]
        SEq3SEq4 = 0
        twom[0] = 2*m[0]
        twom[1] = 2*m[1]
        twom[2] = 2*m[2]

        #normalize the accelerometer measurement
        norm = sqrt(a[1]*a[1] + a[2]*a[2] + a[3]*a[3])
        a[0] /= norm
        a[1] /= norm
        a[2] /= norm

        #normalize the magnetometer measurement
        norm = sqrt(m[1]*m[1] + m[2]*m[2] + m[3]*m[3])
        m[0] /= norm
        m[1] /= norm
        m[2] /= norm

        #compute the objective function and Jacobian
        f[0] = twoSEq[1]*SEq[3] - twoSEq[0]*SEq[2] - a[0]
        f[1] = twoSEq[0]*SEq[1] + twoSEq[2]*SEq[3] - a[1]
        f[2] = 1 - twoSEq[1]*SEq[1] - twoSEq[2]*SEq[2] - a[2]
        f[3] = twob[0] * (0.5 - SEq[2]*SEq[2] - SEq[3]*SEq[3]) + twob[2] * (SEq2SEq4 - SEq1SEq3) - m[0]
        f[4] = twob[0] * (SEq[1]*SEq[2] - SEq[0]*SEq[3]) + twob[2] * (SEq[0]*SEq[1] + SEq[2]*SEq[3]) - m[1]
        f[5] = twob[0] * (SEq1SEq3 + SEq2SEq4) + twob[2] * (0.5 - SEq[1]*SEq[1] - SEq[2]*SEq[2]) - m[2]
        J_11or24 = twoSEq[2]    #J_11 negated in matrix multiplication
        J_12or23 = 2*SEq[3]
        J_13or22 = twoSEq[0]    #J_12 negated in matrix multiplication
        J_14or21 = twoSEq[1]
        J_32 = 2*J_14or21   #negated in matrix multiplication
        J_33 = 2*J_11or24   #negated in matrix multiplication
        J_41 = twobzSEq[2]  #negated in matrix multiplication
        J_42 = twobzSEq[3]
        J_43 = 2*twobxSEq[2] + twobzSEq[0]  #negated in matrix multiplication
        J_44 = 2*twobzSEq[3] + twobzSEq[1]  #negated in matrix multiplication
        J_51 = twobxSEq[3] - twobzSEq[1]    #negated in matrix multiplication
        J_52 = twobxSEq[2] + twobxSEq[0]
        J_53 = twobxSEq[1] + twobxSEq[3]
        J_54 = twobxSEq[0] - twobxSEq[2]    #negated in matrix multiplication
        J_61 = twobxSEq[2]
        J_62 = twobxSEq[3] - 2*twobzSEq[1]
        J_63 = twobzSEq[0] - 2*twobzSEq[2]
        J_64 = twobxSEq[1]

        #compute the gradient (matrix multiplication)
        SEqHatDot[0] = J_14or21*f[1] - J_11or24*f[0] - J_41*f[3] - J_51*f[4] + J_61*f[5]
        SEqHatDot[1] = J_12or23*f[0] - J_13or22*f[1] - J_32*f[2] + J_42*f[3] + J_52*f[4] + J_62*f[5]
        SEqHatDot[2] = J_12or23*f[1] - J_33*f[2] - J_13or22*f[0] - J_43*f[3] + J_53*f[4] + J_63*f[5]
        SEqHatDot[3] = J_14or21*f[0] - J_11or24*f[1] - J_44*f[3] - J_54*f[4] + J_64*f[5]

        #normalize the gradient to estimate direction of the gyroscope error
        norm = sqrt(SEqHatDot[0]*SEqHatDot[0] + SEqHatDot[1]*SEqHatDot[1] + SEqHatDot[2]*SEqHatDot[2] + SEqHatDot[3]*SEqHatDot[3])
        SEqHatDot[0] /= norm
        SEqHatDot[1] /= norm
        SEqHatDot[2] /= norm
        SEqHatDot[3] /= norm

        #compute angular estimated direction of the gyroscope error
        w_err[0] = twoSEq[0] * SEqHatDot[1] - twoSEq[1] * SEqHatDot[0] - twoSEq[2] * SEqHatDot[3] + twoSEq[3] * SEqHatDot[2]
        w_err[1] = twoSEq[0] * SEqHatDot[2] + twoSEq[1] * SEqHatDot[3] - twoSEq[2] * SEqHatDot[0] - twoSEq[3] * SEqHatDot[1]
        w_err[2] = twoSEq[0] * SEqHatDot[3] - twoSEq[1] * SEqHatDot[2] + twoSEq[2] * SEqHatDot[1] - twoSEq[3] * SEqHatDot[0]

        #compute and remove the gyroscope biases
        wb[0] += w_err[0]*deltat*zeta
        wb[1] += w_err[1]*deltat*zeta
        wb[2] += w_err[2]*deltat*zeta
        w[0] -= wb[0]
        w[1] -= wb[1]
        w[2] -= wb[2]

        #compute the quaternion rate measured by gyroscopes
        SEqDot_omega[0] = -halfSEq[1]*w[0] -halfSEq[2]*w[1] - halfSEq[3]*w[2]
        SEqDot_omega[1] = halfSEq[0]*w[0] + halfSEq[2]*w[2] - halfSEq[3]*w[1]
        SEqDot_omega[2] = halfSEq[0]*w[1] - halfSEq[1]*w[2] + halfSEq[3]*w[0]
        SEqDot_omega[3] = halfSEq[0]*w[2] + halfSEq[1]*w[1] - halfSEq[2]*w[0]

        #compute then integrate the estimated quaternion rate
        SEq[0] += (SEqDot_omega[0] - (beta*SEqHatDot[0]))*deltat
        SEq[1] += (SEqDot_omega[1] - (beta*SEqHatDot[1]))*deltat
        SEq[2] += (SEqDot_omega[2] - (beta*SEqHatDot[2]))*deltat
        SEq[3] += (SEqDot_omega[3] - (beta*SEqHatDot[3]))*deltat

        #normalise quaternion
        norm = sqrt(SEq[0]*SEq[0] + SEq[1]*SEq[1] + SEq[2]*SEq[2] + SEq[3]*SEq[3])
        SEq[0] /= norm
        SEq[1] /= norm
        SEq[2] /= norm
        SEq[3] /= norm

        #compute flux in the earth frame
        SEq1SEq2 = SEq[0] * SEq[1]
        SEq1SEq3 = SEq[0] * SEq[2]
        SEq1SEq4 = SEq[0] * SEq[3]
        SEq3SEq4 = SEq[2] * SEq[3]
        SEq2SEq3 = SEq[1] * SEq[2]
        SEq2SEq4 = SEq[1] * SEq[3]

        h[0] = twom[0] * (0.5 - SEq[2]*SEq[2] - SEq[3]*SEq[3]) + twom[1]*(SEq2SEq3 - SEq1SEq4) + twom[2]*(SEq2SEq4 + SEq1SEq3)
        h[1] = twom[0] * (SEq2SEq3 + SEq1SEq4) + twom[1] * (0.5 - SEq[1]*SEq[1] - SEq[3]*SEq[3]) + twom[2]*(SEq3SEq4 - SEq1SEq2)
        h[2] = twom[0] * (SEq2SEq4 + SEq1SEq3) + twom[1] * (SEq3SEq4 + SEq1SEq2) + twom[2] * (0.5 - SEq[1]*SEq[1] - SEq[2]*SEq[2])

        #normalize the flux vector to have only components in the x and z
        b[0] = sqrt((h[0]*h[0]) + (h[1]*h[1]))
        b[2] = h[2]

        return (SEq, b, wb)

    def __quaternConj(self, q):
        qC[0] = q[0]
        qC[1] = -q[1]
        qC[2] = -q[2]
        qC[3] = -q[3]

        return qC

    def __quatern2euler(self, q):
        norm = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        q = q/norm

        a[0] = atan2(2*(q[2]*q[3]-q[0]*q[1]),2*q[0]*q[0] -1 + 2*q[3]*q[3])
        a[1] = -atan((2*(q[1]*q[3] + q[0]*q[2]))/sqrt(2*(q[1]*q[3]+ q[0]*q[2])))
        a[2] = atan2(2*(q[1]*q[2]-q[0]*q[3]),2*q[0]*q[0] -1 + 2*q[1]*q[1])

        return a

