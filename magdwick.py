##Julien Leimer
##Python implementation of the Magdwick Attitude estimator

import rospy
import time
import math

class MadgwickAHRS:
    def __init__(self, deltat=0.2, gyroMeasError=5, gyroMeasDrift=0.2):

        #System constants
        self.deltat = deltat #sampling period in seconds
        self.gyroMeasError = 3.14159265358979 * (gyroMeasError/180) #gyroscope measurement error in rad/s
        self.gyroMeasDrift = 3.14159265358979 * (gyroMeasDrift/180) #gyroscope measurement error in rad/s/s
        self.beta = math.sqrt(3/4)*self.gyroMeasError #compute beta
        self.zeta = math.sqrt(3/4)*self.gyroMeasDrift #compute zeta
	#self.beta = 0.041

	self.globSEq = [1.0, 0.0, 0.0, 0.0]
	self.globb = [1.0, 0.0, 0.0]
	self.globwb = [0.0, 0.0, 0.0]	

    def filterUpdate(self, w, a, m, SEq, b, wb):
	
	#local system variables	
	halfSEq = [0.0, 0.0, 0.0, 0.0]       	
	twoSEq = [0.0, 0.0, 0.0, 0.0]
	twob = [0.0, 0.0, 0.0]
	twobxSEq = [0.0, 0.0, 0.0, 0.0]
	twobzSEq = [0.0, 0.0, 0.0, 0.0]
	twom = [0.0, 0.0, 0.0]
	f = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	SEqHatDot = [0.0, 0.0, 0.0, 0.0]
	w_err = [0.0, 0.0, 0.0]
	SEqDot_omega=[0.0, 0.0, 0.0, 0.0]
	h = [0.0, 0.0, 0.0]

	#auxiliary variables to avoid repeated calculations
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
        norm = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
        a[0] /= norm
        a[1] /= norm
        a[2] /= norm

        #normalize the magnetometer measurement
        norm = math.sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2])
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
        J_44 = 2*twobzSEq[3] - twobzSEq[1]  #negated in matrix multiplication
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
        SEqHatDot[1] = J_12or23*f[0] + J_13or22*f[1] - J_32*f[2] + J_42*f[3] + J_52*f[4] + J_62*f[5]
        SEqHatDot[2] = J_12or23*f[1] - J_33*f[2] - J_13or22*f[0] - J_43*f[3] + J_53*f[4] + J_63*f[5]
        SEqHatDot[3] = J_14or21*f[0] + J_11or24*f[1] - J_44*f[3] - J_54*f[4] + J_64*f[5]

        #normalize the gradient to estimate direction of the gyroscope error
        norm = math.sqrt(SEqHatDot[0]*SEqHatDot[0] + SEqHatDot[1]*SEqHatDot[1] + SEqHatDot[2]*SEqHatDot[2] + SEqHatDot[3]*SEqHatDot[3])
        SEqHatDot[0] /= norm
        SEqHatDot[1] /= norm
        SEqHatDot[2] /= norm
        SEqHatDot[3] /= norm

        #compute angular estimated direction of the gyroscope error
        w_err[0] = twoSEq[0] * SEqHatDot[1] - twoSEq[1] * SEqHatDot[0] - twoSEq[2] * SEqHatDot[3] + twoSEq[3] * SEqHatDot[2]
        w_err[1] = twoSEq[0] * SEqHatDot[2] + twoSEq[1] * SEqHatDot[3] - twoSEq[2] * SEqHatDot[0] - twoSEq[3] * SEqHatDot[1]
        w_err[2] = twoSEq[0] * SEqHatDot[3] - twoSEq[1] * SEqHatDot[2] + twoSEq[2] * SEqHatDot[1] - twoSEq[3] * SEqHatDot[0]

        #compute and remove the gyroscope biases
        wb[0] += w_err[0]*self.deltat*self.zeta
        wb[1] += w_err[1]*self.deltat*self.zeta
        wb[2] += w_err[2]*self.deltat*self.zeta
        w[0] -= wb[0]
        w[1] -= wb[1]
        w[2] -= wb[2]

        #compute the quaternion rate measured by gyroscopes
        SEqDot_omega[0] = -halfSEq[1]*w[0] -halfSEq[2]*w[1] - halfSEq[3]*w[2]
        SEqDot_omega[1] = halfSEq[0]*w[0] + halfSEq[2]*w[2] - halfSEq[3]*w[1]
        SEqDot_omega[2] = halfSEq[0]*w[1] - halfSEq[1]*w[2] + halfSEq[3]*w[0]
        SEqDot_omega[3] = halfSEq[0]*w[2] + halfSEq[1]*w[1] - halfSEq[2]*w[0]

        #compute then integrate the estimated quaternion rate
        SEq[0] += (SEqDot_omega[0] - (self.beta*SEqHatDot[0]))*self.deltat
        SEq[1] += (SEqDot_omega[1] - (self.beta*SEqHatDot[1]))*self.deltat
        SEq[2] += (SEqDot_omega[2] - (self.beta*SEqHatDot[2]))*self.deltat
        SEq[3] += (SEqDot_omega[3] - (self.beta*SEqHatDot[3]))*self.deltat

        #normalise quaternion
        norm = math.sqrt(SEq[0]*SEq[0] + SEq[1]*SEq[1] + SEq[2]*SEq[2] + SEq[3]*SEq[3])
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
        b[0] = math.sqrt((h[0]*h[0]) + (h[1]*h[1]))
        b[2] = h[2]

        return (SEq, b, wb)
    
    def filterUpdateSimple(self, w, a, m, SEq):
        
        #local system variables
        halfSEq = [0.0, 0.0, 0.0, 0.0]
        twoSEq = [0.0, 0.0, 0.0, 0.0]
        twob = [0.0, 0.0, 0.0]
        twobxSEq = [0.0, 0.0, 0.0, 0.0]
        twobzSEq = [0.0, 0.0, 0.0, 0.0]
        twom = [0.0, 0.0, 0.0]
        f = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        SEqHatDot = [0.0, 0.0, 0.0, 0.0]
        w_err = [0.0, 0.0, 0.0]
        SEqDot_omega=[0.0, 0.0, 0.0, 0.0]
        h = [0.0, 0.0, 0.0, 0.0]
        
        
        
        #normalize the accelerometer measurement
        norm = math.sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2])
        a[0] /= norm
        a[1] /= norm
        a[2] /= norm
        
        #normalize the magnetometer measurement
        norm = math.sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2])
        m[0] /= norm
        m[1] /= norm
        m[2] /= norm
        
        #Reference direction of Earth's magnetic field
        h[0] = SEq[0]*(SEq[1]*m[0 + SEq[2]*m[1] + SEq[3]*m[2]) - SEq[1]*(SEq[0]*m[0] + SEq[2]*m[2] - SEq[3]*m[1]) - SEq[2]*(SEq[0]*m[1] - SEq[1]*m[2] + SEq[3]*m[0]) - SEq[3]*(SEq[0]*m[2] + SEq[1]*m[1] - SEq[2]*m[0])
        h[1] = SEq[0]*(SEq[0]*m[0] + SEq[2]*m[2] - SEq[3]*m[1]) + SEq[2]*(SEq[0]*m[2] + SEq[1]*m[1] - SEq[2]*m[0]) + SEq[1]*(SEq[1]*m[0] + SEq[2]*m[1] + SEq[3]*m[2]) - SEq[3]*(SEq[0]*m[1] - SEq[1]*m[2] + SEq[3]*m[0])
        h[2] = SEq[0]*(SEq[0]*m[1] - SEq[1]*m[2] + SEq[3]*m[0]) - SEq[1]*(SEq[0]*m[2] + SEq[1]*m[1] - SEq[2]*m[0]) + SEq[2]*(SEq[1]*m[0] + SEq[2]*m[1] + SEq[3]*m[2]) + SEq[3]*(SEq[0]*m[0] + SEq[2]*m[2] - SEq[3]*m[1])
        h[3] = SEq[0]*(SEq[0]*m[2] + SEq[1]*m[1] - SEq[2]*m[0]) + SEq[1]*(SEq[0]*m[1] - SEq[1]*m[2] + SEq[3]*m[0]) - SEq[2]*(SEq[0]*m[0] + SEq2*m[2] - SEq[3]*m[1]) + SEq[3]*(SEq[1]*m[0] + SEq[2]*m[1] + SEq[3]*m[2])
        
        b[0] = math.sqrt((h[1]*h[1]) + (h[2]*h[2]))
        b[2] = h[3]
                                
                                
        #auxiliary variables to avoid repeated calculations
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
        J_44 = 2*twobzSEq[3] - twobzSEq[1]  #negated in matrix multiplication
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
        SEqHatDot[1] = J_12or23*f[0] + J_13or22*f[1] - J_32*f[2] + J_42*f[3] + J_52*f[4] + J_62*f[5]
        SEqHatDot[2] = J_12or23*f[1] - J_33*f[2] - J_13or22*f[0] - J_43*f[3] + J_53*f[4] + J_63*f[5]
        SEqHatDot[3] = J_14or21*f[0] + J_11or24*f[1] - J_44*f[3] - J_54*f[4] + J_64*f[5]
        
        #normalize the gradient to estimate direction of the gyroscope error
        norm = math.sqrt(SEqHatDot[0]*SEqHatDot[0] + SEqHatDot[1]*SEqHatDot[1] + SEqHatDot[2]*SEqHatDot[2] + SEqHatDot[3]*SEqHatDot[3])
        SEqHatDot[0] /= norm
        SEqHatDot[1] /= norm
        SEqHatDot[2] /= norm
        SEqHatDot[3] /= norm
        
        #compute the quaternion rate measured by gyroscopes
        SEqDot_omega[0] = -halfSEq[1]*w[0] -halfSEq[2]*w[1] - halfSEq[3]*w[2]
        SEqDot_omega[1] = halfSEq[0]*w[0] + halfSEq[2]*w[2] - halfSEq[3]*w[1]
        SEqDot_omega[2] = halfSEq[0]*w[1] - halfSEq[1]*w[2] + halfSEq[3]*w[0]
        SEqDot_omega[3] = halfSEq[0]*w[2] + halfSEq[1]*w[1] - halfSEq[2]*w[0]
        
        #compute then integrate the estimated quaternion rate
        SEq[0] += ((SEqDot_omega[0] - (self.beta*SEqHatDot[0]))*self.deltat)
        SEq[1] += ((SEqDot_omega[1] - (self.beta*SEqHatDot[1]))*self.deltat)
        SEq[2] += ((SEqDot_omega[2] - (self.beta*SEqHatDot[2]))*self.deltat)
        SEq[3] += ((SEqDot_omega[3] - (self.beta*SEqHatDot[3]))*self.deltat)
        
        #normalise quaternion
        norm = math.sqrt(SEq[0]*SEq[0] + SEq[1]*SEq[1] + SEq[2]*SEq[2] + SEq[3]*SEq[3])
        SEq[0] /= norm
        SEq[1] /= norm
        SEq[2] /= norm
        SEq[3] /= norm
        
        return SEq


    def quaternConj(self, q):
	#function to compute the conjugate of a quaternion

	qC = [0.0, 0.0, 0.0, 0.0]
        qC[0] = q[0]
        qC[1] = -q[1]
        qC[2] = -q[2]
        qC[3] = -q[3]

        return qC

    def quatern2euler(self, q):
	#function to convert a quaterion to the corresponding Euler angles
	a = [0.0, 0.0, 0.0]

        a[0] = math.atan2(2*(q[2]*q[3]-q[0]*q[1]),2*q[0]*q[0] -1 + 2*q[3]*q[3])
        a[1] = -math.atan((2*(q[1]*q[3] + q[0]*q[2]))/(math.sqrt(1 - 2*(q[1]*q[3]+ q[0]*q[2]))))
	a[2] = math.atan2(2*(q[1]*q[2]-q[0]*q[3]),2*q[0]*q[0] -1 + 2*q[1]*q[1])

        return a
                                
                            

    def EulerUpdateFilter(self, gyro, accelero, magneto):
	
	Euler = [0.0, 0.0, 0.0]

	#convert accelero and magneto units
	accelero[0] /= 9.81
	accelero[1] /= 9.81
	accelero[2] /= 9.81
	magneto[0] /= 100
	magneto[1] /= 100
	magneto[2] /= 100

	#update Magdwick filter
	(self.globSEq, self.globb, self.globwb) = self.filterUpdate(gyro, accelero, magneto, self.globSEq, self.globb, self.globwb)
	
	#convert to euler angle
	rad2deg = 180/3.14159265358979
	Euler = self.quatern2euler(self.quaternConj(self.globSEq))
	Euler[0] *= rad2deg
	Euler[1] *= rad2deg
	Euler[2] *= rad2deg
	
	return Euler
                                
    def EulerUpdateFilterSimple(self, gyro, accelero, magneto):
                                
    Euler = [0.0, 0.0, 0.0]
                                
    #convert accelero and magneto units
    accelero[0] /= 9.81
    accelero[1] /= 9.81
    accelero[2] /= 9.81
    magneto[0] /= 100
    magneto[1] /= 100
    magneto[2] /= 100
                                
    #update Magdwick simple filter
    (self.globSEq) = self.filterUpdateSimple(gyro, accelero, magneto, self.globSEq)
                                
    #convert to euler angle
    #rad2deg = 180/3.14159265358979
    rad2deg = 57.29577951
    Euler = self.quatern2euler(self.quaternConj(self.globSEq))
    Euler[0] *= rad2deg
    Euler[1] *= rad2deg
    Euler[2] *= rad2deg
                                
    return Euler
	

