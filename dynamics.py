#
# pendulum
#

from matplotlib import pyplot as plt
import sys


TAU = 0.02
U_MAX = 12.
G = 9.81
C = 0.1
m = 1.
L = 1.
Z_END = 2.


def control(z,dz,z_end):
	#return lim(U_MAX, m*G + 5*(z_end - z))  # proportional (1)
	return lim(U_MAX, m*G + 5*(z_end - z) - 3.7*dz)  # prop + dir (2)



def PID_control(de,e,ie,z,z_end,tau):
	Kd = 2.
	Kp = 7.
	Ki = 5.
	de, e, ie = (z_end - z - e)/tau, z_end - z, ie + (z_end - z)*TAU
	U = Kd*de + Kp*e + Ki*ie
	return U,de,e,ie


def run(T=1):
	t = 0
	z = 0.
	dz = 0.
	log = []

	while t < T:
		u = control(z,dz,Z_END)
		z,dz = dynamics(z,dz,u,TAU) 
		log.append((t,z,dz,u))
		t += TAU

	[Ti, Z, dZ, U] = zip(*log)
	plt.plot(Ti, Z, '-')
	plt.show()


def run_pid(T=1):
	t = 0
	z = 0.
	dz = 0.
	de,e,ie = 0., 0., 0.
	log = []

	while t < T:
		u,de,e,ie = PID_control(de,e,ie,z,Z_END,TAU)
		#print de,e,ie
		z,dz = dynamics(z,dz,u,TAU) 
		log.append((t,z,dz,u))
		t += TAU

	[Ti, Z, dZ, U] = zip(*log)
	plt.plot(Ti, Z, '-')
	plt.show()


def lim(LIM, val):
	if val < - LIM: return -LIM
	elif val > LIM: return LIM
	return val


def dynamics(z,dz,u,tau):
	dz += (-C*dz + u - m*G) * tau
	z += dz * tau
	if z < 0:
		return 0,0
	return z,dz


if __name__ == '__main__':
	args = sys.argv[:]
	if len(args) == 2:
		run(float(args[1]))
	else:
		run()

