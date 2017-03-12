#
# drone stabilization
#

from matplotlib import pyplot as plt
import sys


TAU = 0.02
Z_END = 2.
U_MAX = 12.
G = 9.81
C = 0.1
k = 5.
m = 1.


def control(z,dz,z_end):
	#return lim(U_MAX, m*G + 5*(z_end - z))
	return lim(U_MAX, m*G + 5*(z_end - z) - 3.7*dz)


def P_control(z,dz,z_end):
	return lim(U_MAX, k*(z_end - z) )


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

