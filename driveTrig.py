import math

def findDist(r1, r2, theta):
	theta = theta * (math.pi/180)
	numerator = r1 * r2 * math.sin(theta)
	denom_wo_sqrt = r1 ** 2 + r2 ** 2 + (-2*r1*r2*math.cos(theta))
	denom = math.sqrt(denom_wo_sqrt)
	d = numerator / denom
	return d
