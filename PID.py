
class PIDController:
	def __init__(self,time, Kp=0, Kd=0, Ki=0):
		self._Kp = Kp
		self._Kd = Kd
		self._Ki = Ki

		self._integrator = 0
		self._error=0
		self._lastTime = time

	def update(self, error, time):
		P = self._Kp * error

		dt = float(time - self._lastTime)

		self._integrator += error * dt

		I = self._integrator * self._Ki

		D = self._Kd * (error - self._error) / dt

		PID = P + I + D

		self._lastTime = time

		self._error = error
		
		return PID


