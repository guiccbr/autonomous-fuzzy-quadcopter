# Universidade Estadual de Campinas - UNICAMP
# Trabalho Final de Curso
# Diego Pereira Domingos RA 090923
# Guilherme Campos Camargo RA

import numpy as np

class model():

	def __init__(self,params=None):
		if params == None:
			self.m = 1
			self.g = 9.81
			self.k = 0.000003
			self.kd = 0.25
			self.I = np.diag([0.005, 0.005, 0.010])
			self.L = 0.25
			self.b = 0.00007
		else:
			self.m = params[0]
			self.g = params[1]
			self.k = params[2]
			self.kd = params[3]
			self.I = params[4]
			self.L = params[5]
			self.b = params[6]




