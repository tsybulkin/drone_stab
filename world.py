#
# Reinforcement learning
#

import numpy as np
import dynamics

TAU = 0.01


class World():
	def __init__(self):
		self.height = None
		self.velocity = None
		
	
	def initialize(self):
		self.height = 0
		self.velocity = 0
		

	
	def take_agents_action(self, action):	
		self.height, self.velocity = dynamics.dynamics(self.height, self.velocity, action, TAU)

		if self.height > 0:
			return False

		return True # drone landed
		


	def get_legal_actions(self, coords):
		return [-12, -7, -3, 0, 3, 7, 12]



	def show(self):
		pass


	

