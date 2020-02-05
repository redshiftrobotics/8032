#!/usr/bin/env python3

import weakref

"""
This file holds our state management. The state of the robot is used to
determine what should be happening and when.
"""

""" Constants that represent various robot states """
DRIVE_FORWARD_TWO_SEC = "DRIVE_FORWARD_TWO_SEC"
TANK_DRIVE_NORMAL = "TANK_DRIVE_NORMAL"
FREEZE = "FREEZE"

"""
\class StateOpts is a simple utility that acts as an enum wrapper for all 
our states.
"""
class StateOpts(set):
	def __init__(self):
		super().__init__([
			DRIVE_FORWARD_TWO_SEC,
			TANK_DRIVE_NORMAL,
			FREEZE
		])
	
	""" Gets a state and throws if the state does not exist. """
	def __getattr__(self, name):
		if name in self:
			return name
		print('Could not find %s in StateOpts.', name)
		raise AttributeError

"""
\class State handles all state of our robot. It handles updating what state 
we should currently have, what state we do have, and manually overriding 
state.
"""
class State:
	"""
	Creates a state based on our parent and an optional \param default_state
	
	\param parent should be a reference to `Robot`.
	"""
	def __init__(self, parent, default_state = FREEZE):
		"""
		TODO: assert that other things we need exist here
		TODO: use protocol here
		"""
		assert parent.timer is not None
		"""
		It's really important that we have a weakref here. If the ref count
		is incremented here, then we will create a circle reference and the 
		garbage collector will never be able to delete either object creating
		a leak.
		"""
		self.parent = weakref.ref(parent)
		
		self.opts = StateOpts()
		
		if default_state not in self.opts:
			raise AttributeError
		self.state = default_state

	def dispatch(self, new_state):
		if new_state not in self.opts:
			raise AttributeError
		self.state = new_state
	
	""" 
	Updates state to reflect the current state of the robot. This method 
	should be called in every periodic method on the robot.
	"""
	def update(self):
		""" kTeleop = 3; for now, just drive strait in teleop. """
		if 3 == 3:
			self.dispatch(TANK_DRIVE_NORMAL)
			return
		
		""" Otherwise, we're in auto so, drive strait for two seconds. """
		if self.parent().timer.get() < 2:
			self.dispatch(DRIVE_FORWARD_TWO_SEC)
		else:
			self.dispatch(FREEZE)
	