import numpy as np
import sympy as sp
from helper import remove_small_terms
from step_fk import HexapodLegFK


class FootTippleIKSolver(HexapodLegFK):

    def __init__(self, step_design_dict):
        
        # get dh
        super().__init__(self)
        
        # design parameters
        self.step_length = step_design_dict["step_length"]  # unit in meters
        self.lift_height = step_design_dict["lift_height"]
        self.phase_offset = step_design_dict["phase_offset"]
        self.num_steps = step_design_dict["num_steps"]

        
    def _sin_wave_generator(self):
        
        # define the omega
        period_span = self.step_length / 2
        omega = np.pi * 2 / period_span

        # define the sine function
        steps = np.linspace(0, self.step_length, self.num_steps)
        sin_wave = self.lift_height * np.sin(omega * steps + self.phase_offset)

        return sin_wave
    
    def _fk_foot_tipple(self):
        pass

    def _ik_foot_tipple(self):
        pass
    
    

    
    
    
    



    
    def _jacobian(self):

        # get the theta variable list
        theta_list = self._form_theta_var()

    def _jacobian_w(self):
        pass

    def _jacobian_v(self):
        pass



