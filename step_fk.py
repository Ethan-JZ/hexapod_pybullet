import sympy as sp
import numpy as np
from helper import remove_small_terms

class HexapodLegFK:
    
    def __init__(self, dh_dict: dict, dof: int):
        
        self.a_list     = dh_dict["a"]  # should be a list of values
        self.alpha_list = dh_dict["alpha"]
        self.d_list     = dh_dict["d"]
        self.dof        = dof
    
    @staticmethod
    def _form_theta_var():
        """form theta variable"""
        theta1, theta2, theta3 = sp.symbols('theta1 theta2 theta3')
        return theta1, theta2, theta3
    
    @staticmethod
    def _dh_transform_sym(theta: sp.Symbol, d: float, a: float, alpha: float) -> sp.Matrix:
        """
        Compute symbolic DH transformation matrix.
        Input:
        theta: symbolic variable, theta var in DH table
        d: float variable, d constant in DH table
        a: float variable, a constant in DH table
        alpha: float variable, alpha constant in DH table
        Output:
        sp.Matrix: symbolic matrix with theta as variable
        """
        
        symbol_matrix = sp.Matrix([
            [sp.cos(theta), -sp.sin(theta)*np.cos(alpha),  sp.sin(theta)*np.sin(alpha), a*sp.cos(theta)],
            [sp.sin(theta),  sp.cos(theta)*np.cos(alpha), -sp.cos(theta)*np.sin(alpha), a*sp.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0,              0,                            0,                           1]
        ])
        
        # remove small terms in the matrix
        for i in range(symbol_matrix.shape[0]):
            for j in range(symbol_matrix.shape[1]):
                symbol_matrix[i, j] = remove_small_terms(symbol_matrix[i, j])

        return symbol_matrix
    
    @staticmethod
    def _dh_transform_numerical(theta: float, a: float, d: float, alpha: float) -> np.ndarray:
        """
        Compute the numerical DH transformation matrix.
        Input:
        theta: symbolic variable, theta var in DH table
        d: float variable, d constant in DH table
        a: float variable, a constant in DH table
        alpha: float variable, alpha constant in DH table
        Output:
        4x4 matrix of the transform
        """
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_alpha = np.cos(alpha)
        s_alpha = np.sin(alpha)

        return np.array([
            [c_theta, -s_theta * c_alpha, s_theta * s_alpha, a * c_theta],
            [s_theta, c_theta * c_alpha, -c_theta * s_alpha, a * s_theta],
            [0, s_alpha, c_alpha, d],
            [0, 0, 0, 1]
        ])
    
    def _get_transforms(self):
        """
        get symbolic transformation matrix from DH table dict

        Output:
        a list of transforms [[4x4 matrix], ...]
        """

        # get the theta variable list
        theta_list = self._form_theta_var()
        
        # init transforms list
        transforms = []
        
        # loop to get each transform
        for i in range(self.dof):
            transforms.append(self._dh_transform_sym(theta_list[i], self.d_list[i], self.a_list[i], self.alpha_list[i]))
        
        return transforms
    
    def _get_rotation_matrix(self):
        """
        get symbolic rotation transformation matrix from DH table dict

        Output:
        a list of rotation matrix transforms [[3x3 matrix], ...]
        """
        
        # get all lhs and rhs transformation matrix
        transforms = self._get_transforms()

        rotation_matrices = []

        # loop to get the rotation matrix
        for transform in transforms:
            rotation_matrices.append(transform[:3, :3])
        
        return rotation_matrices
    
    def compute_fk_transform(self, flag: str ="symbol", theta_values=None):
        """
        compute the complete fk transforms along the kinematic chain
        Input: 
        flag: a str to determine the matrix is in which form
        theta_values: a list of theta values

        Output:
        a 4x4 matrix of the transform
        """

        # Get dh parameters
        if flag == "symbol":
            theta_list = self._form_theta_var()
        elif flag == "numerical":
            if theta_values == None:
                raise ValueError("Need to have theta values to continue computation")
            theta_list = theta_values

        # compute forward kinematics of the whole chain for both lhs and rhs
        transform = np.eye(4)

        for i in range(self.dof):  # lhs
            
            # get dh parameters for each link on the kinematic chain
            qi        = theta_list[i]
            di        = self.d_list[i]
            ai        = self.a_list[i]
            alphai    = self.alpha_list[i]
            
            # compute lhs transform
            transform = np.dot(transform, self._dh_transform_sym(qi, di, ai, alphai))
        
        return transform
    

if __name__ == "__main__":
    
    # init dh table
    l1 = 0.07109
    l2 = 0.08109
    l3 = 0.15209
    dh_dict = {
        "a": [l1, l2, l3],
        "alpha":[np.pi/2, 0, 0],
        "d":[0, 0, 0]
        }

    fk_model_per_leg = HexapodLegFK(dh_dict=dh_dict, dof=3)
    transform = fk_model_per_leg.compute_fk_transform(flag = "symbol")
    print(f"if theta is symbolic variable: ")
    print(f'x part of the transform: {transform[0, -1]}')
    print(f'y part of the transform: {transform[1, -1]}')
    print(f'z part of the transform: {transform[2, -1]}\n')

    theta_values = [0, 0, 0]
    transform = fk_model_per_leg.compute_fk_transform(flag = "numerical", theta_values=theta_values)
    print(f"if theta has certain values: {theta_values}")
    print(f'x part of the transform: {transform[0, -1]}')
    print(f'y part of the transform: {transform[1, -1]}')
    print(f'z part of the transform: {transform[2, -1]}\n')

    theta_values = [0, np.pi/2, 0]
    transform = fk_model_per_leg.compute_fk_transform(flag = "numerical", theta_values=theta_values)
    print(f"if theta has certain values: {theta_values}")
    print(f'x part of the transform: {transform[0, -1]}')
    print(f'y part of the transform: {transform[1, -1]}')
    print(f'z part of the transform: {transform[2, -1]}\n')
