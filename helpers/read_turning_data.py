import json
import numpy as np


class MapSimToReal:
    def __init__(self, file_path):
        self.file_path = file_path

    def _read_turning_data(self):
        """
        Reads turning data from a JSON file and returns the data as a dictionary.
        Input:
            file_path (str): The path to the JSON file containing the turning data.
        Output:
            list: A list containing the turning data.
        """
        with open(self.file_path, 'r') as file:
            turning_data = json.load(file)
        
        return turning_data

    def _convert_turning_data_to_degrees(self):
        """
        Converts the turning data from radians to degrees.

        Output:
            list: A list of turning data in degrees.
        """
        turning_data_dict = self._read_turning_data()
        loop_count = len(turning_data_dict["joint_11"])

        for joint_name in turning_data_dict.keys():
            for i in range(loop_count):
                turning_data_dict[joint_name][i] = np.degrees(turning_data_dict[joint_name][i])
        
        return turning_data_dict

    def convert_degree_to_motor_frame(self):
        """
        Converts the turning data from the robot frame to the motor frame.
        Output:
            list: A list of turning data converted to the motor frame.
        """
        joint_1_names_group16 = ["joint_11", "joint_61"]
        joint_1_names_group25 = ["joint_21", "joint_51"]
        joint_1_names_group34 = ["joint_41", "joint_31"]

        joint_2_names = ["joint_12", "joint_22", "joint_32", "joint_42", "joint_52", "joint_62"]
        joint_3_names = ["joint_13", "joint_23", "joint_33", "joint_43", "joint_53", "joint_63"]

        turning_data_dict = self._convert_turning_data_to_degrees()
        loop_count = len(turning_data_dict["joint_11"])
        motor_frame_data = []
        
        # change each joint angle to motor frame
        for joint_name in turning_data_dict.keys():
            for i in range(loop_count):
                if joint_name in joint_1_names_group16:
                    turning_data_dict[joint_name][i] = round(turning_data_dict[joint_name][i], 2) + 90
                elif joint_name in joint_1_names_group25:
                    turning_data_dict[joint_name][i] = round(turning_data_dict[joint_name][i], 2) + 120
                elif joint_name in joint_1_names_group34:
                    turning_data_dict[joint_name][i] = round(turning_data_dict[joint_name][i], 2) + 150
                
                elif joint_name in joint_2_names:
                    turning_data_dict[joint_name][i] = 120 - round(turning_data_dict[joint_name][i], 2)
                elif joint_name in joint_3_names:
                    turning_data_dict[joint_name][i] = 120 + round(turning_data_dict[joint_name][i], 2)

        # create a list of lists for each data instance
        for i in range(loop_count):
            motor_frame_data.append([
                turning_data_dict["joint_11"][i], turning_data_dict["joint_12"][i], turning_data_dict["joint_13"][i],
                turning_data_dict["joint_41"][i], turning_data_dict["joint_42"][i], turning_data_dict["joint_43"][i],
                turning_data_dict["joint_21"][i], turning_data_dict["joint_22"][i], turning_data_dict["joint_23"][i],
                turning_data_dict["joint_51"][i], turning_data_dict["joint_52"][i], turning_data_dict["joint_53"][i],
                turning_data_dict["joint_61"][i], turning_data_dict["joint_62"][i], turning_data_dict["joint_63"][i],
                turning_data_dict["joint_31"][i], turning_data_dict["joint_32"][i], turning_data_dict["joint_33"][i]
            ])
            
        return motor_frame_data
