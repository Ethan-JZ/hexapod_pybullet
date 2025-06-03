import json

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
        turning_data = self._read_turning_data()

        turning_data_list = []
        for data_instance in turning_data:
            temp = [round(angle * (180 / 3.141592653589793), 2) for angle in data_instance]
            turning_data_list.append(temp)
        return turning_data_list

    def convert_degree_to_motor_frame(self):
        """
        Converts the turning data from the robot frame to the motor frame.
        Output:
            list: A list of turning data converted to the motor frame.
        """
        turning_data_list = self._convert_turning_data_to_degrees()
        motor_frame_data = []

        for data_instance in turning_data_list:
            
            # leg 1
            j11_motor = data_instance[0] + 120
            j12_motor = data_instance[1] + 120
            j13_motor = data_instance[2] + 120

            # leg 4
            j41_motor = data_instance[3] + 120
            j42_motor = data_instance[4] + 120
            j43_motor = data_instance[5] + 120

            # leg 2
            j21_motor = data_instance[6] + 120
            j22_motor = data_instance[7] + 120
            j23_motor = data_instance[8] + 120

            # leg 5
            j51_motor = data_instance[9] + 120
            j52_motor = data_instance[10] + 120
            j53_motor = data_instance[11] + 120

            # leg 6
            j61_motor = data_instance[12] + 120
            j62_motor = data_instance[13] + 120
            j63_motor = data_instance[14] + 120

            motor_frame_data.append([
                j11_motor, j12_motor, j13_motor,
                j41_motor, j42_motor, j43_motor,
                j21_motor, j22_motor, j23_motor,
                j51_motor, j52_motor, j53_motor,
                j61_motor, j62_motor, j63_motor
            ])
            
        return motor_frame_data
