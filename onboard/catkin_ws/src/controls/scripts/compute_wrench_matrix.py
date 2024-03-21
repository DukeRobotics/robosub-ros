#!/usr/bin/env python3

import yaml
import pandas as pd
import numpy as np
from sympy import Matrix, rad
from sympy.matrices import rot_ccw_axis1, rot_ccw_axis2, rot_ccw_axis3
import os
import resource_retriever as rr
import rospy

CONTROLS_PACKAGE_PATH = 'package://controls/'
CONFIG_FILE_PATH_TEMPLATE = CONTROLS_PACKAGE_PATH + 'config/%s.yaml'


class ComputeWrenchMatrix:
    """
    Computes the wrench matrix and wrench matrix pseudoinverse for a given robot using symbolic math.
    Saves the results to CSV files.

    Attributes:
        wrench_matrix: The wrench matrix as a SymPy matrix
        wrench_matrix_pinv: The wrench matrix pseudoinverse as a SymPy matrix
        wrench_matrix_array: The wrench matrix as a numpy array
        wrench_matrix_pinv_array: The wrench matrix pseudoinverse as a numpy array
        wrench_matrix_df: The wrench matrix as a pandas dataframe
        wrench_matrix_pinv_df: The wrench matrix pseudoinverse as a pandas dataframe
    """

    # Get the robot name from the user
    def get_robot_name(self):
        # Get the default value from the ROBOT_NAME environment variable
        default_robot_name = os.getenv('ROBOT_NAME')

        # Ask the user for the robot name
        user_robot_name = input(f"Enter the robot name (press enter for default '{default_robot_name}'): ")

        # Use the default value if the user input is empty
        return user_robot_name.strip() if user_robot_name.strip() else default_robot_name

    # Get the rotation matrix for a given roll, pitch, and yaw
    # Order of rotations: roll -> pitch -> yaw
    # See https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions
    def rotation_matrix(self, roll, pitch, yaw):
        R_roll = rot_ccw_axis1(roll)
        R_pitch = rot_ccw_axis2(pitch)
        R_yaw = rot_ccw_axis3(yaw)
        return R_yaw * R_pitch * R_roll

    # Compute the force and torque vectors for a given thruster
    def compute_force_torque(self, thruster):
        # Get thruster data
        pos = Matrix(thruster['pos'])
        rpy = thruster['rpy']
        flipped = -1 if thruster['flipped'] else 1

        # Compute force vector
        rpy_radians = map(rad, rpy)
        R = self.rotation_matrix(*rpy_radians)
        force = R * Matrix([1, 0, 0]) * flipped

        # Compute torque vector
        torque = pos.cross(force)

        return force, torque

    def run(self):
        # Get the path to the config file for the robot the user would like to compute the wrench matrix for
        robot_name = self.get_robot_name()
        config_file_path = rr.get_filename(CONFIG_FILE_PATH_TEMPLATE % robot_name, use_protocol=False)

        # Read YAML data
        with open(config_file_path, 'r') as file:
            vehicle = yaml.safe_load(file)

        # Initialize wrench matrix
        self.wrench_matrix = Matrix.zeros(6, len(vehicle['thrusters']))

        # Compute force and torque vectors and add them to the wrench matrix
        for idx, thruster in enumerate(vehicle['thrusters']):
            force, torque = self.compute_force_torque(thruster)

            # Add force and torque to the wrench matrix
            self.wrench_matrix[0:3, idx] = force
            self.wrench_matrix[3:6, idx] = torque

        # Compute pseudoinverse of wrench matrix
        self.wrench_matrix_pinv = self.wrench_matrix.pinv()

        # Convert sympy matrices to numpy matrices
        self.wrench_matrix_array = np.array(self.wrench_matrix).astype(np.float64)
        self.wrench_matrix_pinv_array = np.array(self.wrench_matrix_pinv).astype(np.float64)

        # Convert numpy matrices to pandas dataframes
        self.wrench_matrix_df = pd.DataFrame(self.wrench_matrix_array)
        self.wrench_matrix_pinv_df = pd.DataFrame(self.wrench_matrix_pinv_array)

        # Set all values less than e-10 to 0
        self.wrench_matrix_df = self.wrench_matrix_df.round(10)
        self.wrench_matrix_pinv_df = self.wrench_matrix_pinv_df.round(10)

        # Convert negative zeroes to positive zeroes
        self.wrench_matrix_df = self.wrench_matrix_df.replace(-0.0, 0.0)
        self.wrench_matrix_pinv_df = self.wrench_matrix_pinv_df.replace(-0.0, 0.0)

        # Get full paths to CSV files
        wrench_matrix_file_path = rr.get_filename(CONTROLS_PACKAGE_PATH + vehicle["wrench_matrix_file_path"],
                                                  use_protocol=False)
        wrench_matrix_pinv_file_path = rr.get_filename(CONTROLS_PACKAGE_PATH + vehicle["wrench_matrix_pinv_file_path"],
                                                       use_protocol=False)

        # Export data to CSV files
        self.wrench_matrix_df.to_csv(wrench_matrix_file_path, index=False, header=False)
        self.wrench_matrix_pinv_df.to_csv(wrench_matrix_pinv_file_path, index=False, header=False)

        # Print data to console
        print("Wrench matrix:")
        print(self.wrench_matrix_df)
        print()
        print("Wrench matrix pseudoinverse:")
        print(self.wrench_matrix_pinv_df)
        print()
        print("Saved wrench matrix to %s" % wrench_matrix_file_path)
        print("Saved wrench matrix pseudoinverse to %s" % wrench_matrix_pinv_file_path)


if __name__ == '__main__':
    try:
        ComputeWrenchMatrix().run()
    except rospy.ROSInterruptException:
        pass
