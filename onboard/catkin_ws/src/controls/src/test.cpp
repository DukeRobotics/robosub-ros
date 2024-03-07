#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>

#include "controls_utils.h"

int osqp_eigen_test() {
    Eigen::MatrixXd W;
    ControlsUtils::read_matrix_from_csv("/root/dev/robosub-ros/onboard/catkin_ws/src/controls/data/oogway_wrench.csv",
                                        W);

    Eigen::MatrixXd W_pinv;
    ControlsUtils::read_matrix_from_csv(
        "/root/dev/robosub-ros/onboard/catkin_ws/src/controls/data/oogway_wrench_pinv.csv", W_pinv);

    Eigen::VectorXd b(6);
    b << 2, 1.3, -1.5, 0.05, 0.1, 0.3;
    std::cout << "b: " << std::endl;
    std::cout << b << std::endl;

    OsqpEigen::Solver solver;

    int numberOfVariables = W.cols();
    int numberOfConstraints = W.cols();
    Eigen::SparseMatrix<double> hessian = (W.transpose() * W).sparseView();
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(6).transpose() * W;
    Eigen::SparseMatrix<double> linearMatrix = Eigen::MatrixXd::Identity(W.cols(), W.cols()).sparseView();
    Eigen::VectorXd lowerBound = -Eigen::VectorXd::Ones(W.cols());
    Eigen::VectorXd upperBound = Eigen::VectorXd::Ones(W.cols());

    solver.data()->setNumberOfVariables(numberOfVariables);
    solver.data()->setNumberOfConstraints(numberOfConstraints);
    if (!solver.data()->setHessianMatrix(hessian)) return 1;
    if (!solver.data()->setGradient(gradient)) return 1;
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
    if (!solver.data()->setLowerBound(lowerBound)) return 1;
    if (!solver.data()->setUpperBound(upperBound)) return 1;

    solver.settings()->setPolish(true);
    solver.settings()->setVerbosity(false);

    if (!solver.initSolver()) return 1;

    // gradient = -b.transpose() * W;
    // std::cout << "b transpose" << std::endl;

    gradient = -W.transpose() * b;
    // std::cout << "W transpose" << std::endl;
    if (!solver.updateGradient(gradient)) return 1;
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;
    Eigen::VectorXd QPSolution = solver.getSolution();

    // cout qp solution
    std::cout << "QP solution: " << std::endl;
    std::cout << W * QPSolution << std::endl;
    std::cout << std::endl;

    Eigen::VectorXd std_solution = W_pinv * b;

    // If maximum absolute value in std_solution is greater than 1, scale it down
    double max_abs = std_solution.cwiseAbs().maxCoeff();
    if (max_abs > 1) {
        std_solution /= max_abs;
    }

    std::cout << "Standard solution: " << std::endl;
    // std::cout << std_solution << std::endl;
    // std::cout << std::endl;
    std::cout << W * std_solution << std::endl;
    std::cout << std::endl;

    return 0;

    b << 5, 2.3, -0.5, 0.2, 0.7, 0.4;
    std::cout << "b: " << std::endl;
    std::cout << b << std::endl;

    gradient = -b.transpose() * W;
    if (!solver.updateGradient(gradient)) return 1;

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;
    QPSolution = solver.getSolution();

    std::cout << "QP solution: " << std::endl;
    std::cout << W * QPSolution << std::endl;
    std::cout << std::endl;

    std_solution = W_pinv * b;
    std::cout << "Standard solution: " << std::endl;
    std::cout << std_solution << std::endl;
    std::cout << std::endl;
    std::cout << W * std_solution << std::endl;
    std::cout << std::endl;

    // If maximum absolute value in std_solution is greater than 1, scale it down
    max_abs = std_solution.cwiseAbs().maxCoeff();
    if (max_abs > 1) {
        std_solution /= max_abs;
    }

    std::cout << "Scaled standard solution: " << std::endl;
    std::cout << std_solution << std::endl;
    std::cout << std::endl;
    std::cout << W * std_solution << std::endl;
    std::cout << std::endl;

    return 0;
}

void vector_to_quat() {
    tf2::Vector3 v_orig(0, 0, -1);
    tf2::Quaternion q(0, 0.7071068, 0, 0.7071068);
    tf2::Vector3 v_new = quatRotate(q.inverse(), v_orig);
    std::cout << "v_orig: " << v_orig.getX() << ", " << v_orig.getY() << ", " << v_orig.getZ() << std::endl;
    std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;
    std::cout << "v_new: " << v_new.getX() << ", " << v_new.getY() << ", " << v_new.getZ() << std::endl;
}

void rpy_from_quat() {
    // Get rpy from quaternion
    tf2::Quaternion q(0, 0, 0.9999912, -0.0042037);
    tf2::Quaternion nq = q.normalized();
    std::cout << "q: " << q.getX() << ", " << q.getY() << ", " << q.getZ() << ", " << q.getW() << std::endl;
    std::cout << "nq: " << nq.getX() << ", " << nq.getY() << ", " << nq.getZ() << ", " << nq.getW() << std::endl;

    std::cout << std::endl;

    // Equivalent to ZYX and python's quaternion_from_euler
    // In other words, given a set of roll, pitch, yaw angles (in radians) converted into a quaternion with
    // quaternion_from_euler with the default value for its axes arg (sxyz), the code below (getRPY), called on that
    // quaternion, will return the same original roll, pitch, yaw angles (in radians).
    double roll, pitch, yaw;
    tf2::Matrix3x3(nq).getRPY(roll, pitch, yaw);
    std::cout << "roll: " << roll << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "yaw: " << yaw << std::endl;

    std::cout << std::endl;

    tf2::Matrix3x3(nq).getEulerYPR(roll, pitch, yaw);
    std::cout << "roll: " << roll << std::endl;
    std::cout << "pitch: " << pitch << std::endl;
    std::cout << "yaw: " << yaw << std::endl;
}

void yaml_test() {
    YAML::Emitter out;
    out << "Hello, World!";

    std::cout << "Here's the output YAML:\n" << out.c_str();  // prints "Hello, World!"
}

void ros_test(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "hello_world_publisher");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a publisher for the "hello_world" topic
    ros::Publisher pub = nh.advertise<std_msgs::String>("hello_world_cpp", 10);

    // Set the publishing rate (in Hz)
    ros::Rate rate(50000);  // Publish at 1 Hz (1 message per second)

    while (ros::ok()) {
        // Create a message and set its data
        std_msgs::String msg;
        msg.data = "Hello, World!";

        // Publish the message
        pub.publish(msg);

        // Spin once to process callbacks
        ros::spinOnce();

        // Sleep to maintain the desired publishing rate
        rate.sleep();
    }
}

std::unordered_map<uint8_t, double> map_copy_func1(std::unordered_map<uint8_t, double> &map) {
    std::unordered_map<uint8_t, double> map_copy = map;
    map_copy[1] = 2;
    return map_copy;
}

std::unordered_map<uint8_t, double> global_map = {{1, 3}};

std::unordered_map<uint8_t, double> get_global_map() { return global_map; }

void map_copy_test() {
    std::unordered_map<uint8_t, double> map;
    map[1] = 1;

    std::unordered_map<uint8_t, double> map_copy = map_copy_func1(map);

    std::unordered_map<uint8_t, double> local_map = get_global_map();
    local_map[1] = 4;

    std::cout << "map[1]: " << map[1] << std::endl;
    std::cout << "map_copy[1]: " << map_copy[1] << std::endl;
    std::cout << "global_map[1]: " << global_map[1] << std::endl;
    std::cout << "local_map[1]: " << local_map[1] << std::endl;
}

void eigen_copy_test_helper(Eigen::MatrixXd &A, Eigen::MatrixXd &B) {
    B = A;
    B(0, 0) = 2;
}

void eigen_copy_test() {
    Eigen::MatrixXd A(2, 2);
    A << 1, 2, 3, 4;
    Eigen::MatrixXd B(2, 2);
    eigen_copy_test_helper(A, B);
    std::cout << "A: " << std::endl;
    std::cout << A << std::endl;
    std::cout << "B: " << std::endl;
    std::cout << B << std::endl;
}

int main(int argc, char **argv) {
    // map_copy_test();
    eigen_copy_test();
    return 0;
}