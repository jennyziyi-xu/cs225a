// some standard library includes
#include <math.h>

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

// sai2 main libraries includes
#include "Sai2Model.h"

// sai2 utilities from sai2-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm_controller.urdf";

int main(int argc, char** argv) {
    Sai2Model::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);

    // check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{HW_NUMBER}-control {QUESTION_NUMBER}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 6) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new Sai2Model::Sai2Model(robot_file);

    // prepare controller
    int dof = robot->dof();
    VectorXd control_torques = VectorXd::Zero(dof);

    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = VectorXd::Zero(dof);
    VectorXd robot_dq = VectorXd::Zero(dof);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);

    redis_client.receiveAllFromGroup();
    redis_client.sendAllFromGroup();

    // update robot model from simulation configuration
    robot->setQ(robot_q);
    robot->setDq(robot_dq);
    robot->updateModel();

    // record initial configuration
    VectorXd initial_q = robot->q();

    // create a loop timer
    const double control_freq = 1000;
    Sai2Common::LoopTimer timer(control_freq);

    // File for question 1. 
    ofstream file_1;
    file_1.open("../../homework/hw1/data_files/q1.txt");

    // File for question 2
    ofstream file_2;
    file_2.open("../../homework/hw1/data_files/q2.txt");

    // File for question 3
    ofstream file_3;
    file_3.open("../../homework/hw1/data_files/q3.txt");

    // File for question 4
    ofstream file_4;
    file_4.open("../../homework/hw1/data_files/q4.txt");

    // File for question 5
    ofstream file_5;
    file_5.open("../../homework/hw1/data_files/q5.txt");

    // File for question 6
    ofstream file_6;
    file_6.open("../../homework/hw1/data_files/q6.txt");


    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

        // read robot state from redis
        redis_client.receiveAllFromGroup();
        robot->setQ(robot_q);
        robot->setDq(robot_dq);
        robot->updateModel();

        // **********************
        // WRITE YOUR CODE AFTER
        // **********************

        // ---------------------------  question 1 ---------------------------------------
        if(controller_number == 1) {

            double kp = 400.0;      // chose your p gain
            double kv = 52.0;      // chose your d gain

            file_1 << time << "\t" << robot->q()(0) <<  "\t" << robot->q()(1) << "\t" << robot->q()(2) <<  "\t" << robot->q()(3) <<  "\t" << robot->q()(4) <<  "\t" << robot->q()(5) << "\t" << robot->q()(6) << "\n"; 

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, -(125*M_PI/180), 0, 80 * M_PI / 180, 0;

            control_torques.setZero();  // change to the control torques you compute

            control_torques = - kp*(robot->q() - q_desired) - kv * robot->dq();

        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            file_2 << time << "\t" << robot->q()(0) <<  "\t" << robot->q()(1) << "\t" << robot->q()(2) <<  "\t" << robot->q()(3) <<  "\t" << robot->q()(4) <<  "\t" << robot->q()(5) << "\n"; 

            double kp = 400.0;      // chose your p gain
            double kv = 52.0;      // chose your d gain

            VectorXd q_desired = initial_q;   
            q_desired << M_PI/2, - M_PI/4, 0, -(125*M_PI/180), 0, 80 * M_PI / 180, 0;

            control_torques.setZero();  

            control_torques = - kp * (robot->q() - q_desired) - kv * robot->dq()  + robot->jointGravityVector();

        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            file_3 << time << "\t" << robot->q()(0) <<  "\t" << robot->q()(1) << "\t" << robot->q()(2) <<  "\t" << robot->q()(3) <<  "\t" << robot->q()(4) <<  "\t" << robot->q()(5) << "\n"; 

            double kp = 400.0;      // chose your p gain
            double kv = 40.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, -(125*M_PI/180), 0, 80 * M_PI / 180, 0;

            control_torques.setZero();  // change to the control torques you compute

            control_torques = robot->M() * (- kp*(robot->q() - q_desired) - kv * robot->dq())  + robot->jointGravityVector();
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            file_4 << time << "\t" << robot->q()(0) <<  "\t" << robot->q()(1) << "\t" << robot->q()(2) <<  "\t" << robot->q()(3) <<  "\t" << robot->q()(4) <<  "\t" << robot->q()(5) << "\n"; 

            double kp = 400.0;      // chose your p gain
            double kv = 39.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, -(125 * M_PI/180), 0, 80 * M_PI / 180, 0;

            control_torques.setZero();  // change to the control torques you compute

            control_torques = robot->M() * (- kp*(robot->q() - q_desired) - kv * robot->dq()) + robot -> coriolisForce() + robot->jointGravityVector();

        }

        // ---------------------------  question 5 ---------------------------------------
        else if(controller_number == 5) {

            file_5 << time << "\t" << robot->q()(0) <<  "\t" << robot->q()(1) << "\t" << robot->q()(2) <<  "\t" << robot->q()(3) <<  "\t" << robot->q()(4) <<  "\t" << robot->q()(5) << "\n"; 

            double kp = 400.0;      // chose your p gain
            double kv = 39.0;      // chose your d gain

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, - (125 * M_PI/180), 0, 80 * M_PI / 180, 0;

            control_torques.setZero();  // change to the control torques you compute

            control_torques = robot->M()  * (- kp*(robot->q() - q_desired) - kv * robot->dq()) + robot -> coriolisForce() + robot->jointGravityVector();
        }

        else if(controller_number == 6) {

            file_6 << time << "\t" << robot->q()(0) <<  "\t" << robot->q()(1) << "\t" << robot->q()(2) <<  "\t" << robot->q()(3) <<  "\t" << robot->q()(4) <<  "\t" << robot->q()(5) << "\n"; 

            double kp = 400.0;      // chose your p gain
            double kv = 38.0;      // chose your d gain

            const string ee_link_name = "link7"; 
            Vector3d ee_pos_in_link = Vector3d(0.0, 0.0, 0.17); 
            int dof = robot->dof();
            MatrixXd ee_jacobian(3, dof);
            ee_jacobian = robot->Jv(ee_link_name, ee_pos_in_link);

            Vector3d gravity_ee = Vector3d(0.0, 0.0, 9.8);
            

            VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
            q_desired << M_PI/2, - M_PI/4, 0, -(125*M_PI/180), 0, 80 * M_PI / 180, 0;

            control_torques.setZero();  // change to the control torques you compute

            control_torques = (robot->M() + 2.5 * ee_jacobian.transpose() * ee_jacobian) * (- kp*(robot->q() - q_desired) - kv * robot->dq()) + robot -> coriolisForce() + robot->jointGravityVector() + ee_jacobian.transpose() * 2.5 * gravity_ee;
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.setInt("sai2::interfaces::simviz::gravity_comp_enabled", 0);
        redis_client.sendAllFromGroup();
    }

    control_torques.setZero();
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
