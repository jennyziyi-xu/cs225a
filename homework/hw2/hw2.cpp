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
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm.urdf";

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
        else if (controller_number < 1 || controller_number > 4) {
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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.10);
	VectorXd control_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Jv = robot->Jv(link_name, pos_in_link);
	Lambda = robot->taskInertiaMatrix(Jv);
	J_bar = robot->dynConsistentInverseJacobian(Jv);
	N = robot->nullspaceMatrix(Jv);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = Sai2Common::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = redis_client.getEigen(JOINT_ANGLES_KEY);
    VectorXd robot_dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client.addToSendGroup(GRAVITY_COMP_ENABLED_KEY, gravity_comp_enabled);

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
    file_1.open("../../homework/hw2/data_files/q1.txt");

    ofstream file_2;
    file_2.open("../../homework/hw2/data_files/q2.txt");
    
    ofstream file_2_c;
    file_2_c.open("../../homework/hw2/data_files/q2_c.txt");

    ofstream file_2_d;
    file_2_d.open("../../homework/hw2/data_files/q2_d.txt");

    ofstream file_3;
    file_3.open("../../homework/hw2/data_files/q3.txt");

    ofstream file_4_i;
    file_4_i.open("../../homework/hw2/data_files/q4_i.txt");

    ofstream file_4_ii;
    file_4_ii.open("../../homework/hw2/data_files/q4_ii.txt");

    ofstream file_4_iii;
    file_4_iii.open("../../homework/hw2/data_files/q4_iii.txt");

    ofstream file_4_iv;
    file_4_iv.open("../../homework/hw2/data_files/q4_iv.txt");


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
            

            file_1 << time << "\t" << robot->q()(6) << "\n"; 

            control_torques.setZero();
            VectorXd kp(7);
            kp << 400.0, 400.0, 400.0, 400.0, 400.0, 400.0, 50.0;
            VectorXd kv(7);
            kv << 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, -0.34;

            VectorXd q_desired = initial_q;  
            q_desired[6] = 0.1;
            // cout << kp.size() << endl;
            control_torques = - kp.cwiseProduct(robot->q() - q_desired) - kv.cwiseProduct(robot->dq())  + robot -> coriolisForce() + robot->jointGravityVector();
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            // part a) 

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_2 << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 
            // double kp = 200.0;      
            // double kv = 45.0;      
            // control_torques.setZero();
            // Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5); 

            // Jv = robot->Jv(link_name, pos_in_link);
	        // Lambda = robot->taskInertiaMatrix(Jv);

            // VectorXd F = Lambda * (-kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            // control_torques  = Jv.transpose() * F + robot->jointGravityVector();

            // // part c)
            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_2_c << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 
            // double kp = 200.0;      
            // double kv = 45.0;      
            // double kvj = 8;
            // control_torques.setZero();
            // Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5); 

            // Jv = robot->Jv(link_name, pos_in_link);
	        // Lambda = robot->taskInertiaMatrix(Jv);

            // VectorXd F = Lambda * (-kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            // control_torques  = Jv.transpose() * F + robot->jointGravityVector() - kvj * robot->dq();

            
            //part d
            Vector3d ee_pos = robot->position(link_name, pos_in_link);
            VectorXd robot_q_pos  = robot->q();
            file_2_d << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 
            double kp = 200.0;      
            double kv = 45.0;      
            double kvj = 8;
            control_torques.setZero();
            Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5); 

            Jv = robot->Jv(link_name, pos_in_link);
	        Lambda = robot->taskInertiaMatrix(Jv);
            N = robot->nullspaceMatrix(Jv);

            VectorXd F = Lambda * (-kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            control_torques  = Jv.transpose() * F + robot->jointGravityVector() - N.transpose() * robot->M() * kvj * robot->dq();
            
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            Vector3d ee_pos = robot->position(link_name, pos_in_link);
            VectorXd robot_q_pos  = robot->q();
            file_3 << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 

            double kp = 200.0;      
            double kv = 45.0;      // chose your d gain
            double kvj = 8;

            control_torques.setZero();
            Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5); 
            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            J_bar = robot->dynConsistentInverseJacobian(Jv);
            N = robot->nullspaceMatrix(Jv);
            VectorXd p = J_bar.transpose() * robot->jointGravityVector();
            VectorXd F = Lambda * (kp * (pos_desired - robot->position(link_name, pos_in_link)) - kv * robot->linearVelocity(link_name, pos_in_link)) + p;
            control_torques = Jv.transpose() * F - N.transpose() * robot->M() * kvj * robot->dq();

        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            // control_torques.setZero();

            // // part i

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_4_i << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 

            // double kp = 250.0;      
            // double kv = 45.0;      // chose your d gain
            // double kvj = 10;

            // Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5) + 0.1 * Vector3d(sin(M_PI * time), cos(M_PI * time), 0); 
            // Jv = robot->Jv(link_name, pos_in_link);
            // Lambda = robot->taskInertiaMatrix(Jv);
            // J_bar = robot->dynConsistentInverseJacobian(Jv);
            // N = robot->nullspaceMatrix(Jv);
            // VectorXd p = J_bar.transpose() * robot->jointGravityVector();
            // VectorXd F = Lambda * (kp * (pos_desired - robot->position(link_name, pos_in_link)) - kv * robot->linearVelocity(link_name, pos_in_link)) + p;
            // control_torques = Jv.transpose() * F - N.transpose() * robot->M() * kvj * robot->dq();


            // // part ii

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_4_ii << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 

            // double kp = 250.0;      
            // double kv = 45.0;      
            // double kvj = 10;

            // Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5) + 0.1 * Vector3d(sin(M_PI * time), cos(M_PI * time), 0); 
            // Jv = robot->Jv(link_name, pos_in_link);
            // J_bar = robot->dynConsistentInverseJacobian(Jv);
            // N = robot->nullspaceMatrix(Jv);
            // VectorXd p = J_bar.transpose() * robot->jointGravityVector();
            // VectorXd F = kp * (pos_desired - robot->position(link_name, pos_in_link)) - kv * robot->linearVelocity(link_name, pos_in_link) + p;
            // control_torques = Jv.transpose() * F - N.transpose() * robot->M() * kvj * robot->dq();

            // // part iii

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_4_iii << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 

            // double kp = 250.0;      
            // double kv = 45.0;     
            // double kvj = 10;          
            // double kpj = 10;

            // Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5) + 0.1 * Vector3d(sin(M_PI * time), cos(M_PI * time), 0); 
            // Jv = robot->Jv(link_name, pos_in_link);
            // J_bar = robot->dynConsistentInverseJacobian(Jv);
            // N = robot->nullspaceMatrix(Jv);
            // VectorXd p = J_bar.transpose() * robot->jointGravityVector();
            // VectorXd F = Lambda * (kp * (pos_desired - robot->position(link_name, pos_in_link)) - kv * robot->linearVelocity(link_name, pos_in_link)) + p;
            // VectorXd q_desired = initial_q;
            // q_desired << 0,0,0,0,0,0,0;
            // control_torques = Jv.transpose() * F - N.transpose() * robot->M() * (kvj * robot->dq() + kpj * (robot->q() - q_desired));


            // // part iv

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_4_iv << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(0) << "\t" << robot_q_pos(1) << "\t" << robot_q_pos(2) << "\t" << robot_q_pos(3) << "\t" << robot_q_pos(4) << "\t" << robot_q_pos(5)  << "\t" << robot_q_pos(6) << "\n"; 

            // double kp = 250.0;      // increase this to better track the trajectory 
            // double kv = 45.0;      // reduce this to reduce the damping. 
            // double kvj = 10;        // If I reduce it, it will increase the damping. 
            // double kpj = 10;       // this can be increased to like 20. 

            // Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5) + 0.1 * Vector3d(sin(M_PI * time), cos(M_PI * time), 0); 
            // Jv = robot->Jv(link_name, pos_in_link);
            // J_bar = robot->dynConsistentInverseJacobian(Jv);
            // N = robot->nullspaceMatrix(Jv);
            // VectorXd p = J_bar.transpose() * robot->jointGravityVector();
            // VectorXd F = Lambda * (kp * (pos_desired - robot->position(link_name, pos_in_link)) - kv * robot->linearVelocity(link_name, pos_in_link)) + p;
            // VectorXd q_desired = initial_q;
            // q_desired << 0,0,0,0,0,0,0;
            // control_torques = Jv.transpose() * F - N.transpose() * robot->M() * (kvj * robot->dq() + kpj * (robot->q() - q_desired)) + robot->jointGravityVector();
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
    }

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
