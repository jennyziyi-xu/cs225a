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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
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

    ofstream file_1a;
    file_1a.open("../../homework/hw3/data_files/q1a.txt");

    // ofstream file_1c;
    // file_1c.open("../../homework/hw3/data_files/q1c.txt");

    ofstream file_2d;
    file_2d.open("../../homework/hw3/data_files/q2d.txt");

    // ofstream file_2e;
    // file_2e.open("../../homework/hw3/data_files/q2e.txt");

    // ofstream file_2f;
    // file_2f.open("../../homework/hw3/data_files/q2f.txt");

    // ofstream file_2g;
    // file_2g.open("../../homework/hw3/data_files/q2g.txt");

    ofstream file_3;
    file_3.open("../../homework/hw3/data_files/q3.txt");

    // ofstream file_4a;
    // file_4a.open("../../homework/hw3/data_files/q4a.txt");

    ofstream file_4b;
    file_4b.open("../../homework/hw3/data_files/q4b.txt");

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


            /////// a) 

            Vector3d ee_pos = robot->position(link_name, pos_in_link);
            VectorXd robot_q_pos  = robot->q();
            file_1a << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\n"; 

            double kp = 100.0;      
            double kv = 20.0;      
            double kpj = 50;       
            double kvj = 14;        
            

            Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5) + 0.1 * Vector3d(sin(M_PI * time), cos(M_PI * time), 0); 
            VectorXd q_desired = initial_q;
            q_desired << 0,0,0,0,0,0,0;
            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            
            N = robot->nullspaceMatrix(Jv);
            
            VectorXd F = Lambda * (-kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            control_torques.setZero();
            control_torques = Jv.transpose() * F + N.transpose() * (-kpj * (robot->q() - q_desired) - kvj * robot->dq()) + robot->jointGravityVector();
        
            ///////// c)
            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_1c << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\n"; 

            // double kp = 100.0;      
            // double kv = 20.0;      
            // double kpj = 50;       
            // double kvj = 14;        
            

            // Vector3d pos_desired = Vector3d(0.3, 0.1, 0.5) + 0.1 * Vector3d(sin(M_PI * time), cos(M_PI * time), 0); 
            // Vector3d xdot_desired = 0.1 * Vector3d(cos(M_PI * time) * M_PI , - sin(M_PI * time)* M_PI , 0); 
            // Vector3d xdotdot_desired = 0.1 * M_PI * M_PI * Vector3d(-sin(M_PI * time) , - cos(M_PI * time) , 0); 
            // VectorXd q_desired = initial_q;
            // q_desired << 0,0,0,0,0,0,0;
            // Jv = robot->Jv(link_name, pos_in_link);
            // Lambda = robot->taskInertiaMatrix(Jv);
            // N = robot->nullspaceMatrix(Jv);
            
            // VectorXd F = Lambda * (xdotdot_desired - kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * (robot->linearVelocity(link_name, pos_in_link) - xdot_desired));
            // control_torques.setZero();
            // control_torques = Jv.transpose() * F + N.transpose() * (-kpj * (robot->q() - q_desired) - kvj * robot->dq()) + robot->jointGravityVector();

        
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {

            
            ////// part d

            Vector3d ee_pos = robot->position(link_name, pos_in_link);
            VectorXd robot_q_pos  = robot->q();
            file_2d << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(3)  << "\t" << robot_q_pos(5)   << "\n"; 

            double kp = 100.0;      
            double kv = 20.0;            
            double kdamp = 14;        
            Vector3d pos_desired = Vector3d(-0.1, 0.15, 0.2); 

            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            N = robot->nullspaceMatrix(Jv);

            VectorXd torque_damp = VectorXd::Zero(dof);
            torque_damp = -kdamp * robot->dq();

            VectorXd F = Lambda * (- kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            control_torques.setZero();
            control_torques = Jv.transpose() * F + N.transpose() * torque_damp + robot->jointGravityVector();

            //// part e

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_2e << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(3)  << "\t" << robot_q_pos(5)   << "\n"; 

            // double kp = 100.0;      
            // double kv = 20.0;            
            // double kdamp = 14;     
            // double kmid = 25;       
            // Vector3d pos_desired = Vector3d(-0.1, 0.15, 0.2); 

            // Jv = robot->Jv(link_name, pos_in_link);
            // Lambda = robot->taskInertiaMatrix(Jv);
            // N = robot->nullspaceMatrix(Jv);

            // VectorXd torque_damp = VectorXd::Zero(dof);
            // torque_damp = -kdamp * robot->dq();

            // VectorXd q_lower = VectorXd::Zero(dof);;
            // q_lower << -165 * M_PI/180, -100 * M_PI/180, -165 * M_PI/180, -170 * M_PI/180, -165 * M_PI/180, 0, -165 * M_PI/180;
            // VectorXd q_upper = VectorXd::Zero(dof);;
            // q_upper << 165 * M_PI/180, 100 * M_PI/180, 165 * M_PI/180, -30 * M_PI/180, 165 * M_PI/180, 210 * M_PI/180, 165 * M_PI/180;

            // VectorXd torque_mid = VectorXd::Zero(dof);
            // torque_mid = kmid * (q_upper + q_lower - 2 * robot->q());

            // VectorXd F = Lambda * (- kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            // control_torques.setZero();
            // control_torques = Jv.transpose() * F + N.transpose() * torque_mid + N.transpose() * torque_damp + robot->jointGravityVector();

            /// part f

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_2f << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(3)  << "\t" << robot_q_pos(5)   << "\n"; 

            // double kp = 100.0;      
            // double kv = 20.0;            
            // double kdamp = 14;     
            // double kmid = 25;       
            // Vector3d pos_desired = Vector3d(-0.65, -0.45, 0.7); 

            // Jv = robot->Jv(link_name, pos_in_link);
            // Lambda = robot->taskInertiaMatrix(Jv);
            // N = robot->nullspaceMatrix(Jv);

            // VectorXd torque_damp = VectorXd::Zero(dof);
            // torque_damp = -kdamp * robot->dq();

            // VectorXd q_lower = VectorXd::Zero(dof);;
            // q_lower << -165 * M_PI/180, -100 * M_PI/180, -165 * M_PI/180, -170 * M_PI/180, -165 * M_PI/180, 0, -165 * M_PI/180;
            // VectorXd q_upper = VectorXd::Zero(dof);;
            // q_upper << 165 * M_PI/180, 100 * M_PI/180, 165 * M_PI/180, -30 * M_PI/180, 165 * M_PI/180, 210 * M_PI/180, 165 * M_PI/180;

            // VectorXd torque_mid = VectorXd::Zero(dof);
            // torque_mid = kmid * (q_upper + q_lower - 2 * robot->q());

            // VectorXd F = Lambda * (- kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            // control_torques.setZero();
            // control_torques = Jv.transpose() * F + N.transpose() * torque_mid + N.transpose() * torque_damp + robot->jointGravityVector();


            /// part g
            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // VectorXd robot_q_pos  = robot->q();
            // file_2g << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << robot_q_pos(3)  << "\t" << robot_q_pos(5)   << "\n"; 

            // double kp = 100.0;      
            // double kv = 20.0;            
            // double kdamp = 14;     
            // double kmid = 25;       
            // Vector3d pos_desired = Vector3d(-0.65, -0.45, 0.7); 

            // Jv = robot->Jv(link_name, pos_in_link);
            // Lambda = robot->taskInertiaMatrix(Jv);
            // N = robot->nullspaceMatrix(Jv);

            // VectorXd torque_damp = VectorXd::Zero(dof);
            // torque_damp = -kdamp * robot->dq();

            // VectorXd q_lower = VectorXd::Zero(dof);;
            // q_lower << -165 * M_PI/180, -100 * M_PI/180, -165 * M_PI/180, -170 * M_PI/180, -165 * M_PI/180, 0, -165 * M_PI/180;
            // VectorXd q_upper = VectorXd::Zero(dof);;
            // q_upper << 165 * M_PI/180, 100 * M_PI/180, 165 * M_PI/180, -30 * M_PI/180, 165 * M_PI/180, 210 * M_PI/180, 165 * M_PI/180;

            // VectorXd torque_mid = VectorXd::Zero(dof);
            // torque_mid = kmid * (q_upper + q_lower - 2 * robot->q());

            // VectorXd F = Lambda * (- kp * (robot->position(link_name, pos_in_link) - pos_desired) - kv * robot->linearVelocity(link_name, pos_in_link));
            // control_torques.setZero();
            // control_torques = Jv.transpose() * F + torque_mid + N.transpose() * torque_damp + robot->jointGravityVector();
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {

            control_torques.setZero();

            double kp = 70.0;      
            double kv = 40.0;            
            double kvj = 20;   

            Vector3d phi = Vector3d::Zero(3);
            Matrix3d R_d;
            R_d << cos(M_PI /3), 0, sin(M_PI / 3),
                   0, 1, 0,
                   -sin(M_PI / 3), 0, cos(M_PI / 3);
            Matrix3d R = robot->rotation(link_name);

            for (int i=0; i<3; i++){
                Vector3d Ri = R(all, i);
                Vector3d Rdi = R_d(all, i);
                Vector3d crossp = Ri.cross(Rdi);
                cout << crossp << endl;
                phi += crossp;
            }
            phi = -0.5 * phi;

             
            MatrixXd J_0 = robot->J(link_name, pos_in_link);
            MatrixXd Lambda_0 = MatrixXd::Zero(6,6);
            Lambda_0 = robot->taskInertiaMatrix(J_0);

            Vector3d x_d = Vector3d(0.6, 0.3, 0.5);
            Vector3d ee_pos = robot->position(link_name, pos_in_link);
            Vector3d F_v = kp * (x_d - ee_pos) - kv * robot->linearVelocity(link_name, pos_in_link);
            Vector3d F_w = kp * (-phi) - kv * robot->angularVelocity(link_name);
            Vector6d F_vw;
            F_vw << F_v, F_w;

            VectorXd F = Lambda_0 * F_vw;

            N = robot->nullspaceMatrix(J_0);
            control_torques = J_0.transpose() * F - N.transpose() * kvj * robot->dq() + robot->jointGravityVector();

            file_3 << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << phi(0) << "\t" << phi(1) << "\t" << phi(2) << "\n"; 
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            //// Part a

            // control_torques.setZero();

            // Vector3d ee_pos = robot->position(link_name, pos_in_link);
            // Vector3d x_dot = robot->linearVelocity(link_name, pos_in_link);
            
            // // Plot x, x_dot
            // file_4a << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << x_dot(0) << "\t" << x_dot(1) << "\t" << x_dot(2) << "\n"; 

            // double kp = 200.0;      
            // double kv = 40.0;        // This is tuned to be critically damped for end effector xyz. 
            // double kpj = 50;       
            // double kvj = 14;        
        
            // Vector3d pos_desired = Vector3d(0.6, 0.3, 0.4); 
            // VectorXd q_desired = initial_q;
            // q_desired << 0,0,0,0,0,0,0;
            // Jv = robot->Jv(link_name, pos_in_link);
            // Lambda = robot->taskInertiaMatrix(Jv);
            
            // N = robot->nullspaceMatrix(Jv);
            
            // VectorXd F = Lambda * (kp * (pos_desired - robot->position(link_name, pos_in_link)) - kv * x_dot);
            // control_torques = Jv.transpose() * F + N.transpose() * robot->M() * (-kpj * (robot->q() - q_desired) - kvj * robot->dq()) + robot->jointGravityVector();

            //// Part b

            control_torques.setZero();

            Vector3d ee_pos = robot->position(link_name, pos_in_link);
            Vector3d x_dot = robot->linearVelocity(link_name, pos_in_link);

            // // Plot x, x_dot
            file_4b << time << "\t" << ee_pos(0)  << "\t" << ee_pos(1) << "\t" << ee_pos(2) << "\t" << x_dot(0) << "\t" << x_dot(1) << "\t" << x_dot(2) << "\n"; 

            Vector3d pos_desired = Vector3d(0.6, 0.3, 0.4); 
            double kp = 200.0;      
            double kv = 40.0;        // This is tuned to be critically damped for end effector xyz. 
            double kpj = 50;       
            double kvj = 14;     

            Vector3d x_d_dot = kp / kv * (pos_desired - ee_pos);
            // calculate v
            double v = x_d_dot.norm();

            v = 0.1 / v;
            if (v > 1) {
                v = 1;
            }

            VectorXd q_desired = initial_q;
            q_desired << 0,0,0,0,0,0,0;
            Jv = robot->Jv(link_name, pos_in_link);
            Lambda = robot->taskInertiaMatrix(Jv);
            N = robot->nullspaceMatrix(Jv);
            VectorXd F = Lambda * (- kv * (x_dot - v * x_d_dot));

            control_torques = Jv.transpose() * F + N.transpose() * robot->M() * (-kpj * (robot->q() - q_desired) - kvj * robot->dq()) + robot->jointGravityVector();

            cout << robot->linearVelocity(link_name, pos_in_link).norm() << endl;

            
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
