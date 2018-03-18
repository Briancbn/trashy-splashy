#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cpg/Fin_servos.h>
#include <cpg/Servo.h>

#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include "CPG.h"

#define FREQUENCY 1.75
#define NUM_SERVO 3
float alpha = 11.68 * FREQUENCY;
float beta = 11.68 * FREQUENCY;
float mu = 5.84 * FREQUENCY;
float update_interval = 0.02;


int main(int argc, char **argv){
    ros::init(argc, argv, "cpg_test");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<cpg::Fin_servos>("cpg_test", 1000);
    ros::Rate loop_rate(1 / update_interval);

    CPG::CPG cpg_gen{NUM_SERVO, FREQUENCY, alpha, beta, mu, update_interval};
    std::vector<float> input_amplitude(NUM_SERVO, 0);
    std::vector<float> input_neutral_position(NUM_SERVO, 0);
    std::vector<float> input_phase_shift(NUM_SERVO, 0);
    for(int i = 0; i < NUM_SERVO; i++){
        input_phase_shift[i] = i * M_PI / 3; 
    }
    int counter = 0;
    srand(time(NULL));

    while(ros::ok()){
        counter++;
        if(counter % (int)(2 * 1 / update_interval) == 0){
            float new_amplitude = (rand() % 100) / 100.0;
            float new_neutral = (rand() % 100) / 100.0;
            float new_neutral_diff = (rand() % 100) / 100.0;
            for(int i = 0; i < NUM_SERVO; i++){
                input_amplitude[i] = new_amplitude;
                input_neutral_position[i] = new_neutral + (i - 1) * new_neutral_diff;
            }
        }

        std::vector<float> output(NUM_SERVO);
        output =  cpg_gen.generate_new_pose(input_amplitude, input_neutral_position, input_phase_shift);
        cpg::Fin_servos cpg;
        cpg.L1.amp_in = input_amplitude[0];
        cpg.L1.neu_in = input_neutral_position[0];
        cpg.L1.phi_in = input_phase_shift[0];
        cpg.L1.output = output[0];

        cpg.L2.amp_in = input_amplitude[1];
        cpg.L2.neu_in = input_neutral_position[1];
        cpg.L2.phi_in = input_phase_shift[1];
        cpg.L2.output = output[1];

        cpg.L3.amp_in = input_amplitude[2];
        cpg.L3.neu_in = input_neutral_position[2];
        cpg.L3.phi_in = input_phase_shift[2];
        cpg.L3.output = output[2];



        cpg.header.stamp = ros::Time::now();

        pub.publish(cpg);
        ros::spinOnce();

        loop_rate.sleep();


            
        

    }
}
