#include <servo_pixhawk/servoPixhawk.h>

int main(int argc,char * argv[]){
    ros::init(argc, argv, "ibvs_node");
    ros::NodeHandle nh;

    ServoPixhawk ibvs(nh);
    ros::Rate rate(ibvs.acq_fps); 

    while (ros::ok()) {
        ros::spinOnce();
        ibvs.run();
        ros::spinOnce();  
        rate.sleep(); 
    }
}