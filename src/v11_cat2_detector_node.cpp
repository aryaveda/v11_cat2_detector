#include <v11_cat2_detector/v11_cat2_detector.h>

int main(int argc,char **argv){

    ros::init(argc,argv,"v11_cat2_detector_node");

    Cat2Detector cat2_detector;

    ros::Rate loop_rate(30);

    while(ros::ok()){

        cat2_detector.process();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
