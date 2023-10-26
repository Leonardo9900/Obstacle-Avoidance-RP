#include "tf/tf.h"
#include <Eigen/Geometry>


/*
THIS FUNCTION IS USED TO CONVERT THE tf::StampedTransform POSE INTO Eigen::Isometry2f
*/

inline Eigen::Isometry2f convertPose2D(const tf::StampedTransform& t) {
    
    double yaw,pitch,roll;
    
    //EXTRACT THE BASIS MATRIX TO OBTAIN THE ORIENTATION
    tf::Matrix3x3 mat =  t.getBasis();
    
    //REPRESENT THE MATRIX USING ROLL, PITCH, YAW ANGLES AND FOCUS YOUR ATTENTION ON THE LAST ONE(ROTATION ON Z-axis)
    mat.getRPY(roll, pitch, yaw);
    Eigen::Isometry2f T;
    
    //INITIALIZE THE FINAL OUTPUT TO A IDENTITY TRANSFORMATION
    T.setIdentity();
    Eigen::Matrix2f R;
    
    //SET THE ROTATION PART
    R << std::cos(yaw), -std::sin(yaw),
        std::sin(yaw), std::cos(yaw);
    T.linear() = R;
    
    //SET THE TRANSLATION PART
    T.translation() = Eigen::Vector2f(t.getOrigin().x(), t.getOrigin().y());
    
    return T;
}
