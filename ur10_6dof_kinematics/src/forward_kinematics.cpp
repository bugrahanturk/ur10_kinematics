#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

void Matris_Carpim(const float A[4][4], const float B[4][4], float C[4][4]) 
{
    //!< 4x4 iki adet matrisin carpimini yapar
    for (int i = 0; i < 4; i++) 
    {
        for (int j = 0; j < 4; j++) 
        {
            C[i][j] = 0;
            for (int k = 0; k < 4; k++) 
            {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void Matris_Yazdir(const float A[3][3])
{
    //!< Matrisi yazdir
    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; ++j) 
        {
            std::cout << A[i][j] << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

void UR10_Ileri_Kinematik_Hesabi(const std::vector<float>& q, float T[4][4]) 
{
    //!< UR10 robotun ileri kinematik hesaplamalarini yapalim
    const float E_J7[4][4] = {{1, 0, 0, 0},
                              {0, 1, 0, 0.1716059},
                              {0, 0, 1, 0},
                              {0, 0, 0, 1}};
    
    const float J7_J6[4][4] = {{cos(q[5]),  0, sin(q[5]), 0},
                                {0,          1, 0        , 0},
                                {-sin(q[5]), 0, cos(q[5]), -0.1157},
                                {0,          0, 0        , 1}};
    
    const float J6_J5[4][4] = {{cos(q[4]), -sin(q[4]), 0, 0},
                                {sin(q[4]), cos(q[4]) , 0, 0.1149},
                                {0,         0,          1, 0},
                                {0,         0,          0, 1}};
    
    const float J5_J4[4][4] = {{cos(q[3]),  0, sin(q[3]), 0.5723},
                                {0,          1, 0        , 0},
                                {-sin(q[3]), 0, cos(q[3]), 0},
                                {0,          0, 0        , 1}};
    
    const float J4_J3[4][4] = {{1, 0, 0, 0},
                                {0, 1, 0, -0.1719},
                                {0, 0, 1, 0},
                                {0, 0, 0, 1}};
    
    const float J3_J2[4][4] = {{cos(q[2]),  0, sin(q[2]), 0.612},
                                {0,          1, 0        , 0},
                                {-sin(q[2]), 0, cos(q[2]), 0},
                                {0,          0, 0        , 1}};
    
    const float J2_J1[4][4] = {{cos(q[1]),  0, sin(q[1]), 0},
                                {0        ,  1, 0        , 0.220941},
                                {-sin(q[1]), 0, cos(q[1]), 0},
                                {0,          0, 0        , 1}};
    
    const float J1_J0[4][4] = {{cos(q[0]), -sin(q[0]),0, 0},
                                {sin(q[0]), cos(q[0]), 0, 0},
                                {0,         0,         1, 0.1273},
                                {0,         0,         0, 1}};
    
    const float J0[4][4] = {{1, 0, 0, 0},
                             {0, 1, 0, 0},
                             {0, 0, 1, 2},
                             {0, 0, 0, 1}};

    float T_temp[4][4];
    Matris_Carpim(J0, J1_J0, T_temp);
    Matris_Carpim(T_temp, J2_J1, T);
    Matris_Carpim(T, J3_J2, T_temp);
    Matris_Carpim(T_temp, J4_J3, T);
    Matris_Carpim(T, J5_J4, T_temp);
    Matris_Carpim(T_temp, J6_J5, T);
    Matris_Carpim(T, J7_J6, T_temp);
    Matris_Carpim(T_temp, E_J7, T);
}

void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    //!< Odometri mesajindan gelen quaternion'u al
    tf2::Quaternion quat(msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z,
                         msg->pose.pose.orientation.w);

    tf2::Matrix3x3 rotation_matrix(quat);

    std::cout << "[Odometri Pozisyon Bilgisi]" << std::endl;
    std::cout << "Konum: x=" << msg->pose.pose.position.x 
              << ", y=" << msg->pose.pose.position.y 
              << ", z=" << msg->pose.pose.position.z << std::endl;

    std::cout << "[Odometri Yonelim Matrisi]:" << std::endl; 

    //!< tf2::Matrix3x3 degerlerini float dizisine donustur
    float matris[3][3];
    for (int i = 0; i < 3; ++i) 
    {
        for (int j = 0; j < 3; ++j) 
        {
            matris[i][j] = rotation_matrix[i][j];
        }
    }

    Matris_Yazdir(matris);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "ur10_control_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("/ur10_arm/odometri", 10, Odom_Callback);
    ros::Publisher joint_pub = nh.advertise<std_msgs::Float32MultiArray>("/ur10_arm/acilar", 10);

    std_msgs::Float32MultiArray eklem_msg;
    ros::Rate loop_rate(1);

    //!< Ornek test eklem acilari
    const std::vector<float> acilar = {0.218669, -0.383317, 1.58017, 0, 0, 0};

    float T[4][4];

    while (ros::ok()) 
    {
        UR10_Ileri_Kinematik_Hesabi(acilar, T);

        //!< Teorik konum
        float teorik_pozisyon_x = T[0][3];
        float teorik_pozisyon_y = T[1][3];
        float teorik_pozisyon_z = T[2][3];

        //!< Teorik yönelim (3x3 rotasyon matrisini kullanarak manuel analiz)
        float teorik_oryantasyon[3][3] = 
        {
            {T[0][0], T[0][1], T[0][2]},
            {T[1][0], T[1][1], T[1][2]},
            {T[2][0], T[2][1], T[2][2]}
        };

        std::cout << "[Teorik Pozisyon]" << std::endl;
        std::cout << "x: " << teorik_pozisyon_x << ", y: " << teorik_pozisyon_y << ", z: " << teorik_pozisyon_z << std::endl;

        std::cout << "[Teorik Yonelim Matrisi]" << std::endl;

        Matris_Yazdir(teorik_oryantasyon);

        eklem_msg.data = acilar;
        joint_pub.publish(eklem_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }   
    
    return 0;
}
