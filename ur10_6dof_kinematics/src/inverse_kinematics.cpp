#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define MAX_WORKSPACE_RADIUS 1.5 //!< Robotun maksimum erisim yaricapi
#define D1 2.0                   //!< Taban yuksekligi
#define D6 0.1716059             //!< Uc efektor uzunlugu
#define L1 0.5723                //!< Link 1 uzunlugu
#define L2 0.612                 //!< Link 2 uzunlugu  


//!< Ara hedef noktalarini hesaplar
std::vector<std::vector<float>> Ara_Hedef_Noktalarini_Hesapla(const std::vector<float>& P0, const std::vector<float>& P1, float max_step) 
{
    std::vector<std::vector<float>> hedef_noktalar;
    float dx = P1[0] - P0[0];
    float dy = P1[1] - P0[1];
    float dz = P1[2] - P0[2];
    float mesafe = sqrt(dx * dx + dy * dy + dz * dz);

    //!< Ara hedeflerin sayisi
    int num_steps = std::ceil(mesafe / max_step); 
    for (int i = 0; i <= num_steps; i++) 
    {
        //!< 0 ile 1 arasinda ilerleme orani
        float t = static_cast<float>(i) / num_steps; 
        float x = P0[0] + t * dx;
        float y = P0[1] + t * dy;
        float z = P0[2] + t * dz;

        //!<  Calisma alani kontrolu
        float r_xy = sqrt(x * x + y * y);
        if (r_xy > MAX_WORKSPACE_RADIUS) 
        {
            ROS_WARN("Ara hedef calisma alani sinirini asiyor: x=%.2f, y=%.2f, z=%.2f", x, y, z);
            continue;
        }

        hedef_noktalar.push_back({x, y, z});
    }
    return hedef_noktalar;
}

//!< Theta1 hesaplama
float HesaplaTheta1(float wx, float wy) 
{
    return atan2(wy, wx);
}

//!< Theta3 hesaplama
float HesaplaTheta3(float r, float s) 
{
    //!< Hipotenus uzaklik
    float d = sqrt(r * r + s * s); 
    float cos_theta3 = (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2);

    //!<  Aralik kontrolu
    cos_theta3 = std::max(-1.0f, std::min(1.0f, cos_theta3)); 
    return acos(cos_theta3);
}

//!< Theta2 hesaplama
float HesaplaTheta2(float r, float s,float theta3) 
{
    //!<  Bilek merkezinin aci degeri
    float alpha = atan2(s, r); 
    float beta = atan2(L2 * sin(theta3), L1 + L2 * cos(theta3));
    return alpha - beta; // Radyan
}

//!< Ters kinematik hesaplama
void Ters_Kinematik_Hesabi(const std::vector<float>& hedef_pozisyon, std::vector<float>& eklem_acilari, bool& basarili) 
{
    basarili = true;

    //!< Hedef pozisyon
    float x = hedef_pozisyon[0];
    float y = hedef_pozisyon[1];
    float z = hedef_pozisyon[2];

    //!< Bilek merkezi hesaplama
    float wx = x;
    float wy = y;
    float wz = z - D6;

    //!< Taban yuksekligi kontrolü
    if (wz < D1) 
    {
        ROS_ERROR("Hedef pozisyon taban seviyesinin altinda.");
        basarili = false;
        return;
    }

    //!< Yatay ve dikey uzakliklar
    float r = sqrt(wx * wx + wy * wy); 
    float s = wz - D1;                 

    //!< Calisma alani kontrolu
    float d = sqrt(r * r + s * s);
    if (d > (L1 + L2)) 
    {
        ROS_ERROR("Hedef pozisyon robotun caslima alani disinda.");
        basarili = false;
        return;
    }

    //!< Theta1 hesapla
    float theta1 = HesaplaTheta1(wx, wy);

    //!< Theta3 hesapla
    float theta3 = HesaplaTheta3(r, s);

    //!< Theta2 hesapla
    float theta2 = HesaplaTheta2(r, s, theta3);

    //!< Soruda oryantasyon bagimsiz dendigi icin sabit değerler: Theta4, Theta5, Theta6
    float theta4 = 0.0;
    float theta5 = 0.0;
    float theta6 = 0.0;

    eklem_acilari = {theta1, theta2, theta3, theta4, theta5, theta6};
}

void Odom_Callback(const nav_msgs::Odometry::ConstPtr& msg) 
{
    //!< Quaternion al ve normalizasyon yap
    tf2::Quaternion quat(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    //!< Normalizasyon
    quat.normalize();

    //!< Quaternion'un gecerli olup olmadigini kontrol et
    if (std::isnan(quat.x()) || std::isnan(quat.y()) || std::isnan(quat.z()) || std::isnan(quat.w())) 
    {
        ROS_ERROR("Odometri verisinde gecersiz quaternion (NaN) tespit edildi.");
        return;
    }

    //!< Pozisyonu hesapla
    float x = msg->pose.pose.position.x;
    float y = msg->pose.pose.position.y;
    float z = msg->pose.pose.position.z;

    ROS_INFO("Uc efektor Pozisyonu: x=%.2f, y=%.2f, z=%.2f", x, y, z);
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "robot_linear_motion_node");
    ros::NodeHandle nh;

    if (argc != 7) 
    {
        ROS_ERROR("Lutfen 6 parametre girin: P0 (x0, y0, z0) ve P1 (x1, y1, z1)");
        return -1;
    }

    //!< P0 ve P1 noktalarini alalim
    std::vector<float> P0 = {std::stof(argv[1]), std::stof(argv[2]), std::stof(argv[3])};
    std::vector<float> P1 = {std::stof(argv[4]), std::stof(argv[5]), std::stof(argv[6])};

    //!< Ara hedef noktalarini hesaplayalim (10 cm mesafede)
    float maks_adim_uzunlugu = 0.1; 
    std::vector<std::vector<float>> hedef_noktalar = Ara_Hedef_Noktalarini_Hesapla(P0, P1, maks_adim_uzunlugu);

    ros::Subscriber odom_sub = nh.subscribe("/ur10_arm/odometri", 10, Odom_Callback);
    ros::Publisher joint_acilar_pub = nh.advertise<std_msgs::Float32MultiArray>("/ur10_arm/acilar", 10);
    std_msgs::Float32MultiArray joint_acilar_msg;
    ros::Rate loop_rate(10);

    for (const auto& hedef : hedef_noktalar) 
    {
        ROS_INFO("Hedef Nokta: x=%.2f, y=%.2f, z=%.2f", hedef[0], hedef[1], hedef[2]);
        
        std::vector<float> acilar;
        bool basarili = false;
        Ters_Kinematik_Hesabi(hedef, acilar, basarili);

        if (!basarili) 
        {
            ROS_WARN("Ters kinematik verilen kordinatlara gore basarili oldu. Mesaj yayinlanmiyor.");
            continue;
        }

        // Eklem açılarını yayınla
        joint_acilar_msg.data = acilar;
        joint_acilar_pub.publish(joint_acilar_msg);
        loop_rate.sleep();
    }

    ROS_INFO("P1 noktasina ulasildi!");
    return 0;
}
