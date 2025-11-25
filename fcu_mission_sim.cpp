#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <geometry_msgs/InertiaStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "quadrotor_msgs/PositionCommand.h"
#include "../mavlink/common/mavlink.h"
#include <yaml-cpp/yaml.h>

// -------------------------- 全局配置 --------------------------
static const float SIM_HEIGHT = 1.0f;       // NNU仿真高度（1m）
static const bool SIMULATE_MODE = true;     // 仿真模式（无需无人机）
static const float SIM_SPEED = 0.02f;       // 仿真轨迹移动速度
static const float REACH_THRESHOLD = 0.1f;  // 目标点到达判定阈值
static const std::string FRAME_ID = "map";  // 全局坐标系（与RViz一致）

// -------------------------- 数据结构 --------------------------
struct Pt { 
    float x; 
    float y; 
    float z; 
    Pt(float x_=0, float y_=0, float z_=0) : x(x_), y(y_), z(z_) {}
};

// -------------------------- 全局变量 --------------------------
// 轨迹与路径消息
static nav_msgs::Path path_global_001, path_global_002, path_global_003;  // 3架机实际轨迹
static nav_msgs::Path path_target_001, path_target_002, path_target_003;  // 3架机目标轨迹
static ros::Publisher pub_odom_001, pub_odom_002, pub_odom_003;          // 模拟里程计发布
static ros::Publisher pub_path_global_001, pub_path_global_002, pub_path_global_003;
static ros::Publisher pub_path_target_001, pub_path_target_002, pub_path_target_003;
static ros::Publisher pub_mission_001, pub_mission_002, pub_mission_003;  // Mission指令发布

// 无人机状态（仿真用）
static Pt uav1_pos(SIM_HEIGHT, 0, SIM_HEIGHT), uav1_goal;  // UAV1：画第一个N
static Pt uav2_pos(SIM_HEIGHT, 0, SIM_HEIGHT), uav2_goal;  // UAV2：画第二个N（右移2m）
static Pt uav3_pos(SIM_HEIGHT, 0, SIM_HEIGHT), uav3_goal;  // UAV3：画U（右移4m）
static size_t traj_idx = 0;                                 // 轨迹点索引
static std::vector<Pt> traj1, traj2, traj3;                 // 3架机NNU轨迹缓存

// -------------------------- 辅助函数：生成N/U字母轨迹 --------------------------
// 生成垂直N字（x：上下方向，y：左右方向）
// 生成垂直 N 字（x：上下方向，y：左右方向）
// 生成垂直 N 字（左下角为起点，x:上下，y:左右）
static std::vector<Pt> gen_N(float height, float width, int samples) {
    std::vector<Pt> out;

    // 1) 左竖线：左下 (0, -width/2) → 左上 (height, -width/2)
    for (int i = 0; i < samples; i++) {
        float t = (float)i / (samples - 1);
        out.emplace_back(t * height, -width * 0.5f, 0.0f);
    }

    // 2) 斜线：左上 (height, -width/2) → 右下 (0, +width/2)
    for (int i = 0; i < samples; i++) {
        float t = (float)i / (samples - 1);
        float x = height * (1 - t);      // height → 0
        float y = -width * 0.5f + t * width;  // -width/2 → +width/2
        out.emplace_back(x, y, 0.0f);
    }

    // 3) 右竖线：右下 (0, +width/2) → 右上 (height, +width/2)
    for (int i = 0; i < samples; i++) {
        float t = (float)i / (samples - 1);
        out.emplace_back(t * height, +width * 0.5f, 0.0f);
    }

    return out;
}


// 生成垂直 U 字（左下角为起点，x:上下，y:左右）
static std::vector<Pt> gen_U(float height, float width, int samples_side, int samples_bottom) {
    std::vector<Pt> out;

    // 左竖线（下→上）
   for (int i = 0; i < samples_side; i++) {
    float t = -(float)(samples_side-1-i) / (samples_side - 1);
    out.emplace_back(t * height, 0.0f, 0.0f);
}

    // 底部半圆（左→右）
    for (int i = 0; i < samples_bottom; i++) {
        float t = (float)i / (samples_bottom - 1);
        float angle = M_PI * t; // 0 → π

        float cx = 0.0f;        // 圆心 x 在底部起点
        float cy = width * 0.5f; // 圆心 y 居中

        float rx = width * 0.5f; // 半径 y 方向
        float ry = height * 0.2f; // 半圆深度，控制圆弧的弯曲度

        out.emplace_back(
            ry * sinf(angle),       // x: 0 → 弧形向上
            cy - rx * cosf(angle),  // y: 左→右
            0.0f
        );
    }

    // 右竖线（下→上）
    for (int i = 0; i < samples_side; i++) {
        float t = -(float)i / (samples_side - 1);
        out.emplace_back(t * height, width, 0.0f);
    }

    return out;
}


// -------------------------- 辅助函数：轨迹初始化 --------------------------
static void init_trajectory() {
    // N/U字参数（优化后比例，确保清晰）
    const float letter_height = 2.5f;  // 字母高度（上下方向）
    const float letter_width = 1.0f;   // 字母宽度（左右方向）
    const int stroke_samples = 60;     // 单笔画采样点（平滑度）
    const int bottom_samples = 80;     // U字底部弧线采样点

    // 生成基础轨迹（本地坐标系）
    traj1 = gen_N(letter_height, letter_width, stroke_samples);  // UAV1：N
    traj2 = gen_N(letter_height, letter_width, stroke_samples);  // UAV2：N
    traj3 = gen_U(letter_height, letter_width, stroke_samples, bottom_samples);  // UAV3：U

    // 轨迹 densify（提升平滑度：每段插值6个点）
    auto densify = [](std::vector<Pt>& in, int factor) {
        std::vector<Pt> out;
        for (size_t i = 0; i < in.size() - 1; i++) {
            Pt a = in[i], b = in[i + 1];
            for (int k = 0; k < factor; k++) {
                float t = (float)k / factor;
                out.emplace_back(
                    a.x + (b.x - a.x) * t,
                    a.y + (b.y - a.y) * t,
                    a.z + (b.z - a.z) * t
                );
            }
        }
        out.push_back(in.back());
        return out;
    };
    traj1 = densify(traj1, 6);
    traj2 = densify(traj2, 6);
    traj3 = densify(traj3, 6);
    float y_off1 = -1.0f; // UAV1 left letter
                  float y_off2 =  0.0f; // UAV2 middle letter
                  float y_off3 =  1.0f; // UAV3 right letter
    // 初始化第一个目标点
    uav1_goal = traj1[0];
    uav2_goal = traj2[0];
    uav3_goal = traj3[0];

    ROS_INFO("NNU轨迹初始化完成！轨迹点数量：UAV1=%zu, UAV2=%zu, UAV3=%zu", 
             traj1.size(), traj2.size(), traj3.size());
}

// -------------------------- 辅助函数：模拟无人机移动 --------------------------
static void update_uav_pos() {
    // UAV1：向目标点移动
    if (fabs(uav1_pos.x - uav1_goal.x) > REACH_THRESHOLD || 
        fabs(uav1_pos.y - uav1_goal.y) > REACH_THRESHOLD) {
        float dx = uav1_goal.x - uav1_pos.x;
        float dy = uav1_goal.y - uav1_pos.y;
        float dist = sqrtf(dx*dx + dy*dy);
        uav1_pos.x += dx / dist * SIM_SPEED;
        uav1_pos.y += dy / dist * SIM_SPEED;
    } else if (traj_idx < traj1.size() - 1) {
        // 切换到下一个目标点
        traj_idx++;
        uav1_goal = traj1[traj_idx];
        uav2_goal = traj2[traj_idx];
        uav3_goal = traj3[traj_idx];
    }

    // UAV2/3 同理（与UAV1同步轨迹索引）
    if (fabs(uav2_pos.x - uav2_goal.x) > REACH_THRESHOLD || 
        fabs(uav2_pos.y - uav2_goal.y) > REACH_THRESHOLD) {
        float dx = uav2_goal.x - uav2_pos.x;
        float dy = uav2_goal.y - uav2_pos.y;
        float dist = sqrtf(dx*dx + dy*dy);
        uav2_pos.x += dx / dist * SIM_SPEED;
        uav2_pos.y += dy / dist * SIM_SPEED;
    }
    if (fabs(uav3_pos.x - uav3_goal.x) > REACH_THRESHOLD || 
        fabs(uav3_pos.y - uav3_goal.y) > REACH_THRESHOLD) {
        float dx = uav3_goal.x - uav3_pos.x;
        float dy = uav3_goal.y - uav3_pos.y;
        float dist = sqrtf(dx*dx + dy*dy);
        uav3_pos.x += dx / dist * SIM_SPEED;
        uav3_pos.y += dy / dist * SIM_SPEED;
    }
}

// -------------------------- 辅助函数：发布模拟数据 --------------------------
static void publish_sim_data(const ros::Time& now) {
    // 1. 发布模拟里程计（UAV1/2/3）
    auto publish_odom = [&](ros::Publisher& pub, const Pt& pos, int uav_id) {
        nav_msgs::Odometry odom;
        odom.header.frame_id = FRAME_ID;
        odom.header.stamp = now;
        // 位置：Y轴取反（适配FRU坐标系，与fcu_bridge一致）
        odom.pose.pose.position.x = pos.x;
        odom.pose.pose.position.y = -pos.y;  // 关键：Y轴取反（FLU→FRU）
        odom.pose.pose.position.z = pos.z;
        // 姿态：默认水平（w=1，无旋转）
        odom.pose.pose.orientation.w = 1.0f;
        odom.pose.pose.orientation.x = 0.0f;
        odom.pose.pose.orientation.y = 0.0f;
        odom.pose.pose.orientation.z = 0.0f;
        pub.publish(odom);

        // 2. 发布Path（实际轨迹）
        nav_msgs::Path& path = (uav_id == 1) ? path_global_001 : (uav_id == 2) ? path_global_002 : path_global_003;
        ros::Publisher& path_pub = (uav_id == 1) ? pub_path_global_001 : (uav_id == 2) ? pub_path_global_002 : pub_path_global_003;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;
        path.header = odom.header;
        path.poses.push_

// -------------------------- 主函数 --------------------------
int main(int argc, char **argv) {
    ros::init(argc, argv, "fcu_mission_sim");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);  // 100Hz发布频率

    // -------------------------- 初始化发布者 --------------------------
    // 模拟里程计发布
    pub_odom_001 = nh.advertise<nav_msgs::Odometry>("odom_global_001", 10);
    pub_odom_002 = nh.advertise<nav_msgs::Odometry>("odom_global_002", 10);
    pub_odom_003 = nh.advertise<nav_msgs::Odometry>("odom_global_003", 10);
    // 实际轨迹Path发布
    pub_path_global_001 = nh.advertise<nav_msgs::Path>("path_global_001", 10);
    pub_path_global_002 = nh.advertise<nav_msgs::Path>("path_global_002", 10);
    pub_path_global_003 = nh.advertise<nav_msgs::Path>("path_global_003", 10);
    // 目标轨迹Path发布
    pub_path_target_001 = nh.advertise<nav_msgs::Path>("path_target_001", 10);
    pub_path_target_002 = nh.advertise<nav_msgs::Path>("path_target_002", 10);
    pub_path_target_003 = nh.advertise<nav_msgs::Path>("path_target_003", 10);
    // Mission指令发布（兼容fcu_bridge）
    pub_mission_001 = nh.advertise<std_msgs::Float32MultiArray>("mission_001", 10);
    pub_mission_002 = nh.advertise<std_msgs::Float32MultiArray>("mission_002", 10);
    pub_mission_003 = nh.advertise<std_msgs::Float32MultiArray>("mission_003", 10);

    // -------------------------- 初始化NNU轨迹 --------------------------
    init_trajectory();
    ROS_INFO("NNU仿真启动！在RViz中订阅以下话题：");
    ROS_INFO("  - 实际轨迹：path_global_001 (N)、path_global_002 (N)、path_global_003 (U)");
    ROS_INFO("  - 目标轨迹：path_target_001、path_target_002、path_target_003");

    // -------------------------- 仿真循环 --------------------------
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        
        // 1. 更新无人机仿真位置
        update_uav_pos();
        
        // 2. 发布所有仿真数据（里程计、Path、Mission）
        publish_sim_data(now);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}back(pose_stamped);
        path_pub.publish(path);

        // 3. 发布Mission指令（目标点）
        std_msgs::Float32MultiArray mission;
        mission.layout.dim.push_back(std_msgs::MultiArrayDimension());
        mission.layout.dim[0].label = "mission_00" + std::to_string(uav_id);
        mission.layout.dim[0].size = 11;
        mission.data.resize(11);
        mission.data[0] = 0.0f;          // yaw（水平）
        mission.data[1] = 0.0f;          // yaw_rate
        mission.data[2] = (uav_id == 1) ? uav1_goal.x : (uav_id == 2) ? uav2_goal.x : uav3_goal.x;  // 目标x
        mission.data[3] = -((uav_id == 1) ? uav1_goal.y : (uav_id == 2) ? uav2_goal.y : uav3_goal.y);  // 目标y（取反）
        mission.data[4] = SIM_HEIGHT;    // 目标z（固定1m）
        mission.data[5] = 0.0f;          // vx
        mission.data[6] = 0.0f;          // vy
        mission.data[7] = 0.0f;          // vz
        mission.data[8] = 0.0f;          // ax
        mission.data[9] = 0.0f;          // ay
        mission.data[10] = 0.0f;         // az
        (uav_id == 1) ? pub_mission_001.publish(mission) : (uav_id == 2) ? pub_mission_002.publish(mission) : pub_mission_003.publish(mission);

        // 4. 发布目标轨迹Path
        nav_msgs::Path& target_path = (uav_id == 1) ? path_target_001 : (uav_id == 2) ? path_target_002 : path_target_003;
        ros::Publisher& target_pub = (uav_id == 1) ? pub_path_target_001 : (uav_id == 2) ? pub_path_target_002 : pub_path_target_003;
        geometry_msgs::PoseStamped target_pose;
        target_pose.header = odom.header;
        target_pose.pose.position.x = mission.data[2];
        target_pose.pose.position.y = mission.data[3];  // 已取反
        target_pose.pose.position.z = mission.data[4];
        target_pose.pose.orientation.w = 1.0f;
        target_path.header = odom.header;
        target_path.poses.push_back(target_pose);
        target_pub.publish(target_path);
    };

    // 发布3架机数据
    publish_odom(pub_odom_001, uav1_pos, 1);
    publish_odom(pub_odom_002, uav2_pos, 2);
    publish_odom(pub_odom_003, uav3_pos, 3);

    // 发布TF变换（map→uwb/world，确保RViz坐标系匹配）
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, now, FRAME_ID, "uwb"));
    br.sendTransform(tf::StampedTransform(transform, now, FRAME_ID, "world"));
}

// -------------------------- 主函数 --------------------------
int main(int argc, char **argv) {
    ros::init(argc, argv, "fcu_mission_sim");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(100);  // 100Hz发布频率

    // -------------------------- 初始化发布者 --------------------------
    // 模拟里程计发布
    pub_odom_001 = nh.advertise<nav_msgs::Odometry>("odom_global_001", 10);
    pub_odom_002 = nh.advertise<nav_msgs::Odometry>("odom_global_002", 10);
    pub_odom_003 = nh.advertise<nav_msgs::Odometry>("odom_global_003", 10);
    // 实际轨迹Path发布
    pub_path_global_001 = nh.advertise<nav_msgs::Path>("path_global_001", 10);
    pub_path_global_002 = nh.advertise<nav_msgs::Path>("path_global_002", 10);
    pub_path_global_003 = nh.advertise<nav_msgs::Path>("path_global_003", 10);
    // 目标轨迹Path发布
    pub_path_target_001 = nh.advertise<nav_msgs::Path>("path_target_001", 10);
    pub_path_target_002 = nh.advertise<nav_msgs::Path>("path_target_002", 10);
    pub_path_target_003 = nh.advertise<nav_msgs::Path>("path_target_003", 10);
    // Mission指令发布（兼容fcu_bridge）
    pub_mission_001 = nh.advertise<std_msgs::Float32MultiArray>("mission_001", 10);
    pub_mission_002 = nh.advertise<std_msgs::Float32MultiArray>("mission_002", 10);
    pub_mission_003 = nh.advertise<std_msgs::Float32MultiArray>("mission_003", 10);

    // -------------------------- 初始化NNU轨迹 --------------------------
    init_trajectory();
    ROS_INFO("NNU仿真启动！在RViz中订阅以下话题：");
    ROS_INFO("  - 实际轨迹：path_global_001 (N)、path_global_002 (N)、path_global_003 (U)");
    ROS_INFO("  - 目标轨迹：path_target_001、path_target_002、path_target_003");

    // -------------------------- 仿真循环 --------------------------
    while (ros::ok()) {
        ros::Time now = ros::Time::now();
        
        // 1. 更新无人机仿真位置
        update_uav_pos();
        
        // 2. 发布所有仿真数据（里程计、Path、Mission）
        publish_sim_data(now);
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
