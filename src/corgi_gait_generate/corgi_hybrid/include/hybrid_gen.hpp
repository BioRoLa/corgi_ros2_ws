#ifndef HYBRID_GEN_HPP
#define HYBRID_GEN_HPP

#include "Simple_fsm.hpp"
#include <chrono>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <cmath>
#include <random>
#include <iomanip>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ros/ros.h"
#include <corgi_msgs/MotorState.h>
#include <corgi_msgs/MotorStateStamped.h>
#include <corgi_msgs/MotorCmd.h>
#include <corgi_msgs/MotorCmdStamped.h>

#include "leg_model.hpp"
#include "hybrid_swing.hpp"

class Hybrid{   
    public:
        // Hybrid(ros::NodeHandle& nh);
        Hybrid(std::shared_ptr<GaitSelector> gait_selector_ptr);
        ~Hybrid()= default;

        void Initialize(int swing_index, int set_type);
        std::array<double, 2> find_pose(double height, float shift, float steplength, double slope);
        void Swing(double relative[4][2], std::array<double, 2> &target, std::array<double, 2> &variation, int swing_leg);
        void Swing_step(std::array<double, 2> target, std::array<double, 2> variation, int swing_leg, double duty_ratio);
 
        void Step();
        // void change_Height(double new_value);
        void change_Height(double new_value, int leg_index);
        void change_Step_length(double new_value);
        void change_Velocity(double new_value);
        void csv_title(std::ofstream &file);
        void save_to_csv(std::ofstream &file, int step);

        std::array<double, 2> swing_pose;
        std::array<double, 2> swing_variation;
        SwingType swing_type = SwingType::OPTIMIZE;
        TerrainType terrain_type = TerrainType::Plain;
        double terrain_slope = -14 * M_PI / 180.0; // 10 degree
        double swing_desired_height = 0.0; 
        
    private:
        std::shared_ptr<GaitSelector> gaitSelector;
        std::vector<SwingPoint> swing_traj[4];  // 每條腿都有自己的swing軌跡
        double clamp(double value, double min_val, double max_val);
        void update_nextFrame();
        void computeNextBody();
        
};


class TerrainInfo {
public:
    TerrainInfo(TerrainType type = TerrainType::Zigzag);

    // 通用
    void setTerrainType(TerrainType type);

    // Zigzag 參數
    //   startX, startY: 起點座標
    //   segmentLength: 每段水平長度 (m)
    //   slopeDeg:      每段坡度角度 (°)
    //   numSegments:   段數
    void setZigzagParameters(double startX, double startY,
                              double segmentLength,
                              double slopeDeg,
                              int    numSegments);

    // Plain 參數
    void setPlainHeight(double height);

    // Slope 參數 (global slope over x)
    void setSlopeAngle(double slopeDeg);

    // 查詢地形高度
    // 回傳該 x 座標下的地形高度 y
    double getTerrainHeight(double x) const;

    // 查詢目標高度: standHeight - terrainHeight(x)
    double getTargetHeight(double x, double standHeight) const;

private:
    void generateZigzagPoints();

    TerrainType type_;

    // --- Zigzag ---
    double             startX_;
    double             startY_;
    double             segmentLength_;
    double             slopeAngleRad_;
    int                numSegments_;
    std::vector<std::pair<double,double>> zigzagPoints_;

    // --- Plain ---
    double plainHeight_;

    // --- Slope ---
    // reuse slopeAngleRad_
};

#endif