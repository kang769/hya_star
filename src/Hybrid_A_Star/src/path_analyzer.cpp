/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "hybrid_a_star/path_analyzer.h"
#include <fstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <ros/ros.h>

PathAnalyzer::PathAnalyzer(const std::shared_ptr<HybridAStar>& hybrid_a_star_ptr)
    : hybrid_a_star_ptr_(hybrid_a_star_ptr), csv_file_path_("/home/jjk/code_hub/hybrid_a_star_ws/src/Hybrid_A_Star/data/path_analysis.csv") {
}

PathAnalyzer::PathAnalyzer(const std::shared_ptr<HybridAStar>& hybrid_a_star_ptr, const std::string& csv_file_path)
    : hybrid_a_star_ptr_(hybrid_a_star_ptr), csv_file_path_(csv_file_path) {
}

void PathAnalyzer::SetHybridAStar(const std::shared_ptr<HybridAStar>& hybrid_a_star_ptr) {
    hybrid_a_star_ptr_ = hybrid_a_star_ptr;
}

void PathAnalyzer::SetCsvFilePath(const std::string& csv_file_path) {
    csv_file_path_ = csv_file_path;
}

void PathAnalyzer::AnalyzePaths(const VectorVec3d& original_path, const VectorVec3d& optimized_path) {
    if (original_path.empty() || optimized_path.empty()) {
        ROS_WARN("路径分析失败：原始路径或优化路径为空");
        return;
    }

    // 创建CSV文件
    std::ofstream csv_file(csv_file_path_);
    if (!csv_file.is_open()) {
        ROS_ERROR("无法创建CSV文件：%s", csv_file_path_.c_str());
        return;
    }

    // 写入CSV文件头
    csv_file << "path_type,point_index,x,y,theta,curvature,obstacle_distance,path_length,segment_length,steering_angle\n";

    // 分析原始路径
    AnalyzePath(csv_file, "original", original_path);

    // 分析优化路径
    AnalyzePath(csv_file, "optimized", optimized_path);

    csv_file.close();
    ROS_INFO("路径分析完成，CSV文件已保存至：%s", csv_file_path_.c_str());
}

void PathAnalyzer::AnalyzePath(std::ofstream& csv_file, const std::string& path_type, const VectorVec3d& path) {
    double path_length = 0.0;
    
    for (size_t i = 0; i < path.size(); ++i) {
        double curvature = 0.0;
        double obstacle_distance = CalculateObstacleDistance(path[i].x(), path[i].y());
        double segment_length = 0.0;
        double steering_angle = 0.0;

        // 计算曲率（至少需要三个点）
        if (i > 0 && i < path.size() - 1) {
            curvature = CalculateCurvature(path[i-1], path[i], path[i+1]);
        }

        // 计算路径段长度
        if (i > 0) {
            segment_length = std::sqrt(std::pow(path[i].x() - path[i-1].x(), 2) + 
                                       std::pow(path[i].y() - path[i-1].y(), 2));
            path_length += segment_length;
            
            // 估计转向角度（根据航向角变化）
            double angle_diff = path[i].z() - path[i-1].z();
            while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
            
            steering_angle = angle_diff;
        }

        // 写入CSV
        csv_file << path_type << ","
                 << i << ","
                 << std::fixed << std::setprecision(6) << path[i].x() << ","
                 << std::fixed << std::setprecision(6) << path[i].y() << ","
                 << std::fixed << std::setprecision(6) << path[i].z() << ","
                 << std::fixed << std::setprecision(6) << curvature << ","
                 << std::fixed << std::setprecision(6) << obstacle_distance << ","
                 << std::fixed << std::setprecision(6) << path_length << ","
                 << std::fixed << std::setprecision(6) << segment_length << ","
                 << std::fixed << std::setprecision(6) << steering_angle << "\n";
    }
}

double PathAnalyzer::CalculateCurvature(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3) {
    // 使用三点法计算曲率
    double x1 = p1.x(), y1 = p1.y();
    double x2 = p2.x(), y2 = p2.y();
    double x3 = p3.x(), y3 = p3.y();
    
    double a = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    double b = std::sqrt(std::pow(x2 - x3, 2) + std::pow(y2 - y3, 2));
    double c = std::sqrt(std::pow(x3 - x1, 2) + std::pow(y3 - y1, 2));
    
    // 计算半周长
    double s = (a + b + c) / 2.0;
    
    // 计算三角形面积（海伦公式）
    double area = std::sqrt(s * (s - a) * (s - b) * (s - c));
    
    // 计算曲率（K = 4 * 面积 / (a * b * c)）
    double curvature = 0.0;
    if (a > 0 && b > 0 && c > 0) {
        curvature = 4.0 * area / (a * b * c);
    }
    
    return curvature;
}

double PathAnalyzer::CalculateObstacleDistance(double x, double y) {
    double min_dist = std::numeric_limits<double>::max();
    
    const double search_radius = 10.0;
    const double step = hybrid_a_star_ptr_->GetMapGridResolution();
    
    for (double dx = -search_radius; dx <= search_radius; dx += step) {
        for (double dy = -search_radius; dy <= search_radius; dy += step) {
            double nx = x + dx;
            double ny = y + dy;
            
            Vec2i grid_index = hybrid_a_star_ptr_->Coordinate2MapGridIndex(Vec2d(nx, ny));
            if (hybrid_a_star_ptr_->HasObstacle(grid_index)) {
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
        }
    }
    
    // 如果在搜索范围内没有找到障碍物，返回搜索半径
    if (min_dist == std::numeric_limits<double>::max()) {
        return search_radius;
    }
    
    return min_dist;
} 