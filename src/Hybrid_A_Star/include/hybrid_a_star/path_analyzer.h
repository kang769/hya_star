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

#ifndef HYBRID_A_STAR_PATH_ANALYZER_H
#define HYBRID_A_STAR_PATH_ANALYZER_H

#include "hybrid_a_star/hybrid_a_star.h"
#include <string>
#include <memory>
#include <fstream>

class PathAnalyzer {
public:
    PathAnalyzer(const std::shared_ptr<HybridAStar>& hybrid_a_star_ptr);
    PathAnalyzer(const std::shared_ptr<HybridAStar>& hybrid_a_star_ptr, const std::string& csv_file_path);
    
    void SetHybridAStar(const std::shared_ptr<HybridAStar>& hybrid_a_star_ptr);
    void SetCsvFilePath(const std::string& csv_file_path);
    
    // 分析原始路径和优化路径，并生成CSV文件
    void AnalyzePaths(const VectorVec3d& original_path, const VectorVec3d& optimized_path);
    
private:
    // 分析单个路径并写入CSV文件
    void AnalyzePath(std::ofstream& csv_file, const std::string& path_type, const VectorVec3d& path);
    
    // 计算曲率
    double CalculateCurvature(const Vec3d& p1, const Vec3d& p2, const Vec3d& p3);
    
    // 计算点到最近障碍物的距离
    double CalculateObstacleDistance(double x, double y);
    
    std::shared_ptr<HybridAStar> hybrid_a_star_ptr_;
    std::string csv_file_path_;
};

#endif // HYBRID_A_STAR_PATH_ANALYZER_H 