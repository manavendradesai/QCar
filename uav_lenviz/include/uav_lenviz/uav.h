// This header file makes declarations required for Bidirectional A* planning on parallel threads.

// The code is adapted from https://github.com/akshay-antony/BiDirectionalWeightedAstar/blob/main/src/main.cpp

#pragma once
#include <iostream>
#include <vector>
#include <utility>
#include <bits/stdc++.h>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <cmath>
#include <queue>
#include <limits>
#include <cfloat>
#include <chrono>
#include <thread>
#include <mutex>

namespace gp{

    // 1024 node map
    float height = 32;
    float width = 32;
    float w = 1;

    std::set<std::pair<float,float>> visited_fwd;
    std::set<std::pair<float,float>> visited_bwd;

    bool found = false;
    
    std::pair<float,float> meet_point;
    typedef std::priority_queue<std::vector<float>, std::vector<std::vector<float>>, std::greater<std::vector<float>>> pq;
}