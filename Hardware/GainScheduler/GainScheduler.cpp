/**
 * LQR Gain Scheduler Implementation
 * 
 * Reads CSV from MATLAB solveLQR.m and performs bilinear interpolation.
 */

#include "GainScheduler.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// Internal scheduler state
struct GainScheduler {
    std::vector<float> vel_breakpoints;
    std::vector<float> h_breakpoints;
    std::vector<float> K_data;  // Flat array: [vel_idx][h_idx][k_element]
    int num_vel;
    int num_h;
    bool loaded;
    
    GainScheduler() : num_vel(0), num_h(0), loaded(false) {}
};

// ============================================================================
// Internal helpers
// ============================================================================

namespace {

inline float clamp(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

// Find lower index for interpolation
int findLowerIndex(const std::vector<float>& breakpoints, float value) {
    if (breakpoints.empty()) return 0;
    
    // Clamp to valid range
    if (value <= breakpoints.front()) return 0;
    if (value >= breakpoints.back()) return static_cast<int>(breakpoints.size()) - 2;
    
    // Binary search for lower bound
    auto it = std::lower_bound(breakpoints.begin(), breakpoints.end(), value);
    int idx = static_cast<int>(std::distance(breakpoints.begin(), it));
    
    // lower_bound gives first element >= value, we want the one before
    if (idx > 0 && (it == breakpoints.end() || *it > value)) {
        idx--;
    }
    
    // Ensure valid range for interpolation
    return std::min(idx, static_cast<int>(breakpoints.size()) - 2);
}

// Get K data offset for given indices
inline size_t getKOffset(int vel_idx, int h_idx, int num_h) {
    return static_cast<size_t>((vel_idx * num_h + h_idx) * LQR_K_SIZE);
}

// Parse CSV and load data
bool loadCSV(const char* csvPath, GainScheduler* sched) {
    std::ifstream file(csvPath);
    if (!file.is_open()) {
        return false;
    }
    
    std::string line;
    std::vector<float> all_vels;
    std::vector<float> all_hs;
    std::vector<std::vector<float>> all_rows;
    
    // Skip header line
    if (!std::getline(file, line)) {
        return false;
    }
    
    // Parse data rows
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        std::string cell;
        std::vector<float> row;
        
        while (std::getline(iss, cell, ',')) {
            try {
                row.push_back(std::stof(cell));
            } catch (...) {
                return false;
            }
        }
        
        // Expected: velocity, height, K11..K46 (2 + 24 = 26 values)
        if (row.size() != 2 + LQR_K_SIZE) {
            return false;
        }
        
        all_vels.push_back(row[0]);
        all_hs.push_back(row[1]);
        all_rows.push_back(std::move(row));
    }
    
    if (all_rows.empty()) {
        return false;
    }
    
    // Extract unique sorted breakpoints
    std::vector<float> vels = all_vels;
    std::sort(vels.begin(), vels.end());
    vels.erase(std::unique(vels.begin(), vels.end()), vels.end());
    
    std::vector<float> hs = all_hs;
    std::sort(hs.begin(), hs.end());
    hs.erase(std::unique(hs.begin(), hs.end()), hs.end());
    
    const int num_vel = static_cast<int>(vels.size());
    const int num_h = static_cast<int>(hs.size());
    
    // Verify we have complete grid
    if (num_vel * num_h != static_cast<int>(all_rows.size())) {
        return false;
    }
    
    // Allocate and fill K_data
    sched->vel_breakpoints = std::move(vels);
    sched->h_breakpoints = std::move(hs);
    sched->num_vel = num_vel;
    sched->num_h = num_h;
    sched->K_data.resize(static_cast<size_t>(num_vel) * num_h * LQR_K_SIZE);
    
    // Map each row to correct position in grid
    for (const auto& row : all_rows) {
        float vel = row[0];
        float h = row[1];
        
        // Find indices
        auto vel_it = std::lower_bound(sched->vel_breakpoints.begin(), 
                                        sched->vel_breakpoints.end(), vel);
        auto h_it = std::lower_bound(sched->h_breakpoints.begin(), 
                                      sched->h_breakpoints.end(), h);
        
        int vel_idx = static_cast<int>(std::distance(sched->vel_breakpoints.begin(), vel_it));
        int h_idx = static_cast<int>(std::distance(sched->h_breakpoints.begin(), h_it));
        
        // Clamp indices
        vel_idx = std::min(vel_idx, num_vel - 1);
        h_idx = std::min(h_idx, num_h - 1);
        
        // Copy K values (columns 2-25 in row)
        size_t offset = getKOffset(vel_idx, h_idx, num_h);
        for (int k = 0; k < LQR_K_SIZE; ++k) {
            sched->K_data[offset + k] = row[2 + k];
        }
    }
    
    sched->loaded = true;
    return true;
}

// Bilinear interpolation for K matrix
void interpolateGains(const GainScheduler* sched, float velocity, float height, LQRGains_t* gains) {
    // Clamp inputs to table range
    velocity = clamp(velocity, sched->vel_breakpoints.front(), sched->vel_breakpoints.back());
    height = clamp(height, sched->h_breakpoints.front(), sched->h_breakpoints.back());
    
    // Find grid cell
    int i0 = findLowerIndex(sched->vel_breakpoints, velocity);
    int j0 = findLowerIndex(sched->h_breakpoints, height);
    int i1 = std::min(i0 + 1, sched->num_vel - 1);
    int j1 = std::min(j0 + 1, sched->num_h - 1);
    
    // Compute interpolation weights
    float v0 = sched->vel_breakpoints[i0];
    float v1 = sched->vel_breakpoints[i1];
    float h0 = sched->h_breakpoints[j0];
    float h1 = sched->h_breakpoints[j1];
    
    float tv = (i1 > i0) ? (velocity - v0) / (v1 - v0) : 0.0f;
    float th = (j1 > j0) ? (height - h0) / (h1 - h0) : 0.0f;
    tv = clamp(tv, 0.0f, 1.0f);
    th = clamp(th, 0.0f, 1.0f);
    
    // Get offsets for 4 corners
    size_t off00 = getKOffset(i0, j0, sched->num_h);
    size_t off10 = getKOffset(i1, j0, sched->num_h);
    size_t off01 = getKOffset(i0, j1, sched->num_h);
    size_t off11 = getKOffset(i1, j1, sched->num_h);
    
    // Bilinear interpolation for each K element
    for (int k = 0; k < LQR_K_SIZE; ++k) {
        float k00 = sched->K_data[off00 + k];
        float k10 = sched->K_data[off10 + k];
        float k01 = sched->K_data[off01 + k];
        float k11 = sched->K_data[off11 + k];
        
        // Interpolate along velocity
        float k0 = lerp(k00, k10, tv);
        float k1 = lerp(k01, k11, tv);
        
        // Interpolate along height
        float val = lerp(k0, k1, th);
        
        // Store in 2D array (row-major)
        int row = k / LQR_K_COLS;
        int col = k % LQR_K_COLS;
        gains->K[row][col] = val;
    }
}

}  // anonymous namespace

// ============================================================================
// Public API
// ============================================================================

extern "C" {

GainSchedulerHandle_t GainScheduler_Create(const char* csvPath) {
    if (!csvPath) return nullptr;
    
    GainScheduler* sched = new GainScheduler();
    if (!loadCSV(csvPath, sched)) {
        delete sched;
        return nullptr;
    }
    
    return sched;
}

void GainScheduler_Destroy(GainSchedulerHandle_t* handle) {
    if (!handle || !*handle) return;
    
    delete *handle;
    *handle = nullptr;
}

bool GainScheduler_GetConfig(GainSchedulerHandle_t handle, GainSchedulerConfig_t* config) {
    if (!handle || !config || !handle->loaded) return false;
    
    config->vel_min = handle->vel_breakpoints.front();
    config->vel_max = handle->vel_breakpoints.back();
    config->h_min = handle->h_breakpoints.front();
    config->h_max = handle->h_breakpoints.back();
    config->num_vel = handle->num_vel;
    config->num_h = handle->num_h;
    
    // Compute step sizes (assuming uniform spacing)
    config->vel_step = (handle->num_vel > 1) ? 
        (config->vel_max - config->vel_min) / (handle->num_vel - 1) : 0.0f;
    config->h_step = (handle->num_h > 1) ? 
        (config->h_max - config->h_min) / (handle->num_h - 1) : 0.0f;
    
    return true;
}

bool GainScheduler_GetGains(GainSchedulerHandle_t handle, float velocity, float height, LQRGains_t* gains) {
    if (!handle || !gains || !handle->loaded) return false;
    
    interpolateGains(handle, velocity, height, gains);
    return true;
}

bool GainScheduler_ComputeControl(GainSchedulerHandle_t handle, 
                                   float velocity, float height,
                                   const float x[LQR_NUM_STATES], 
                                   float u[LQR_NUM_INPUTS]) {
    if (!handle || !x || !u || !handle->loaded) return false;
    
    LQRGains_t gains;
    interpolateGains(handle, velocity, height, &gains);
    
    // u = -K * x
    for (int i = 0; i < LQR_NUM_INPUTS; ++i) {
        float sum = 0.0f;
        for (int j = 0; j < LQR_NUM_STATES; ++j) {
            sum += gains.K[i][j] * x[j];
        }
        u[i] = -sum;
    }
    
    return true;
}

bool GainScheduler_GetGainsAtIndex(GainSchedulerHandle_t handle, 
                                    int vel_idx, int h_idx, 
                                    LQRGains_t* gains) {
    if (!handle || !gains || !handle->loaded) return false;
    if (vel_idx < 0 || vel_idx >= handle->num_vel) return false;
    if (h_idx < 0 || h_idx >= handle->num_h) return false;
    
    size_t offset = getKOffset(vel_idx, h_idx, handle->num_h);
    
    for (int k = 0; k < LQR_K_SIZE; ++k) {
        int row = k / LQR_K_COLS;
        int col = k % LQR_K_COLS;
        gains->K[row][col] = handle->K_data[offset + k];
    }
    
    return true;
}

}  // extern "C"
