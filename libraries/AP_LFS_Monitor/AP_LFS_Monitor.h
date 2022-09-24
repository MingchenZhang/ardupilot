/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <limits>

// #define LFS_DEBUG

class AP_LFS_Monitor
{
public:
    AP_LFS_Monitor();

    static AP_LFS_Monitor *get_singleton(void)
    {
        return _singleton;
    }
    static const struct AP_Param::GroupInfo var_info[];

    /* Do not allow copies */
    AP_LFS_Monitor(const AP_LFS_Monitor &other) = delete;

    AP_LFS_Monitor &operator=(const AP_LFS_Monitor&) = delete;

    // init threads and lookup for io uart.
    bool init();

    int32_t lfs_terrain_ms = 0.0;

private:
    typedef struct PACKED sample{
        bool valid:1;
        int32_t below:31; 
    } sample;
    static const int32_t SAMPLE_BELOW_MAX = (1 << 30) - 1;
    static const int32_t SAMPLE_BELOW_MIN = -(1 << 30);

    // Pointer to singleton
    static AP_LFS_Monitor* _singleton;

    AP_Int8 _enabled;  // false
    AP_Int16 _check_interval_meter;  // 30
    AP_Int16 _test_interval_ms;  // 1000
    AP_Int16 _min_test_dist_m;  // 100
    // altitude unit is not important, we just want to know how fast it is approaching 0
    unsigned int last_alt_list_size = 10;
    sample* last_alt_list = nullptr;
    
    // checking periodically
    void loop();
};
