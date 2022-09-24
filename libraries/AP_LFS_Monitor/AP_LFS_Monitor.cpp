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

// #include <sstream>

#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>
#include "AP_LFS_Monitor.h"

extern const AP_HAL::HAL &hal;

#ifdef LFS_DEBUG
# define debug(fmt, args...)	do { hal.console->printf("LFS monitor: " fmt "\n", ##args); } while (0)
#else
# define debug(fmt, args...)	do {} while(0)
#endif


AP_LFS_Monitor::AP_LFS_Monitor()
{
    if(_singleton != nullptr) {
        AP_HAL::panic("AP_LFS_Monitor reinitalized");
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

AP_LFS_Monitor *AP_LFS_Monitor::_singleton;

const AP_Param::GroupInfo AP_LFS_Monitor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Toggle line of sight check
    // @Description: Toggle line of sight check
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_LFS_Monitor, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: INTV_DIST
    // @DisplayName: Sample AGL every x meter
    // @Description: Sample AGL every x meter
    // @User: Standard
    // @Units: m
    // @Range: 1 1000
    AP_GROUPINFO("INTV_DIST",    2, AP_LFS_Monitor, _check_interval_meter, 30),

    // @Param: INTV_TIME
    // @DisplayName: Perform AGL check every x miliseconds
    // @Description: Perform AGL check every x miliseconds
    // @User: Standard
    // @Units: ms
    // @Range: 1 30000
    AP_GROUPINFO("INTV_TIME",  3, AP_LFS_Monitor, _test_interval_ms, 1000),

    // @Param: MIN_DIST
    // @DisplayName: minimum test distance in meter 
    // @Description: lower bound of home distance that would trigger line of sight checks
    // @User: Standard
    // @Units: m
    // @Range: 1 10000
    AP_GROUPINFO("MIN_DIST",  4, AP_LFS_Monitor, _min_test_dist_m, 100),

    AP_GROUPEND
};

// initialization start making a request settings to the vtx
bool AP_LFS_Monitor::init()
{
    last_alt_list = (sample*)malloc(sizeof(sample) * last_alt_list_size);
    for (unsigned int i=0; i<last_alt_list_size; i++) {
        last_alt_list[i].valid = false;
    }
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_LFS_Monitor::loop, void),
                                      "LFS_MONI",
                                      2048, AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        return false;
    }
    return true;
}

void AP_LFS_Monitor::loop()
{
    int last_check_ms = AP_HAL::millis();
    while (1) {
        hal.scheduler->delay(_test_interval_ms);
        if(!_enabled) {
            continue;
        }
#if AP_TERRAIN_AVAILABLE
        AP_Terrain *terrain = AP::terrain();
        if (!terrain || !terrain->enabled()) {
            debug("LFS monitor: terrain is not enabled. ");
            continue;
        }
#else 
        continue;
#endif
        AP_AHRS &ahrs = AP::ahrs();
        const Location &real_home_loc = ahrs.get_home();
        Location home_loc = real_home_loc;
        Location cur_loc;
        if (!hal.util->get_soft_armed()) {
            continue;
        }
        if (!ahrs.get_location(cur_loc) || !ahrs.home_is_set()) {
            continue;
        }
        home_loc.alt += 300;  // add 3 meters to help avoid close by terrain
        ftype distance = home_loc.get_distance(cur_loc);
        if (distance < _min_test_dist_m) {
            continue;
        }
        ftype bearing_at_home = home_loc.get_bearing(cur_loc);
        ftype height_diff;
        if (!cur_loc.get_alt_distance(home_loc, height_diff)) {
            debug("Unable to get altitude difference");
            continue;
        }
        ftype pitch_deg = degrees(atanF(height_diff / distance));
        // debug("dist=%.1f, bearing=%.1f, hgt=%.1f, pitch=%.1f", distance, degrees(bearing_at_home), height_diff, pitch_deg);

        // check if we need to upsize
        while (distance >= last_alt_list_size * _check_interval_meter) {
            auto new_size = last_alt_list_size << 1;
            sample* new_list = (sample*)realloc(last_alt_list, sizeof(sample) * new_size);
            if (!new_list) {
                debug("Unable to allocate more memory for %d", new_size);
                break;
            }
            for(unsigned int i=last_alt_list_size; i<new_size; i++) {
                new_list[i].valid = false;
            }
            last_alt_list_size = new_size;
            last_alt_list = new_list;
        }

        int min_ms_to_ground = std::numeric_limits<int>::max();
        int ms_since_last_check = AP_HAL::millis() - last_check_ms;
        last_check_ms = AP_HAL::millis();

        // std::stringstream ss;
        ftype range=0.0;
        ftype ha_ratio = 1/cosF(radians(pitch_deg));
        unsigned int i=0;
        for (; range<distance && i<last_alt_list_size; range += _check_interval_meter, i++) {
            Location test_loc = home_loc;
            test_loc.offset_bearing_and_pitch(degrees(bearing_at_home), pitch_deg, range*ha_ratio);
            int32_t terrain_alt_cm = 0;
            if (!test_loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, terrain_alt_cm)) {
                debug("Unable to get altitude above terrain");
                // TODO: find a way to reduce impact of thrashing
                last_alt_list[i].valid = false;
                continue;
            }

            // float alt_terr_cm;
            // if(terrain->height_amsl(test_loc, alt_terr_cm)) {
            //     ss << "[" << alt_terr_cm << "]";
            // }
            // int32_t ret_alt_cm;
            // if(test_loc.get_alt_cm(Location::AltFrame::ABSOLUTE, ret_alt_cm)) {
            //     ss << "[" << ret_alt_cm << "]";
            // }

            // convert to the store unit of altitude
            // TODO: test more detailed alt, make sure overflow is safe
            sample cur_sample;
            cur_sample.below = constrain_value(terrain_alt_cm, SAMPLE_BELOW_MIN, SAMPLE_BELOW_MAX);
            cur_sample.valid = true;
            sample prev_sample = last_alt_list[i];
            // ss << cur_sample.below;
            last_alt_list[i] = cur_sample;
            // if there is no valid prev altitide, skip the check
            if (!prev_sample.valid) {
                // ss << "(-), ";
                continue;
            }
            if (cur_sample.below <= 0) {
                min_ms_to_ground = 0;
                gcs().send_text(MAV_SEVERITY_ERROR, "ground reached");
                // ss << "(G), ";
                continue;
            }
            // calculate time to reach 0
            ftype ms_to_ground = std::numeric_limits<int>::max();
            if (cur_sample.below < prev_sample.below) {
                ms_to_ground = (ftype)cur_sample.below / ((ftype)(prev_sample.below - cur_sample.below) / ms_since_last_check);
                // ss << "(" << (int)ms_to_ground << "), ";
                if (ms_to_ground >= 0 && min_ms_to_ground > ms_to_ground) {
                    min_ms_to_ground = ms_to_ground;
                }
            } else {
                // ss << "(++), ";
            }
        }
        // ss << "[" << i << "]";
        for(;i<last_alt_list_size; i++) {
            last_alt_list[i].valid = false;
        }
        // debug("poi %s", ss.str().c_str());

        lfs_terrain_ms = min_ms_to_ground;

        if (min_ms_to_ground < 15000) {
            gcs().send_text(MAV_SEVERITY_ERROR, "%.2f sec to ground", ((ftype)min_ms_to_ground)/1000);
        }
    }
    gcs().send_text(MAV_SEVERITY_ERROR, "LFS monitor exited");
    while (1) {
        hal.scheduler->delay(10000);
    }
}
