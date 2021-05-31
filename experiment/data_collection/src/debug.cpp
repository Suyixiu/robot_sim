#include "debug.h"

bool next_pose = false;
bool save_data = false;

void data_collection_dynamic_callback(robot_sim::data_collectionConfig &config)
{
    next_pose = config.next_pose;
    save_data = config.save_data;
}
