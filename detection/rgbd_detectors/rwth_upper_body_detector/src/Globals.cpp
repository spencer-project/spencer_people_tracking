#include "Globals.h"
#include <string>

////////////////////////////////////////
// Distance Range Accepted Detections
////////////////////////////////////////
double Globals::distance_range_accepted_detections;

////////////////////////////////////////
// ROI
////////////////////////////////////////
double Globals::inc_width_ratio;
double Globals::inc_height_ratio;
int Globals::region_size_threshold;

////////////////////////////////////////
// Freespace Parameters
////////////////////////////////////////
double Globals::freespace_scaleZ;
double Globals::freespace_scaleX;
double Globals::freespace_minX;
double Globals::freespace_minZ;
double Globals::freespace_maxX;
double Globals::freespace_maxZ;
double Globals::freespace_threshold;
int Globals::freespace_max_depth_to_cons;

////////////////////////////////////////
// Evaluation Parameters
////////////////////////////////////////
double Globals::evaluation_NMS_threshold;
double Globals::evaluation_NMS_threshold_LM;
double Globals::evaluation_NMS_threshold_Border;
double Globals::evaluation_inc_height_ratio;
int Globals::evaluation_stride;
double Globals::evaluation_scale_stride;
int Globals::evaluation_nr_scales;
int Globals::evaluation_inc_cropped_height;
double Globals::evaluation_greedy_NMS_overlap_threshold;
double Globals::evaluation_greedy_NMS_threshold;

////////////////////////////////////////
// World scale
////////////////////////////////////////
double  Globals::WORLD_SCALE;

////////////////////////////////////////
// height and width for precomputed Segmentations
////////////////////////////////////////
int Globals::dImHeight;
int Globals::dImWidth;

////////////////////////////////////////
// Number of Frames / offset
////////////////////////////////////////
int Globals::numberFrames;
int Globals::nOffset;

////////////////////////////////////////
// Size of Template
////////////////////////////////////////
int Globals::template_size = 30;

double Globals::max_height;
double Globals::min_height;


