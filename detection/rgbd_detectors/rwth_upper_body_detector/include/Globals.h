/* 
 * File:   Globals.h
 * Author: dennis
 *
 * Created on May 15, 2009, 10:02 AM
 */

#ifndef _GLOBALS_DENNIS_H
#define	_GLOBALS_DENNIS_H

#include <string>
using namespace std;

class Globals {
public:

    ////////////////////////////////////////
    // Distance Range Accepted Detections
    ////////////////////////////////////////
    static double distance_range_accepted_detections;

    ////////////////////////////////////////
    // ROI
    ////////////////////////////////////////
    static double inc_width_ratio;
    static double inc_height_ratio;
    static int region_size_threshold;

    ////////////////////////////////////////
    // Freespace Parameters
    ////////////////////////////////////////
    static double freespace_scaleZ;
    static double freespace_scaleX;
    static double freespace_minX;
    static double freespace_minZ;
    static double freespace_maxX;
    static double freespace_maxZ;
    static double freespace_threshold;
    static int freespace_max_depth_to_cons;

    ////////////////////////////////////////
    // Evaluation Parameters
    ////////////////////////////////////////
    static double evaluation_NMS_threshold;
    static double evaluation_NMS_threshold_LM;
    static double evaluation_NMS_threshold_Border;
    static double evaluation_inc_height_ratio;
    static int evaluation_stride;
    static double evaluation_scale_stride;
    static int evaluation_nr_scales;
    static int evaluation_inc_cropped_height;
    static double evaluation_greedy_NMS_overlap_threshold;
    static double evaluation_greedy_NMS_threshold;

    ////////////////////////////////////////
    // World scale
    ////////////////////////////////////////
    static  double  WORLD_SCALE;

    ////////////////////////////////////////
    // height and width for precomputed Segmentations
    ////////////////////////////////////////
    static  int dImHeight;
    static  int dImWidth;

    ////////////////////////////////////////
    // Number of Frames / offset
    ////////////////////////////////////////
    static  int numberFrames;
    static int nOffset;

    ////////////////////////////////////////
    // Size of Template
    ////////////////////////////////////////
    static int template_size;

    static double max_height;
    static double min_height;

};

#endif	/* _GLOBALS_DENNIS_H */
