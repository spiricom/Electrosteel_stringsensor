/*
 * pluck_detect.h
 *
 *  Created on: Aug 4, 2020
 *      Author: josnyder
 */

#ifndef PLUCK_DETECT_H_
#define PLUCK_DETECT_H_
#include "leaf.h"
	/* Angie Hugeback Pluck Detection Algorithm designed for Snyderphonics Electrobass and Electrosteel */
	typedef struct _tPluckDetectorInt
	{
		tMempool mempool;


		tRingBufferInt smoothed_array;
		tRingBufferInt super_smoothed_array;
		tRingBufferInt minmax_samples;

		int current_dir;
		int envelope_min;
		int envelope_max;
		int prior_smoothed;
		int prior_super_smoothed;
		int prior_super_smoothed_dir;
		int prior_dirs[3];
		int prior_changepoints_index[5];//    # empty vector of length 5
		int prior_changepoints_value[5];//        # empty vector of length 5
		int prior_detect_1_index;
		int prior_detect_1_value;
		int prior_detect_2_index;
		int prior_detect_2_value;
		int prior_detect_3_index;
		int prior_detect_3_value;
		uint midpoint_estimate;
		uint is_midpoint_calculated;
		int delay_since_last_detect;
		int dir_count;
		int ready_for_pluck;
		uint64_t Pindex;
		int totalNumChangepoints;

		uint smoothed;//Mean of the last [SMOOTHING_WINDOW] samples
		uint smoothedAccum;
		uint super_smoothed;//Mean of the last [SUPER_SMOOTHING_WINDOW] smoothed values
		uint super_smoothedAccum;

		int pluck_strength;

		int max_samples_still_same_pluck;
		int max_var_diff_width;
		int max_width_is_resonating;
		float max_ratio_value_diffs;
		int min_value_spread;
		int min_same_direction_steps;
		int smoothing_window;
		int super_smoothing_window;
		int minmax_window;
		uint min_recent_value;
		uint max_recent_value;
		uint minMaxIncrementsBetweenSamples;
		tRingBufferInt prior_midpoint_estimates;
		tRingBufferInt midpoint_samples;
		uint midpoint_accum;
		uint prior_midpoints_window;
		uint midpoint_estimate_window;
		uint samples_per_midpoint_estimate_check;
		uint midpoint_estimate_acceptance_threshold;

		float inv_smoothing_window;
		float inv_super_smoothing_window;
		float inv_minmax_window;
		float inv_midpoint_estimate_window;
		float inv_prior_midpoints_window;

	} _tPluckDetectorInt;

	typedef _tPluckDetectorInt* tPluckDetectorInt;

	void    tPluckDetectorInt_init          (tPluckDetectorInt* const);
	void    tPluckDetectorInt_initToPool    (tPluckDetectorInt* const, tMempool* const mp);
	void    tPluckDetectorInt_free          (tPluckDetectorInt* const);
	int   tPluckDetectorInt_calculateMinMax          (tPluckDetectorInt* const pd, int input);
	int   tPluckDetectorInt_tick          (tPluckDetectorInt* const pd, int input);


#endif /* PLUCK_DETECT_H_ */
