/*
 * pluck_detect.h
 *
 *  Created on: Aug 4, 2020
 *      Author: josnyder
 */

#ifndef PLUCK_DETECT_H_
#define PLUCK_DETECT_H_
//#include "AVLtree.h"

	/* Angie Hugeback Pluck Detection Algorithm designed for Snyderphonics Electrobass and Electrosteel */
	typedef struct _tPluckDetectorInt
	{
		tMempool mempool;


		tRingBufferInt smoothed_array;
		tRingBufferInt super_smoothed_array;
		tRingBufferInt last400_smoothed; // making this 512 for now to use ringbuffer object


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
		int envelope_window;

		uint myMin;
		uint myMax;

		//struct AVLNode *root = NULL;

		//char* AVL_array;

		//int AVL_arraySize;

	} _tPluckDetectorInt;

	typedef _tPluckDetector* tPluckDetectorInt;

	void    tPluckDetectorInt_init          (tPluckDetectorInt* const);
	void    tPluckDetectorInt_initToPool    (tPluckDetectorInt* const, tMempool* const);
	void    tPluckDetectorInt_free          (tPluckDetectorInt* const);

	uint   tPluckDetectorInt_tick          (tPluckDetectorInt* const, int input);


#endif /* PLUCK_DETECT_H_ */
