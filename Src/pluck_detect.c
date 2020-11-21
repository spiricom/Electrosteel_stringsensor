#include "pluck_detect.h"


int LEAF_sign(int x) {
    return (x > 0) - (x < 0);
}

void    tPluckDetectorInt_init          (tPluckDetectorInt* const pd)
{
    tPluckDetectorInt_initToPool(pd, &leaf.mempool);
}
void    tPluckDetectorInt_initToPool    (tPluckDetectorInt* const pd, tMempool* const mp)
{
    _tMempool* m = *mp;
    _tPluckDetectorInt* p = *pd = (_tPluckDetectorInt*) mpool_alloc(sizeof(_tPluckDetectorInt), m);
    p->mempool = m;

	//INITIALIZE VARIABLES
	p->current_dir = 1;
	p->envelope_min = 0;//maybe could be 32 bit?
	p->envelope_max = 65535;//maybe could be 32 bit?
	p->prior_smoothed = 0;
	p->prior_super_smoothed = 0;
	p->prior_super_smoothed_dir = 1;
	for (int i = 0; i < 3; i++)
	{
		p->prior_dirs[i] = 1;
	}
	for (int i = 0; i < 5; i++)
	{
		p->prior_changepoints_index[i] = 0;//
		p->prior_changepoints_value[i] = 0;//
	}

	p->prior_detect_1_index = 0;
	p->prior_detect_1_value = 0;
	p->prior_detect_2_index = 0;
	p->prior_detect_2_value = 0;
	p->prior_detect_3_index = 0;
	p->prior_detect_3_value = 0;
	p->midpoint_estimate = 48552;
	p->is_midpoint_calculated = 0;
	p->delay_since_last_detect = 0;
	p->dir_count = 0;
	p->ready_for_pluck = 1;
	p->Pindex = 1;
	p->totalNumChangepoints = 0;

	p->smoothed = 0;//Mean of the last [SMOOTHING_WINDOW] samples
	p->smoothedAccum = 0;
	p->super_smoothed = 0;//Mean of the last [SUPER_SMOOTHING_WINDOW] smoothed values
	p->super_smoothedAccum = 0;

	p->pluck_strength = 0;

	p->smoothing_window = 8;
    p->super_smoothing_window = 128;
    p->minmax_window = 8;


    tRingBufferInt_initToPool(&p->smoothed_array, p->smoothing_window, mp);

	tRingBufferInt_initToPool(&p->super_smoothed_array, p->super_smoothing_window, mp);

	tRingBufferInt_initToPool(&p->minmax_samples, p->minmax_window, mp);

	p->min_recent_value = 0;
	p->max_recent_value = 0;
    p->max_samples_still_same_pluck = 2400; //400
    p->max_var_diff_width = 100;
    p->max_width_is_resonating = 2000;
    p->max_ratio_value_diffs = 100.0f;
    p->min_value_spread = 100; //500
    p->min_same_direction_steps = 10; //150
    p->minMaxIncrementsBetweenSamples = 8;
    p->midpoint_estimate_window = 4096;
    p->samples_per_midpoint_estimate_check = 4800;
    p->midpoint_estimate_acceptance_threshold = 10;
    p->midpoint_accum = 0;
    p->prior_midpoints_window = 4;
    tRingBufferInt_initToPool(&p->prior_midpoint_estimates, p->prior_midpoints_window, mp);
    tRingBufferInt_initToPool(&p->midpoint_samples, p->midpoint_estimate_window, mp);

	p->inv_smoothing_window = 1.0f / p->smoothing_window;
	p->inv_super_smoothing_window = 1.0f / p->super_smoothing_window;
	p->inv_minmax_window = 1.0f / p->minmax_window;
	p->inv_midpoint_estimate_window = 1.0f / p->midpoint_estimate_window;
	p->inv_prior_midpoints_window = 1.0f / p->prior_midpoints_window;

}
void    tPluckDetectorInt_free          (tPluckDetectorInt* const pd)
{
    _tPluckDetectorInt* p = *pd;

    tRingBufferInt_free(&p->smoothed_array);
	tRingBufferInt_free(&p->super_smoothed_array);
	tRingBufferInt_free(&p->minmax_samples);

    mpool_free((char*)p, p->mempool);
}


int   tPluckDetectorInt_tick          (tPluckDetectorInt* const pd, int input)
{
	 _tPluckDetectorInt* p = *pd;

	int pluckHappened = -1;



	//get the smoothed mean of that array
	int oldSmoothed = tRingBufferInt_getOldest(&p->smoothed_array);
	p->smoothedAccum -= oldSmoothed;
	p->smoothedAccum += input;
	p->smoothed = (uint)(((float)p->smoothedAccum) * p->inv_smoothing_window); // divide by 16
	//update smoothed for current sample
	tRingBufferInt_push(&p->smoothed_array, input);


	//get the smoothed mean of that array
	int oldSuperSmoothed = tRingBufferInt_getOldest(&p->super_smoothed_array);
	p->super_smoothedAccum -= oldSuperSmoothed;
	p->super_smoothedAccum += p->smoothed;
	p->super_smoothed = (uint)(((float)p->super_smoothedAccum) * p->inv_super_smoothing_window); // divide by 128

	//update super_smoothed for current sample
	tRingBufferInt_push(&p->super_smoothed_array, p->smoothed);

	//### Collect a new data point every MINMAX_INCREMENTS_BETWEEN_SAMPLES steps,
	//### and collect the current min and max values within this loop since they won't change until the next update
	if ((p->Pindex % p->minMaxIncrementsBetweenSamples) == 0) //maybe have this update as a separate function to avoid the branch every sample?
	{
			tRingBufferInt_push(&p->minmax_samples, p->smoothed);
			int tempMin1 = 65535;
			int tempMax1 = 0;

			for (int i = 0; i < p->minmax_window; i++)
			{
				int tempSample = tRingBufferInt_get(&p->minmax_samples, i);
				if (tempSample > tempMax1)
				{
					tempMax1 = tempSample;
				}
				if (tempSample < tempMin1)
				{
					tempMin1 = tempSample;
				}

			}
			p->max_recent_value = tempMax1;
			p->min_recent_value = tempMin1;
	}

	//only update the midpoint if not in a current pluck
	//if (p->ready_for_pluck == 1)
	{
		int oldMidpointSample = tRingBufferInt_getOldest(&p->midpoint_samples);
		p->midpoint_accum -= oldMidpointSample;
		p->midpoint_accum += p->smoothed;
		float midPointSmoothedAverage = ((float)p->midpoint_accum) * p->inv_midpoint_estimate_window; // divide by 100

		tRingBufferInt_push(&p->midpoint_samples, p->smoothed);


		 if (p->Pindex % p->samples_per_midpoint_estimate_check==0)
		 {

			 tRingBufferInt_push(&p->prior_midpoint_estimates, (int)midPointSmoothedAverage);

			 uint maxMidpointEstimate = 0;
			 uint minMidpointEstimate = 65535;
			 float midpointMeanAccum = 0.0f;
			 for (int i = 0; i < p->prior_midpoints_window; i++)
			 {
				 uint testVal = (uint)tRingBufferInt_get(&p->prior_midpoint_estimates, i);
				 if (testVal > maxMidpointEstimate)
				 {
					 maxMidpointEstimate = testVal;
				 }
				 else if (testVal < minMidpointEstimate)
				 {
					 minMidpointEstimate = testVal;
				 }
				 midpointMeanAccum += (float)testVal;
			 }

			 if ((maxMidpointEstimate - minMidpointEstimate) < p->midpoint_estimate_acceptance_threshold)
			 {
				 p->midpoint_estimate = (uint)(midpointMeanAccum * p->inv_prior_midpoints_window);
				 p->is_midpoint_calculated = 1;
			 }

		}
	}

	int outside_envelope = (( p->min_recent_value < p->envelope_min) || (p->max_recent_value > p->envelope_max ));   //# Logical: TRUE/FALSE

	//COLLECT THE DIRECTION OF MOVEMENT FOR SUPER-SMOOTHED SEQUENCE (FOR DETECTING IF READY FOR NEXT PLUCK)
	//Here we're basically counting how many times we've taken consecutive steps in the same direction
	//If we move in a different direction (up or down) then it resets
	//This is helpful for detecting that movement up or down right at the start of a pluck signal

	int super_smoothed_dir = LEAF_sign(p->super_smoothed - p->prior_super_smoothed);
	if (super_smoothed_dir != p->prior_super_smoothed_dir)
	{
		p->dir_count = 0;
	}
	else
	{
		p->dir_count = p->dir_count + 1;
	}

	//CHECK IF WE SEE THE SIGNS THAT WE ARE READY FOR NEXT PLUCK
	//We are ready for a new pluck if we've both:
	//(1) seen enough steps in same direction, and
	//(2) moved outside our current envelope

	if (p->ready_for_pluck==0)
	{
		if ((p->dir_count > p->min_same_direction_steps) && (outside_envelope == 1))
		{
			p->ready_for_pluck = 1;
		}
	}

	//COLLECT THE DIRECTION OF MOVEMENT FOR SMOOTHED SEQUENCE (FOR CHANGEPOINT DETECTION)
	int direction = LEAF_sign(p->smoothed-p->prior_smoothed);
	//prior_dirs = c(prior_dirs[-1],direction); //Update by removing first element and adding new value to end
	p->prior_dirs[0] = p->prior_dirs[1];
	p->prior_dirs[1] = p->prior_dirs[2];
	p->prior_dirs[2] = direction;


	//BEGIN TO CHECK IF WE ARE AT A "CHANGEPOINT"
	//A changepoint is when both of the following are true:
	// 		(1) Several consistent steps all up (or all down) in sequence, and then suddenly a change
	// 		(2) There is enough overall vertical movement in the recent samples

	int tempMin2 = 1;
	int tempMax2 = -1;
	for (int i = 0; i < 3; i++)
	{
		if (p->prior_dirs[i] < tempMin2)
		{
			tempMin2 = p->prior_dirs[i];
		}
		if (p->prior_dirs[i] > tempMax2)
		{
			tempMax2 = p->prior_dirs[i];
		}
	}
	if (((p->current_dir == 1) && (tempMax2 == -1)) || ((p->current_dir == -1) && (tempMin2 == 1)))
	{
		//UPDATE THE DIRECTION THAT WE'll BE COMPARING AGAINST NEXT TIME
		p->current_dir = -p->current_dir;

		if (p->totalNumChangepoints < 5)
		{
			p->totalNumChangepoints++;
		}

		//UPDATE VECTORS THAT STORE THE LAST 5 CHANGEPOINTS

		p->prior_changepoints_index[0] = p->prior_changepoints_index[1];
		p->prior_changepoints_index[1] = p->prior_changepoints_index[2];
		p->prior_changepoints_index[2] = p->prior_changepoints_index[3];
		p->prior_changepoints_index[3] = p->prior_changepoints_index[4];
		p->prior_changepoints_index[4] = p->Pindex;


		p->prior_changepoints_value[0] = p->prior_changepoints_value[1];
		p->prior_changepoints_value[1] = p->prior_changepoints_value[2];
		p->prior_changepoints_value[2] = p->prior_changepoints_value[3];
		p->prior_changepoints_value[3] = p->prior_changepoints_value[4];
		p->prior_changepoints_value[4] = p->smoothed;


		//ONCE THERE HAVE BEEN AT LEAST THREE CHANGEPOINTS
		//I'm doing this as 5 so I don't need to check any NULL values
	    if (p->totalNumChangepoints >= 5)
	    {

	    	//COMPUTE NUMBER OF SAMPLES BETWEEN EACH CHANGEPOINT
			//### 	Eg. if prior_changepoints_index = [NULL,NULL,40,60,90] then
			//###		width_differences = [NULL,NULL,20,30]
	    	int width_differences[4];
	    	width_differences[0] = p->prior_changepoints_index[1] - p->prior_changepoints_index[0];
	    	width_differences[1] = p->prior_changepoints_index[2] - p->prior_changepoints_index[1];
	    	width_differences[2] = p->prior_changepoints_index[3] - p->prior_changepoints_index[2];
	    	width_differences[3] = p->prior_changepoints_index[4] - p->prior_changepoints_index[3];

	    	//= vector of incremental differences between values in prior_changepoints_index

			//### COMPUTE THE VALUE DEVIATIONS FROM THE MIDPOINT (ONLY IF THE MIDPOINT IS NON-NULL)
			//### 	Eg. if prior_changepoints_value = [NULL,NULL,100,200,300] and midpoint_estimate = 150
			//###     	then dirs_from_midpoint = [NULL,NULL,-1,1,1]
	    	int dirs_from_midpoint[5];
	    	if (p->is_midpoint_calculated == 1)
	    	{
	    		for (int i = 0; i < 5; i++)
	    		{
	    			dirs_from_midpoint[i] = LEAF_sign(p->prior_changepoints_value[i] - p->midpoint_estimate);
	    		}
	    	}
			//### ASSEMBLE STATISTICS RELATED TO A 3-POINT PATTERN (UP/DOWN/UP or vice versa)
	    	int tempZeroCheck = abs(p->prior_changepoints_value[4] - p->prior_changepoints_value[3]);
	    	if (tempZeroCheck < 1)
	    	{
	    		tempZeroCheck = 1; //prevent divide by zero
	    	}

			float ratio_value_diffs_1 = ((float)abs(p->prior_changepoints_value[4] - p->prior_changepoints_value[2])) / (float)tempZeroCheck;
			int spread_value_1 = abs(p->prior_changepoints_value[4] - p->prior_changepoints_value[3]);
			int falls_about_midpoint_1 = 1;
			if (p->is_midpoint_calculated == 1) //if you've got a valid midpoint, then calculate this
			{
				falls_about_midpoint_1 = ((dirs_from_midpoint[2] == dirs_from_midpoint[4]) && (dirs_from_midpoint[3] != dirs_from_midpoint[4]));
			}

			float ratio_value_diffs_2 = ((float)abs(p->prior_changepoints_value[4] - p->prior_changepoints_value[0])) / (float)tempZeroCheck;
			int spread_value_2 = abs(p->prior_changepoints_value[0] - p->prior_changepoints_value[1]);
			int falls_about_midpoint_2 = 1;
			if (p->is_midpoint_calculated == 1) //if you've got a valid midpoint, then calculate this
			{
				falls_about_midpoint_2 = ( (dirs_from_midpoint[0] == dirs_from_midpoint[4]) && (dirs_from_midpoint[0] != dirs_from_midpoint[1]) && (dirs_from_midpoint[0] != dirs_from_midpoint[3]));
			}



			//### CHECK WHETHER WE ARE IN A DETECTION PATTERN
			//### 	All of the following must be satisfied in order to be a detected event:
			//### 		(1) outer two values are much closer to each other than they are to middle value
			//### 		(2) distance between first and second values is far enough apart
			//### 		(3) differences in widths are consistently spaced
			//### 		(4) differences in widths are small enough that it could be a vibration signal
			//### 		(5) points fall on the correct sides of the midpoint estimate for the given pattern


			//### 3-POINT PATTERN
			//### NOTE: var() here means the "sample variance". Tell me if you need help with it.
			//### See link: https://www.mathsisfun.com/data/standard-deviation.html

			//compute var of width differences using just elements [2] and [3]
			//first take the mean
			int tempMean = (width_differences[2] + width_differences[3]) / 2; //divide by 2
			int tempVar1 = width_differences[2] - tempMean;
			int tempVar2 = width_differences[3] - tempMean;
			int tempVariance = ((tempVar1 * tempVar1) + (tempVar2 * tempVar2));

			int tempMax3 = width_differences[2];
			if (width_differences[3] > tempMax3)
			{
				tempMax3 = width_differences[3];
			}

			int firstTest = (ratio_value_diffs_1 < p->max_ratio_value_diffs) && (spread_value_1 > p->min_value_spread) && (tempVariance < p->max_var_diff_width) && (tempMax3 < p->max_width_is_resonating) && (falls_about_midpoint_1 == 1);


			//### 5-POINT PATTERN

			//compute var of width differences using all elements
			//first take the mean
			tempMean = (width_differences[0] + width_differences[1] + width_differences[2] + width_differences[3]) / 4;
			tempVar1 = width_differences[0] - tempMean;
			tempVar2 = width_differences[1] - tempMean;
			int tempVar3 = width_differences[2] - tempMean;
			int tempVar4 = width_differences[3] - tempMean;
			tempVariance = ((tempVar1 * tempVar1) + (tempVar2 * tempVar2) + (tempVar3 * tempVar3) + (tempVar4 * tempVar4)) / 3; // divide by 3;

			tempMax3 = width_differences[0];
			for (int i = 1; i < 4; +i++)
			if (width_differences[i] > tempMax3)
			{
				tempMax3 = width_differences[i];
			}

			int secondTest = (ratio_value_diffs_2 < p->max_ratio_value_diffs) && (spread_value_2 > p->min_value_spread) && (tempVariance < p->max_var_diff_width) && (tempMax3 < p->max_width_is_resonating) && (falls_about_midpoint_2 == 1);

			if (firstTest || secondTest)
			{
				//### UPDATE THE ENVELOPE
				p->envelope_min = p->min_recent_value;
				p->envelope_max = p->max_recent_value;
				int is_pluck = 0;

				//### CHECK IF THIS IS A NEW PLUCK (NOT JUST FURTHER DETECTION OF RESONANCE ON EXISTING PLUCK)
				//### 	If it is an actual pluck, then also collect its strength
				if (p->ready_for_pluck==1)
				{
					is_pluck = (p->delay_since_last_detect > p->max_samples_still_same_pluck);
					if (is_pluck == 1)
					{
						p->pluck_strength = p->envelope_max - p->envelope_min;
						pluckHappened = p->pluck_strength;
						//adding this - not sure if it's what Angie meant:
						p->ready_for_pluck = 0;
					}
				}

				/*
				//### IF WE HAVE HAD AT LEAST THREE DETECTIONS OF RESONANCE WITHIN THE SAME PLUCK'S SIGNAL
				//### THEN WE CAN COMPUTE OR UPDATE THE MIDPOINT ESTIMATE
				if (p->prior_detect_3_index > 0)
				{
					if ((p->Pindex - p->prior_detect_1_index < p->max_samples_still_same_pluck) && (p->prior_detect_1_index - p->prior_detect_2_index < p->max_samples_still_same_pluck) && (p->prior_detect_2_index - p->prior_detect_3_index < p->max_samples_still_same_pluck))
					{
						//### Note: This can be rounded to the nearest int, but doesn't need to be
						p->midpoint_estimate = (p->max_recent_value + p->min_recent_value) / 2;
						p->is_midpoint_calculated = 1;
					}
				}
				*/

				//### RESET THE DELAY SINCE LAST DETECTION
				p->delay_since_last_detect = 0;

				//### UPDATE THE INFORMATION FOR THE PRIOR THREE DETECT EVENTS
				p->prior_detect_3_index = p->prior_detect_2_index;
				p->prior_detect_3_value = p->prior_detect_2_value;
				p->prior_detect_2_index = p->prior_detect_1_index;
				p->prior_detect_2_value = p->prior_detect_1_value;
				p->prior_detect_1_index = p->Pindex;
				p->prior_detect_1_value = p->smoothed;

			}
	    }

	}

	//### INCREMENT THE TIME DELAY SINCE THE LAST PLUCK
	p->delay_since_last_detect = p->delay_since_last_detect + 1;

	//### INCREMENT INDEX COUNTER THAT TRACKS HOW MANY SAMPLES WE'VE SEEN SO FAR
	//still need to figure out how to manage integer rollover.
	p->Pindex = p->Pindex + 1;
	if (p->Pindex == 0)
	{
		p->Pindex = 1;
	}

	//### STORE CURRENT VALUES TO COMPARE AGAINST IN NEXT ITERATION
	p->prior_super_smoothed = p->super_smoothed;
	p->prior_smoothed = p->smoothed;
	return pluckHappened;
}

