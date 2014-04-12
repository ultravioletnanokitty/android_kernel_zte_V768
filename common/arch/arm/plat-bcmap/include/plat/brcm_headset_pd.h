/* Header file for HEASET DRIVER */

#ifndef __BCM_HEADSET_PD__
#define __BCM_HEADSET_PD__

struct brcm_headset_pd {
	int hsirq;
	int hsbirq;
	void (*check_hs_state) (int *);
	int hsgpio;
	int debounce_ms;
    int keypress_threshold_fp;
    int keypress_threshold_lp;
};

#endif /*  __BCM_HEADSET_PD__ */
