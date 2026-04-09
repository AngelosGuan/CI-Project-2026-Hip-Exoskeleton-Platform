#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
http://t-filter.appspot.com

sampling frequency: 500 Hz

* 0 Hz - 3 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 4.16697758870432 dB

* 6 Hz - 250 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -40.01799805778509 dB

*/

#define FILTER_TAP_NUM 301
#define MOV_AVG_NUM 70
#define SHORT_MOV_AVG_NUM 11


double fir_filter(double xn, double xv[FILTER_TAP_NUM]);
double mov_avg_filter(double xn, double xv[MOV_AVG_NUM]);
double short_mov_avg_filter(double xn, double xv[SHORT_MOV_AVG_NUM]);

#endif
