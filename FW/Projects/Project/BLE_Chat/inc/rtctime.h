#ifndef _RTCTIME_H_
#define _RTCTIME_H_

void RTC_Config(void);
void RTC_TimeShow(void);
void RTC_TimeRegulate(void);
void RTC_DateRegulate(void);
void RTC_SetMyTime(uint32_t hh, uint32_t mm, uint32_t ss);
void RTC_SetMyDate(uint32_t yy, uint32_t mm, uint32_t dd);
#endif /* _RTCTIME_H_ */