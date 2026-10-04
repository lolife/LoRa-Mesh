#pragma once
using TaskHandle_t=void*;
inline unsigned notifications=0, completionWaits=0, retryDelays=0;
inline int xTaskCreate(void(*)(void*),const char*,unsigned,void*,int,TaskHandle_t* handle){*handle=reinterpret_cast<void*>(1);return 1;}
inline void xTaskNotifyGive(TaskHandle_t){notifications++;}
inline unsigned ulTaskNotifyTake(int,unsigned wait){if(wait)completionWaits++;unsigned count=notifications;notifications=0;return count;}
inline void vTaskDelay(unsigned){retryDelays++;}
