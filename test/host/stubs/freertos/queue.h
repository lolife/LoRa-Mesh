#pragma once
#include <deque>
#include <vector>
#include <cstring>
struct MockQueue { size_t capacity, itemSize; std::deque<std::vector<uint8_t>> items; };
using QueueHandle_t=MockQueue*;
struct EndOfWork {};
inline QueueHandle_t xQueueCreate(size_t n,size_t size){return new MockQueue{n,size,{}};}
inline void vQueueDelete(QueueHandle_t q){delete q;}
inline size_t uxQueueSpacesAvailable(QueueHandle_t q){return q->capacity-q->items.size();}
inline int xQueueSend(QueueHandle_t q,const void* value,unsigned){if(q->items.size()==q->capacity)return 0;const auto* p=static_cast<const uint8_t*>(value);q->items.emplace_back(p,p+q->itemSize);return 1;}
inline int xQueueSendToFront(QueueHandle_t q,const void* value,unsigned){if(q->items.size()==q->capacity)return 0;const auto* p=static_cast<const uint8_t*>(value);q->items.emplace_front(p,p+q->itemSize);return 1;}
inline int xQueueReceive(QueueHandle_t q,void* out,unsigned wait){if(q->items.empty()){if(!wait)return 0;throw EndOfWork{};}memcpy(out,q->items.front().data(),q->itemSize);q->items.pop_front();return 1;}
