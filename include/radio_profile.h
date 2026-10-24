#pragma once

// Select one shared profile in platformio.ini's [radio] section.
#define LORA_PROFILE_SF9_125 1
#define LORA_PROFILE_SF7_125 2
#define LORA_PROFILE_SF7_250 3

#ifndef LORA_PROFILE
#define LORA_PROFILE LORA_PROFILE_SF9_125
#endif

#if LORA_PROFILE == LORA_PROFILE_SF9_125
#define LORA_SF 9
#define LORA_BW 125000L
#elif LORA_PROFILE == LORA_PROFILE_SF7_125
#define LORA_SF 7
#define LORA_BW 125000L
#elif LORA_PROFILE == LORA_PROFILE_SF7_250
#define LORA_SF 7
#define LORA_BW 250000L
#else
#error "Unknown LORA_PROFILE: choose LORA_PROFILE_SF9_125, LORA_PROFILE_SF7_125, or LORA_PROFILE_SF7_250"
#endif
