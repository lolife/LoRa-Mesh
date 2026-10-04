#pragma once

void networkRecoveryBegin();
void networkRecoveryLoop();
void networkOtaLoop();
bool networkAvailable();
void resetMqttConnection();
