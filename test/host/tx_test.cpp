#include <cassert>
#include "../../src/mesh_tx.cpp"
const MeshOptions& meshApplicationOptions() {
    static const MeshOptions options = {ROLE_NONE, nullptr, nullptr, nullptr};
    return options;
}
int main(){
 const uint8_t addr[6]={1,2,3,4,5,6};uint8_t payload[sizeof(StatusMessage)+1]={};
 assert(!queueMeshPacket(addr,payload,40,false));assert(initMeshTx());
 for(int i=0;i<28;i++)assert(queueMeshPacket(addr,payload,40,false));
 assert(!queueMeshPacket(addr,payload,40,false));
 payload[0]=42;assert(queueMeshPacket(addr,payload,8,true));
 assert(!queueMeshPacket(addr,payload,sizeof(payload),true));
 int sends=0;
 onSend=[&](){
   ++sends;
   if(sends==1){assert(sentPacket[0]==42);return ESP_ERR_ESPNOW_NO_MEM;}
   // Every previous accepted packet must have waited for completion first.
   assert(completionWaits==static_cast<unsigned>(sends-2));
   meshTxComplete();return ESP_OK;
 };
 try{transmit(nullptr);}catch(const EndOfWork&){}
 assert(sends==30 && completionWaits==29 && retryDelays==1);
 assert(uxQueueSpacesAvailable(txQueue)==32);
 puts("PASS TX: bounded queue, relay reserve, priority, size validation, low-memory retry, completion pacing");
}
