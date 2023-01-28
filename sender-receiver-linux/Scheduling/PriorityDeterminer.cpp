//
// Created by david on 28.01.23.
//

#include "PriorityDeterminer.h"

int PriorityDeterminer::toEthernetFrameSizeBytes(int payloadSizeBytes) {
    int ethernetFrameHeaderSize = 18;
    int ipPacketHeaderSize = 20;
    int udpPacketHeaderSize = 8;
    return payloadSizeBytes + ethernetFrameHeaderSize + ipPacketHeaderSize + udpPacketHeaderSize;
}
