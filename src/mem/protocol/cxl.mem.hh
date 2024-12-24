#ifndef __CXL_PROTOCOL_HH__
#define __CXL_PROTOCOL_HH__

#include "base/types.hh"

#define FLIT_SIZE (528 / 8)
#define DATA_FLIT   65
struct M2SReq
{
    unsigned val:1;
    unsigned MemOp:4;
    unsigned SnpType:3;
    unsigned MetaField:2;
    unsigned MetaValue:2;
    unsigned Tag3_0:4;
    unsigned Tag11_4:8;
    unsigned Tag15_12:4;
    Addr Addr51_5:47;
    unsigned TC:2;
    unsigned Spare2_0:3;
    unsigned Spare9_3:7;
    unsigned RSVD1:1;
    unsigned RSVD2:8;
};

struct M2SReqWD{
    unsigned val:1;
    unsigned MemOp:4;
    unsigned SnpType:3;
    unsigned MetaField:2;
    unsigned MetaValue:2;
    unsigned Tag3_0:4;
    unsigned Tag11_4:8;
    unsigned Tag15_12:4;
    Addr Addr51_6:46;
    unsigned Poi:1;
    unsigned TC:2;
    unsigned Spare2_0:3;
    unsigned Spare9_3:7;
    unsigned RSVD1:1;
    unsigned RSVD2:8;
};

class hslot{
private:
    unsigned type:1;
    unsigned RSVD_0:1;
    unsigned Ak:1;
    unsigned BE:1;
    unsigned Sz:1;
    unsigned Slot0:3;
    unsigned Slot1:3;
    unsigned Slot2:3;
    unsigned Slot3_2_1:2;
    unsigned Sl3:1;
    unsigned RSVD_1:3;
    unsigned RspCrd:4;
    unsigned ReqCrd:4;
    unsigned DataCrd:4;
public:
    char buffer[14];
    hslot(){

    };
    ~hslot(){};
};

class M2S : public hslot
{
private:
    union m2s
    {
        struct M2SReq m2sreq;
        struct M2SReqWD m2sreqwd;
    };
    union m2s *data = (union m2s *)hslot::buffer;
public:
    M2S(Addr dest, unsigned memop, unsigned snptype,
        unsigned metafield, unsigned metavalue){
        data->m2sreq.Addr51_5 = dest >> 5;
        data->m2sreq.MemOp = memop;
        data->m2sreq.SnpType = snptype;
        data->m2sreq.MetaField = metafield;
        data->m2sreq.MetaValue = metavalue;
    };
    ~M2S(){};
};

class MemRd : public M2S{
private:

public:
    MemRd(Addr dest) :
    M2S(dest, 0b0001, 0b010, 0b11, 0b01){

    }
};

class gslot{
public:
    gslot(){};
    ~gslot(){};
};

class ProtocolFlit
{
private:
    /* data */
public:
    ProtocolFlit(/* args */);
    ~ProtocolFlit();
};

#endif