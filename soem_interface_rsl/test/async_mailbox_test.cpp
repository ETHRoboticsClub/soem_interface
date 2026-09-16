#include <soem_interface_rsl/AsyncMailbox.hpp>
#include <cassert>
#include <cstring>
#include <map>
using namespace soem_interface_rsl;
struct Wire : MailboxTransport {
  bool flight=false, frozen=false, pending=false, unrelated=false, badSize=false, abort=false, badWkc=false;
  uint16_t reg=0, size=0, object=0; uint8_t sub=0, count=0;
  bool write=false, upload=false;
  int polls=0, submissions=0;
  uint32_t value=0;
  std::array<uint8_t,1486> tx{};
  std::map<std::pair<uint16_t,uint8_t>,uint32_t> objects;
  MailboxEndpoint endpoint(uint16_t) const override {return {0x1000,128,0x1100,128};}
  uint8_t nextCounter(uint16_t) override {count=count%7+1;return count;}
  bool start(uint16_t, uint16_t r, bool w, const uint8_t* data,uint16_t n) override {
    assert(!flight);flight=true;reg=r;write=w;size=n;std::copy_n(data,n,tx.begin());++submissions;return true;
  }
  int poll(uint8_t* data) override {
    ++polls;
    assert(flight);
    if(frozen)return -1;
    flight=false;
    if(badWkc)return 0;
    std::fill_n(data,size,0);
    if(reg==0x080d)data[0]=pending?8:0;
    else if(reg==0x1000){
      assert(write && tx[0]==10 && tx[5]==(3|(count<<4)) && tx[7]==0x20);
      object=tx[9]|(uint16_t(tx[10])<<8);sub=tx[11];upload=tx[8]==0x40;
      if(!upload){
        assert((tx[8]&0xf3)==0x23);
        unsigned n=4-((tx[8]>>2)&3);value=0;
        for(unsigned i=0;i<n;++i)value|=uint32_t(tx[12+i])<<(8*i);
        if(!abort)objects[{object,sub}]=value;
      }
      pending=true;
    }else if(reg==0x1100){
      assert(pending);pending=false;
      data[0]=10;data[5]=3|(count<<4);data[7]=0x30;
      data[8]=abort?0x80:(upload?(object==0x603f?0x4b:0x43):0x60);
      if(badSize)data[8]=0x4f;
      data[9]=object;data[10]=object>>8;data[11]=sub;
      value=abort?0x06020000:objects[{object,sub}];
      for(unsigned i=0;i<4;++i)data[12+i]=value>>(8*i);
      if(unrelated){data[11]++;unrelated=false;pending=true;}
    }else if(reg==0x0130){data[0]=0x14;data[4]=0x1b;}
    else assert(reg==0x0805);
    return 1;
  }
  void cancel() override {flight=false;}
};
void drive(AsyncMailbox& box, Wire& wire, MailboxRequest::Ptr request){
  for(int i=0;i<200 && request->status==MailboxStatus::Pending;++i){
    int calls=wire.polls+wire.submissions;box.tick();
    assert(wire.polls+wire.submissions-calls<=1);
  }
  assert(request->status!=MailboxStatus::Pending);
}
#ifndef MAILBOX_TEST_NO_MAIN
int main(){
  Wire wire;AsyncMailbox box(wire);box.enable(2);
  auto write=box.submit(MailboxRequest::Kind::Write,1,0x34c6,1,4,0x12345678);
  drive(box,wire,write);assert(write->status==MailboxStatus::Success);
  auto read=box.submit(MailboxRequest::Kind::Read,1,0x34c6,1,4);
  wire.unrelated=true;drive(box,wire,read);assert(read->status==MailboxStatus::Success && read->value==0x12345678);
  wire.objects[{0x603f,0}]=0x8611;
  auto fault=box.submit(MailboxRequest::Kind::Read,2,0x603f,0,2);
  drive(box,wire,fault);assert(fault->status==MailboxStatus::Success && fault->value==0x8611);
  auto al=box.submit(MailboxRequest::Kind::Register,1,0x0130,0,6);
  drive(box,wire,al);assert(al->registers[0]==0x14 && al->registers[4]==0x1b);
  wire.abort=true;auto aborted=box.submit(MailboxRequest::Kind::Write,1,0x34c6,1,4,4);
  drive(box,wire,aborted);assert(aborted->status==MailboxStatus::Abort && aborted->abortCode==0x06020000);
  wire.abort=false;wire.badSize=true;
  auto malformed=box.submit(MailboxRequest::Kind::Read,1,0x603f,0,2);
  drive(box,wire,malformed);assert(malformed->status==MailboxStatus::Invalid);wire.badSize=false;
  // Missing mailbox response never blocks PDO ownership; hundreds of ticks remain available.
  auto missing=box.submit(MailboxRequest::Kind::Write,1,0x34c6,1,4,99);
  while(!wire.pending)box.tick();wire.frozen=true;
  int pdoCycles=0;for(int i=0;i<200;++i){box.tick();++pdoCycles;}
  assert(pdoCycles==200 && missing->status==MailboxStatus::Pending);
  box.tick(AsyncMailbox::Clock::now()+std::chrono::seconds(1));
  assert(missing->status==MailboxStatus::Timeout && !wire.flight);
  auto unsafeRetry=box.submit(MailboxRequest::Kind::Write,1,0x34c6,1,4,100);
  assert(unsafeRetry->status==MailboxStatus::Unavailable);
  wire.frozen=false;wire.pending=false;
  auto other=box.submit(MailboxRequest::Kind::Read,2,0x603f,0,2);
  drive(box,wire,other);assert(other->status==MailboxStatus::Success);
  box.disable();assert(box.submit(MailboxRequest::Kind::Read,2,0x603f,0,2)->status==MailboxStatus::Unavailable);
  box.enable(2);auto cancelled=box.submit(MailboxRequest::Kind::Read,2,0x603f,0,2);box.tick();box.disable();
  assert(cancelled->status==MailboxStatus::Cancelled && !wire.flight);
  box.enable(2);assert(box.submit(MailboxRequest::Kind::Read,1,0x603f,0,2)->status==MailboxStatus::Unavailable);
  assert(box.submit(MailboxRequest::Kind::Read,2,0x603f,0,8)->status==MailboxStatus::Invalid);
  for(size_t i=0;i<AsyncMailbox::kQueueCapacity;++i)assert(box.submit(MailboxRequest::Kind::Read,2,0x603f,0,2)->status==MailboxStatus::Pending);
  assert(box.submit(MailboxRequest::Kind::Read,2,0x603f,0,2)->status==MailboxStatus::Unavailable);
  box.disable();
}

#endif
