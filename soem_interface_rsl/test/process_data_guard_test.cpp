// The process-data guard: a receive must not trust a reply pointer the send did not queue,
// and a sealed group table faults in its writer.
#include <gtest/gtest.h>

#include "ProcessDataGuard.hpp"

#include <csignal>

using namespace soem_interface_rsl;

namespace {

char ioMap[256];

ec_idxstackT queued() {
  ec_idxstackT stack{};
  stack.pushed = 2;
  stack.idx[0] = 3;
  stack.data[0] = &ioMap[64];
  stack.length[0] = 32;
  stack.dcoffset[0] = 44;
  stack.idx[1] = 4;
  stack.data[1] = &ioMap[0];
  stack.length[1] = 64;
  return stack;
}

}  // namespace

TEST(ProcessDataGuard, RepliesInsideTheMapPass) {
  EXPECT_EQ(indexStackOutsideMap(queued(), ioMap, sizeof(ioMap)), "");
}

TEST(ProcessDataGuard, ANullOrOverlongReplyIsNamed) {
  auto stack = queued();
  stack.data[0] = nullptr;
  stack.length[1] = sizeof(ioMap) + 1;
  const auto report = indexStackOutsideMap(stack, ioMap, sizeof(ioMap));
  EXPECT_NE(report.find("entry 0 (idx 3) copies 32 bytes to 0"), std::string::npos) << report;
  EXPECT_NE(report.find("entry 1 (idx 4) copies 257 bytes"), std::string::npos) << report;
}

TEST(ProcessDataGuard, AnUnchangedQueuePasses) {
  EXPECT_EQ(indexStackDifference(queued(), queued()), "");
}

TEST(ProcessDataGuard, EveryChangedFieldIsNamed) {
  const auto sent = queued();
  auto actual = sent;
  actual.data[0] = nullptr;
  actual.pulled = 1;
  const auto report = indexStackDifference(sent, actual);
  EXPECT_NE(report.find("pulled 0 -> 1"), std::string::npos) << report;
  EXPECT_NE(report.find("entry 0:"), std::string::npos) << report;
  EXPECT_EQ(report.find("entry 1:"), std::string::npos) << report;
}

TEST(ProcessDataGuardDeathTest, ASealedPageFaultsInItsWriter) {
  SealablePages<ec_groupt, EC_MAXGROUP> groups;
  groups.data()[1].blockLRW = 1;
  ASSERT_TRUE(groups.seal());
  EXPECT_EQ(groups.data()[1].blockLRW, 1) << "sealed pages stay readable";
  EXPECT_EXIT(groups.data()[0].inputs = nullptr, testing::KilledBySignal(SIGSEGV), "");
  ASSERT_TRUE(groups.unseal());
  groups.data()[0].Ibytes = 7;
  EXPECT_EQ(groups.data()[0].Ibytes, 7u);
}
