#include <gtest/gtest.h>

#include "m1_local_fast2d/candidate_selector.hpp"

namespace
{
using m1_local_fast2d::CandidateSelector;
using m1_local_fast2d::CandidateSelectorConfig;
using m1_local_fast2d::msg::CandidateRiskScoreArray;
using m1_local_fast2d::msg::LocalTrajectoryCandidateArray;

LocalTrajectoryCandidateArray routes(std::initializer_list<double> ys, uint64_t id = 9)
{
  LocalTrajectoryCandidateArray set; set.planning_result_id = id; set.header.frame_id = "odom";
  uint32_t i = 0; for (double y : ys) {m1_local_fast2d::msg::LocalTrajectoryCandidate c; c.candidate_index=i++; c.valid=true; c.trajectory.planning_result_id=id; c.trajectory.header=set.header; for(int n=0;n<4;++n){m1_local_fast2d::msg::TimedTrajectoryPoint p;p.x=n;p.y=y;p.time_from_start=n*.1;c.trajectory.points.push_back(p);} set.candidates.push_back(c);} set.valid_candidate_count=set.candidates.size(); return set;
}
CandidateRiskScoreArray scores(const LocalTrajectoryCandidateArray & set, std::initializer_list<std::pair<double,double>> risks, double coverage=.8)
{
  CandidateRiskScoreArray a; a.planning_result_id=set.planning_result_id; a.header=set.header; a.scope_prediction_id=3; uint32_t i=0; for(auto [mean,max]:risks){m1_local_fast2d::msg::CandidateRiskScore s;s.planning_result_id=a.planning_result_id;s.candidate_index=i++;s.mean_risk=mean;s.max_risk=max;s.coverage_ratio=coverage;s.valid_query_count=8;s.score_valid=true;s.coverage_sufficient=true;a.scores.push_back(s);} return a;
}
CandidateSelectorConfig cfg(){CandidateSelectorConfig c;c.w_mean=1;c.w_max=1;c.w_high=0;c.baseline_preference=0;c.switch_margin=.10;c.switch_penalty=.10;c.route_match_threshold=.2; return c;}
}

TEST(CandidateSelector, FallbackAndCoverageAndPairing)
{
  auto set=routes({0,1}); auto a=scores(set, {{.9,1},{.1,.2}}); auto c=cfg();
  auto no=CandidateSelector::select(set,nullptr,nullptr,c); EXPECT_EQ(no.selected_candidate_index,0u); EXPECT_EQ(no.selection_reason,m1_local_fast2d::msg::SelectedLocalTrajectory::FALLBACK_NO_SCORE);
  auto low=scores(set,{{.9,1},{.1,.2}},.05); for(auto &s:low.scores)s.coverage_sufficient=false;
  auto r=CandidateSelector::select(set,&low,nullptr,c); EXPECT_EQ(r.selected_candidate_index,0u); EXPECT_FALSE(r.prediction_used); EXPECT_EQ(r.selection_reason,m1_local_fast2d::msg::SelectedLocalTrajectory::FALLBACK_LOW_COVERAGE);
  a.planning_result_id=77; r=CandidateSelector::select(set,&a,nullptr,c); EXPECT_EQ(r.selected_candidate_index,0u); EXPECT_EQ(r.selection_reason,m1_local_fast2d::msg::SelectedLocalTrajectory::FALLBACK_SCORE_MISMATCH);
  a=scores(set,{{.9,1},{.1,.2}}); r=CandidateSelector::select(set,&a,nullptr,c,false); EXPECT_EQ(r.selected_candidate_index,0u); EXPECT_EQ(r.selection_reason,m1_local_fast2d::msg::SelectedLocalTrajectory::FALLBACK_STALE);
  r=CandidateSelector::select(set,&a,nullptr,c,true,true,false); EXPECT_EQ(r.selection_reason,m1_local_fast2d::msg::SelectedLocalTrajectory::FALLBACK_FRAME_MISMATCH);
}
TEST(CandidateSelector, PredictiveAndHysteresis)
{
  auto set=routes({0,1}); auto c=cfg(); auto high=scores(set,{{.9,1},{.1,.2}});
  auto r=CandidateSelector::select(set,&high,nullptr,c); EXPECT_EQ(r.selected_candidate_index,1u); EXPECT_TRUE(r.prediction_used); // temporal dynamic-obstacle route change
  auto previous=set.candidates[0].trajectory; auto marginal=scores(set,{{.30,.30},{.26,.26}});
  r=CandidateSelector::select(set,&marginal,&previous,c); EXPECT_EQ(r.selected_candidate_index,0u); EXPECT_FALSE(r.switched);
  auto strong=scores(set,{{.9,1},{.1,.2}}); r=CandidateSelector::select(set,&strong,&previous,c); EXPECT_EQ(r.selected_candidate_index,1u); EXPECT_TRUE(r.switched);
}
TEST(CandidateSelector, RouteAssociationAndUnavailable)
{
  auto old=routes({3,0,1}); auto previous=old.candidates[1].trajectory; auto next=routes({0,1}); auto c=cfg();
  auto s=scores(next,{{.5,.5},{.7,.7}}); auto r=CandidateSelector::select(next,&s,&previous,c);
  EXPECT_EQ(r.selected_candidate_index,0u); EXPECT_TRUE(r.previous_matched_candidate.has_value()); EXPECT_EQ(*r.previous_matched_candidate,0u); EXPECT_FALSE(r.switched);
  auto gone=routes({-2,2}); s=scores(gone,{{.9,.9},{.1,.1}}); r=CandidateSelector::select(gone,&s,&previous,c);
  EXPECT_EQ(r.selected_candidate_index,1u); EXPECT_TRUE(r.switched); EXPECT_EQ(r.selection_reason,m1_local_fast2d::msg::SelectedLocalTrajectory::CURRENT_ROUTE_UNAVAILABLE);
}
TEST(CandidateSelector, FiveCycleHysteresisSequence)
{
  auto set=routes({0,1}); auto c=cfg(); auto previous=set.candidates[0].trajectory;
  auto one=scores(set,{{.1,.1},{.9,.9}}); auto r=CandidateSelector::select(set,&one,&previous,c); EXPECT_EQ(r.selected_candidate_index,0u);
  auto marginal=scores(set,{{.30,.30},{.26,.26}}); r=CandidateSelector::select(set,&marginal,&previous,c); EXPECT_EQ(r.selected_candidate_index,0u); r=CandidateSelector::select(set,&marginal,&previous,c); EXPECT_EQ(r.selected_candidate_index,0u);
  auto strong=scores(set,{{.9,1},{.1,.2}}); r=CandidateSelector::select(set,&strong,&previous,c); EXPECT_EQ(r.selected_candidate_index,1u); previous=set.candidates[1].trajectory;
  auto reverse_marginal=scores(set,{{.26,.26},{.30,.30}}); r=CandidateSelector::select(set,&reverse_marginal,&previous,c); EXPECT_EQ(r.selected_candidate_index,1u); EXPECT_FALSE(r.switched);
}
