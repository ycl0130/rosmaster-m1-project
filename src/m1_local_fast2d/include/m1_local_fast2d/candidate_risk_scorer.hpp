#ifndef M1_LOCAL_FAST2D__CANDIDATE_RISK_SCORER_HPP_
#define M1_LOCAL_FAST2D__CANDIDATE_RISK_SCORER_HPP_
#include "m1_local_fast2d/msg/timed_trajectory.hpp"
#include "m1_local_fast2d/msg/candidate_risk_score.hpp"
#include "m1_scope_risk/risk_field_snapshot.hpp"
namespace m1_local_fast2d { struct CandidateRiskScoringConfig {double dt{0.1}; double high_threshold{0.7}; double minimum_coverage_ratio{0.25};}; class CandidateRiskScorer {public: static m1_local_fast2d::msg::CandidateRiskScore score(const m1_local_fast2d::msg::TimedTrajectory &, uint32_t, const m1_scope_risk::RiskFieldSnapshot &, int64_t, int64_t, const CandidateRiskScoringConfig &);}; }
#endif
