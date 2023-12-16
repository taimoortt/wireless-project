#include "radiosaber-downlink-scheduler.h"
#include "../mac-entity.h"
#include "../../packet/Packet.h"
#include "../../packet/packet-burst.h"
#include "../../../device/NetworkNode.h"
#include "../../../flows/radio-bearer.h"
#include "../../../protocolStack/rrc/rrc-entity.h"
#include "../../../flows/application/Application.h"
#include "../../../device/ENodeB.h"
#include "../../../protocolStack/mac/AMCModule.h"
#include "../../../phy/lte-phy.h"
#include "../../../core/spectrum/bandwidth-manager.h"
#include "../../../core/idealMessages/ideal-control-messages.h"
#include "../../../utility/eesm-effective-sinr.h"
#include <cassert>
#include <unordered_set>

using SchedulerAlgo = SliceContext::SchedulerAlgo;

RadioSaberDownlinkScheduler::RadioSaberDownlinkScheduler(std::string config_fname)
: DownlinkPacketScheduler(config_fname) {
  SetMacEntity (0);
  CreateFlowsToSchedule ();
  slice_rbs_share_.resize(slice_ctx_.num_slices_, 0);
  slice_rbs_offset_.resize(slice_ctx_.num_slices_, 0);
  slice_rbgs_quota_.resize(slice_ctx_.num_slices_, 0);
  metrics_perrbg_slice_.resize(slice_ctx_.num_slices_, 0);
  rollover_slice_quota_.resize(slice_ctx_.num_slices_, 0);
  actual_slice_rbgs_metric_sum_.resize(slice_ctx_.num_slices_, 0);
  std::cout << "construct RadioSaber Downlink Scheduler." << std::endl;
}

RadioSaberDownlinkScheduler::~RadioSaberDownlinkScheduler()
{
  Destroy ();
}

void RadioSaberDownlinkScheduler::CalculateSliceQuota()
{
  int nb_rbgs = GetMacEntity()->GetDevice()->GetPhy()
    ->GetBandwidthManager()->GetDlSubChannels().size() / RBG_SIZE;
  for (int i = 0; i < slice_ctx_.num_slices_; i++) {
    slice_rbgs_quota_[i] = nb_rbgs * slice_ctx_.weights_[i];
  }
  // now we reallocate the RBGs of slices with no traffic to slices with traffic
  // step1: find those slices with data
  FlowsToSchedule* flows = GetFlowsToSchedule();
  std::unordered_set<int> slice_with_data;
  double extra_rbgs = nb_rbgs;
  for (auto it = flows->begin(); it != flows->end(); ++it) {
    assert(*it != nullptr);
    int slice_id = (*it)->GetSliceID();
    if (slice_with_data.find(slice_id) != slice_with_data.end()) {
      continue;
    }
    slice_with_data.insert(slice_id);
    extra_rbgs -= slice_rbgs_quota_[slice_id];
  }
  // assert(slice_with_data.size() > 0);
  // step2: update slice rbgs quota(set 0 to slice without data, and increase a random slice with extra_rbgs)
  int rand_idx = rand() % slice_with_data.size();
  for (int i = 0; i < slice_ctx_.num_slices_; i++) {
    if (slice_with_data.find(i) == slice_with_data.end()) {
      slice_rbgs_quota_[i] = 0;
    }
    else {
      if (rand_idx == 0)
        slice_rbgs_quota_[i] += extra_rbgs;
      rand_idx -= 1;
    }
  }
  std::cout << "slice quota: ";
  for (int i = 0; i < slice_ctx_.num_slices_; i++) {
    std::cout << slice_rbgs_quota_[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "rolled over: ";
    for (int i = 0; i < slice_ctx_.num_slices_; i++) {
    std::cout << rollover_slice_quota_[i] << ", ";
  }
  std::cout << std::endl;

  for (int i = 0; i < slice_ctx_.num_slices_; i++) {
    slice_rbgs_quota_[i] += rollover_slice_quota_[i];
  }

  std::cout << "after rollover: ";
  for (int i = 0; i < slice_ctx_.num_slices_; i++) {
    std::cout << slice_rbgs_quota_[i] << ", ";
  }
  std::cout << std::endl;
}

double
RadioSaberDownlinkScheduler::ComputeSchedulingMetric(
  RadioBearer *bearer, double spectralEfficiency, int subChannel)
{
  double metric = 0;
  double averageRate = bearer->GetAverageTransmissionRate();
  int slice_id = bearer->GetDestination()->GetSliceID();
  averageRate /= 1000.0; // set the unit of both params to kbps
  SchedulerAlgo param = slice_ctx_.algo_params_[slice_id];
  if (param.alpha == 0) {
    metric = pow(spectralEfficiency, param.epsilon) / pow(averageRate, param.psi);
    if (bearer->GetPriority() == 1) {
      metric = 100 * metric;
    }
  }
  else {
    if (param.beta) {
      double HoL = bearer->GetHeadOfLinePacketDelay();
      metric = HoL * pow(spectralEfficiency, param.epsilon)
        / pow(averageRate, param.psi);
    }
    else {
      metric = pow(spectralEfficiency, param.epsilon)
        / pow(averageRate, param.psi);
    }
  }
  return metric;
}

void
RadioSaberDownlinkScheduler::CalculateMetricsPerRB(int rb_id)
{
  FlowsToSchedule* flows = GetFlowsToSchedule();
  const int num_slice = slice_ctx_.num_slices_;
  AMCModule* amc = GetMacEntity()->GetAmcModule();
  int nb_of_rbs = GetMacEntity()->GetDevice()->GetPhy()
    ->GetBandwidthManager()->GetDlSubChannels().size ();
  std::vector<double> slice_sum_metrics(num_slice, 0);
  std::vector<double> slice_rbgs_quota(slice_rbgs_quota_);
  rb_id = 0;
  while (rb_id < nb_of_rbs) {
    std::vector<FlowToSchedule*> slice_flow(num_slice, nullptr);
    std::vector<double> slice_spectraleff(num_slice, -1);
    std::vector<double> max_metrics(num_slice, -1);
    for (auto it = flows->begin(); it != flows->end(); it++) {
      FlowToSchedule* flow = *it;
      auto& rsrp_report = flow->GetRSRPReport();
      // under the current muting assumption
      double spectraleff_rbg = 0.0;
      for (int j = rb_id; j < RBG_SIZE+rb_id; j++) {
        double int_noise = rsrp_report.noise_interfere_watt[j]; // Interference under Muting
        int cqi = amc->GetCQIFromSinr(rsrp_report.rx_power[j] - 10. * log10(int_noise));
        spectraleff_rbg += amc->GetEfficiencyFromCQI(cqi); // This returns TBS
      }
      double metric = ComputeSchedulingMetric(
        flow->GetBearer(), spectraleff_rbg, rb_id);
      int slice_id = flow->GetSliceID();
      // enterprise schedulers
      if (metric > max_metrics[slice_id]) {
        max_metrics[slice_id] = metric;
        slice_flow[slice_id] = flow;
        slice_spectraleff[slice_id] = spectraleff_rbg;
      }
    }
    double max_slice_spectraleff = -1;
    int selected_slice = -1;
    double slice_metric = -1;
    for (int j = 0; j < num_slice; j++) {
      if (slice_spectraleff[j] > max_slice_spectraleff
          && slice_rbgs_quota[j] >= 1) {
        max_slice_spectraleff = slice_spectraleff[j];
        selected_slice = j;
        slice_metric = max_metrics[j];
      }
    }
    // no slice has more than one RB quota, allocate to a slice with fractional
    if (selected_slice == -1) {
      for (int j = 0; j < num_slice; j++) {
        if (slice_spectraleff[j] > max_slice_spectraleff
            && slice_rbgs_quota[j] >= 0) {
          max_slice_spectraleff = slice_spectraleff[j];
          selected_slice = j;
          slice_metric = max_metrics[j];
        }
      }
    }
    slice_rbgs_quota[selected_slice] -= 1;
    slice_sum_metrics[selected_slice] += slice_metric;
    rb_id += RBG_SIZE;
  }
  for (int j = 0; j < num_slice; j++) {
    metrics_perrbg_slice_[j] = slice_sum_metrics[j] / slice_rbgs_quota_[j];
  }
}