/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010,2011,2012,2013 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of LTE-Sim
 *
 * LTE-Sim is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * LTE-Sim is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LTE-Sim; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Giuseppe Piro <g.piro@poliba.it>
 */



#include "FrameManager.h"
#include "../load-parameters.h"
#include "../device/ENodeB.h"
#include "../device/HeNodeB.h"
#include "../device/UserEquipment.h"
#include "../protocolStack/mac/packet-scheduler/downlink-packet-scheduler.h"
#include "../protocolStack/mac/AMCModule.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../utility/eesm-effective-sinr.h"
#include "../flows/radio-bearer.h"
#include "../flows/application/Application.h"
#include "../phy/enb-lte-phy.h"
#include <cassert>
#include <unordered_set>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <random>
#include <stdexcept>
#include "../protocolStack/mac/packet-scheduler/radiosaber-downlink-scheduler.h"

int min_number_users_per_cell = 5;

FrameManager* FrameManager::ptr=NULL;

FrameManager::FrameManager() {
  m_nbFrames = 0;
  m_nbSubframes = 0;
  m_TTICounter = 0;
  m_frameStructure = FrameManager::FRAME_STRUCTURE_FDD; //Default Value
  m_TDDFrameConfiguration = 1; //Default Value
  Simulator::Init()->Schedule(0.0, &FrameManager::Start, this);
}

FrameManager::~FrameManager()
{}

void
FrameManager::SetFrameStructure (FrameManager::FrameStructure frameStructure)
{
  m_frameStructure = frameStructure;
}

FrameManager::FrameStructure
FrameManager::GetFrameStructure (void) const
{
  return m_frameStructure;
}


void
FrameManager::SetTDDFrameConfiguration (int configuration)
{
  if (configuration < 0 && configuration > 6)
    {
	  m_TDDFrameConfiguration = 0; //Default Value
    }
  else
    {
	  m_TDDFrameConfiguration = configuration;
    }
}

int
FrameManager::GetTDDFrameConfiguration (void) const
{
  return m_TDDFrameConfiguration;
}

int
FrameManager::GetSubFrameType (int nbSubFrame)
{
  return TDDConfiguration [GetTDDFrameConfiguration ()][nbSubFrame-1];
}


void
FrameManager::UpdateNbFrames (void)
{
  m_nbFrames++;
}

int
FrameManager::GetNbFrames (void) const
{
  return m_nbFrames;
}

void
FrameManager::UpdateNbSubframes (void)
{
  m_nbSubframes++;
}

void
FrameManager::ResetNbSubframes (void)
{
  m_nbSubframes = 0;
}

int
FrameManager::GetNbSubframes (void) const
{
  return m_nbSubframes;
}

void
FrameManager::UpdateTTIcounter (void)
{
  m_TTICounter++;
}

unsigned long
FrameManager::GetTTICounter () const
{
  return m_TTICounter;
}

void
FrameManager::Start (void)
{
#ifdef FRAME_MANAGER_DEBUG
  std::cout << " LTE Simulation starts now! "<< std::endl;
#endif

  Simulator::Init()->Schedule(0.0, &FrameManager::StartFrame, this);
}

void
FrameManager::StartFrame (void)
{
  UpdateNbFrames ();

#ifdef FRAME_MANAGER_DEBUG
  std::cout << " +++++++++ Start Frame, time =  "
      << Simulator::Init()->Now() << " +++++++++" << std::endl;
#endif

  Simulator::Init()->Schedule(0.0,
							  &FrameManager::StartSubframe,
							  this);
}

void
FrameManager::StopFrame (void)
{
  Simulator::Init()->Schedule(0.0,
							  &FrameManager::StartFrame,
							  this);
}

void
FrameManager::StartSubframe (void)
{
#ifdef FRAME_MANAGER_DEBUG
  std::cout << " --------- Start SubFrame, time =  "
      << Simulator::Init()->Now() << " --------- " << "TTI: " << GetTTICounter() << std::endl;
#endif

  // Prints UE positioning logs after the handovers have occured.
  if(GetTTICounter() == 120){
    std::vector<ENodeB*> *enodebs = GetNetworkManager ()->GetENodeBContainer ();
    cout << "ENB Size: " << enodebs->size() << endl;
    for (auto iter = enodebs->begin (); iter != enodebs->end (); iter++) {
  	  ENodeB* enb = *iter;
      ENodeB::UserEquipmentRecords* ue_records = enb->GetUserEquipmentRecords();
      cout << "UE Records Size: " << ue_records->size() << endl;
      for (size_t i = 0; i < ue_records->size(); i++){
          UserEquipment* x = (*ue_records)[i]->GetUE();
          cout << GetTTICounter();
          x->Print();
      }
      // if (ue_records->size() < min_number_users_per_cell){
      //   cout << "UE Attachment too low for Cell: " << enb->GetIDNetworkNode() 
      // << "\t No of UEs: " << ue_records->size() << endl;  
      //   cout << "Central Downlink RBs Allocation!" << endl;
      //   std::exit(0);
      // }
    }
  }


  UpdateTTIcounter ();
  UpdateNbSubframes ();

  /*
   * At the beginning of each sub-frame the simulation
   * update user position. This function could be
   * implemented also every TTI.
   */
  //UpdateUserPosition(); --> moved to each UE class


#ifdef PLOT_USER_POSITION
  NetworkManager::Init()->PrintUserPosition();
#endif

  /*
   * According to the Frame Structure, the DW/UL link scheduler
   * will be called for each sub-frame.
   * (RBs allocation)
   */
  Simulator::Init()->Schedule(0, &FrameManager::CentralResourceAllocation, this);
  Simulator::Init()->Schedule(0.001,
							  &FrameManager::StopSubframe,
							  this);
}

void
FrameManager::StopSubframe (void)
{
  if (GetNbSubframes () == 10)
    {
	  ResetNbSubframes ();
	  Simulator::Init()->Schedule(0.0,
								  &FrameManager::StopFrame,
								  this);
    }
  else
    {
	  Simulator::Init()->Schedule(0.0,
								  &FrameManager::StartSubframe,
								  this);
    }
}


NetworkManager*
FrameManager::GetNetworkManager (void)
{
  return NetworkManager::Init();
}

void
FrameManager::UpdateUserPosition(void)
{
  GetNetworkManager ()->UpdateUserPosition (GetTTICounter () / 1000.0);
}


void
FrameManager::ResourceAllocation(void)
{
  std::vector<ENodeB*> *records = GetNetworkManager ()->GetENodeBContainer ();
  std::vector<ENodeB*>::iterator iter;
  ENodeB *record;
  for (iter = records->begin (); iter != records->end (); iter++)
	{
	  record = *iter;

#ifdef FRAME_MANAGER_DEBUG
	  std::cout << " FRAME_MANAGER_DEBUG: RBs allocation for eNB " <<
		  record->GetIDNetworkNode() << std::endl;
#endif


	  if (GetFrameStructure () == FrameManager::FRAME_STRUCTURE_FDD)
		{
		  //record->ResourceBlocksAllocation ();
		  Simulator::Init()->Schedule(0.0, &ENodeB::ResourceBlocksAllocation,record);
		}
	  else
		{
		  //see frame configuration in TDDConfiguration
		  if (GetSubFrameType (GetNbSubframes ()) == 0)
			{
#ifdef FRAME_MANAGER_DEBUG
			  std::cout << " FRAME_MANAGER_DEBUG: SubFrameType = "
				  "	SUBFRAME_FOR_DOWNLINK " << std::endl;
#endif
			  //record->DownlinkResourceBlockAllocation();
			  Simulator::Init()->Schedule(0.0, &ENodeB::DownlinkResourceBlockAllocation,record);
			}
		  else if(GetSubFrameType (GetNbSubframes ()) == 1)
			{
#ifdef FRAME_MANAGER_DEBUG
			  std::cout << " FRAME_MANAGER_DEBUG: SubFrameType = "
				  "	SUBFRAME_FOR_UPLINK " << std::endl;
#endif
			  //record->UplinkResourceBlockAllocation();
			  Simulator::Init()->Schedule(0.0, &ENodeB::UplinkResourceBlockAllocation,record);
			}
		  else
			{
#ifdef FRAME_MANAGER_DEBUG
			  std::cout << " FRAME_MANAGER_DEBUG: SubFrameType = "
				  "	SPECIAL_SUBFRAME" << std::endl;
#endif
			}
		}
	}

  std::vector<HeNodeB*> *records_2 = GetNetworkManager ()->GetHomeENodeBContainer();
    std::vector<HeNodeB*>::iterator iter_2;
    HeNodeB *record_2;
    for (iter_2 = records_2->begin (); iter_2 != records_2->end (); iter_2++)
  	{
  	  record_2 = *iter_2;

  #ifdef FRAME_MANAGER_DEBUG
  	  std::cout << " FRAME_MANAGER_DEBUG: RBs allocation for eNB " <<
  		  record_2->GetIDNetworkNode() << std::endl;
  #endif


  	  if (GetFrameStructure () == FrameManager::FRAME_STRUCTURE_FDD)
  		{
  		  //record_2->ResourceBlocksAllocation ();
  		  Simulator::Init()->Schedule(0.0, &ENodeB::ResourceBlocksAllocation,record_2);
  		}
  	  else
  		{
  		  //see frame configuration in TDDConfiguration
  		  if (GetSubFrameType (GetNbSubframes ()) == 0)
  			{
  #ifdef FRAME_MANAGER_DEBUG
  			  std::cout << " FRAME_MANAGER_DEBUG: SubFrameType = "
  				  "	SUBFRAME_FOR_DOWNLINK " << std::endl;
  #endif
  			  //record_2->DownlinkResourceBlockAllocation();
  			  Simulator::Init()->Schedule(0.0, &ENodeB::DownlinkResourceBlockAllocation,record_2);
  			}
  		  else if(GetSubFrameType (GetNbSubframes ()) == 1)
  			{
  #ifdef FRAME_MANAGER_DEBUG
  			  std::cout << " FRAME_MANAGER_DEBUG: SubFrameType = "
  				  "	SUBFRAME_FOR_UPLINK " << std::endl;
  #endif
  			  //record_2->UplinkResourceBlockAllocation();
  			  Simulator::Init()->Schedule(0.0, &ENodeB::UplinkResourceBlockAllocation,record_2);
  			}
  		  else
  			{
  #ifdef FRAME_MANAGER_DEBUG
  			  std::cout << " FRAME_MANAGER_DEBUG: SubFrameType = "
  				  "	SPECIAL_SUBFRAME" << std::endl;
  #endif
  			}
  		}
  	}
}

void
FrameManager::CentralResourceAllocation(void)
{
  std::vector<ENodeB*> *enodebs = GetNetworkManager ()->GetENodeBContainer ();
  assert(GetFrameStructure() == FrameManager::FRAME_STRUCTURE_FDD);
#ifdef FRAME_MANAGER_DEBUG
	std::cout << "Resource Allocation for eNB:";
  for (auto iter = enodebs->begin (); iter != enodebs->end (); iter++) {
	  ENodeB* enb = *iter;
    std::cout << " " << enb->GetIDNetworkNode();
	}
  std::cout << std::endl;
#endif

#ifdef SET_CENTRAL_SCHEDULER
  std::cout << "Central Downlink RBs Allocation!" << std::endl;
  CentralDownlinkRBsAllocation();
#else
  std::cout << "Original Per-Cell Downlink RBs Allocation!" << std::endl;
for (auto iter = enodebs->begin (); iter != enodebs->end (); iter++) {
	  ENodeB* enb = *iter;
    enb->DownlinkResourceBlockAllocation();
	}
#endif
  for (auto iter = enodebs->begin (); iter != enodebs->end (); iter++) {
	  ENodeB* enb = *iter;
    enb->UplinkResourceBlockAllocation();
	}
}

void
FrameManager::CentralDownlinkRBsAllocation(void)
{
  std::vector<ENodeB*> *enodebs =
    GetNetworkManager ()->GetENodeBContainer ();
  std::vector<DownlinkPacketScheduler*> schedulers;
  // initialization
  int counter = 0;
  for (auto it = enodebs->begin(); it != enodebs->end(); it++) {
    ENodeB* enb = *it;
    DownlinkPacketScheduler* scheduler =
      (DownlinkPacketScheduler*)enb->GetDLScheduler();
    assert(counter == enb->GetIDNetworkNode());
    schedulers.push_back(scheduler);
    counter += 1;
  }
  // set up schedulers
  bool available_flow = false;
  for (size_t j = 0; j < schedulers.size(); j++) {
    schedulers[j]->UpdateAverageTransmissionRate();
    schedulers[j]->SelectFlowsToSchedule();
    if (schedulers[j]->GetFlowsToSchedule()->size() > 0) {
      available_flow = true;
    }
  }
  // if no flow is available in any slice, stop schedulers
  if (!available_flow) {
    for (size_t j = 0; j < schedulers.size(); j++)
      schedulers[j]->StopSchedule();
    return;
  }
  // Assume that every eNB has same number of RBs
  int nb_of_rbs = schedulers[0]->GetMacEntity()->GetDevice()->GetPhy()
      ->GetBandwidthManager()->GetDlSubChannels().size ();
  // int nb_of_rbs = no_of_schedulers * 4;
  // cout << "Number of RBS: " << nb_of_rbs << endl;
  // bool enable_comp = schedulers[0]->enable_comp_;

  if (schedulers[0]->enable_tune_weights_) {
    ReassignUserToSlice(schedulers);
    TuneWeightsAcrossCells(schedulers);
    schedulers[0]->enable_tune_weights_ = false;
  }

  if (schedulers[0]->enable_sliceserve_) {
    CalcSliceServedQuota(schedulers);
    int rb_id = 0;
    while (rb_id < nb_of_rbs) {
      SliceServedAllocateOneRBExhaustMute(schedulers, rb_id);
      // SliceServedAllocateOneRBHeuristicMute(schedulers, rb_id);
      rb_id += RBG_SIZE;
    }
  }
  else {
    for (auto it = schedulers.begin(); it != schedulers.end(); it++) {
      RadioSaberDownlinkScheduler* scheduler = (RadioSaberDownlinkScheduler*)(*it);
      scheduler->CalculateSliceQuota();
      if (schedulers[0]->current_tti_cost_) {
        scheduler->CalculateMetricsPerRB();
      }
    }
    int rb_id = 0;
    std::vector<Allocation> allocations;
    while (rb_id < nb_of_rbs) { // Allocate in the form of PRBG of size 4
      if (!schedulers[0]->enable_comp_){
        SingleObjMutingExhaust(schedulers, rb_id);
      }
      else if (schedulers[0]->muting_system_ == 0) {
        SingleObjMutingExhaust(schedulers, rb_id);
      }
      else if (schedulers[0]->muting_system_ == 1) {
        SingleObjMutingOneCell(schedulers, rb_id);
      }
      else if (schedulers[0]->muting_system_ == 2) {
        MultiObjMutingOld(schedulers, rb_id);
      }
      else if (schedulers[0]->muting_system_ == 3){
        allocations = MultiObjMuting(schedulers, rb_id);
        Allocation allocation_no_mute = allocations[0];
        Allocation allocation_mute = allocations[1];
        for (auto &t: allocation_mute.per_cell_metric_map){
          int cell_id = t.first;
          double metric = t.second;
          if (allocation_mute.success){
            assert(allocation_mute.per_cell_slice_map[cell_id] == allocation_no_mute.per_cell_slice_map[cell_id]);
          }
          cout << "ALM: " << "TI: " << GetTTICounter() << " RB: " << rb_id 
          << " CellID: " << cell_id 
          << " SliceID: " << allocation_mute.per_cell_slice_map[cell_id]
          << " Mute: " << allocation_mute.per_cell_metric_map[cell_id] 
          << " No_Mute: " << allocation_no_mute.per_cell_metric_map[cell_id]
          << " Per_Cell_Imp: " << (allocation_mute.per_cell_metric_map[cell_id] - allocation_no_mute.per_cell_metric_map[cell_id])
           / allocation_no_mute.per_cell_metric_map[cell_id]
          << " Overall_Imp: " << allocation_mute.percentage_improvement
          << " Muting_Success: " << allocation_mute.success << endl;
        }
      }
      else {
        SingleObjMutingExhaust(schedulers, rb_id);
      }
      rb_id += RBG_SIZE;
    }
  }

  std::map<int, double> slice_rollover_quotas;
  for (size_t j = 0; j < schedulers.size(); j++) {
    RadioSaberDownlinkScheduler* rs_scheduler = (RadioSaberDownlinkScheduler*) schedulers[j];
    for (int k = 0; k < schedulers[j]->slice_ctx_.num_slices_; k++){
      rs_scheduler->slice_rbgs_quota_[k] = round(rs_scheduler->slice_rbgs_quota_[k] * 100000.) / 100000.;
      cout << "Rolling over " << rs_scheduler->slice_rbgs_quota_[k] << " for slice " << k << endl;
      slice_rollover_quotas[k] += rs_scheduler->slice_rbgs_quota_[k];
      rs_scheduler->rollover_slice_quota_[k] = rs_scheduler->slice_rbgs_quota_[k];
    }
  }
  for (size_t j = 0; j < slice_rollover_quotas.size(); j++) {
    cout << "TI: " << GetTTICounter() << " Slice " << j << " has TOTAL " << slice_rollover_quotas[j] << " rollover quota" << endl;
  }
  for (size_t j = 0; j < schedulers.size(); j++) {
    schedulers[j]->FinalizeScheduledFlows();
    schedulers[j]->StopSchedule();
  }
}

inline int provision_metric(double weight, double ideal_weight) {
  double weight_diff = weight - ideal_weight;
  double delta = 0.01;
  if (abs(weight_diff) < delta) {
      return 0;
  }
  if (weight_diff >= delta) {
    return 1;
  }
  else {
    return -1;
  }
}

inline bool within_range(double v) {
  return v >= 0 && v <= 1;
}

void
FrameManager::TuneWeightsAcrossCells(std::vector<DownlinkPacketScheduler*>& schedulers)
{
  std::vector<std::vector<int>> slice_users_per_cell;
  std::vector<std::vector<double>> ideal_weight_per_cell;
  auto& slice_weight = schedulers[0]->slice_ctx_.weights_;
  std::vector<int> slice_users(slice_weight.size(), 0);
  assert((int)slice_weight.size() == schedulers[0]->slice_ctx_.num_slices_);
  // calculate ideal weights of slices in every cell
  for (size_t j = 0; j < schedulers.size(); j++) {
    slice_users_per_cell.emplace_back(slice_weight.size(), 0);
    ideal_weight_per_cell.emplace_back(slice_weight.size(), 0);
    auto flows = schedulers[j]->GetFlowsToSchedule();
    for (auto it = flows->begin(); it != flows->end(); it++) {
      FlowToSchedule* flow = *it;
      int slice_id = flow->GetSliceID();
      slice_users_per_cell[j][slice_id] += 1;
      slice_users[slice_id] += 1;
    }
  }
  for (size_t j = 0; j < schedulers.size(); j++) {
    for (size_t i = 0; i < slice_weight.size(); i++) {
      ideal_weight_per_cell[j][i] = (double)slice_users_per_cell[j][i] / slice_users[i]
        * slice_weight[i] * schedulers.size();
    }
  }
  const double delta = 0.01;
  // 1 indicates over provision; -1 indicates under-provision
  std::vector<std::vector<int>> provision_matrix;
  for (size_t cell_id = 0; cell_id < schedulers.size(); cell_id++) {
    provision_matrix.emplace_back();
    for (size_t slice_id = 0; slice_id < slice_weight.size(); slice_id++) {
      double metric = provision_metric(
          schedulers[cell_id]->slice_ctx_.weights_[slice_id],
          ideal_weight_per_cell[cell_id][slice_id]);
      provision_matrix[cell_id].push_back(metric);
    }
  }
  
  /*
  // simulated annealing
  int counter = 0;
  double temp = 1;
  const double cooling_alpha = 0.99;
  std::random_device rd;
  std::mt19937 e2(rd());
  std::uniform_real_distribution<> distribution(0, 1);
  while (counter < 2000) {
    int lcell = rand() % schedulers.size();
    int rcell = rand() % schedulers.size();
    while (rcell == lcell) rcell = rand() % schedulers.size();
    int lslice = rand() % slice_weight.size();
    int rslice = rand() % slice_weight.size();
    while (rslice == lslice) rslice = rand() % slice_weight.size();
    if (provision_matrix[lcell][lslice] == 0)
      continue;
    int over = provision_matrix[lcell][lslice];
    double sum_diff = 0;
    for (int cell_id = 0; cell_id < schedulers.size(); cell_id++) {
      for (int slice_id = 0; slice_id < slice_weight.size(); slice_id++) {
        sum_diff += pow(ideal_weight_per_cell[cell_id][slice_id] - 
          schedulers[cell_id]->slice_ctx_.weights_[slice_id], 2);
      }
    }
    double lc_ls = schedulers[lcell]->slice_ctx_.weights_[lslice];
    double lc_rs = schedulers[lcell]->slice_ctx_.weights_[rslice];
    double rc_ls = schedulers[rcell]->slice_ctx_.weights_[lslice];
    double rc_rs = schedulers[rcell]->slice_ctx_.weights_[rslice];
    double origin_loss = 
        pow(lc_ls - ideal_weight_per_cell[lcell][lslice], 2)
      + pow(lc_rs - ideal_weight_per_cell[lcell][rslice], 2)
      + pow(rc_ls - ideal_weight_per_cell[rcell][lslice], 2)
      + pow(rc_rs - ideal_weight_per_cell[rcell][rslice], 2);
    double new_loss = 
        pow(lc_ls - ideal_weight_per_cell[lcell][lslice] - over * delta, 2)
      + pow(lc_rs - ideal_weight_per_cell[lcell][rslice] + over * delta, 2)
      + pow(rc_ls - ideal_weight_per_cell[rcell][lslice] + over * delta, 2)
      + pow(rc_rs - ideal_weight_per_cell[rcell][rslice] - over * delta, 2);
    bool inrange = within_range(lc_ls - over*delta) 
      && within_range(lc_rs + over * delta) && within_range(rc_ls + over * delta)
      && within_range(rc_rs - over * delta);
    double delta_loss = new_loss - origin_loss;
    std::cout << counter << " temp: " << temp << " delta: " << delta_loss
      << " total_loss: " << sum_diff << std::endl;
    if ( inrange && (delta_loss < 0 || distribution(e2) < exp(-delta_loss / temp)) ) {
      std::vector<int> cells;
      cells.push_back(lcell);
      cells.push_back(rcell);
      std::vector<int> slices;
      slices.push_back(lslice);
      slices.push_back(rslice);
      for (auto it = cells.begin(); it != cells.end(); it++) {
        for (auto it_s = slices.begin(); it_s != slices.end(); it_s++) {
          int cell_id = *it;
          int slice_id = *it_s;
          std::cout << "(" << schedulers[cell_id]->slice_ctx_.weights_[slice_id]
            << ", " << ideal_weight_per_cell[cell_id][slice_id]
            << ")" << "; ";  
        }
      }
      std::cout << std::endl;
      schedulers[lcell]->slice_ctx_.weights_[lslice] -= over * delta;
      schedulers[lcell]->slice_ctx_.weights_[rslice] += over * delta;
      schedulers[rcell]->slice_ctx_.weights_[lslice] += over * delta;
      schedulers[rcell]->slice_ctx_.weights_[rslice] -= over * delta;
      provision_matrix[lcell][lslice] = provision_metric(
        schedulers[lcell]->slice_ctx_.weights_[lslice], ideal_weight_per_cell[lcell][lslice]);
      provision_matrix[lcell][rslice] = provision_metric(
        schedulers[lcell]->slice_ctx_.weights_[rslice], ideal_weight_per_cell[lcell][rslice]);
      provision_matrix[rcell][lslice] = provision_metric(
        schedulers[rcell]->slice_ctx_.weights_[lslice], ideal_weight_per_cell[rcell][lslice]);
      provision_matrix[rcell][rslice] = provision_metric(
        schedulers[rcell]->slice_ctx_.weights_[rslice], ideal_weight_per_cell[rcell][rslice]);

      for (auto it = cells.begin(); it != cells.end(); it++) {
        for (auto it_s = slices.begin(); it_s != slices.end(); it_s++) {
          int cell_id = *it;
          int slice_id = *it_s;
          std::cout << "(" << schedulers[cell_id]->slice_ctx_.weights_[slice_id]
            << ", " << ideal_weight_per_cell[cell_id][slice_id]
            << ")" << "; ";  
        }
      }
      std::cout << std::endl;
    }
    temp *= cooling_alpha;
    counter += 1;
  }
  double sum_diff = 0;
  for (int cell_id = 0; cell_id < schedulers.size(); cell_id++) {
    std::cout << "cell " << cell_id << ": ";
    for (int slice_id = 0; slice_id < slice_weight.size(); slice_id++) {
      std::cout << "(" << schedulers[cell_id]->slice_ctx_.weights_[slice_id]
        << ", " << provision_matrix[cell_id][slice_id]
        << ", " << ideal_weight_per_cell[cell_id][slice_id]
        << ")" << "; ";
      sum_diff += pow(ideal_weight_per_cell[cell_id][slice_id] - 
        schedulers[cell_id]->slice_ctx_.weights_[slice_id], 2);
    }
    std::cout << std::endl;
  }
  std::cout << "total_loss: " << sum_diff << std::endl;
  */
  // initial greedy algorithm: we keep reallocating when there's a chance
  int counter = 0;
  while (true) {
    bool swap_once = false;
    std::cout << "swap_iteration: " << counter << std::endl;
    counter += 1;
    for (size_t lcell = 0; lcell < schedulers.size(); lcell++) {
      for (size_t lslice = 0; lslice < slice_weight.size(); lslice++) {
        for (size_t rslice = 0; rslice < slice_weight.size(); rslice++) {
          if (lslice == rslice) continue;
          for (size_t rcell = 0; rcell < schedulers.size(); rcell++) {
            if (lcell == rcell) continue;
            if (provision_matrix[lcell][lslice] * provision_matrix[lcell][rslice] == -1) {
              // both provision or under-provision for both slices across two cells, skip
              if (provision_matrix[lcell][lslice] * provision_matrix[rcell][lslice] == 1
                && provision_matrix[lcell][rslice] * provision_matrix[rcell][rslice] == 1)
                continue;
              // if any 0 is achieved in rcell, skip
              if (provision_matrix[rcell][lslice] * provision_matrix[rcell][rslice] == 0)
                continue;
              int over = provision_matrix[lcell][lslice];
              schedulers[lcell]->slice_ctx_.weights_[lslice] -= over * delta;
              schedulers[rcell]->slice_ctx_.weights_[lslice] += over * delta;
              schedulers[lcell]->slice_ctx_.weights_[rslice] += over * delta;
              schedulers[rcell]->slice_ctx_.weights_[rslice] -= over * delta;
              swap_once = true;
              provision_matrix[lcell][lslice] = provision_metric(
                schedulers[lcell]->slice_ctx_.weights_[lslice],
                ideal_weight_per_cell[lcell][lslice]);
              provision_matrix[lcell][rslice] = provision_metric(
                schedulers[lcell]->slice_ctx_.weights_[rslice],
                ideal_weight_per_cell[lcell][rslice]);
              provision_matrix[rcell][lslice] = provision_metric(
                schedulers[rcell]->slice_ctx_.weights_[lslice],
                ideal_weight_per_cell[rcell][lslice]);
              provision_matrix[rcell][rslice] = provision_metric(
                schedulers[rcell]->slice_ctx_.weights_[rslice],
                ideal_weight_per_cell[rcell][rslice]);
            }
          }
        }
      }
    }
    if (!swap_once) break;
  }
  double sum_diff = 0;
  for (size_t cell_id = 0; cell_id < schedulers.size(); cell_id++) {
    std::cout << "cell " << cell_id << ": ";
    for (size_t slice_id = 0; slice_id < slice_weight.size(); slice_id++) {
      std::cout << "(" << schedulers[cell_id]->slice_ctx_.weights_[slice_id]
        << ", " << provision_matrix[cell_id][slice_id]
        << ", " << ideal_weight_per_cell[cell_id][slice_id]
        << ", " << slice_users_per_cell[cell_id][slice_id]
        << ")" << "; ";
      sum_diff += pow(ideal_weight_per_cell[cell_id][slice_id] - 
        schedulers[cell_id]->slice_ctx_.weights_[slice_id], 2);
    }
    std::cout << std::endl;
  }
  std::cout << "total_loss: " << sum_diff << std::endl;
}

void
FrameManager::ReassignUserToSlice(std::vector<DownlinkPacketScheduler*>& schedulers)
{
  for (size_t j = 0; j < schedulers.size(); j++) {
    int num_slices = schedulers[j]->slice_ctx_.num_slices_;
    auto flows = schedulers[j]->GetFlowsToSchedule();
    std::cout << "cell: " << j << " n_flows: " << flows->size() << std::endl;
    int slice_id = 0;
    for (auto it = flows->begin(); it != flows->end(); it++) {
      FlowToSchedule* flow = *it;
      flow->SetSliceID(slice_id % num_slices);
      flow->GetBearer()->GetDestination()->SetSliceID(slice_id % num_slices);
      slice_id += 1;
    }
  }
}

// void
// FrameManager::ReassignUserToSlice(std::vector<DownlinkPacketScheduler*>& schedulers)
// {
//   for (size_t j = 0; j < schedulers.size(); j++) {
//     int num_slices = schedulers[j]->slice_ctx_.num_slices_;
//     auto flows = schedulers[j]->GetFlowsToSchedule();
//     std::cout << "cell: " << j << " n_flows: " << flows->size() << std::endl;
//     for (size_t i = 0; i < flows->size(); i++) {
//       double w = (double)i / flows->size();
//       FlowToSchedule* flow = flows->at(i);
//       if (w >= 0 && w < 0.25) {
//         if (j % 2 == 0)
//           flow->SetSliceID(rand() % 4);
//         else
//           flow->SetSliceID(rand() % 4 + 4);
//       }
//       else if (w >= 0.25 && w < 1) {
//         if (j % 2 == 0)
//           flow->SetSliceID(rand() % 4 + 4);
//         else
//           flow->SetSliceID(rand() % 4);
//       }
//       flow->GetBearer()->GetDestination()->SetSliceID(
//         flow->GetSliceID());
//     }
//   }
// }

void FrameManager::UpdateNoiseInterferenceWithMute(std::vector<DownlinkPacketScheduler*>& schedulers,
  int rb_id, int mute_id) {
  if (mute_id == -1)
    return;
  for (int j = 0; j < (int)schedulers.size(); j++) {
    if (j != mute_id) {
      FlowsToSchedule* flows = schedulers[j]->GetFlowsToSchedule();
      for (auto it = flows->begin(); it != flows->end(); it++) {
        auto& rsrp_report = (*it)->GetRSRPReport();
        for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
            rsrp_report.noise_interfere_watt[i] -= pow(10., rsrp_report.rsrp_interference[mute_id][i]/10.);
        }
      }
    }
  }
}

Allocation FrameManager::DoAllocation (std::vector<DownlinkPacketScheduler*>& schedulers,
  int rb_id, int mute_id, bool mute_cell, std::map<int, int> slice_map) {
    std::vector<FlowToSchedule*> cell_flow (schedulers.size(), nullptr);
    AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
    std::map<int, int> per_cell_slice_map; 
    std::map<int, double> per_cell_metric_map;
    std::map<int, double> per_slice_metric_map; 
    double sum_metric = 0;
    double sum_tbs = 0;
    for (size_t i = 0; i < schedulers.size(); i++) {
      if(i == mute_id){
        continue;
      }
      RadioSaberDownlinkScheduler* scheduler = (RadioSaberDownlinkScheduler*)schedulers[i];
      FlowsToSchedule* flows = scheduler->GetFlowsToSchedule();
      int num_slice = scheduler->slice_ctx_.num_slices_;
      std::vector<FlowToSchedule*> slice_flow(num_slice, nullptr);
      std::vector<double> slice_spectraleff(num_slice, 0);
      std::vector<double> max_metrics(num_slice, 0);
      for (auto it = flows->begin(); it != flows->end(); it++) {
        FlowToSchedule* flow = *it;
        auto& rsrp_report = flow->GetRSRPReport();
        double spectraleff_rbg = 0.0;
        if (mute_cell)
        {
          assert(mute_id != -1);
          for (int j = rb_id; j < RBG_SIZE + rb_id; j++) {
            double in_under_mute = rsrp_report.noise_interfere_watt[j];
            in_under_mute -= pow(10.0, rsrp_report.rsrp_interference[mute_id][j]/10.);
            int cqi_under_mute = amc->GetCQIFromSinr(
                rsrp_report.rx_power[j] - 10. * log10(in_under_mute));
            spectraleff_rbg += amc->GetEfficiencyFromCQI(cqi_under_mute);
          }
        }
        else {
          for (int j = rb_id; j < RBG_SIZE+rb_id; j++) {
            double int_noise = rsrp_report.noise_interfere_watt[j];
            int cqi = amc->GetCQIFromSinr(rsrp_report.rx_power[j] - 10. * log10(int_noise));
            flow->GetCqiFeedbacks()[j] = cqi; // **
            spectraleff_rbg += amc->GetEfficiencyFromCQI(cqi); // This returns TBS
          }
        }

        double metric = scheduler->ComputeSchedulingMetric(
            flow->GetBearer(), spectraleff_rbg, rb_id);
        int slice_id = flow->GetSliceID();
        if (metric > max_metrics[slice_id]) {
          max_metrics[slice_id] = metric;
          slice_flow[slice_id] = flow;
          slice_spectraleff[slice_id] = spectraleff_rbg;
        }
      }
      double max_slice_spectraleff = 0;
      double max_metric = 0;
      FlowToSchedule* selected_flow = nullptr;
      if (slice_map.find(i) == slice_map.end()){
        for (int j = 0; j < num_slice; j++) {
          if (slice_spectraleff[j] > max_slice_spectraleff 
              && scheduler->slice_rbgs_quota_[j] >0) {
            max_slice_spectraleff = slice_spectraleff[j];
            max_metric = max_metrics[j];
            selected_flow = slice_flow[j];
          }
        }
      } else {
        max_slice_spectraleff = slice_spectraleff[slice_map[i]];
        max_metric = max_metrics[slice_map[i]];
        selected_flow = slice_flow[slice_map[i]];
      }

      assert(selected_flow);
      cell_flow[i] = selected_flow;
      int slice_policy = schedulers[i]->slice_ctx_.algo_params_[selected_flow->GetSliceID()].psi;
      if (slice_policy == 1){ // PF
        per_cell_metric_map[i] = max_metric;
      } else {
        per_cell_metric_map[i] = max_slice_spectraleff;
      }
      sum_metric += max_metric;
      sum_tbs += max_slice_spectraleff;
    }

    for (int i = 0; i < cell_flow.size(); i++){
      if (cell_flow[i] != nullptr){
        per_cell_slice_map[i] = cell_flow[i]->GetSliceID();
        per_slice_metric_map[cell_flow[i]->GetSliceID()] += per_cell_metric_map[i];
      }
    }
    if (slice_map.size() > 0){
      for (auto &t: per_cell_metric_map){
        int cell_id = t.first;
        double metric = t.second;
        assert(per_cell_slice_map[cell_id] == slice_map[cell_id]);
      }
    }


  Allocation result;
  result.cell_flow = cell_flow;
  result.sum_metric = sum_metric;
  result.sum_tbs = sum_tbs;
  result.per_cell_metric_map = per_cell_metric_map;
  result.per_cell_slice_map = per_cell_slice_map;
  result.per_slice_metric_map = per_slice_metric_map;
  return result;
}



void
FrameManager::CalcSliceServedQuota(std::vector<DownlinkPacketScheduler*>& schedulers)
{
  // assume only the macrocell(0th) and other cells are interfered
  const int num_slices = schedulers[0]->slice_ctx_.num_slices_;
  std::vector<std::pair<int, double> > linking_index(num_slices, {0, 0});
  for (int i = 0; i < num_slices; i++) {
    double total_index = 0;
    double macro_weight = schedulers[0]->slice_ctx_.weights_[i];
    for (size_t j = 1; j < schedulers.size(); j++) {
      double small_weight = schedulers[j]->slice_ctx_.weights_[i];
      total_index += ( macro_weight > small_weight ? small_weight : macro_weight );
    }
    linking_index[i].first = i;
    linking_index[i].second = total_index;
  }
  sort(linking_index.begin(), linking_index.end(),
    [](const auto& a, const auto& b) -> bool {
      return a.second > b.second;
  });
  int nb_of_rbs = schedulers[0]->GetMacEntity()->GetDevice()->GetPhy()
      ->GetBandwidthManager()->GetDlSubChannels().size ();
  for (size_t i = 0; i < schedulers.size(); i++) {
    schedulers[i]->rbs_to_slice_.resize(nb_of_rbs, 0);
  }
  std::vector<int> cells_begin_id(schedulers.size(), 0);
  for (size_t i = 0; i != linking_index.size(); i++) {
    int sid = linking_index[i].first;
    for (size_t cid = 0; cid != schedulers.size(); cid++) {
      DownlinkPacketScheduler* scheduler = schedulers[cid];
      int slice_rbs = (int)((double)nb_of_rbs * scheduler->slice_ctx_.weights_[sid]);
      // assgin rbs to slices
      for (int i = 0; i < slice_rbs; i++) {
        scheduler->rbs_to_slice_[cells_begin_id[cid] + i] = sid;
      }
      cells_begin_id[cid] += slice_rbs;
      if (i == ( linking_index.size() - 1) ) {
        // The quota RBs of every slice can be fractional, round it up in the last slice
        for (int i = cells_begin_id[cid]; i< nb_of_rbs; i++) {
          scheduler->rbs_to_slice_[i] = sid;
        }
      }
    }
  }
}

void
FrameManager::SliceServedAllocateOneRBExhaustMute(
  std::vector<DownlinkPacketScheduler*>& schedulers,
  int rb_id) {
  const int slice_serve = schedulers[0]->rbs_to_slice_[rb_id];
  bool enable_comp = true;
  if (!schedulers[0]->enable_comp_) {
    enable_comp = false;
  }
  for (auto it = schedulers.begin(); it != schedulers.end(); it++) {
    if ((*it)->rbs_to_slice_[rb_id] != slice_serve) {
      enable_comp = false;
    }
  }
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  std::unordered_set<int> global_cells_muted;
  std::vector<FlowToSchedule*> global_cell_flow;
  const int id_no_mute = schedulers.size();
  double tbs_weight = schedulers[0]->max_tp_weight_;
  double pf_metric_weight = 1 - tbs_weight;
  while (true) {
    int final_mute_id = -1;
    int mute_id;
    if (enable_comp) {
      mute_id = 0;
    } else {
      mute_id = id_no_mute;
    }
    std::vector<int> mute_ids;
    std::vector<double> sum_tbs_under_mute;
    std::vector<double> sum_metric_under_mute;
    std::vector<std::vector<FlowToSchedule*>> cell_flow_under_mute;
    for (; mute_id <= (int)schedulers.size(); mute_id++) {
      if (global_cells_muted.count(mute_id)) {
        continue;
      }
      // the aggregate tbs or pf-metric across cells
      double sum_tbs = 0;
      double sum_metric = 0;
      std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
      for (size_t i = 0; i < schedulers.size(); i++) {
        int cell_id = i;
        RadioSaberDownlinkScheduler* scheduler =
          (RadioSaberDownlinkScheduler*) schedulers[cell_id];
        int cell_slice = scheduler->rbs_to_slice_[rb_id];
        if (cell_id == mute_id || global_cells_muted.count(cell_id)) {
          continue;
        }
        FlowsToSchedule* flows = scheduler->GetFlowsToSchedule();
        double max_metric = -1;
        double flow_tbs = -1;
        FlowToSchedule* chosen_flow = nullptr;
        for (auto it = flows->begin(); it != flows->end(); it++) {
          FlowToSchedule* flow = *it;
          // exclusively for this slice, no inter-slice scheduler
          if (flow->GetSliceID() != cell_slice)
            continue;
          auto& rsrp_report = flow->GetRSRPReport();
          double tbs = 0.0;
          for (int i = rb_id; i < RBG_SIZE + rb_id; i++) {
            double in_under_mute = rsrp_report.noise_interfere_watt[i];
            if (mute_id != id_no_mute) {
              in_under_mute -= pow(10.0, rsrp_report.rsrp_interference[mute_id][i]/10.);
            }
            int cqi_under_mute = amc->GetCQIFromSinr(
              rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
            tbs += amc->GetEfficiencyFromCQI(cqi_under_mute);
          }
          double metric = scheduler->ComputeSchedulingMetric(
            flow->GetBearer(), tbs, rb_id);
          if (metric > max_metric) {
            max_metric = metric;
            flow_tbs = tbs;
            chosen_flow = flow;
          }
        }
        if (chosen_flow) {
          cell_flow[cell_id] = chosen_flow;
          sum_tbs += flow_tbs;
          sum_metric += max_metric;
        }
        else {
          std::cout << "Warn: slice " << cell_slice
            << " has no user in cell " << cell_id << std::endl;
        }
      }
      mute_ids.push_back(mute_id);
      sum_tbs_under_mute.push_back(sum_tbs);
      sum_metric_under_mute.push_back(sum_metric);
      cell_flow_under_mute.push_back(cell_flow);
    }
    auto it = find(mute_ids.begin(), mute_ids.end(), id_no_mute);
    int no_mute_index = it - mute_ids.begin();
    double no_mute_tbs = sum_tbs_under_mute[no_mute_index];
    double no_mute_metric = sum_metric_under_mute[no_mute_index];
    // Calculate the percentage improvement over the no muting scenario
    std::vector<double> percentage_improvements;
    for (size_t i = 0; i < mute_ids.size(); i++){
      double tbs_percentage_improvement =
        (double)(sum_tbs_under_mute[i] - no_mute_tbs) / no_mute_tbs;
      double pf_percentage_improvement =
        (double)(sum_metric_under_mute[i] - no_mute_metric) / no_mute_metric;
      percentage_improvements.push_back(
        (tbs_weight * tbs_percentage_improvement) + (pf_metric_weight * pf_percentage_improvement));
    }
    double max = *max_element(percentage_improvements.begin(), percentage_improvements.end());
    auto it_2 = find(percentage_improvements.begin(), percentage_improvements.end(), max);
    int id = it_2 - percentage_improvements.begin();
    final_mute_id = mute_ids[id];
    if (final_mute_id == id_no_mute) {
      global_cell_flow = cell_flow_under_mute[id];
      break;
    }
    else {
      global_cell_flow = cell_flow_under_mute[id];
      global_cells_muted.insert(final_mute_id);
      UpdateNoiseInterferenceWithMute(schedulers, rb_id, final_mute_id);
    }
  }
  for (size_t j = 0; j < schedulers.size(); j++) {
    if (global_cells_muted.count(j)) {
      continue;
    }
    RadioSaberDownlinkScheduler* scheduler =
      (RadioSaberDownlinkScheduler*)schedulers[j];
    FlowToSchedule* flow = global_cell_flow[j];
    if (!flow) {
      continue;
    }
    auto& rsrp_report = flow->GetRSRPReport();
    // under the current muting assumption
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
      double in_under_mute = rsrp_report.noise_interfere_watt[i];
      int cqi_under_mute = amc->GetCQIFromSinr(
        rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
      flow->GetCqiFeedbacks()[i] = cqi_under_mute;
    }
    scheduler->slice_rbgs_quota_[flow->GetSliceID()] -= 1;
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++){
      flow->GetListOfAllocatedRBs()->push_back(i);
    }
  }
}

std::vector<int> GetHeuristicMutingOrder(
  std::vector<FlowToSchedule*> cell_flow, int rb_id, bool consider_all_cells){
    std::map<int, double> user_int_levels;
    for(int i = 0; i < cell_flow.size(); i++){
      double interference = cell_flow[i]->GetRSRPReport().noise_interfere_watt[rb_id];
      user_int_levels.emplace(i, interference);
    }

    // for (const auto &pair : user_int_levels) {
    //   std::cout << "User ID: " << pair.first << " Int Level: " << pair.second << "\n";
    // }

    std::vector<std::pair<int, double>> vectorPairs(user_int_levels.begin(), user_int_levels.end());
    std::sort(vectorPairs.begin(), vectorPairs.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.second > rhs.second;
    });

    std::vector<int> sorted_user_int_levels;
    for (const auto &pair : vectorPairs) {
        sorted_user_int_levels.push_back(pair.first);
    }

    // for (const auto &pair : sorted_user_int_levels) {
    //   std::cout << "User ID: " << pair << "\n";
    // }

    // for (int i = 0; i < cell_flow.size(); i++) {
    //   std::cout << "User ID: " << i << endl;
    //   auto &t = cell_flow[i]->GetRSRPReport().rsrp_interference;
    //   for (int j = 0; j < cell_flow.size(); j++){
    //     cout <<  "Neighbor Cell ID: " << j << " Interference: " << t[j][rb_id] << endl;
    //   }
    // }

    std::vector<int> cell_muting_order;
    int next_highest_interferer = 0;
    while (cell_muting_order.size() < cell_flow.size()){
      for (int i = 0; i < sorted_user_int_levels.size(); i++){
        std::map<int, std::vector<double>> rsrp_interference = 
          cell_flow[sorted_user_int_levels[i]]->GetRSRPReport().rsrp_interference;

        std::map<int, double> interference_vals_for_this_rb;
        for (int j = 0; j < cell_flow.size(); j++){
          if (sorted_user_int_levels[i] != j){ // Self interference 
            interference_vals_for_this_rb.emplace(j, rsrp_interference[j][rb_id]);
          }
        }
        std::vector<std::pair<int, double>> sorted_interfering_neighbors
          (interference_vals_for_this_rb.begin(), interference_vals_for_this_rb.end());
        std::sort(sorted_interfering_neighbors.begin(), 
          sorted_interfering_neighbors.end(), [](const auto &lhs, const auto &rhs) {
            return lhs.second > rhs.second;
        });

        std::pair<int, double> highest_pair = sorted_interfering_neighbors[next_highest_interferer];

        auto it = std::find(cell_muting_order.begin(), cell_muting_order.end(), highest_pair.first);
        if (it == cell_muting_order.end()){
          cell_muting_order.push_back(highest_pair.first);
        }
      }
      if(!consider_all_cells){break;}
      next_highest_interferer++;
    }

    // cout << "Muting Order: " << endl;
    // for (int i = 0; i < cell_muting_order.size(); i++){
    //   cout << cell_muting_order[i] << "\t";
    // }
    // cout << endl;
    return cell_muting_order;
}

std::vector<int> GetMutingOrder(int num_cells, bool random){
  vector<int> cell_muting_order;
  for (int i = 0; i < num_cells; i++){
    cell_muting_order.push_back(i);
  }
  if (random){
    std::random_device rd;
    std::default_random_engine rng(1);
    std::shuffle(cell_muting_order.begin(), cell_muting_order.end(), rng);
  }
  return cell_muting_order;
}

int FindCellFacingHighestInterference(
  std::vector<FlowToSchedule*> cell_flow, int rb_id){
  int selected_cell = -1;
  int selected_aggressor_cell = -1;
  double max_rsrp_diff = (-std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < cell_flow.size(); i++){
    int cell_id = i;
    FlowToSchedule* flow = cell_flow[cell_id];
    auto& rsrp_interference = flow->GetRSRPReport().rsrp_interference;
    for (auto it = rsrp_interference.begin(); it != rsrp_interference.end(); it++) {
      int neighbor_cell_id = it->first;
      if (neighbor_cell_id != cell_id) {
        double rsrp_diff = it->second[rb_id] - flow->GetRSRPReport().rx_power[rb_id];
        if (max_rsrp_diff < rsrp_diff) {
          max_rsrp_diff = rsrp_diff;
          selected_cell = cell_id;
          selected_aggressor_cell = neighbor_cell_id;
        }
      }
    }
  }
  // cout << "Returning Aggressor Cell: " << selected_aggressor_cell << endl;
  return selected_aggressor_cell;
}

void
FrameManager::SliceServedAllocateOneRBHeuristicMute(
  std::vector<DownlinkPacketScheduler*>& schedulers,
  int rb_id) {
  const int slice_serve = schedulers[0]->rbs_to_slice_[rb_id];
  assert(schedulers[0]->enable_comp_);
  bool enable_comp = true;
  for (auto it = schedulers.begin(); it != schedulers.end(); it++) {
    if ((*it)->rbs_to_slice_[rb_id] != slice_serve) {
      enable_comp = false;
    }
  }
  // Get the adaptive modulation module for the eNB
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  std::unordered_set<int> global_cells_muted;
  std::vector<FlowToSchedule*> global_cell_flow;
  const int id_no_mute = schedulers.size();
  double tbs_weight = schedulers[0]->max_tp_weight_;
  double pf_metric_weight = 1 - tbs_weight;
  std::vector<int> mute_ids;
  std::vector<double> sum_tbs_under_mute;
  std::vector<double> sum_metric_under_mute;
  std::vector<std::vector<FlowToSchedule*>> cell_flow_under_mute;
  for (int mute_id = 0; mute_id <= (int)schedulers.size(); mute_id++) {
    // the aggregate tbs or pf-metric across cells
    double sum_tbs = 0;
    double sum_metric = 0;
    std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
    for (size_t i = 0; i < schedulers.size(); i++) {
      int cell_id = i;
      RadioSaberDownlinkScheduler* scheduler =
        (RadioSaberDownlinkScheduler*) schedulers[cell_id];
      int cell_slice = scheduler->rbs_to_slice_[rb_id];
      if (cell_id == mute_id || global_cells_muted.count(cell_id)) {
        continue;
      }
      FlowsToSchedule* flows = scheduler->GetFlowsToSchedule();
      double max_metric = -1;
      double flow_tbs = -1;
      FlowToSchedule* chosen_flow = nullptr;
      for (auto it = flows->begin(); it != flows->end(); it++) {
        FlowToSchedule* flow = *it;
        // exclusively for this slice, no inter-slice scheduler
        if (flow->GetSliceID() != cell_slice)
          continue;
        auto& rsrp_report = flow->GetRSRPReport();
        double tbs = 0.0;
        for (int i = rb_id; i < RBG_SIZE + rb_id; i++) {
          double in_under_mute = rsrp_report.noise_interfere_watt[i];
          if (mute_id != id_no_mute) {
            in_under_mute -= pow(10.0, rsrp_report.rsrp_interference[mute_id][i]/10.);
          }
          int cqi_under_mute = amc->GetCQIFromSinr(
              rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
          tbs += amc->GetEfficiencyFromCQI(cqi_under_mute);
        }
        double metric = scheduler->ComputeSchedulingMetric(
            flow->GetBearer(), tbs, rb_id);
        if (metric > max_metric) {
          max_metric = metric;
          flow_tbs = tbs;
          chosen_flow = flow;
        }
      }
      if (chosen_flow) {
        cell_flow[cell_id] = chosen_flow;
        sum_tbs += flow_tbs;
        sum_metric += max_metric;
      }
      else {
        std::cout << "Warn: slice " << cell_slice
          << " has no user in cell " << cell_id << std::endl;
      }
    }
    mute_ids.push_back(mute_id);
    sum_tbs_under_mute.push_back(sum_tbs);
    sum_metric_under_mute.push_back(sum_metric);
    cell_flow_under_mute.push_back(cell_flow);
  }
  double no_mute_tbs = sum_tbs_under_mute[id_no_mute];
  double no_mute_metric = sum_metric_under_mute[id_no_mute];
  std::vector<FlowToSchedule*> no_mute_cell_flow = cell_flow_under_mute[id_no_mute];
  int id_heuristic = FindCellFacingHighestInterference(no_mute_cell_flow, rb_id);
  double tbs_improve = (double)(sum_tbs_under_mute[id_heuristic] - no_mute_tbs) / no_mute_tbs;
  double pf_improve = (double)(sum_metric_under_mute[id_heuristic] - no_mute_metric) / no_mute_metric;
  double final_improve = tbs_weight * tbs_improve + pf_metric_weight * pf_improve;
  int final_mute_id = id_no_mute;
  if (final_improve > 0 && enable_comp) {
    final_mute_id = id_heuristic;
  }
  if (final_mute_id == id_no_mute) {
    global_cell_flow = cell_flow_under_mute[final_mute_id];
  }
  else {
    global_cell_flow = cell_flow_under_mute[final_mute_id];
    global_cells_muted.insert(final_mute_id);
    UpdateNoiseInterferenceWithMute(schedulers, rb_id, final_mute_id);
  }
  for (size_t j = 0; j < schedulers.size(); j++) {
    if (global_cells_muted.count(j)) {
      continue;
    }
    RadioSaberDownlinkScheduler* scheduler =
      (RadioSaberDownlinkScheduler*)schedulers[j];
    FlowToSchedule* flow = global_cell_flow[j];
    if (!flow) {
      continue;
    }
    auto& rsrp_report = flow->GetRSRPReport();
    // under the current muting assumption
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
      double in_under_mute = rsrp_report.noise_interfere_watt[i];
      int cqi_under_mute = amc->GetCQIFromSinr(
        rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
      flow->GetCqiFeedbacks()[i] = cqi_under_mute;
    }
    scheduler->slice_rbgs_quota_[flow->GetSliceID()] -= 1;
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++){
      flow->GetListOfAllocatedRBs()->push_back(i);
    }
  }
}

void FrameManager::MultiObjMutingOld(
  std::vector<DownlinkPacketScheduler*>& schedulers,
    int rb_id){
  assert(schedulers[0]->enable_comp_);
  // Get the adaptive modulation module of the eNB
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  std::map<int, double> per_cell_metric;
  std::map<int, double> per_cell_metric_under_mute;
  std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
  bool current_tti_cost = schedulers[0]->current_tti_cost_;
  bool do_realloc = schedulers[0]->do_reallocation_;
  bool consider_all_cells_for_int = schedulers[0]->consider_all_cells_for_int_;

  Allocation initial_allocation = DoAllocation(schedulers, rb_id);
  per_cell_metric = initial_allocation.per_cell_metric_map;
  cell_flow = initial_allocation.cell_flow;
  bool mute_aggressor = false;
  int aggressor_cell = -1;
  int muted_flow_slice_id = -1;
  std::map<int, FlowToSchedule*> flows_benefit;
  const int num_slice = schedulers[0]->slice_ctx_.num_slices_;
  double per_slice_quota_deduction;
  RadioSaberDownlinkScheduler* muted_cell = nullptr;
  if (GetTTICounter() >= 150){
    std::vector<int> aggressor_cells = GetMutingOrder(schedulers.size(), true);
    // std::vector<int> aggressor_cells = GetHeuristicMutingOrder(cell_flow, rb_id, false);
    for (int i = 0; i < aggressor_cells.size(); i++){
      aggressor_cell = aggressor_cells[i];
      for (int i = 0; i < (int)schedulers.size(); i++){
        double spectraleff_rbg_under_mute = 0.0;
        if (i != aggressor_cell){
          RSRPReport rsrp_report = cell_flow[i]->GetRSRPReport();
          for (int j = rb_id; j < RBG_SIZE+rb_id; j++) {
            double in_under_mute = rsrp_report.noise_interfere_watt[j];
            in_under_mute -= pow(10., rsrp_report.rsrp_interference[aggressor_cell][j]/10.);
            int cqi_under_mute = amc->GetCQIFromSinr(
              rsrp_report.rx_power[rb_id] - 10. * log10(in_under_mute));
            spectraleff_rbg_under_mute += amc->GetEfficiencyFromCQI(cqi_under_mute); 
          }
          double metric_under_mute = schedulers[i]->ComputeSchedulingMetric(
            cell_flow[i]->GetBearer(), spectraleff_rbg_under_mute, rb_id);
          per_cell_metric_under_mute[i] =  metric_under_mute;
          // std::cerr << "cell: " << i << " slice: " << cell_flow[i]->GetSliceID()
          //   << " metric_under_mute: " << per_cell_metric_under_mute[i]
          //   << " metric_no_mute: " << per_cell_metric[i]
          //   << std::endl;
        }
      }
      // Find the slice&flow that benefits from this muting
      for (int i = 0; i < (int)schedulers.size(); i++){
        if (per_cell_metric_under_mute[i] > per_cell_metric[i]
            && i != aggressor_cell){
          flows_benefit[i] = cell_flow[i];
        }
      }
      muted_cell = (RadioSaberDownlinkScheduler*)schedulers[aggressor_cell];
      int muted_slice_id = cell_flow[aggressor_cell]->GetSliceID();
      // For multi-utility muting, this trade-off approach uses history allocation
      if (current_tti_cost) {
        while(flows_benefit.size() >= 1){
          per_slice_quota_deduction = 1.0 / flows_benefit.size();
          for (const auto& pair : flows_benefit) {
            // mute_aggressor is always initialized to false for every flow
            mute_aggressor = false;
            int slice_id = pair.second->GetSliceID();
            double gain = per_cell_metric_under_mute[pair.first] - per_cell_metric[pair.first];
            assert(gain > 0);
            if (muted_cell->slice_rbgs_quota_[slice_id] >= per_slice_quota_deduction){
              // Should only be able to mute aggressor if it has quota left on the aggressor
              double metric_per_rbg = muted_cell->metrics_perrbg_slice_[slice_id];
              double metric_loss_penalty = per_slice_quota_deduction * metric_per_rbg;
              if (metric_loss_penalty <= gain) {
                mute_aggressor = true;
              }
            }
            if (!mute_aggressor){
              flows_benefit.erase(pair.first);
              break;
            }
          }
          if (mute_aggressor){
            break;
          }
        }
      }
      else
      {
        while(flows_benefit.size() >= 1){
          per_slice_quota_deduction = 1.0 / flows_benefit.size();
          for (const auto& pair : flows_benefit){
            mute_aggressor = false;
            int slice_id = pair.second->GetSliceID();
            double gain = per_cell_metric_under_mute[pair.first] - per_cell_metric[pair.first];
            assert(gain > 0);
            if (muted_cell->slice_rbgs_quota_[slice_id] >= per_slice_quota_deduction){
              // Should only be able to mute aggressor if it has quota left on the aggressor
              mute_aggressor = EvaluateTradeoff(muted_cell, per_slice_quota_deduction, slice_id, gain);
            }
            if (!mute_aggressor){
              flows_benefit.erase(pair.first);
              break;
            }
          }
          if (mute_aggressor){
            break;
          }
        }
      }
      if(mute_aggressor){
        break;
      }
    }
    // For multi-utility muting, this trade-off approach uses current TTI allocation to approximate

    double total_metric_under_mute = 0.0;
    for (size_t i = 0; i < per_cell_metric_under_mute.size(); i++){
      if (per_cell_metric_under_mute[i] > 0){
        total_metric_under_mute += per_cell_metric_under_mute[i];
      }
    }
    double total_metric = 0.0;
    for (size_t i = 0; i < per_cell_metric.size(); i++){
      if (per_cell_metric[i] > 0){
        total_metric += per_cell_metric[i];
      }
    }
    if(mute_aggressor){
      for (const auto& pair: flows_benefit){
        FlowToSchedule* flow = pair.second;
        // cout << "Cell " << aggressor_cell
        //   << " muted to benefit Cell " << flow->GetBearer()->GetSource()->GetIDNetworkNode()
        //   << ", UE: " << flow->GetBearer()->GetDestination()->GetIDNetworkNode()
        //   << " of Slice: " << flow->GetSliceID() << endl;
        muted_cell->slice_rbgs_quota_[flow->GetSliceID()] -= per_slice_quota_deduction;
      }
      #ifdef MUTING_FREQ_GAIN_DEBUG
      std::cout<< "Muting Decision: " << rb_id << " - Muting Cell: " <<
      aggressor_cell << " Percentage Improvement: "<<
      (total_metric_under_mute - total_metric) / total_metric << endl;
      #endif
    }
  }

// REMOVE INTERFERNCE FROM ALL FLOWS ONCE DECISION IS FINAL
  if (mute_aggressor){
    assert(aggressor_cell != -1);
    UpdateNoiseInterferenceWithMute(schedulers, rb_id, aggressor_cell);
    if (do_realloc){
      Allocation allocated_flows;
      allocated_flows = DoAllocation(schedulers, rb_id, aggressor_cell);
      cell_flow = allocated_flows.cell_flow;
    }
  }
  if(mute_aggressor){
    std::vector<int> benefitting_cells;
    for (auto it = flows_benefit.begin(); it != flows_benefit.end(); it++){
      FlowToSchedule* flow = it->second;
      benefitting_cells.push_back(flow->GetBearer()->GetSource()->GetIDNetworkNode());
    }
    for (int i = 0; i < schedulers.size(); i ++){
      if(i == aggressor_cell){
        continue;
      }
      if (std::find(benefitting_cells.begin(), benefitting_cells.end(), i) ==
      benefitting_cells.end()){ // This cell is not present in the benefitting cells
        schedulers[i]->UpdatePerSlicePerformance(
          cell_flow[i]->GetSliceID(), per_cell_metric_under_mute[i]);
      }
    }
  } else {
    for (int i = 0; i < schedulers.size(); i++){
      schedulers[i]->UpdatePerSlicePerformance(cell_flow[i]->GetSliceID(), per_cell_metric[i]);
    }
  }
  for (int j = 0; j < (int)schedulers.size(); j++) {
    if (j == aggressor_cell && mute_aggressor) {
      continue;
    }
    RadioSaberDownlinkScheduler* scheduler = (RadioSaberDownlinkScheduler*)schedulers[j];
    FlowToSchedule* flow = cell_flow[j];
    auto& rsrp_report = flow->GetRSRPReport();
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
      double int_noise = rsrp_report.noise_interfere_watt[i];
      int cqi = amc->GetCQIFromSinr(
        rsrp_report.rx_power[i] - 10. * log10(int_noise));
      flow->GetCqiFeedbacks()[i] = cqi;
    }
    scheduler->slice_rbgs_quota_[flow->GetSliceID()] -= 1;
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++){
      flow->GetListOfAllocatedRBs()->push_back(i);
    }
  }
}

bool 
FrameManager::EvaluateTradeoff
  (DownlinkPacketScheduler* muted_cell, double cost_in_rbs, int slice_id, double gain_in_metric){
  // cout << "In Eval Tradeoff\n";
  double metric_per_rbg = muted_cell->GetSliceRollingAverage(slice_id);
  double loss_in_metric = metric_per_rbg * cost_in_rbs;
  // cout << "Slice ID: " << slice_id << " Metric(per RBG): " << metric_per_rbg
  //   << " Cost in RBs: " << cost_in_rbs << " Gain in Metric: " << gain_in_metric
  //   << " Loss in Metric: " << loss_in_metric << endl;
  if (gain_in_metric > loss_in_metric / 2){
    return true;
  } else {
    return false;
  }
}

std::map<int, double>
FrameManager::GetMetricsOnMutedCell(
  DownlinkPacketScheduler* muted_cell, std::map<int, double>& rbgs_deduction, int rb_id) {
    RadioSaberDownlinkScheduler* scheduler = (RadioSaberDownlinkScheduler*)muted_cell;
    FlowsToSchedule* flows = scheduler->GetFlowsToSchedule();
    const int num_slice = scheduler->slice_ctx_.num_slices_;
    AMCModule* amc = scheduler->GetMacEntity()->GetAmcModule();
    int nb_of_rbs = scheduler->GetMacEntity()->GetDevice()->GetPhy()
      ->GetBandwidthManager()->GetDlSubChannels().size ();
    std::vector<double> slice_sum_metrics(num_slice, 0);
    std::vector<double> slice_rbgs_quota(scheduler->slice_rbgs_quota_);
    for (auto it = rbgs_deduction.begin(); it != rbgs_deduction.end(); it++) {
      slice_rbgs_quota[it->first] -= it->second;
    }
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
        double metric = scheduler->ComputeSchedulingMetric(
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
    std::map<int, double> benefit_slice_metrics;
    for (const auto& pair : rbgs_deduction) {
      benefit_slice_metrics[pair.first] = slice_sum_metrics[pair.first];
    }
    return benefit_slice_metrics;
}

void
FrameManager::RadioSaberAllocateOneRBMute(
    std::vector<DownlinkPacketScheduler*>& schedulers,
    int rb_id) {
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  double tbs_weight = schedulers[0]->max_tp_weight_;
  double pf_metric_weight = 1 - tbs_weight;
  std::vector<int> mute_ids;
  std::vector<double> sum_tbs_under_mute;
  std::vector<double> sum_metric_under_mute;
  std::vector<std::vector<FlowToSchedule*>> cell_flow_under_mute;
  const int macro_cell = 0;
  int small_cell_mute = -1;
  // mute_scheme: 0(no mute), 1(mute macro), 2(mute a small cell)
  for (int mute_scheme = 0; mute_scheme < 3; mute_scheme++) {
    double sum_tbs = 0;
    double sum_metric = 0;
    std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
    // capture the slice that benefits most from muting(across all cells)
    for (size_t j = 0; j < schedulers.size(); j++) {
      // skip the muted cell
      if (mute_scheme == 1 && (int)j == macro_cell)
        continue;
      if (mute_scheme == 2 && (int)j == small_cell_mute) {
        continue;
      }
      RadioSaberDownlinkScheduler* scheduler =
        (RadioSaberDownlinkScheduler*)schedulers[j];
      FlowsToSchedule* flows = scheduler->GetFlowsToSchedule();
      int num_slice = scheduler->slice_ctx_.num_slices_;
      // RadioSaber Logic Starts Here
      std::vector<FlowToSchedule*> slice_flow(num_slice, nullptr);
      std::vector<double> slice_spectraleff(num_slice, -1);
      std::vector<double> max_metrics(num_slice, -1);
      for (auto it = flows->begin(); it != flows->end(); it++) {
        FlowToSchedule* flow = *it;
        auto& rsrp_report = flow->GetRSRPReport();
        double spectraleff_rbg = 0.0;
        // under the current muting assumption
        for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
          double in_under_mute = rsrp_report.noise_interfere_watt[i];
          if (mute_scheme == 1) {
            in_under_mute -= pow(10., rsrp_report.rsrp_interference[macro_cell][i]/10.);
          }
          else if (mute_scheme == 2) {
            assert(small_cell_mute != -1);
            in_under_mute -= pow(10., rsrp_report.rsrp_interference[small_cell_mute][i]/10.);
          }
          int cqi_under_mute = amc->GetCQIFromSinr(
              rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
          spectraleff_rbg += amc->GetEfficiencyFromCQI(cqi_under_mute);
        }
        double metric = scheduler->ComputeSchedulingMetric(
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
      double max_pf_metric = -1;
      FlowToSchedule* selected_flow = nullptr;
      for (int i = 0; i < num_slice; i++) {
        if (slice_spectraleff[i] > max_slice_spectraleff
            && scheduler->slice_rbgs_quota_[i] > 0) {
          max_slice_spectraleff = slice_spectraleff[i];
          max_pf_metric = max_metrics[i];
          selected_flow = slice_flow[i];
        }
      }
      if (selected_flow) {
        cell_flow[j] = selected_flow;
        sum_tbs += max_slice_spectraleff;
        sum_metric += max_pf_metric;
      }
    }
    if (mute_scheme == 0) {
      mute_ids.push_back(-1);
      FlowToSchedule* flow = cell_flow[macro_cell];
      assert(flow != nullptr);
      auto& rsrp_report = flow->GetRSRPReport();
      // search the neighbor cell of this macro-cell user
      double neighbor_rsrp = -10000;
      int neighbor_id = -1;
      for (size_t j = 1; j < schedulers.size(); j++) {
        double cell_rsrp = rsrp_report.rsrp_interference[j][rb_id];
        if (cell_rsrp > neighbor_rsrp) {
          neighbor_id = j;
          neighbor_rsrp = cell_rsrp;
        }
      }
      small_cell_mute = neighbor_id;
    }
    else if (mute_scheme == 1) {
      mute_ids.push_back(macro_cell);
    }
    else if (mute_scheme == 2) {
      mute_ids.push_back(small_cell_mute);
    }
    sum_tbs_under_mute.push_back(sum_tbs);
    sum_metric_under_mute.push_back(sum_metric);
    cell_flow_under_mute.push_back(cell_flow);
  }
  double no_mute_tbs = sum_tbs_under_mute[0];
  double no_mute_pf = sum_metric_under_mute[0];
  // Calculate the percentage improvement over the no muting scenario
  std::vector<double> percentage_improvements;
  for (size_t i = 0; i < mute_ids.size(); i++){
    double tbs_percentage_improvement = (double) (sum_tbs_under_mute[i] - no_mute_tbs) / no_mute_tbs;
    double pf_percentage_improvement = (double) (sum_metric_under_mute[i] - no_mute_pf) / no_mute_pf;
    percentage_improvements.push_back(
      (tbs_weight * tbs_percentage_improvement) + (pf_metric_weight * pf_percentage_improvement));
  }
  // Find the index of the maximum percentage improvement
  double max = *max_element(percentage_improvements.begin(), percentage_improvements.end());
  auto it = find(percentage_improvements.begin(), percentage_improvements.end(), max);
  int final_scheme = it - percentage_improvements.begin();
  int final_mute_id = mute_ids[final_scheme];
  auto final_cell_flow = cell_flow_under_mute[final_scheme];
  if (final_mute_id != -1) {
    for (size_t j = 0; j < schedulers.size(); j++) {
      FlowsToSchedule* flows = schedulers[j]->GetFlowsToSchedule();
      for (auto it = flows->begin(); it != flows->end(); it++) {
        auto& rsrp_report = (*it)->GetRSRPReport();
        for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
            rsrp_report.noise_interfere_watt[i] -= pow(10., rsrp_report.rsrp_interference[final_mute_id][i]/10.);
        }
      }
    }
  }
  // Allocation begins
  double highest_interfere = -10000;
  int slice_benefit = -1;
  for (size_t j = 0; j < schedulers.size(); j++) {
    if (final_mute_id == (int)j) {
      continue;
    }
    RadioSaberDownlinkScheduler* scheduler = (RadioSaberDownlinkScheduler*)schedulers[j];
    FlowToSchedule* flow = final_cell_flow[j];
    auto& rsrp_report = flow->GetRSRPReport();
    // under the current muting assumption
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
      double in_under_mute = rsrp_report.noise_interfere_watt[i];
      int cqi_under_mute = amc->GetCQIFromSinr(
        rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
      flow->GetCqiFeedbacks()[i] = cqi_under_mute;
    }
    scheduler->slice_rbgs_quota_[flow->GetSliceID()] -= 1;
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++){
      flow->GetListOfAllocatedRBs()->push_back(i);
    }
    if (final_mute_id != -1) {
      if (rsrp_report.rsrp_interference[final_mute_id][rb_id] > highest_interfere) {
        highest_interfere = rsrp_report.rsrp_interference[final_mute_id][rb_id];
        slice_benefit = flow->GetSliceID();
      }
    }
  }
  if (final_mute_id != -1) {
    assert(slice_benefit != -1);
    RadioSaberDownlinkScheduler* scheduler =
      (RadioSaberDownlinkScheduler*) schedulers[final_mute_id];
    scheduler->slice_rbgs_quota_[slice_benefit] -= 1;
  }
}

void
FrameManager::SingleObjMutingExhaust(
    std::vector<DownlinkPacketScheduler*>& schedulers,
    int rb_id) {
  std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  std::unordered_set<int> global_cells_muted;
  std::vector<FlowToSchedule*> global_cell_flow; // WHich flow will be scheduled for this RB
  const int index_no_mute = schedulers.size();
  bool do_realloc = schedulers[0]->do_reallocation_;

  double tbs_weight = schedulers[0]->max_tp_weight_;
  double pf_metric_weight = 1 - tbs_weight;
  double total_quota = 0.;
  for (int i = 0; i < schedulers.size(); i++){
    for (int j = 0; j < schedulers[i]->slice_ctx_.num_slices_; j++){
      RadioSaberDownlinkScheduler* rs_scheduler = (RadioSaberDownlinkScheduler*)schedulers[i];
      double quota_remaining = rs_scheduler->slice_rbgs_quota_[j];
      total_quota += quota_remaining;
    }
    assert(total_quota == (500 - rb_id) / 4.);
    cout << "TI: " << GetTTICounter() << " rb_id: " << (500 - rb_id) / 4. << " total_quota: " << total_quota << endl;
    total_quota = 0.;
  }

  
  while (true) {
    int final_mute_id = -1;
    double max_sum_tbs = 0;
    double max_sum_metric = 0;
    std::vector<FlowToSchedule*> max_cell_flow;
    int mute_id;
    if (schedulers[0]->enable_comp_) { // if enable_comp, start from every possible muting cell
      mute_id = 0;
    } else { // if not enable_comp, start from schedulers.size() which means no muting
      mute_id = index_no_mute;
    }
    std::vector<int> mute_ids;
    std::vector<double> sum_tbs_under_mute;
    std::vector<double> sum_metric_under_mute;
    std::vector<std::vector<FlowToSchedule*>> cell_flow_under_mute;
    for (; mute_id <= (int)schedulers.size(); mute_id++) {
      // already muted
      // cout << "Testing Muting: " << mute_id << endl;
      if (global_cells_muted.count(mute_id)) {
        continue;
      }
      double sum_tbs = 0;
      double sum_metric = 0;
      std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
      // capture the slice that benefits most from muting(across all cells)
      for (int j = 0; j < (int)schedulers.size(); j++) {
        RadioSaberDownlinkScheduler* scheduler =
          (RadioSaberDownlinkScheduler*)schedulers[j];
        // skip the muted cell
        if (j == mute_id || global_cells_muted.count(j)) {
          continue; 
        }
        FlowsToSchedule* flows = scheduler->GetFlowsToSchedule();
        int num_slice = scheduler->slice_ctx_.num_slices_;
        // enterprise scheduling for every slice
        // RadioSaber Logic Starts Here
        std::vector<FlowToSchedule*> slice_flow(num_slice, nullptr);
        std::vector<double> slice_spectraleff(num_slice, -1);
        std::vector<double> max_metrics(num_slice, -1);
        std::vector<double> max_pf_metrics(num_slice, -1);
        // Radiosaber Logic
        if (global_cells_muted.size() > 0 && !do_realloc){
          // cout << "At least one cell muted: " << global_cells_muted.size() << endl;
          for (int i = 0; i < global_cell_flow.size(); i++){
            FlowToSchedule* scheduled_flow = global_cell_flow[i];
            if (scheduled_flow == nullptr){
              // cout << i << " is NULL\n";
              continue;
            }
            auto& rsrp_report = scheduled_flow->GetRSRPReport();
            // under the current muting assumption
            double spectraleff_rbg = 0.0;
            for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
              double in_under_mute = rsrp_report.noise_interfere_watt[i]; // Interference under Muting
              if (mute_id != index_no_mute) {
                // Muting one cell would change the SINR of every user in every cell
                in_under_mute -= pow(10., rsrp_report.rsrp_interference[mute_id][i]/10.);
              }
              int cqi_under_mute = amc->GetCQIFromSinr(
                  rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
              spectraleff_rbg += amc->GetEfficiencyFromCQI(cqi_under_mute); // This returns TBS
            }
            double metric = scheduler->ComputeSchedulingMetric(
                scheduled_flow->GetBearer(), spectraleff_rbg, rb_id);
          
            assert(scheduled_flow);
            cell_flow[i] = scheduled_flow;
            sum_tbs += spectraleff_rbg;
            sum_metric += metric;
          }

          #ifdef REALLOCATION_DEBUG
          cout << "Sticking to same allocation: " << endl;
          for (int i = 0; i < cell_flow.size(); i++){
            if (global_cell_flow[i] == nullptr){
                  cout << i << " is muted\n";
            } else {
                cout << "RB ID: " << rb_id << " Cell " << i << " UE: " 
                << cell_flow[i]->GetBearer()->GetDestination()->GetIDNetworkNode() << endl;
            }
          }
          #endif
        } else {
          for (auto it = flows->begin(); it != flows->end(); it++) {
            FlowToSchedule* flow = *it;
            auto& rsrp_report = flow->GetRSRPReport();
            // under the current muting assumption
            double spectraleff_rbg = 0.0;
            for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
              double in_under_mute = rsrp_report.noise_interfere_watt[i]; // Interference under Muting
              if (mute_id != index_no_mute) {
                // Muting one cell would change the SINR of every user in every cell
                in_under_mute -= pow(10., rsrp_report.rsrp_interference[mute_id][i]/10.);
              }
              int cqi_under_mute = amc->GetCQIFromSinr(
                  rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
              spectraleff_rbg += amc->GetEfficiencyFromCQI(cqi_under_mute); // This returns TBS
            }
            double metric = scheduler->ComputeSchedulingMetric(
                flow->GetBearer(), spectraleff_rbg, rb_id);
            double historical = (flow->GetBearer()->GetAverageTransmissionRate())/1000.;
            double instantaneous = spectraleff_rbg;
            double pf_metric_for_flow = instantaneous / historical;
            int slice_id = flow->GetSliceID();
            // enterprise schedulers
            if (metric > max_metrics[slice_id]) {
              max_metrics[slice_id] = metric;
              slice_flow[slice_id] = flow;
              slice_spectraleff[slice_id] = spectraleff_rbg;
              max_pf_metrics[slice_id] = pf_metric_for_flow;
            }
          }
          double max_slice_spectraleff = -1;
          double max_pf_metric = -1;
          FlowToSchedule* selected_flow = nullptr;
          for (int i = 0; i < num_slice; i++) {
            if (slice_spectraleff[i] <= 0) {
              std::string error_msg = "Slice " + std::to_string(i) + " has no flow in cell " + std::to_string(j);
              throw std::runtime_error(error_msg);
            }
            if (slice_spectraleff[i] > max_slice_spectraleff
                && scheduler->slice_rbgs_quota_[i] >= 1) {
              max_slice_spectraleff = slice_spectraleff[i];
              // max_pf_metric = max_metrics[i];
              max_pf_metric = max_pf_metrics[i];
              selected_flow = slice_flow[i];
            }
          }
          if (selected_flow == nullptr) {
            // cout << "Insufficient quota for all slices, assigning randomly\n";
            vector<int> eligible_slices;
            for (int i = 0; i < scheduler->slice_ctx_.num_slices_; i++){
              if (scheduler->slice_rbgs_quota_[i] > 0){
                eligible_slices.push_back(i);
              }
            }
            int selected_slice_index = rand() % eligible_slices.size();
            int selected_slice = eligible_slices[selected_slice_index];
            selected_flow = slice_flow[selected_slice];
            max_slice_spectraleff = slice_spectraleff[selected_slice];
            max_pf_metric = max_pf_metrics[selected_slice];
          }
          assert(selected_flow);
          cell_flow[j] = selected_flow;
          sum_tbs += max_slice_spectraleff;
          sum_metric += max_pf_metric;
          // cout << "TI: " << GetTTICounter() << " RB: " << rb_id 
          //   << " PF_Analysis: " << selected_flow->GetSliceID() << " " 
          //   << max_slice_spectraleff << " " <<selected_flow->GetBearer()->GetAverageTransmissionRate() /1000
          //   << " Metric: " << max_pf_metric << endl;
        }
      }
      mute_ids.push_back(mute_id);
      sum_tbs_under_mute.push_back(sum_tbs);
      sum_metric_under_mute.push_back(sum_metric);
      cell_flow_under_mute.push_back(cell_flow);
    }
    // Calculate the TP, PF with No Muting/No further muting scenario.
    auto it = find(mute_ids.begin(), mute_ids.end(), index_no_mute);
    int no_mute_index = it - mute_ids.begin();
    double no_mute_tbs = sum_tbs_under_mute[no_mute_index];
    double no_mute_pf = sum_metric_under_mute[no_mute_index];
    
    // Calculate the percentage improvement over the no muting scenario
    std::vector<double> percentage_improvements;
    for (size_t i = 0; i < mute_ids.size(); i++){
      max_sum_tbs = sum_tbs_under_mute[i];
      max_sum_metric = sum_metric_under_mute[i];
      double tbs_percentage_improvement = (double) (sum_tbs_under_mute[i] - no_mute_tbs) / no_mute_tbs;
      double pf_percentage_improvement = (double) (sum_metric_under_mute[i] - no_mute_pf) / no_mute_pf;
      percentage_improvements.push_back(
        (tbs_weight * tbs_percentage_improvement) + (pf_metric_weight * pf_percentage_improvement));
    }

    // Find the index of the maximum percentage improvement
    double max = *max_element(percentage_improvements.begin(), percentage_improvements.end());
    auto it_2 = find(percentage_improvements.begin(), percentage_improvements.end(), max);
    int index_max_percentage_improvement = it_2 - percentage_improvements.begin();
    final_mute_id = mute_ids[index_max_percentage_improvement];

    max_cell_flow = cell_flow_under_mute[index_max_percentage_improvement];
    max_sum_tbs = sum_tbs_under_mute[index_max_percentage_improvement];
    max_sum_metric = sum_metric_under_mute[index_max_percentage_improvement];

    if (final_mute_id == index_no_mute) { // We stop muting any further
      global_cell_flow = max_cell_flow;
      // std::cout << "RB ID: " << rb_id << " - No More Muting - PF Metric: " << global_sum_metric << "\tTBS: " << global_sum_tbs << endl;
      // terminate the loop since no more benefit by muting a cell
      break;
    }
    else { // We continue muting the next cell and update the noise and interfernce for every user
      global_cell_flow = max_cell_flow;
      global_cell_flow[final_mute_id] = nullptr;
      # ifdef REALLOCATION_DEBUG
      cout << "Initial Allocation: " << global_cell_flow.size() << endl;
        for (int i = 0; i < global_cell_flow.size(); i++){
          if (global_cell_flow[i] == nullptr){
            cout << i << " is muted\n";
          } else {
            cout << "RB ID: " << rb_id << " Cell " << i << " UE: " << global_cell_flow[i]->GetBearer()->GetDestination()->GetIDNetworkNode() << endl;
          }
        }
      # endif

      global_cells_muted.insert(final_mute_id);
      #ifdef MUTING_FREQ_GAIN_DEBUG
      std::cout<< "Muting Decision: " << rb_id << " - Muting Cell: " << final_mute_id 
      << " Percentage Improvement: "<< percentage_improvements[index_max_percentage_improvement] << endl;
      #endif
      // update the noise_interference report of all users in all cells
      UpdateNoiseInterferenceWithMute(schedulers, rb_id, final_mute_id);
    }
  }
  std::vector<int> slices_benefit;
  for (size_t j = 0; j < schedulers.size(); j++) {
    if (global_cells_muted.count(j)) {
      continue;
    }
    RadioSaberDownlinkScheduler* scheduler =
      (RadioSaberDownlinkScheduler*)schedulers[j];
    FlowToSchedule* flow = global_cell_flow[j];
    auto& rsrp_report = flow->GetRSRPReport();
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
      double in_under_mute = rsrp_report.noise_interfere_watt[i];
      int cqi_under_mute = amc->GetCQIFromSinr(
        rsrp_report.rx_power[i] - 10. * log10(in_under_mute));
      flow->GetCqiFeedbacks()[i] = cqi_under_mute;
    }
    scheduler->slice_rbgs_quota_[flow->GetSliceID()] -= 1;
    // cout << "Cell " << j << " allocated to Slice: " << flow->GetSliceID() << " Deducting -1" << endl;
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++){
      flow->GetListOfAllocatedRBs()->push_back(i);
    }
    slices_benefit.push_back(flow->GetSliceID());
  }

  #ifdef REALLOCATION_DEBUG
  cout << "\nFinal Allocation: " << endl;
  for (int i = 0; i < global_cell_flow.size(); i++){
    if (global_cell_flow[i] == nullptr){
          cout << i << " is muted\n";
    } else {
        cout << "RB ID: " << rb_id << " Cell " << i << " UE: " << 
        global_cell_flow[i]->GetBearer()->GetDestination()->GetIDNetworkNode() << endl;
    }
  }
  #endif
  cout << endl;

  for (auto it = global_cells_muted.begin(); it != global_cells_muted.end(); it++) {
    int cell_id = *it;
    RadioSaberDownlinkScheduler* scheduler =
      (RadioSaberDownlinkScheduler*) schedulers[cell_id];
    double deduction = 1. / scheduler->slice_ctx_.num_slices_;
    cout << "Deduction: " << deduction << endl;
    for (int i = 0; i < scheduler->slice_ctx_.num_slices_; i++){
      cout << "Deducting " << deduction << 
      " from Slice: " << i << " at Cell " << 
      scheduler->GetMacEntity()->GetDevice()->GetIDNetworkNode() << endl;
      scheduler->slice_rbgs_quota_[i] -= deduction;
    }
  }
  for (int i = 0; i < schedulers.size(); i++){
    RadioSaberDownlinkScheduler* schedulerr =
        (RadioSaberDownlinkScheduler*)schedulers[i];
    cout << "Quota Remaining for Slices in TI: " << GetTTICounter() << " RB: " << rb_id << "\n";
    for (int k = 0; k < schedulerr->slice_ctx_.num_slices_; k++){
      cout << "C " << i << " S " << k << " has quota remaining: " << schedulerr->slice_rbgs_quota_[k] << endl;
    }
  }
  // cout << "TTI: " << GetTTICounter() << endl;
  for (int i = 0; i < global_cell_flow.size(); i++){
    if (global_cell_flow[i] != nullptr){
      cout << "Scheduled: " << i << " at RB: " << rb_id << " " << global_cell_flow[i]->GetBearer()->GetDestination()->GetSliceID() << endl;
    }
  }
}

void FrameManager::SingleObjMutingOneCell(
  std::vector<DownlinkPacketScheduler*>& schedulers,
    int rb_id){
  // Get the adaptive modulation module of the eNB      
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  std::map<int, int> per_cell_slice_map;
  std:map<int, double> per_slice_metric;
  std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
  bool enable_comp = schedulers[0]->enable_comp_;
  int tbs_weight = schedulers[0]->max_tp_weight_;
  int pf_metric_weight = 1 - tbs_weight;
  double percentage_improvement = 0;
  
  bool mute_aggressor = false;
  int aggressor_cell = -1;

  if (enable_comp){
    std::vector<int> aggressor_cells = GetMutingOrder(schedulers.size(), true);

    // std::pair<int, int> x = std::make_pair(static_cast<int>(GetTTICounter()), rb_id);
    // int mute_cell = schedulers[0]->ea_muting_order[x];
    // std::vector<int> aggressor_cells;
    // if (mute_cell > -1){
    //   aggressor_cells.push_back(mute_cell);
    // }

    for (int i = 0; i < aggressor_cells.size(); i++){
      aggressor_cell = aggressor_cells[i];
      Allocation allocation_under_mute = DoAllocation(schedulers, rb_id, aggressor_cell, true);
      per_cell_slice_map = allocation_under_mute.per_cell_slice_map;
      cell_flow = allocation_under_mute.cell_flow;

      Allocation allocation_no_mute = DoAllocation(schedulers, rb_id);
      double tbs_percentage_improvement = (allocation_under_mute.sum_tbs - allocation_no_mute.sum_tbs) 
        / allocation_no_mute.sum_tbs;
      double pf_percentage_improvement = (allocation_under_mute.sum_metric - allocation_no_mute.sum_metric) 
        / allocation_no_mute.sum_metric;
      percentage_improvement = (tbs_weight * tbs_percentage_improvement) +
         (pf_metric_weight * pf_percentage_improvement);

      if(percentage_improvement > 0){
        mute_aggressor = true;
        break;
      }
    }
    // For multi-utility muting, this trade-off approach uses current TTI allocation to approximate

    if(mute_aggressor){
      #ifdef MUTING_FREQ_GAIN_DEBUG
      std::cout<< "Muting Decision: " << rb_id << " - Muting Cell: " << aggressor_cell 
      << " TTI: " << GetTTICounter() << " Percentage Improvement: "<< percentage_improvement << endl;
      #endif
    }
  }

// REMOVE INTERFERNCE FROM ALL FLOWS ONCE DECISION IS FINAL
  if (mute_aggressor){
    assert(aggressor_cell != -1);
    UpdateNoiseInterferenceWithMute(schedulers, rb_id, aggressor_cell);
  } else {
    Allocation allocation = DoAllocation(schedulers, rb_id);
    cell_flow = allocation.cell_flow;
  }
  std::vector<int> slices_benefit;
  for (int j = 0; j < (int)schedulers.size(); j++) {
    if (j == aggressor_cell && mute_aggressor) {
      continue;
    }
    RadioSaberDownlinkScheduler* scheduler = (RadioSaberDownlinkScheduler*)schedulers[j];
    FlowToSchedule* flow = cell_flow[j];
    auto& rsrp_report = flow->GetRSRPReport();
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
      double int_noise = rsrp_report.noise_interfere_watt[i];
      int cqi = amc->GetCQIFromSinr(
        rsrp_report.rx_power[i] - 10. * log10(int_noise));
      flow->GetCqiFeedbacks()[i] = cqi;
    }
    scheduler->slice_rbgs_quota_[flow->GetSliceID()] -= 1;
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++){
      flow->GetListOfAllocatedRBs()->push_back(i);
    }
    slices_benefit.push_back(flow->GetSliceID());
  }
  if (mute_aggressor){
    RadioSaberDownlinkScheduler* scheduler =
    (RadioSaberDownlinkScheduler*) schedulers[aggressor_cell];
    for (size_t k = 0; k < slices_benefit.size(); k++) {
      scheduler->slice_rbgs_quota_[slices_benefit[k]] -= (1.0 / slices_benefit.size());
    }
  }
}

std::map<int, double> GetPerSliceDeduction(std::map<int, double> per_slice_metric_no_mute, 
  std::map<int, double> per_slice_metric_mute)
{
  bool proportionate_to_benefit_deduction = false;
  double total_percentage_gain = 0.0;
  std::map<int, double> per_slice_percentage_gain_map;
  std::map<int, double> per_slice_cost_map;

  // cout << "In Per Slice Deduction" << endl;

  // cout << "No Mute\n";
  // for (auto const &t: per_slice_metric_no_mute){
  //   cout << t.first << " " << t.second << endl;
  // }

  // cout << "Mute\n";
  // for (auto const &t: per_slice_metric_mute){
  //   cout << t.first << " " << t.second << endl;
  // }
  for (auto const pair: per_slice_metric_no_mute){
    double per_slice_percentage_gain = (per_slice_metric_mute[pair.first] - per_slice_metric_no_mute[pair.first]) 
      / per_slice_metric_no_mute[pair.first];
    if (per_slice_percentage_gain > 0){
      per_slice_percentage_gain_map[pair.first] = per_slice_percentage_gain;
      total_percentage_gain += per_slice_percentage_gain;
    }
  }

  if (proportionate_to_benefit_deduction){
    for (auto const pair: per_slice_percentage_gain_map){
      per_slice_cost_map[pair.first] = pair.second / total_percentage_gain;
    }
  } else {
    double mean_cost = 1.0 / per_slice_percentage_gain_map.size();

    for (auto const pair: per_slice_percentage_gain_map){
      per_slice_cost_map[pair.first] = mean_cost;
    }
  }

  // cout << "Per Slice Cost Map\n";
  // for (auto const &t: per_slice_cost_map){
  //   cout << t.first << " " << t.second << endl;
  // }
  return per_slice_cost_map;
}


double GetMutingScore(std::map<int, double> per_slice_metric_no_mute, 
  std::map<int, double> per_slice_metric_mute,RadioSaberDownlinkScheduler* muted_cell)
{
  std::map<int, double> per_slice_metric_gain_map;
  double muting_score = 0;

  // cout << "Muted Cell: " << muted_cell->GetMacEntity()->GetDevice()->GetIDNetworkNode() << " Muting Score: " << endl;
  for (auto const pair: per_slice_metric_no_mute){
    // cout << "Slice: " << pair.first << " No Mute: " << per_slice_metric_no_mute[pair.first] << " Mute: " << per_slice_metric_mute[pair.first] << endl;
    double per_slice_metric_gain = per_slice_metric_mute[pair.first] - per_slice_metric_no_mute[pair.first];
    // cout << "Slice: " << pair.first << " Gain: " << per_slice_metric_gain << endl;
    assert(per_slice_metric_gain >= 0);
    per_slice_metric_gain_map[pair.first] = per_slice_metric_gain;
  }
  double mean_cost = 1.0 / per_slice_metric_gain_map.size();
  // cout << "Map Size: " << per_slice_metric_gain_map.size() << " Mean Cost: " << mean_cost << endl;
  int counter = 0;
  while (counter < per_slice_metric_gain_map.size()){
    for (auto const pair: per_slice_metric_gain_map){
      double slice_loss = (muted_cell->metrics_perrbg_slice_[pair.first]) * mean_cost;
      // double metric_per_rbg = muted_cell->GetSliceRollingAverage(pair.first);
      // double slice_loss = metric_per_rbg * mean_cost;
      double slice_score = pair.second/slice_loss;
      // cout << "Slice: " << pair.first << " Loss: " << slice_loss << " Score: " << slice_score << endl;
      if (slice_score > 1){
        muting_score += slice_score;
        counter++;
      } else {
        // cout << "Erasing S: " << pair.first << endl;
        per_slice_metric_gain_map.erase(pair.first);
        mean_cost = 1.0 / per_slice_metric_gain_map.size();
        // cout << "Map Size: " << per_slice_metric_gain_map.size() << " Mean Cost: " << mean_cost << endl;
        counter = 0;
        muting_score = 0;
        break;
      }
    }
  }

  return muting_score;
}



std::vector<int> FrameManager::GetExhaustiveMuteOrder(
  std::vector<DownlinkPacketScheduler*>& schedulers,
    int rb_id){
  // cout << "Calling Exahustive MU " << endl;
  // Get the adaptive modulation module of the eNB      
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  std::map<int, int> per_cell_slice_map;
  std:map<int, double> per_slice_metric;
  std::map<int, double> per_slice_metric_under_mute;
  
  int aggressor_cell = -1;
  RadioSaberDownlinkScheduler* muted_cell = nullptr;

  std::vector<int> aggressor_cells = GetMutingOrder(schedulers.size(), false);
  std::map<int, double> muting_scores;
  for (int i = 0; i < aggressor_cells.size(); i++){
    aggressor_cell = aggressor_cells[i];
    Allocation allocation_under_mute = DoAllocation(schedulers, rb_id, aggressor_cell, true);
    per_cell_slice_map = allocation_under_mute.per_cell_slice_map;
    per_slice_metric_under_mute = allocation_under_mute.per_slice_metric_map;

    // cout << "Per Cell Slice Map Under Mute: " << endl;
    // for (const auto& slice_id : per_cell_slice_map) {
    //   cout << "C: " << slice_id.first << " S: " << slice_id.second << ",\t";
    // }
    // cout << endl;
    Allocation allocation_no_mute = DoAllocation(schedulers, rb_id, aggressor_cell, false, per_cell_slice_map);
    // cout << "Per Cell Slice Map No Mute: " << endl;
    // for (const auto& slice_id : per_cell_slice_map) {
    //   cout << "C: " << slice_id.first << " S: " << slice_id.second << ",\t";
    // }
    // cout << endl;
    per_slice_metric = allocation_no_mute.per_slice_metric_map;
    muted_cell = (RadioSaberDownlinkScheduler*)schedulers[aggressor_cell];
        // slice scheduled under no mute 
    Allocation default_allocation = DoAllocation(schedulers, rb_id);
    int victim_slice = default_allocation.cell_flow[aggressor_cell]->GetSliceID();
    double muted_cell_metric = default_allocation.per_cell_metric_map[aggressor_cell];
    double average_metric = muted_cell->metrics_perrbg_slice_[victim_slice];
    double losing_slice_ratio = muted_cell_metric / average_metric;
    double muting_score = GetMutingScore(per_slice_metric, per_slice_metric_under_mute, muted_cell);
    muting_score -= losing_slice_ratio;
    // cout << "Adding muting score: " << muting_score << " for cell " << aggressor_cell << endl;
    if (muting_score > 1.0){
      muting_scores.emplace(aggressor_cell, muting_score);
    }
  }

  // for (auto &t: muting_scores){
  //   cout << "Muting Scores for cell " << t.first << " is " << t.second << endl;
  // }

  std::vector<std::pair<int, double>> mapVector(muting_scores.begin(), muting_scores.end());
  std::sort(mapVector.begin(), mapVector.end(),
            [](const auto &lhs, const auto &rhs) {
                return lhs.second > rhs.second; // Change '<' to '>' for descending order
            });

  // Display the sorted values in descending order
  // cout << "Sorted Muting Scores: " << endl;
  // for (const auto &entry : mapVector) {
  //     std::cout << entry.first << ": " << entry.second << std::endl;
  // }

  // put the values in a vector
  std::vector<int> sorted_keys;
  for (const auto &entry : mapVector) {
      sorted_keys.push_back(entry.first);
  }
  return sorted_keys;
}


std::vector<Allocation> FrameManager::MultiObjMuting(
  std::vector<DownlinkPacketScheduler*>& schedulers,
    int rb_id){
  cout << "\nStarting Multi Obj Muting\n";
  // Get the adaptive modulation module of the eNB      
  AMCModule* amc = schedulers[0]->GetMacEntity()->GetAmcModule();
  std::map<int, int> per_cell_slice_map;
  std:map<int, double> per_slice_metric;
  std::map<int, double> per_slice_metric_under_mute;
  std::vector<FlowToSchedule*> cell_flow(schedulers.size(), nullptr);
  bool enable_comp = schedulers[0]->enable_comp_;
  bool current_tti_cost = schedulers[0]->current_tti_cost_;
  bool consider_all_cells_for_int = schedulers[0]->consider_all_cells_for_int_;
  
  bool mute_aggressor = false;
  int aggressor_cell = -1;
  int muted_flow_slice_id = -1;
  // map<int, double> slice_benefit;
  const int num_slice = schedulers[0]->slice_ctx_.num_slices_;
  double per_slice_quota_deduction;
  RadioSaberDownlinkScheduler* muted_cell = nullptr;
  std::map<int, double> per_slice_cost_map;
  Allocation allocation_under_mute;
  Allocation allocation_no_mute;


  if (enable_comp){
    std::vector<int> aggressor_cells = GetExhaustiveMuteOrder(schedulers, rb_id);
    // std::vector<int> aggressor_cells = GetMutingOrder(schedulers.size(), true);
    // std::pair<int, int> x = std::make_pair(static_cast<int>(GetTTICounter()), rb_id);
    // int mute_cell = schedulers[0]->ea_muting_order[x];
    // std::vector<int> aggressor_cells;
    // if (mute_cell > -1){
    //   aggressor_cells.push_back(mute_cell);
    // }
    for (int i = 0; i < aggressor_cells.size(); i++){
      aggressor_cell = aggressor_cells[i];
      allocation_under_mute = DoAllocation(schedulers, rb_id, aggressor_cell, true);
      per_cell_slice_map = allocation_under_mute.per_cell_slice_map;
      cell_flow = allocation_under_mute.cell_flow;
      per_slice_metric_under_mute = allocation_under_mute.per_slice_metric_map;

      allocation_no_mute = DoAllocation(schedulers, rb_id, aggressor_cell, false, per_cell_slice_map);
      per_slice_metric = allocation_no_mute.per_slice_metric_map;

      // cout << "Allocation No Mute: " << endl;
      for (int i = 0; i < allocation_no_mute.cell_flow.size(); i++){
        if (allocation_no_mute.cell_flow[i] != nullptr){
          // cout << "Cell: " << i << " Flow: " << allocation_no_mute.cell_flow[i]->GetBearer()->GetDestination()->GetIDNetworkNode() << endl;
        } else {
          // cout << "Cell: " << i << " Flow: " << "NULL" << endl;
        }
      }

      // cout << "Allocation Under Mute: " << endl;
      for (int i = 0; i < allocation_under_mute.cell_flow.size(); i++){
        if (allocation_under_mute.cell_flow[i] != nullptr){
          // cout << "Cell: " << i << " Flow: " << allocation_under_mute.cell_flow[i]->GetBearer()->GetDestination()->GetIDNetworkNode() << endl;
        } else {
          // cout << "Cell: " << i << " Flow: " << "NULL" << endl;
        }
      }

      // Check Same Slice Restriction are scheduled
      vector<int> scheduled_cells_no_mute;
      vector<int> scheduled_cells_under_mute;
      for (auto &t: allocation_no_mute.per_cell_slice_map){
        scheduled_cells_no_mute.push_back(t.first);
      }
      for (auto &t: allocation_under_mute.per_cell_slice_map){
        scheduled_cells_under_mute.push_back(t.first);
      }
      assert(scheduled_cells_no_mute.size() == scheduled_cells_under_mute.size());
      for (int i = 0; i < scheduled_cells_no_mute.size(); i++){
        allocation_no_mute.per_cell_slice_map[i] == allocation_under_mute.per_cell_slice_map[i];
      }

      // cout << "Considering Agg C: " << aggressor_cell << endl;
      per_slice_cost_map = GetPerSliceDeduction(per_slice_metric, per_slice_metric_under_mute);
      // cout << "Per Slice Cost Map Below: " << endl;
      for (const auto& slice_cost : per_slice_cost_map) {
        // cout << "Slice: " << slice_cost.first << " Cost: " << slice_cost.second << endl;
      }

      muted_cell = (RadioSaberDownlinkScheduler*)schedulers[aggressor_cell];
      mute_aggressor = true;
      if (current_tti_cost) {
        // make a deep copy of per_slice_metric_under_mute and per_slice_metric
        std::map<int, double> per_slice_metric_under_mute_copy = per_slice_metric_under_mute;
        std::map<int, double> per_slice_metric_copy = per_slice_metric;

        while(per_slice_cost_map.size() >= 1){
          for (const auto& slice_cost : per_slice_cost_map) {
            // mute_aggressor is always initialized to false for every flow
            int slice_id = slice_cost.first;
            mute_aggressor = false;
            double gain = per_slice_metric_under_mute_copy[slice_id] - per_slice_metric_copy[slice_id];
            // cout << "In Cost Tradeoff for Muting Cell: " << aggressor_cell << " for slice " << slice_id << " with gain " << gain << endl;
            assert(gain >= 0);
            if (muted_cell->slice_rbgs_quota_[slice_id] >= slice_cost.second){
              // Should only be able to mute aggressor if it has quota left on the aggressor
              double metric_per_rbg = muted_cell->metrics_perrbg_slice_[slice_id];
              double metric_loss_penalty = slice_cost.second * metric_per_rbg;
              // cout << "Loss: " << metric_loss_penalty << endl;
              if (metric_loss_penalty/2 < gain) {
                mute_aggressor = true;
                // cout << "Muting c " << aggressor_cell << " for s " << slice_id << " w cost " << metric_loss_penalty << " - gain " << gain << endl;
              }
            } else {
              // cout << "Not Enough Quota slice " << slice_id << " cell " << aggressor_cell << endl;
            }
            if (!mute_aggressor){
              per_slice_metric_under_mute_copy.erase(slice_id);
              per_slice_metric_copy.erase(slice_id);
              per_slice_cost_map = GetPerSliceDeduction(per_slice_metric_copy, per_slice_metric_under_mute_copy);
              break;
            }
          }
          if (mute_aggressor){
            break;
          }
        }
      }
      if(mute_aggressor){
        break;
      }
    }
    // For multi-utility muting, this trade-off approach uses current TTI allocation to approximate

    double total_metric_under_mute = 0.0;
    for (size_t i = 0; i < per_slice_metric_under_mute.size(); i++){
      if (per_slice_metric_under_mute[i] > 0){
        total_metric_under_mute += per_slice_metric_under_mute[i];
      }
    }
    double total_metric = 0.0;
    for (size_t i = 0; i < per_slice_metric.size(); i++){
      if (per_slice_metric[i] > 0){
        total_metric += per_slice_metric[i];
      }
    }
    if(mute_aggressor){
      for (const auto& slice_cost: per_slice_cost_map){
        muted_cell->slice_rbgs_quota_[slice_cost.first] -= slice_cost.second;
      }
      #ifdef MUTING_FREQ_GAIN_DEBUG
      std::cout<< "Muting Decision: " << rb_id << " - Muting Cell: " <<
      aggressor_cell << " TTI: " << GetTTICounter() << " Percentage Improvement: "<<
      (total_metric_under_mute - total_metric) / total_metric << endl;
      #endif
      allocation_under_mute.success = true;
      allocation_under_mute.percentage_improvement = (total_metric_under_mute - total_metric) / total_metric;

      // for (int k = 0; k < allocation_no_mute.cell_flow.size(); k++){
      //   if (allocation_no_mute.cell_flow[k]!= nullptr && allocation_under_mute.cell_flow[k]!= nullptr){
      //     if (allocation_no_mute.cell_flow[k]->GetSliceID() == allocation_under_mute.cell_flow[k]->GetSliceID()){
      //       if (allocation_no_mute.cell_flow[k]->GetBearer()->GetDestination()->GetIDNetworkNode() 
      //         != allocation_under_mute.cell_flow[k]->GetBearer()->GetDestination()->GetIDNetworkNode()){
      //         // cout << "Flow Changes for Cell " << k << " when muting cell " << aggressor_cell << endl;
      //       }
      //     } else {
      //       // cout << "Slice Changes for Cell " << k << " when muting cell " << aggressor_cell
      //       // << " S_ID before mute: " << allocation_no_mute.cell_flow[k]->GetSliceID()
      //       // << " S_ID after mute: " << allocation_under_mute.cell_flow[k]->GetSliceID() << endl;
      //       assert(allocation_no_mute.cell_flow[k]->GetSliceID() == allocation_under_mute.cell_flow[k]->GetSliceID());
      //     }
      //   }
      // }

    //   if (aggressor_cell > 0){
    //     //  Muting a small cell
    //     int macro_scheduled_slice = per_cell_slice_map[0];
    //     for (int i = 1; i < per_cell_slice_map.size(); i++){
    //     for (auto const& pair : per_cell_slice_map){
    //       int cell_id = pair.first;
    //       for (auto const &slice : per_slice_cost_map){
    //         int benefitting_slice_id = slice.first;
    //         if (per_cell_slice_map[cell_id] == benefitting_slice_id && benefitting_slice_id != macro_scheduled_slice){
    //           cout << "Benefitting Slice: " << benefitting_slice_id 
    //           << " Muted Cell: " << aggressor_cell
    //           << " at Cell: " << cell_id
    //           << " Gain: " << per_slice_metric_under_mute[benefitting_slice_id] - per_slice_metric[benefitting_slice_id]
    //           << endl;
    //         }
    //       }
    //     }
    //   }
    }
  }

// REMOVE INTERFERNCE FROM ALL FLOWS ONCE DECISION IS FINAL
  if (mute_aggressor){
    assert(aggressor_cell != -1);
    // cout << "Removing Interference of Agg: " << aggressor_cell << endl;
    UpdateNoiseInterferenceWithMute(schedulers, rb_id, aggressor_cell);
  } else {
    // cout << "Running No Mute Allocation\n";
    allocation_no_mute = DoAllocation(schedulers, rb_id);
    cell_flow = allocation_no_mute.cell_flow;
  }
  for (int j = 0; j < (int)schedulers.size(); j++) {
    if (j == aggressor_cell && mute_aggressor) {
      continue;
    }
    RadioSaberDownlinkScheduler* scheduler = (RadioSaberDownlinkScheduler*)schedulers[j];
    FlowToSchedule* flow = cell_flow[j];
    auto& rsrp_report = flow->GetRSRPReport();
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++) {
      double int_noise = rsrp_report.noise_interfere_watt[i];
      int cqi = amc->GetCQIFromSinr(
        rsrp_report.rx_power[i] - 10. * log10(int_noise));
      flow->GetCqiFeedbacks()[i] = cqi;
    }
    scheduler->slice_rbgs_quota_[flow->GetSliceID()] -= 1;
    for (int i = rb_id; i < RBG_SIZE+rb_id; i++){
      flow->GetListOfAllocatedRBs()->push_back(i);
    }
    // if (mute_aggressor){
    //   double slice_metric = allocation_under_mute.per_cell_metric_map[j];
    //   int slice_id = allocation_under_mute.per_cell_slice_map[j];
    //   scheduler->actual_slice_rbgs_metric_sum_[slice_id] += slice_metric;
    //   // scheduler->CalculateMetricsPerRB(rb_id + 4);
    // }
  }
  std::vector<Allocation> allocations;
  allocations.push_back(allocation_no_mute);
  allocations.push_back(allocation_under_mute);
  return allocations;


  // if (mute_aggressor){
  // for (auto const& pair : allocation_under_mute.per_cell_metric_map){
  //   int cell_id = pair.first;
  //   double metric = pair.second;
  //   int slice_id = allocation_no_mute.per_cell_slice_map[cell_id];
  //   cout << "Updated-Avg-Perf Cell: " << cell_id << " Slice: " << slice_id << " Metric: " << metric << "\n";
  //   schedulers[cell_id]->UpdateSliceMetric(metric, slice_id);
  // }
  // } else {
  //   for (auto const& pair : allocation_no_mute.per_cell_metric_map){
  //     int cell_id = pair.first;
  //     double metric = pair.second;
  //     int slice_id = allocation_no_mute.per_cell_slice_map[cell_id];
  //     cout << "Updated-Avg-Perf Cell: " << cell_id << " Slice: " << slice_id << " Metric: " << metric << "\n";
  //     schedulers[cell_id]->UpdateSliceMetric(metric, slice_id);
  //   }
  // }

  // cout << "\n\n";
}
