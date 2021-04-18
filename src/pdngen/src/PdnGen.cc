/////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019, OpenROAD
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include "pdn/PdnGen.hh"

#include "openroad/OpenRoad.hh"
#include "utl/Logger.h"

#include <set>

namespace pdn {

using utl::Logger;
using utl::IFP;

using odb::dbBlock;
using odb::dbInst;
using odb::dbITerm;
using odb::dbMTerm;
using odb::dbMaster;
using odb::dbNet;
using odb::dbBox;

using std::regex;
using std::cmatch;

using utl::PDN;

PdnGen::PdnGen() : db_(nullptr),
    logger_(nullptr),
    global_connect_(nullptr)
{
};

void
PdnGen::init(dbDatabase *db, Logger* logger) {
  db_ = db;
  logger_ = logger;
}

void
PdnGen::setSpecialITerms() {
  if (global_connect_ == nullptr) {
    logger_->warn(PDN, 50, "Global connections not set up");
    return;
  }

  std::set<dbNet*> global_nets = std::set<dbNet*>();
  for (auto region : *global_connect_) {
    for (auto nets : *region.second) {
      global_nets.insert(nets.first);
    }
  }

  for (dbNet* net : global_nets)
    setSpecialITerms(net);
}

void
PdnGen::setSpecialITerms(dbNet *net) {
  for (dbITerm* iterm : net->getITerms()) {
    iterm->setSpecial();
  }
}

void
PdnGen::globalConnect(dbBlock* block, std::shared_ptr<regex>& instPattern, std::shared_ptr<regex>& pinPattern, dbNet *net) {
  dbBox* bbox = nullptr;
  globalConnectRegion(block, bbox, instPattern, pinPattern, net);
}

void
PdnGen::globalConnect(dbBlock* block) {
  if (global_connect_ == nullptr) {
    logger_->warn(PDN, 50, "Global connections not set up");
    return;
  }

  // Do non-regions first
  if (global_connect_->find(nullptr) != global_connect_->end()) {
    globalConnectRegion(block, nullptr, global_connect_->at(nullptr));
  }

  // Do regions
  for (auto region_con : *global_connect_) {
    dbBox* region = region_con.first;
    if (region == nullptr) continue;

    std::shared_ptr<netRegexPairs> netRegexes = region_con.second;

    globalConnectRegion(block, region, netRegexes);
  }

  setSpecialITerms();
}

void
PdnGen::globalConnectRegion(dbBlock* block, dbBox* region, std::shared_ptr<netRegexPairs> global_connect) {
  for (auto global_con : *global_connect) {
    dbNet* net = global_con.first;
    std::shared_ptr<regexPairs> netRegex = global_con.second;

    for (auto instpin : *netRegex) {
      globalConnectRegion(block, region, instpin.first, instpin.second, net);
    }
  }
}

void
PdnGen::globalConnectRegion(dbBlock* block, dbBox* region, std::shared_ptr<regex>& instPattern, std::shared_ptr<regex>& pinPattern, dbNet *net) {
  std::vector<dbInst*> insts;
  findInstsInArea(block, region, instPattern, insts);

  std::map<dbMaster*, std::vector<dbMTerm*>> masterpins;
  buildMasterPinMatchingMap(block, pinPattern, masterpins);

  cmatch match;
  for (dbInst* inst : insts) {
    dbMaster* master = inst->getMaster();

    auto masterpin = masterpins.find(master);
    if (masterpin != masterpins.end()) {
      for (dbMTerm* mterm : masterpin->second) {
        dbITerm::connect(inst, net, mterm);
      }
    }
  }
}

void
PdnGen::findInstsInArea(dbBlock* block, dbBox* region, std::shared_ptr<regex>& instPattern, std::vector<dbInst*>& insts) {
  cmatch match;
  for (dbInst* inst : block->getInsts()) {
    if (std::regex_match(inst->getName().c_str(), match, *instPattern)) {
      if (region == nullptr) {
        insts.push_back(inst);
      } else {
        dbBox* box = inst->getBBox();

        if (region->yMin() <= box->yMin() &&
            region->xMin() <= box->xMin() &&
            region->xMax() >= box->xMax() &&
            region->yMax() >= box->yMax()) {
          insts.push_back(inst);
        }
      }
    }
  }
}

void
PdnGen::buildMasterPinMatchingMap(dbBlock* block, std::shared_ptr<regex>& pinPattern, std::map<dbMaster*, std::vector<dbMTerm*>>& masterpins) {
  std::vector<dbMaster*> masters;
  block->getMasters(masters);

  cmatch match;
  for (dbMaster* master : masters) {
    masterpins.emplace(master, std::vector<dbMTerm*>());

    for (dbMTerm* mterm : master->getMTerms()) {
      if (std::regex_match(mterm->getName().c_str(), match, *pinPattern)) {
        masterpins.at(master).push_back(mterm);
      }
    }
  }
}

void
PdnGen::addGlobalConnect(const char* instPattern, const char* pinPattern, dbNet *net) {
  addGlobalConnect(nullptr, instPattern, pinPattern, net);
}

void
PdnGen::addGlobalConnect(dbBox* region, const char* instPattern, const char* pinPattern, dbNet *net) {
  if (global_connect_ == nullptr) {
    global_connect_ = std::make_unique<regionNetRegexPairs>();
  }

  // check if region is present in map, add if not
  if (global_connect_->find(region) == global_connect_->end()) {
    global_connect_->emplace(region, std::make_shared<netRegexPairs>());
  }

  std::shared_ptr<netRegexPairs> netRegexes = global_connect_->at(region);
  // check if net is present in region mapping
  if (netRegexes->find(net) == netRegexes->end()) {
    netRegexes->emplace(net, std::make_shared<regexPairs>());
  }

  netRegexes->at(net)->push_back(std::make_pair(std::make_shared<regex>(instPattern), std::make_shared<regex>(pinPattern)));
}

void
PdnGen::clearGlobalConnect() {
  if (global_connect_ != nullptr) {
    global_connect_.release();
    global_connect_ = nullptr;
  }
}

} // namespace
