<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
/////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2024, Precision Innovations Inc.
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
///////////////////////////////////////////////////////////////////////////////
#include "GuideProcessor.h"

#include "frProfileTask.h"
namespace drt::io {
using Interval = boost::icl::interval<frCoord>;
namespace {
/**
 * @brief Returns the closest point on the perimeter of the rectangle r to the
 * point p
 */
Point getClosestPoint(const frRect& r, const Point& p)
{
  const Rect b = r.getBBox();
  const int x = std::clamp(p.getX(), b.xMin(), b.xMax());
  const int y = std::clamp(p.getY(), b.yMin(), b.yMax());
  return Point(x, y);
}
/**
 * @brief Returns the shapes of the given pin on all layers.
 * @note Assumes pin's typeId() is either frcBTerm or frcInstTerm
 */
std::vector<frRect> getPinShapes(const frBlockObject* pin)
{
  std::vector<frRect> pinShapes;
  if (pin->typeId() == frcBTerm) {
    static_cast<const frBTerm*>(pin)->getShapes(pinShapes);
  } else {
    static_cast<const frInstTerm*>(pin)->getShapes(pinShapes, true);
  }
  return pinShapes;
}

/**
 * @brief Returns bounding box of the given pin.
 * @note Assumes pin's typeId() is either frcBTerm or frcInstTerm
 */
Rect getPinBBox(const frBlockObject* pin)
{
  if (pin->typeId() == frcBTerm) {
    return static_cast<const frBTerm*>(pin)->getBBox();
  } else {
    return static_cast<const frInstTerm*>(pin)->getBBox(true);
  }
}
/**
 * @brief Returns name of the given pin.
 * @note Assumes pin's typeId() is either frcBTerm or frcInstTerm
 */
std::string getPinName(const frBlockObject* pin)
{
  if (pin->typeId() == frcBTerm) {
    return static_cast<const frBTerm*>(pin)->getName();
  } else {
    return static_cast<const frInstTerm*>(pin)->getName();
  }
}

bool isPinCoveredByGuides(const frBlockObject* pin,
                          const std::vector<frRect>& guides)
{
  std::vector<frRect> pin_shapes = getPinShapes(pin);
  // checks if there is a guide that overlaps with any of the pin shapes
  for (const auto& pin_rect : pin_shapes) {
    for (const auto& guide : guides) {
      if (guide.getLayerNum() == pin_rect.getLayerNum()
          && guide.getBBox().overlaps(pin_rect.getBBox())) {
        return true;
      }
    }
  }
  return false;
}

/**
 * @brief Finds intersecting guides with point(x,y).
=======
/* Authors: Lutong Wang and Bangqi Xu */
/*
 * Copyright (c) 2019, The Regents of the University of California
 * All rights reserved.
>>>>>>> writer:src/drt/src/io/io_guide.cpp
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <queue>

#include "io/io.h"

namespace drt {
/* note: M1 guide special treatment. search "no M1 cross-gcell routing allowed"
 */

void getGuide(int x,
              int y,
              std::vector<int>& outGuides,
              std::vector<frRect>& guides,
              frDesign* design)
{
  ;
  Point gCell = design->getTopBlock()->getGCellCenter({x, y});
  for (int i = 0; i < (int) guides.size(); i++) {
    if (guides[i].getBBox().intersects(gCell)) {
      outGuides.push_back(i);
    }
  }
}

// Returns the manhattan distance from Point p to Rect b
int io::Parser::distL1(const Rect& b, const Point& p)
{
  int x = p.getX();
  int y = p.getY();
  int dx = (x < b.xMin()) ? b.xMin() - x : (x > b.xMax()) ? x - b.xMax() : 0;
  int dy = (y < b.yMin()) ? b.yMin() - y : (y > b.yMax()) ? y - b.yMax() : 0;
  return dx + dy;
}

<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
/**
 * @brief logs the number of guides read so far
 *
 * If the number of guides read is less than 1M guides, the function reports
 * every 100k guides, otherwise it reports every 1M guides read
 * @param num_guides number of guides read so far
 */
void logGuidesRead(const int num_guides, utl::Logger* logger)
{
  bool log = false;
  if (num_guides < 1000000) {
    log = (num_guides % 100000 == 0);
  } else {
    log = (num_guides % 1000000 == 0);
  }
  if (log) {
    logger->info(DRT, 156, "guideIn read {} guides.", num_guides);
  }
}
/**
 * @brief Checks the validity of the odb guide layer
 *
 * The db_guide layer is invalid if it is any of the following conditions:
 * - Not in the DRT layer database
 * - Above the sepecified top routing layer
 * - Below the specified bottom routing layer and the via access layer
 * @note If a layer is invalid, this produces an error unless it is above the
 * top routing layer for a net that has pins above the top routing layer. In the
 * latest case, we just ignore the guide and the pin is handled by
 * io::Parser::setBTerms_addPinFig_helper
 * @param layer_num The layer_num of the guide returned by this function if it
 * is a valid layer
 * @returns True if the guide is valid by the previous criteria and False
 * if above top routing layer for a net with bterms above top routing layer
 */
bool isValidGuideLayerNum(odb::dbGuide* db_guide,
                          frTechObject* tech,
                          frNet* net,
                          utl::Logger* logger,
                          frLayerNum& layer_num)
{
  frLayer* layer = tech->getLayer(db_guide->getLayer()->getName());
  if (layer == nullptr) {
    logger->error(
        DRT, 154, "Cannot find layer {}.", db_guide->getLayer()->getName());
  }
  layer_num = layer->getLayerNum();

  // Ignore guide as invalid if above top routing layer for a net with bterms
  // above top routing layer
  const bool guide_above_top_routing_layer = layer_num > TOP_ROUTING_LAYER;
  if (guide_above_top_routing_layer && net->hasBTermsAboveTopLayer()) {
    return false;
  }
  const bool guide_below_bottom_routing_layer
      = layer_num < BOTTOM_ROUTING_LAYER && layer_num != VIA_ACCESS_LAYERNUM;
  if (guide_below_bottom_routing_layer || guide_above_top_routing_layer) {
    logger->error(DRT,
                  155,
                  "Guide in net {} uses layer {} ({})"
                  " that is outside the allowed routing range "
                  "[{} ({}), {} ({})] with via access on [{} ({})].",
                  net->getName(),
                  layer->getName(),
                  layer_num,
                  tech->getLayer(BOTTOM_ROUTING_LAYER)->getName(),
                  BOTTOM_ROUTING_LAYER,
                  tech->getLayer(TOP_ROUTING_LAYER)->getName(),
                  TOP_ROUTING_LAYER,
                  tech->getLayer(VIA_ACCESS_LAYERNUM)->getName(),
                  VIA_ACCESS_LAYERNUM);
  }
  return true;
}

/**
 * @brief Initializes guide intervals for genGuides_merge.
 *
 * The function iterates over all the net guides and identifies each one's
 * coverage of the gcell indices. The data structure merges touching guides as
 * they intersect in their intervals. For a guide on a horizontal layer spanning
 * from index(1, 10) to (10, 12). It adds the following entries to
 * intvs[layer_num]:
 * - Track 10 begin_idx 1 end_idx 10 : intvs[layer_num][10].insert(1, 10)
 * - Track 11 begin_idx 1 end_idx 10 : intvs[layer_num][11].insert(1, 10)
 * - Track 12 begin_idx 1 end_idx 10 : intvs[layer_num][12].insert(1, 10)
 * As you can see the map key/track idx is the vertical index because this is a
 * horizontal layer.
 * @param rects The list of guides of the net.
 * @param intvs The map of guide intervals.
 */
void initGuideIntervals(const std::vector<frRect>& rects,
                        const frDesign* design,
                        TrackIntervalsByLayer& intvs)
{
  for (const auto& rect : rects) {
    const Rect box = rect.getBBox();
    const Point pt1(box.ll());
    const Point idx1 = design->getTopBlock()->getGCellIdx(pt1);
    const frCoord x1 = idx1.x();
    const frCoord y1 = idx1.y();
    const Point pt2(box.xMax() - 1, box.yMax() - 1);
    const Point idx2 = design->getTopBlock()->getGCellIdx(pt2);
    const frCoord x2 = idx2.x();
    const frCoord y2 = idx2.y();
    const frLayerNum layer_num = rect.getLayerNum();
    const bool is_horizontal = design->getTech()->getLayer(layer_num)->getDir()
                               == dbTechLayerDir::HORIZONTAL;
    if (is_horizontal) {
      for (auto track_idx = y1; track_idx <= y2; track_idx++) {
        intvs[layer_num][track_idx].insert(Interval::closed(x1, x2));
      }
    } else {
      for (auto track_idx = x1; track_idx <= x2; track_idx++) {
        intvs[layer_num][track_idx].insert(Interval::closed(y1, y2));
      }
    }
  }
}
/**
 * @brief Checks if there exists an interval with the specified passed arguments
 *
 * The function checks if there exists an interval on layer_num with any track
 * index from begin_idx to end_idx that contains both indices; track_idx1 and
 * track_idx2. This is used for discovering bridge guides over touching guides
 * on consecutive track indices.
 * @param begin_idx The lower bound of track indices for searching.
 * @param end_idx The upper bound of track indices for searching.
 * @returns True if it finds a bridge with the specified criteria and False
 * otherwise
 */
bool hasGuideInterval(const frCoord begin_idx,
                      const frCoord end_idx,
                      const frCoord track_idx1,
                      const frCoord track_idx2,
                      const frLayerNum layer_num,
                      const TrackIntervalsByLayer& intvs)
{
  for (auto it = intvs[layer_num].lower_bound(begin_idx);
       it != intvs[layer_num].end() && it->first <= end_idx;
       it++) {
    if (boost::icl::contains(it->second, track_idx1)
        && boost::icl::contains(it->second, track_idx2)) {
      return true;
    }
  }
  return false;
}
/**
 * @brief Adds Bridge guides for touching guides on the same layer if needed.
 *
 * The function identifies touching guides on the same layer on consecutive
 * track indices. It adds a bridge guide on the upper or lower layer that
 * connects both guides if such one does not exist.
 * @note The function prefers bridging on the upper layer to the lower layer.
 */
void addTouchingGuidesBridges(TrackIntervalsByLayer& intvs, utl::Logger* logger)
{
  struct BridgeGuide
  {
    frCoord track_idx{-1};
    frCoord begin_idx{-1};
    frCoord end_idx{-1};
    frLayerNum layer_num{-1};
    BridgeGuide(frCoord track_idx_in,
                frCoord begin_idx_in,
                frCoord end_idx_in,
                frLayerNum layer_num_in)
        : track_idx(track_idx_in),
          begin_idx(begin_idx_in),
          end_idx(end_idx_in),
          layer_num(layer_num_in)
    {
    }
  };

  std::vector<BridgeGuide> bridge_guides;
  // append touching edges
  for (int layer_num = 0; layer_num < (int) intvs.size(); layer_num++) {
    const auto& curr_layer_intvs = intvs[layer_num];
    int prev_track_idx = -2;
    for (const auto& [curr_track_idx, curr_track_intvs] : curr_layer_intvs) {
      if (curr_track_idx == prev_track_idx + 1) {
        const auto& prev_track_intvs = curr_layer_intvs.at(prev_track_idx);
        const auto intvs_intersection = curr_track_intvs & prev_track_intvs;
        for (const auto& intv : intvs_intersection) {
          const auto begin_idx = intv.lower();
          const auto end_idx = intv.upper();
          bool has_bridge = false;
          // lower layer intersection
          std::optional<frLayerNum> bridge_layer_num;
          if (layer_num - 2 >= 0) {
            bridge_layer_num = layer_num - 2;
            has_bridge = hasGuideInterval(begin_idx,
                                          end_idx,
                                          curr_track_idx,
                                          prev_track_idx,
                                          bridge_layer_num.value(),
                                          intvs);
          }
          if (layer_num + 2 < (int) intvs.size() && !has_bridge) {
            bridge_layer_num = layer_num + 2;
            has_bridge = hasGuideInterval(begin_idx,
                                          end_idx,
                                          curr_track_idx,
                                          prev_track_idx,
                                          bridge_layer_num.value(),
                                          intvs);
          }
          if (!has_bridge) {
            // add bridge guide;
            if (!bridge_layer_num.has_value()) {
              logger->error(
                  DRT, 228, "genGuides_merge cannot find bridge layer.");
            }
            bridge_guides.emplace_back(begin_idx,
                                       prev_track_idx,
                                       curr_track_idx,
                                       bridge_layer_num.value());
          }
        }
      }
      prev_track_idx = curr_track_idx;
    }
  }

  for (const auto& bridge : bridge_guides) {
    intvs[bridge.layer_num][bridge.track_idx].insert(
        Interval::closed(bridge.begin_idx, bridge.end_idx));
  }
}

}  // namespace

bool GuideProcessor::readGuides()
{
  ProfileTask profile("IO:readGuide");
  int num_guides = 0;
  const auto block = db_->getChip()->getBlock();
  for (const auto db_net : block->getNets()) {
    frNet* net = getDesign()->getTopBlock()->findNet(db_net->getName());
    if (net == nullptr) {
      logger_->error(DRT, 153, "Cannot find net {}.", db_net->getName());
    }
    for (auto db_guide : db_net->getGuides()) {
      frLayerNum layer_num;
      if (!isValidGuideLayerNum(db_guide, getTech(), net, logger_, layer_num)) {
        continue;
      }
      frRect rect;
      rect.setBBox(db_guide->getBox());
      rect.setLayerNum(layer_num);
      tmp_guides_[net].emplace_back(rect);
      ++num_guides;
      logGuidesRead(num_guides, logger_);
    }
  }
  if (VERBOSE > 0) {
    logger_->report("");
    logger_->info(utl::DRT, 157, "Number of guides:     {}", num_guides);
    logger_->report("");
  }
  return !tmp_guides_.empty();
=======
// Returns (by reference) the closest point inside r to Point3D p
void io::Parser::getClosestPoint(const frRect& r,
                                 const Point3D& p,
                                 Point3D& result)
{
  int px = p.getX();
  int py = p.getY();
  Rect b = r.getBBox();
  int x = (px < b.xMin()) ? b.xMin() : (px > b.xMax()) ? b.xMax() : px;
  int y = (py < b.yMin()) ? b.yMin() : (py > b.yMax()) ? b.yMax() : py;
  result.set(x, y, r.getLayerNum());
>>>>>>> writer:src/drt/src/io/io_guide.cpp
}

// extends/adds guides to cover (at least the major part of) the pins that were
// considered disconnected from the guides
void io::Parser::patchGuides(frNet* net,
                             frBlockObject* pin,
                             std::vector<frRect>& guides)
{
<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
  buildGCellPatterns_getWidth(GCELLGRIDX, GCELLGRIDY);
  buildGCellPatterns_getOffset(
      GCELLGRIDX, GCELLGRIDY, GCELLOFFSETX, GCELLOFFSETY);
}

void GuideProcessor::buildGCellPatterns_getWidth(frCoord& GCELLGRIDX,
                                                 frCoord& GCELLGRIDY)
{
  std::map<frCoord, int> guideGridXMap, guideGridYMap;
  // get GCell size information loop
  for (auto& [netName, rects] : tmp_guides_) {
    for (auto& rect : rects) {
      frLayerNum layerNum = rect.getLayerNum();
      Rect guideBBox = rect.getBBox();
      frCoord guideWidth = (getTech()->getLayer(layerNum)->getDir()
                            == dbTechLayerDir::HORIZONTAL)
                               ? guideBBox.dy()
                               : guideBBox.dx();
      if (getTech()->getLayer(layerNum)->getDir()
          == dbTechLayerDir::HORIZONTAL) {
        if (guideGridYMap.find(guideWidth) == guideGridYMap.end()) {
          guideGridYMap[guideWidth] = 0;
        }
        guideGridYMap[guideWidth]++;
      } else if (getTech()->getLayer(layerNum)->getDir()
                 == dbTechLayerDir::VERTICAL) {
        if (guideGridXMap.find(guideWidth) == guideGridXMap.end()) {
          guideGridXMap[guideWidth] = 0;
        }
        guideGridXMap[guideWidth]++;
      }
=======
  // get the gCells of the pin, and is shapes (rects)
  Rect pinBBox;
  std::vector<frRect> pinShapes;
  std::string name;
  switch (pin->typeId()) {
    case frcBTerm: {
      frBTerm* term = static_cast<frBTerm*>(pin);
      term->getShapes(pinShapes);
      pinBBox = term->getBBox();
      name = term->getName();
      break;
>>>>>>> writer:src/drt/src/io/io_guide.cpp
    }
    case frcInstTerm: {
      frInstTerm* iTerm = static_cast<frInstTerm*>(pin);
      iTerm->getShapes(pinShapes, true);
      pinBBox = iTerm->getBBox(true);
      name = iTerm->getName();
      break;
    }
    default:
      logger_->error(DRT, 1007, "PatchGuides invoked with non-term object.");
  }
<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
  for (const auto [coord, cnt] : guideGridYMap) {
    if (cnt > tmpGCELLGRIDYCnt) {
      tmpGCELLGRIDYCnt = cnt;
      tmpGCELLGRIDY = coord;
    }
  }
  if (tmpGCELLGRIDX != -1) {
    GCELLGRIDX = tmpGCELLGRIDX;
  } else {
    logger_->error(DRT, 170, "No GCELLGRIDX.");
  }
  if (tmpGCELLGRIDY != -1) {
    GCELLGRIDY = tmpGCELLGRIDY;
  } else {
    logger_->error(DRT, 171, "No GCELLGRIDY.");
  }
}

void GuideProcessor::buildGCellPatterns_getOffset(frCoord GCELLGRIDX,
                                                  frCoord GCELLGRIDY,
                                                  frCoord& GCELLOFFSETX,
                                                  frCoord& GCELLOFFSETY)
{
  std::map<frCoord, int> guideOffsetXMap, guideOffsetYMap;
  // get GCell offset information loop
  for (auto& [netName, rects] : tmp_guides_) {
    for (auto& rect : rects) {
      // frLayerNum layerNum = rect.getLayerNum();
      Rect guideBBox = rect.getBBox();
      frCoord guideXOffset = guideBBox.xMin() % GCELLGRIDX;
      frCoord guideYOffset = guideBBox.yMin() % GCELLGRIDY;
      if (guideXOffset < 0) {
        guideXOffset = GCELLGRIDX - guideXOffset;
      }
      if (guideYOffset < 0) {
        guideYOffset = GCELLGRIDY - guideYOffset;
      }
      if (guideOffsetXMap.find(guideXOffset) == guideOffsetXMap.end()) {
        guideOffsetXMap[guideXOffset] = 0;
      }
      guideOffsetXMap[guideXOffset]++;
      if (guideOffsetYMap.find(guideYOffset) == guideOffsetYMap.end()) {
        guideOffsetYMap[guideYOffset] = 0;
      }
      guideOffsetYMap[guideYOffset]++;
    }
  }
  frCoord tmpGCELLOFFSETX = -1, tmpGCELLOFFSETY = -1;
  int tmpGCELLOFFSETXCnt = -1, tmpGCELLOFFSETYCnt = -1;
  for (const auto [coord, cnt] : guideOffsetXMap) {
    if (cnt > tmpGCELLOFFSETXCnt) {
      tmpGCELLOFFSETXCnt = cnt;
      tmpGCELLOFFSETX = coord;
    }
  }
  for (const auto [coord, cnt] : guideOffsetYMap) {
    if (cnt > tmpGCELLOFFSETYCnt) {
      tmpGCELLOFFSETYCnt = cnt;
      tmpGCELLOFFSETY = coord;
    }
  }
  if (tmpGCELLOFFSETX != -1) {
    GCELLOFFSETX = tmpGCELLOFFSETX;
  } else {
    logger_->error(DRT, 172, "No GCELLGRIDX.");
  }
  if (tmpGCELLOFFSETY != -1) {
    GCELLOFFSETY = tmpGCELLOFFSETY;
  } else {
    logger_->error(DRT, 173, "No GCELLGRIDY.");
  }
}

void GuideProcessor::buildGCellPatterns()
{
  // horizontal = false is gcell lines along y direction (x-grid)
  frGCellPattern xgp, ygp;
  frCoord GCELLOFFSETX, GCELLOFFSETY, GCELLGRIDX, GCELLGRIDY;
  auto gcellGrid = db_->getChip()->getBlock()->getGCellGrid();
  if (gcellGrid != nullptr && gcellGrid->getNumGridPatternsX() == 1
      && gcellGrid->getNumGridPatternsY() == 1) {
    frCoord COUNTX, COUNTY;
    gcellGrid->getGridPatternX(0, GCELLOFFSETX, COUNTX, GCELLGRIDX);
    gcellGrid->getGridPatternY(0, GCELLOFFSETY, COUNTY, GCELLGRIDY);
    xgp.setStartCoord(GCELLOFFSETX);
    xgp.setSpacing(GCELLGRIDX);
    xgp.setCount(COUNTX);
    xgp.setHorizontal(false);

    ygp.setStartCoord(GCELLOFFSETY);
    ygp.setSpacing(GCELLGRIDY);
    ygp.setCount(COUNTY);
    ygp.setHorizontal(true);

  } else {
    Rect dieBox = getDesign()->getTopBlock()->getDieBox();
    buildGCellPatterns_helper(
        GCELLGRIDX, GCELLGRIDY, GCELLOFFSETX, GCELLOFFSETY);
    xgp.setHorizontal(false);
    // find first coord >= dieBox.xMin()
    frCoord startCoordX
        = dieBox.xMin() / (frCoord) GCELLGRIDX * (frCoord) GCELLGRIDX
          + GCELLOFFSETX;
    if (startCoordX > dieBox.xMin()) {
      startCoordX -= (frCoord) GCELLGRIDX;
    }
    xgp.setStartCoord(startCoordX);
    xgp.setSpacing(GCELLGRIDX);
    if ((dieBox.xMax() - (frCoord) GCELLOFFSETX) / (frCoord) GCELLGRIDX < 1) {
      logger_->error(DRT, 174, "GCell cnt x < 1.");
    }
    xgp.setCount((dieBox.xMax() - (frCoord) startCoordX)
                 / (frCoord) GCELLGRIDX);

    ygp.setHorizontal(true);
    // find first coord >= dieBox.yMin()
    frCoord startCoordY
        = dieBox.yMin() / (frCoord) GCELLGRIDY * (frCoord) GCELLGRIDY
          + GCELLOFFSETY;
    if (startCoordY > dieBox.yMin()) {
      startCoordY -= (frCoord) GCELLGRIDY;
    }
    ygp.setStartCoord(startCoordY);
    ygp.setSpacing(GCELLGRIDY);
    if ((dieBox.yMax() - (frCoord) GCELLOFFSETY) / (frCoord) GCELLGRIDY < 1) {
      logger_->error(DRT, 175, "GCell cnt y < 1.");
    }
    ygp.setCount((dieBox.yMax() - startCoordY) / (frCoord) GCELLGRIDY);
  }

  if (VERBOSE > 0 || logger_->debugCheck(DRT, "autotuner", 1)) {
    logger_->info(DRT,
                  176,
                  "GCELLGRID X {} DO {} STEP {} ;",
                  xgp.getStartCoord(),
                  xgp.getCount(),
                  xgp.getSpacing());
    logger_->info(DRT,
                  177,
                  "GCELLGRID Y {} DO {} STEP {} ;",
                  ygp.getStartCoord(),
                  ygp.getCount(),
                  ygp.getSpacing());
  }

  getDesign()->getTopBlock()->setGCellPatterns(
      {std::move(xgp), std::move(ygp)});
}

void GuideProcessor::patchGuides_helper(frNet* net,
                                        std::vector<frRect>& guides,
                                        const Point3D& best_pin_loc_idx,
                                        const Point3D& best_pin_loc_coords,
                                        const int closest_guide_idx)
{
  Point3D guide_pt(
      getClosestPoint(guides[closest_guide_idx], best_pin_loc_coords),
      guides[closest_guide_idx].getLayerNum());
  const Rect& guide_bbox = guides[closest_guide_idx].getBBox();
  const frCoord gcell_half_size_horz
      = getDesign()->getTopBlock()->getGCellSizeHorizontal() / 2;
  const frCoord gcell_half_size_vert
      = getDesign()->getTopBlock()->getGCellSizeVertical() / 2;
  adjustGuidePoint(
      guide_pt, guide_bbox, gcell_half_size_horz, gcell_half_size_vert);
  extendGuide(getDesign(),
              best_pin_loc_coords,
              gcell_half_size_horz,
              gcell_half_size_vert,
              guides[closest_guide_idx],
              guide_pt);
  if (guide_pt == best_pin_loc_coords) {
    return;
  }
  connectGuidesWithBestPinLoc(guide_pt,
                              best_pin_loc_coords,
                              gcell_half_size_horz,
                              gcell_half_size_vert,
                              net,
                              guides);
  // fill the gap between current layer and the best_pin_loc_coords layer with
  // guides
  fillGuidesUpToZ(best_pin_loc_coords,
                  guide_pt.z(),
                  gcell_half_size_horz,
                  gcell_half_size_vert,
                  net,
                  guides);
}

void GuideProcessor::patchGuides(frNet* net,
                                 frBlockObject* pin,
                                 std::vector<frRect>& guides)
{
  if (pin->typeId() != frcBTerm && pin->typeId() != frcInstTerm) {
    logger_->error(DRT, 1007, "patchGuides invoked with non-term object.");
  }
  if (isPinCoveredByGuides(pin, guides)) {
    return;
  }
  // no guide was found that overlaps with any of the pin shapes, then we patch
  // the guides

  const std::string name = getPinName(pin);
=======
>>>>>>> writer:src/drt/src/io/io_guide.cpp
  logger_->info(DRT,
                1000,
                "Pin {} not in any guide. Attempting to patch guides to cover "
                "(at least part of) the pin.",
                name);
  pinBBox.init(
      pinBBox.xMin() + 1,
      pinBBox.yMin() + 1,
      pinBBox.xMax() - 1,
      pinBBox.yMax()
          - 1);  // pins tangent to gcell aren't considered as part of them
  // set pinBBox to gCell coords
  Point llGcell = design_->getTopBlock()->getGCellIdx(pinBBox.ll());
  Point urGcell = design_->getTopBlock()->getGCellIdx(pinBBox.ur());

  // finds the gCell with higher pinShape overlapping area (approximate)
  frArea bestArea = 0, area = 0;
  Point3D bestPinLocIdx;
  std::vector<int> candidateGuides;  // indexes of guides in rects
  for (int x = llGcell.x(); x <= urGcell.x(); x++) {
    for (int y = llGcell.y(); y <= urGcell.y(); y++) {
      Rect intersection;
      Point gCell(x, y);
      Rect gCellBox = design_->getTopBlock()->getGCellBox(gCell);
      for (int z = 0; z < (int) design_->getTech()->getLayers().size(); z++) {
        if (design_->getTech()->getLayer(z)->getType()
            != dbTechLayerType::ROUTING) {
          continue;
        }
        area = 0;
        for (auto& pinRect : pinShapes) {
          if (pinRect.getLayerNum() != z) {
            continue;
          }
          gCellBox.intersection(pinRect.getBBox(), intersection);
          area += intersection.area();
        }
        if (area > bestArea) {
          bestArea = area;
          bestPinLocIdx.set(x, y, z);
        }
      }
      // finds guides in the neighboring gCells
      getGuide(x - 1, y, candidateGuides, guides, design_);
      getGuide(x + 1, y, candidateGuides, guides, design_);
      getGuide(x, y - 1, candidateGuides, guides, design_);
      getGuide(x, y + 1, candidateGuides, guides, design_);
    }
  }
  if (candidateGuides.empty()) {
    logger_->warn(DRT, 1001, "No guide in the pin neighborhood");
    return;
  }
  // get the guide that is closer to the gCell
  int closerGuideIdx = -1;
  int dist = 0;
  int closerDist = std::numeric_limits<int>::max();
  Point center = design_->getTopBlock()->getGCellCenter(bestPinLocIdx);
  Point3D bestPinLocCoords(center.x(), center.y(), 0);
  for (auto& guideIdx : candidateGuides) {
    dist = distL1(guides[guideIdx].getBBox(), bestPinLocCoords);
    dist += abs(guides[guideIdx].getLayerNum() - bestPinLocIdx.z());
    if (dist < closerDist) {
      closerDist = dist;
      closerGuideIdx = guideIdx;
    }
  }
  //    design->getTopBlock()->getGCellIdx(guides[closerGuideIdx].getBBox().ll(),
  //    pl);
  //    design->getTopBlock()->getGCellIdx(guides[closerGuideIdx].getBBox().ur(),
  //    ph); frRect closerGuide(pl.x(), pl.y(), ph.x(), ph.y(),
  //    guides[closerGuideIdx].getLayerNum(), net);
  // gets the point in the closer guide that is closer to the bestPinLoc
<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
  patchGuides_helper(
      net, guides, best_pin_loc_idx, best_pin_loc_coords, closest_guide_idx);
=======
  Point3D guidePt;
  getClosestPoint(guides[closerGuideIdx], bestPinLocCoords, guidePt);
  const Rect& guideBox = guides[closerGuideIdx].getBBox();
  frCoord gCellX = design_->getTopBlock()->getGCellSizeHorizontal();
  frCoord gCellY = design_->getTopBlock()->getGCellSizeVertical();
  if (guidePt.x() == guideBox.xMin()
      || std::abs(guideBox.xMin() - guidePt.x())
             <= std::abs(guideBox.xMax() - guidePt.x())) {
    guidePt.setX(guideBox.xMin() + gCellX / 2);
  } else if (guidePt.x() == guideBox.xMax()
             || std::abs(guideBox.xMax() - guidePt.x())
                    <= std::abs(guideBox.xMin() - guidePt.x())) {
    guidePt.setX(guideBox.xMax() - gCellX / 2);
  }
  if (guidePt.y() == guideBox.yMin()
      || std::abs(guideBox.yMin() - guidePt.y())
             <= std::abs(guideBox.yMax() - guidePt.y())) {
    guidePt.setY(guideBox.yMin() + gCellY / 2);
  } else if (guidePt.y() == guideBox.yMax()
             || std::abs(guideBox.yMax() - guidePt.y())
                    <= std::abs(guideBox.yMin() - guidePt.y())) {
    guidePt.setY(guideBox.yMax() - gCellY / 2);
  }

  // connect bestPinLoc to guidePt by creating "patch" guides
  // first, try to extend closerGuide
  if (design_->isHorizontalLayer(guidePt.z())) {
    if (guidePt.x() != bestPinLocCoords.x()) {
      if (bestPinLocCoords.x() < guideBox.xMin()) {
        guides[closerGuideIdx].setLeft(bestPinLocCoords.x() - gCellX / 2);
      } else if (bestPinLocCoords.x() > guideBox.xMax()) {
        guides[closerGuideIdx].setRight(bestPinLocCoords.x() + gCellX / 2);
      }
      guidePt.setX(bestPinLocCoords.x());
    }
  } else if (design_->isVerticalLayer(guidePt.z())) {
    if (guidePt.y() != bestPinLocCoords.y()) {
      if (bestPinLocCoords.y() < guideBox.yMin()) {
        guides[closerGuideIdx].setBottom(bestPinLocCoords.y() - gCellY / 2);
      } else if (bestPinLocCoords.y() > guideBox.yMax()) {
        guides[closerGuideIdx].setTop(bestPinLocCoords.y() + gCellY / 2);
      }
      guidePt.setY(bestPinLocCoords.y());
    }
  } else {
    logger_->error(DRT, 1002, "Layer is not horizontal or vertical");
  }

  if (guidePt == bestPinLocCoords) {
    return;
  }
  int z = guidePt.z();
  if (guidePt.x() != bestPinLocCoords.x()
      || guidePt.y() != bestPinLocCoords.y()) {
    Point pl, ph;
    pl = {std::min(bestPinLocCoords.x(), guidePt.x()),
          std::min(bestPinLocCoords.y(), guidePt.y())};
    ph = {std::max(bestPinLocCoords.x(), guidePt.x()),
          std::max(bestPinLocCoords.y(), guidePt.y())};

    guides.emplace_back(pl.x() - gCellX / 2,
                        pl.y() - gCellY / 2,
                        ph.x() + gCellX / 2,
                        ph.y() + gCellY / 2,
                        z,
                        net);
  }

  // fill the gap between current layer and the bestPinLocCoords layer with
  // guides
  int inc = z < bestPinLocIdx.z() ? 2 : -2;
  for (z = z + inc; z != bestPinLocIdx.z() + inc; z += inc) {
    guides.emplace_back(bestPinLocCoords.x() - gCellX / 2,
                        bestPinLocCoords.y() - gCellY / 2,
                        bestPinLocCoords.x() + gCellX / 2,
                        bestPinLocCoords.y() + gCellY / 2,
                        z,
                        net);
  }
>>>>>>> writer:src/drt/src/io/io_guide.cpp
}

void io::Parser::genGuides_pinEnclosure(frNet* net, std::vector<frRect>& guides)
{
  for (auto pin : net->getInstTerms()) {
    patchGuides(net, pin, guides);
  }
  for (auto pin : net->getBTerms()) {
    patchGuides(net, pin, guides);
  }
}

<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
void GuideProcessor::genGuides_prep(const std::vector<frRect>& rects,
                                    TrackIntervalsByLayer& intvs)
{
  initGuideIntervals(rects, getDesign(), intvs);
  addTouchingGuidesBridges(intvs, logger_);
=======
void io::Parser::checkPinForGuideEnclosure(frBlockObject* pin,
                                           frNet* net,
                                           std::vector<frRect>& guides)
{
  std::vector<frRect> pinShapes;
  switch (pin->typeId()) {
    case frcBTerm: {
      static_cast<frTerm*>(pin)->getShapes(pinShapes);
      break;
    }
    case frcInstTerm: {
      static_cast<frInstTerm*>(pin)->getShapes(pinShapes, true);
      break;
    }
    default:
      logger_->error(
          DRT, 1008, "checkPinForGuideEnclosure invoked with non-term object.");
  }
  for (auto& pinRect : pinShapes) {
    for (auto& guide : guides) {
      if (pinRect.getLayerNum() == guide.getLayerNum()
          && guide.getBBox().overlaps(pinRect.getBBox())) {
        return;
      }
    }
  }
  patchGuides(net, pin, guides);
}

void io::Parser::genGuides_merge(
    std::vector<frRect>& rects,
    std::vector<std::map<frCoord, boost::icl::interval_set<frCoord>>>& intvs)
{
  for (auto& rect : rects) {
    if (rect.getLayerNum() > TOP_ROUTING_LAYER) {
      logger_->error(DRT,
                     3000,
                     "Guide in layer {} which is above max routing layer {}",
                     rect.getLayerNum(),
                     TOP_ROUTING_LAYER);
    }
    Rect box = rect.getBBox();
    Point pt(box.ll());
    Point idx = design_->getTopBlock()->getGCellIdx(pt);
    frCoord x1 = idx.x();
    frCoord y1 = idx.y();
    pt = {box.xMax() - 1, box.yMax() - 1};
    idx = design_->getTopBlock()->getGCellIdx(pt);
    frCoord x2 = idx.x();
    frCoord y2 = idx.y();
    auto layerNum = rect.getLayerNum();
    if (tech_->getLayer(layerNum)->getDir() == dbTechLayerDir::HORIZONTAL) {
      for (auto i = y1; i <= y2; i++) {
        intvs[layerNum][i].insert(
            boost::icl::interval<frCoord>::closed(x1, x2));
      }
    } else {
      for (auto i = x1; i <= x2; i++) {
        intvs[layerNum][i].insert(
            boost::icl::interval<frCoord>::closed(y1, y2));
      }
    }
  }
  // trackIdx, beginIdx, endIdx, layerNum
  std::vector<std::tuple<frCoord, frCoord, frCoord, frLayerNum>> touchGuides;
  // append touching edges
  for (int lNum = 0; lNum < (int) intvs.size(); lNum++) {
    auto& m = intvs[lNum];
    int prevTrackIdx = -2;
    for (auto& [trackIdx, intvS] : m) {
      if (trackIdx == prevTrackIdx + 1) {
        auto& prevIntvS = m[prevTrackIdx];
        auto newIntv = intvS & prevIntvS;
        for (const auto& intv : newIntv) {
          auto beginIdx = intv.lower();
          auto endIdx = intv.upper();
          bool haveLU = false;
          // lower layer intersection
          if (lNum - 2 >= 0) {
            auto nbrLayerNum = lNum - 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx);
                 it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx;
                 it2++) {
              if (boost::icl::contains(it2->second, trackIdx)
                  && boost::icl::contains(it2->second, prevTrackIdx)) {
                haveLU = true;
                break;
              }
            }
          }
          if (lNum + 2 < (int) intvs.size() && !haveLU) {
            auto nbrLayerNum = lNum + 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx);
                 it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx;
                 it2++) {
              if (boost::icl::contains(it2->second, trackIdx)
                  && boost::icl::contains(it2->second, prevTrackIdx)) {
                haveLU = true;
                break;
              }
            }
          }
          if (!haveLU) {
            // add touching guide;
            // std::cout <<"found touching guide" <<std::endl;
            if (lNum + 2 < (int) intvs.size()) {
              touchGuides.emplace_back(
                  beginIdx, prevTrackIdx, trackIdx, lNum + 2);
            } else if (lNum - 2 >= 0) {
              touchGuides.emplace_back(
                  beginIdx, prevTrackIdx, trackIdx, lNum - 2);
            } else {
              logger_->error(
                  DRT, 228, "genGuides_merge cannot find touching layer.");
            }
          }
        }
      }
      prevTrackIdx = trackIdx;
    }
  }

  for (auto& [trackIdx, beginIdx, endIdx, lNum] : touchGuides) {
    intvs[lNum][trackIdx].insert(
        boost::icl::interval<frCoord>::closed(beginIdx, endIdx));
  }
>>>>>>> writer:src/drt/src/io/io_guide.cpp
}

void io::Parser::genGuides_split(
    std::vector<frRect>& rects,
    TrackIntervalsByLayer& intvs,
    std::map<std::pair<Point, frLayerNum>,
             std::set<frBlockObject*, frBlockObjectComp>>& gCell2PinMap,
    std::map<frBlockObject*,
             std::set<std::pair<Point, frLayerNum>>,
             frBlockObjectComp>& pin2GCellMap,
    bool retry)
{
  rects.clear();
  // layerNum->trackIdx->beginIdx->set of obj
  std::vector<
      std::map<frCoord,
               std::map<frCoord, std::set<frBlockObject*, frBlockObjectComp>>>>
      pin_helper(design_->getTech()->getLayers().size());
  for (auto& [pr, objS] : gCell2PinMap) {
    auto& point = pr.first;
    auto& lNum = pr.second;
    if (design_->getTech()->getLayer(lNum)->getDir()
        == dbTechLayerDir::HORIZONTAL) {
      pin_helper[lNum][point.y()][point.x()] = objS;
    } else {
      pin_helper[lNum][point.x()][point.y()] = objS;
    }
  }

  for (int layerNum = 0; layerNum < (int) intvs.size(); layerNum++) {
    auto dir = design_->getTech()->getLayer(layerNum)->getDir();
    for (auto& [trackIdx, curr_intvs] : intvs[layerNum]) {
      // split by lower/upper seg
      for (const auto& intv : curr_intvs) {
        std::set<frCoord> lineIdx;
        auto beginIdx = intv.lower();
        auto endIdx = intv.upper();
        // hardcode layerNum <= VIA_ACCESS_LAYERNUM not used for GR
        if (!retry && layerNum <= VIA_ACCESS_LAYERNUM) {
          // split by pin
          if (pin_helper[layerNum].find(trackIdx)
              != pin_helper[layerNum].end()) {
            auto& pin_helper_map = pin_helper[layerNum][trackIdx];
            for (auto it2 = pin_helper_map.lower_bound(beginIdx);
                 it2 != pin_helper_map.end() && it2->first <= endIdx;
                 it2++) {
              // add pin2GCellmap
              for (auto obj : it2->second) {
                if (dir == dbTechLayerDir::HORIZONTAL) {
                  pin2GCellMap[obj].insert(
                      std::make_pair(Point(it2->first, trackIdx), layerNum));
                } else {
                  pin2GCellMap[obj].insert(
                      std::make_pair(Point(trackIdx, it2->first), layerNum));
                }
              }
              // std::cout <<"pin split" <<std::endl;
            }
          }
          for (int x = beginIdx; x <= endIdx; x++) {
            frRect tmpRect;
            if (dir == dbTechLayerDir::HORIZONTAL) {
              tmpRect.setBBox(Rect(x, trackIdx, x, trackIdx));
            } else {
              tmpRect.setBBox(Rect(trackIdx, x, trackIdx, x));
            }
            tmpRect.setLayerNum(layerNum);
            rects.push_back(tmpRect);
          }
        } else {
          // lower layer intersection
          if (layerNum - 2 >= 0) {
            auto nbrLayerNum = layerNum - 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx);
                 it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx;
                 it2++) {
              if (boost::icl::contains(it2->second, trackIdx)) {
                lineIdx.insert(
                    it2->first);  // it2->first is intersection frCoord
                // std::cout <<"found split point" <<std::endl;
              }
            }
          }
          if (layerNum + 2 < (int) intvs.size()) {
            auto nbrLayerNum = layerNum + 2;
            for (auto it2 = intvs[nbrLayerNum].lower_bound(beginIdx);
                 it2 != intvs[nbrLayerNum].end() && it2->first <= endIdx;
                 it2++) {
              if (boost::icl::contains(it2->second, trackIdx)) {
                lineIdx.insert(it2->first);
                // std::cout <<"found split point" <<std::endl;
              }
            }
          }
          // split by pin
          if (pin_helper[layerNum].find(trackIdx)
              != pin_helper[layerNum].end()) {
            auto& pin_helper_map = pin_helper[layerNum][trackIdx];
            for (auto it2 = pin_helper_map.lower_bound(beginIdx);
                 it2 != pin_helper_map.end() && it2->first <= endIdx;
                 it2++) {
              lineIdx.insert(it2->first);
              // add pin2GCellMap
              for (auto obj : it2->second) {
                if (dir == dbTechLayerDir::HORIZONTAL) {
                  pin2GCellMap[obj].insert(
                      std::make_pair(Point(it2->first, trackIdx), layerNum));
                } else {
                  pin2GCellMap[obj].insert(
                      std::make_pair(Point(trackIdx, it2->first), layerNum));
                }
              }
              // std::cout <<"pin split" <<std::endl;
            }
          }
          // add rect
          if (lineIdx.empty()) {
            logger_->error(DRT,
                           229,
                           "genGuides_split lineIdx is empty on {}.",
                           design_->getTech()->getLayer(layerNum)->getName());
          } else if (lineIdx.size() == 1) {
            auto x = *(lineIdx.begin());
            frRect tmpRect;
            if (dir == dbTechLayerDir::HORIZONTAL) {
              tmpRect.setBBox(Rect(x, trackIdx, x, trackIdx));
            } else {
              tmpRect.setBBox(Rect(trackIdx, x, trackIdx, x));
            }
            tmpRect.setLayerNum(layerNum);
            rects.push_back(tmpRect);
          } else {
            auto prevIt = lineIdx.begin();
            for (auto currIt = (++(lineIdx.begin())); currIt != lineIdx.end();
                 currIt++) {
              frRect tmpRect;
              if (dir == dbTechLayerDir::HORIZONTAL) {
                tmpRect.setBBox(Rect(*prevIt, trackIdx, *currIt, trackIdx));
              } else {
                tmpRect.setBBox(Rect(trackIdx, *prevIt, trackIdx, *currIt));
              }
              tmpRect.setLayerNum(layerNum);
              prevIt = currIt;
              rects.push_back(tmpRect);
            }
          }
        }
      }
    }
  }
  rects.shrink_to_fit();
}

template <typename T>
void io::Parser::genGuides_gCell2TermMap(
    std::map<std::pair<Point, frLayerNum>,
             std::set<frBlockObject*, frBlockObjectComp>>& gCell2PinMap,
    T* term,
    frBlockObject* origTerm,
    const dbTransform& xform)
{
  for (auto& uPin : term->getPins()) {
    for (auto& uFig : uPin->getFigs()) {
      auto fig = uFig.get();
      if (fig->typeId() != frcRect) {
        logger_->error(DRT, 232, "genGuides_gCell2TermMap unsupported pinfig.");
      }
      auto shape = static_cast<frRect*>(fig);
      auto lNum = shape->getLayerNum();
      auto layer = design_->getTech()->getLayer(lNum);
      Rect box = shape->getBBox();
      xform.apply(box);
      Point pt(box.xMin() + 1, box.yMin() + 1);
      Point idx = design_->getTopBlock()->getGCellIdx(pt);
      frCoord x1 = idx.x();
      frCoord y1 = idx.y();
      pt = {box.ur().x() - 1, box.ur().y() - 1};
      idx = design_->getTopBlock()->getGCellIdx(pt);
      frCoord x2 = idx.x();
      frCoord y2 = idx.y();
      // ispd18_test4 and ispd18_test5 have zero overlap guide
      // excludes double-zero overlap area on the upper-right corner due to
      // initDR requirements
      bool condition2 = false;  // upper right corner has zero-length
                                // overlapped with gcell
      Point tmpIdx = design_->getTopBlock()->getGCellIdx(box.ll());
      Rect gcellBox = design_->getTopBlock()->getGCellBox(tmpIdx);
      if (box.ll() == gcellBox.ll()) {
        condition2 = true;
      }

      bool condition3 = false;  // GR implies wrongway connection but
                                // technology does not allow
      if ((layer->getDir() == dbTechLayerDir::VERTICAL
           && (!USENONPREFTRACKS || layer->isUnidirectional())
           && box.xMin() == gcellBox.xMin())
          || (layer->getDir() == dbTechLayerDir::HORIZONTAL
              && (!USENONPREFTRACKS || layer->isUnidirectional())
              && box.yMin() == gcellBox.yMin())) {
        condition3 = true;
      }
      for (int x = x1; x <= x2; x++) {
        for (int y = y1; y <= y2; y++) {
          if (condition2 && x == tmpIdx.x() - 1 && y == tmpIdx.y() - 1) {
            if (VERBOSE > 0) {
              frString name = (origTerm->typeId() == frcInstTerm)
                                  ? ((frInstTerm*) origTerm)->getName()
                                  : term->getName();
              logger_->warn(DRT,
                            230,
                            "genGuides_gCell2TermMap avoid condition2, may "
                            "result in guide open: {}.",
                            name);
            }
          } else if (condition3
                     && ((x == tmpIdx.x() - 1
                          && layer->getDir() == dbTechLayerDir::VERTICAL)
                         || (y == tmpIdx.y() - 1
                             && layer->getDir()
                                    == dbTechLayerDir::HORIZONTAL))) {
            if (VERBOSE > 0) {
              frString name = (origTerm->typeId() == frcInstTerm)
                                  ? ((frInstTerm*) origTerm)->getName()
                                  : term->getName();
              logger_->warn(DRT,
                            231,
                            "genGuides_gCell2TermMap avoid condition3, may "
                            "result in guide open: {}.",
                            name);
            }
          } else {
            gCell2PinMap[std::make_pair(Point(x, y), lNum)].insert(origTerm);
          }
        }
      }
    }
  }
}

void io::Parser::genGuides_gCell2PinMap(
    frNet* net,
    std::map<std::pair<Point, frLayerNum>,
             std::set<frBlockObject*, frBlockObjectComp>>& gCell2PinMap)
{
  for (auto& instTerm : net->getInstTerms()) {
    dbTransform xform = instTerm->getInst()->getUpdatedXform();
    auto term = instTerm->getTerm();
    if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
      if (!genGuides_gCell2APInstTermMap(gCell2PinMap, instTerm)) {
        genGuides_gCell2TermMap(gCell2PinMap, term, instTerm, xform);
      }
    } else {
      genGuides_gCell2TermMap(gCell2PinMap, term, instTerm, xform);
    }
  }
  for (auto& term : net->getBTerms()) {
    if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
      if (!genGuides_gCell2APTermMap(gCell2PinMap, term)) {
        genGuides_gCell2TermMap(gCell2PinMap, term, term, {});
      }
    } else {
      genGuides_gCell2TermMap(gCell2PinMap, term, term, {});
    }
  }
}

bool io::Parser::genGuides_gCell2APInstTermMap(
    std::map<std::pair<Point, frLayerNum>,
             std::set<frBlockObject*, frBlockObjectComp>>& gCell2PinMap,
    frInstTerm* instTerm)
{
  bool isSuccess = false;

  if (!instTerm) {
    return isSuccess;
  }

  // ap
  frMTerm* trueTerm = instTerm->getTerm();
  std::string name;
  frInst* inst = instTerm->getInst();
  dbTransform shiftXform = inst->getTransform();
  shiftXform.setOrient(dbOrientType(dbOrientType::R0));

  int pinIdx = 0;
  int pinAccessIdx = (inst) ? inst->getPinAccessIdx() : -1;
  int succesPinCnt = 0;
  for (auto& pin : trueTerm->getPins()) {
    frAccessPoint* prefAp = nullptr;
    if (inst) {
      prefAp = (instTerm->getAccessPoints())[pinIdx];
    }
    if (!pin->hasPinAccess()) {
      continue;
    }
    if (pinAccessIdx == -1) {
      continue;
    }

    if (!prefAp) {
      for (auto& ap : pin->getPinAccess(pinAccessIdx)->getAccessPoints()) {
        prefAp = ap.get();
        break;
      }
    }

    if (prefAp) {
      Point bp = prefAp->getPoint();
      auto bNum = prefAp->getLayerNum();
      shiftXform.apply(bp);

      Point idx = design_->getTopBlock()->getGCellIdx(bp);
      gCell2PinMap[std::make_pair(idx, bNum)].insert(
          static_cast<frBlockObject*>(instTerm));
      succesPinCnt++;
      if (succesPinCnt == int(trueTerm->getPins().size())) {
        isSuccess = true;
      }
    }
    pinIdx++;
  }
  return isSuccess;
}

bool io::Parser::genGuides_gCell2APTermMap(
    std::map<std::pair<Point, frLayerNum>,
             std::set<frBlockObject*, frBlockObjectComp>>& gCell2PinMap,
    frBTerm* term)
{
  bool isSuccess = false;

  if (!term) {
    return isSuccess;
  }

  size_t succesPinCnt = 0;
  for (auto& pin : term->getPins()) {
    if (!pin->hasPinAccess()) {
      continue;
    }

    auto& access_points = pin->getPinAccess(0)->getAccessPoints();
    if (access_points.empty()) {
      continue;
    }
    frAccessPoint* prefAp = access_points[0].get();

    const Point& bp = prefAp->getPoint();
    const auto bNum = prefAp->getLayerNum();

    Point idx = design_->getTopBlock()->getGCellIdx(bp);
    gCell2PinMap[{idx, bNum}].insert(term);
    succesPinCnt++;
  }
  return succesPinCnt == term->getPins().size();
}

void io::Parser::genGuides_initPin2GCellMap(
    frNet* net,
    std::map<frBlockObject*,
             std::set<std::pair<Point, frLayerNum>>,
             frBlockObjectComp>& pin2GCellMap)
{
  for (auto& instTerm : net->getInstTerms()) {
    pin2GCellMap[instTerm];
  }
  for (auto& term : net->getBTerms()) {
    pin2GCellMap[term];
  }
}

void io::Parser::genGuides_addCoverGuide(frNet* net, std::vector<frRect>& rects)
{
  for (auto& instTerm : net->getInstTerms()) {
    genGuides_addCoverGuide_helper(instTerm, rects);
  }
}

<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
void GuideProcessor::genGuides_addCoverGuide_helper(frInstTerm* term,
                                                    std::vector<frRect>& rects)
=======
template <typename T>
void io::Parser::genGuides_addCoverGuide_helper(frBlockObject* term,
                                                T* trueTerm,
                                                frInst* inst,
                                                dbTransform& shiftXform,
                                                std::vector<frRect>& rects)
>>>>>>> writer:src/drt/src/io/io_guide.cpp
{
  const frInst* inst = term->getInst();
  dbTransform transform = inst->getTransform();
  transform.setOrient(dbOrientType(dbOrientType::R0));
  int pin_idx = 0;
  const int pin_access_idx = inst->getPinAccessIdx();
  for (const auto& pin : term->getTerm()->getPins()) {
    if (!pin->hasPinAccess()) {
      continue;
    }
    frAccessPoint* pref_ap = term->getAccessPoint(pin_idx);
    if (!pref_ap) {
      const auto pa = pin->getPinAccess(pin_access_idx);
      if (pa->getNumAccessPoints() != 0) {
        pref_ap = pa->getAccessPoint(0);
      }
    }

<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
    if (pref_ap) {
      Point pt = pref_ap->getPoint();
      transform.apply(pt);
      const Point idx = getDesign()->getTopBlock()->getGCellIdx(pt);
      const Rect ll_box = getDesign()->getTopBlock()->getGCellBox(
          Point(idx.x() - 1, idx.y() - 1));
      const Rect ur_box = getDesign()->getTopBlock()->getGCellBox(
          Point(idx.x() + 1, idx.y() + 1));
      const Rect cover_box(
          ll_box.xMin(), ll_box.yMin(), ur_box.xMax(), ur_box.yMax());
      const frLayerNum begin_layer_num = pref_ap->getLayerNum();
      const frLayerNum end_layer_num
          = std::min(begin_layer_num + 4, getTech()->getTopLayerNum());
=======
    if (prefAp) {
      Point bp = prefAp->getPoint();
      auto bNum = prefAp->getLayerNum();
      shiftXform.apply(bp);

      Point idx = design_->getTopBlock()->getGCellIdx(bp);
      Rect llBox = design_->getTopBlock()->getGCellBox(
          Point(idx.x() - 1, idx.y() - 1));
      Rect urBox = design_->getTopBlock()->getGCellBox(
          Point(idx.x() + 1, idx.y() + 1));
      Rect coverBox(llBox.xMin(), llBox.yMin(), urBox.xMax(), urBox.yMax());
      frLayerNum beginLayerNum = bNum;
      frLayerNum endLayerNum
          = std::min(bNum + 4, design_->getTech()->getTopLayerNum());
>>>>>>> writer:src/drt/src/io/io_guide.cpp

      for (auto layer_num = begin_layer_num; layer_num <= end_layer_num;
           layer_num += 2) {
        frRect cover_guide;
        cover_guide.setBBox(cover_box);
        cover_guide.setLayerNum(layer_num);
        rects.push_back(cover_guide);
      }
    }
    pin_idx++;
  }
}

void io::Parser::genGuides(frNet* net, std::vector<frRect>& rects)
{
  net->clearGuides();

  genGuides_pinEnclosure(net, rects);
<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp

  int size = (int) getTech()->getLayers().size();
=======
  int size = (int) tech_->getLayers().size();
>>>>>>> writer:src/drt/src/io/io_guide.cpp
  if (TOP_ROUTING_LAYER < std::numeric_limits<int>::max()
      && TOP_ROUTING_LAYER >= 0) {
    size = std::min(size, TOP_ROUTING_LAYER + 1);
  }
  TrackIntervalsByLayer intvs(size);
  if (DBPROCESSNODE == "GF14_13M_3Mx_2Cx_4Kx_2Hx_2Gx_LB") {
    genGuides_addCoverGuide(net, rects);
  }
  genGuides_prep(rects, intvs);

  std::map<std::pair<Point, frLayerNum>,
           std::set<frBlockObject*, frBlockObjectComp>>
      gCell2PinMap;
  std::map<frBlockObject*,
           std::set<std::pair<Point, frLayerNum>>,
           frBlockObjectComp>
      pin2GCellMap;
  genGuides_gCell2PinMap(net, gCell2PinMap);
  genGuides_initPin2GCellMap(net, pin2GCellMap);

  bool retry = false;
  while (true) {
    genGuides_split(rects,
                    intvs,
                    gCell2PinMap,
                    pin2GCellMap,
                    retry);  // split on LU intersecting guides and pins

    // filter pin2GCellMap with aps

    if (pin2GCellMap.empty()) {
      logger_->warn(DRT, 214, "genGuides empty pin2GCellMap.");
      debugPrint(
          logger_, DRT, "io", 1, "gcell2pin.size() = {}", gCell2PinMap.size());
    }
    for (auto& [obj, locS] : pin2GCellMap) {
      if (locS.empty()) {
        switch (obj->typeId()) {
          case frcInstTerm: {
            auto ptr = static_cast<frInstTerm*>(obj);
            logger_->warn(DRT,
                          215,
                          "Pin {}/{} not covered by guide.",
                          ptr->getInst()->getName(),
                          ptr->getTerm()->getName());
            break;
          }
          case frcBTerm: {
            auto ptr = static_cast<frBTerm*>(obj);
            logger_->warn(
                DRT, 216, "Pin PIN/{} not covered by guide.", ptr->getName());
            break;
          }
          default: {
            logger_->warn(DRT, 217, "genGuides unknown type.");
            break;
          }
        }
      }
    }

    // steiner (i.e., gcell end and pin gcell idx) to guide idx (pin idx)
    std::map<std::pair<Point, frLayerNum>, std::set<int>> nodeMap;
    int gCnt = 0;
    int nCnt = 0;
    genGuides_buildNodeMap(nodeMap, gCnt, nCnt, rects, pin2GCellMap);
    // std::cout <<"build node map done" <<std::endl <<std::flush;

    std::vector<bool> adjVisited;
    std::vector<int> adjPrevIdx;
    if (genGuides_astar(
            net, adjVisited, adjPrevIdx, nodeMap, gCnt, nCnt, false, retry)) {
      // std::cout <<"astar done" <<std::endl <<std::flush;
      genGuides_final(
          net, rects, adjVisited, adjPrevIdx, gCnt, nCnt, pin2GCellMap);
      break;
    }
    if (retry) {
      if (!ALLOW_PIN_AS_FEEDTHROUGH) {
        if (genGuides_astar(net,
                            adjVisited,
                            adjPrevIdx,
                            nodeMap,
                            gCnt,
                            nCnt,
                            true,
                            retry)) {
          genGuides_final(
              net, rects, adjVisited, adjPrevIdx, gCnt, nCnt, pin2GCellMap);
          break;
        }
        logger_->error(DRT, 218, "Guide is not connected to design.");
      } else {
        logger_->error(DRT, 219, "Guide is not connected to design.");
      }
    } else {
      retry = true;
    }
  }
}

void io::Parser::genGuides_final(
    frNet* net,
    std::vector<frRect>& rects,
    std::vector<bool>& adjVisited,
    std::vector<int>& adjPrevIdx,
    int gCnt,
    int nCnt,
    std::map<frBlockObject*,
             std::set<std::pair<Point, frLayerNum>>,
             frBlockObjectComp>& pin2GCellMap)
{
  std::vector<frBlockObject*> pin2ptr;
  pin2ptr.reserve(pin2GCellMap.size());
  for (auto& [obj, idxS] : pin2GCellMap) {
    pin2ptr.push_back(obj);
  }
  // find pin in which guide
  std::vector<std::vector<std::pair<Point, frLayerNum>>> pinIdx2GCellUpdated(
      nCnt - gCnt);
  std::vector<std::vector<int>> guideIdx2Pins(gCnt);
  for (int i = 0; i < (int) adjPrevIdx.size(); i++) {
    if (!adjVisited[i]) {
      continue;
    }
    if (i < gCnt && adjPrevIdx[i] >= gCnt) {
      auto pinIdx = adjPrevIdx[i] - gCnt;
      auto guideIdx = i;
      auto& rect = rects[guideIdx];
      Rect box = rect.getBBox();
      auto lNum = rect.getLayerNum();
      auto obj = pin2ptr[pinIdx];
      // std::cout <<" pin1 id " <<adjPrevIdx[i] <<" prev " <<i <<std::endl;
      if (pin2GCellMap[obj].find(std::make_pair(box.ll(), lNum))
          != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(std::make_pair(box.ll(), lNum));
      } else if (pin2GCellMap[obj].find(std::make_pair(box.ur(), lNum))
                 != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(std::make_pair(box.ur(), lNum));
      } else {
        logger_->warn(
            DRT, 220, "genGuides_final net {} error 1.", net->getName());
      }
      guideIdx2Pins[guideIdx].push_back(pinIdx);
    } else if (i >= gCnt && adjPrevIdx[i] >= 0 && adjPrevIdx[i] < gCnt) {
      auto pinIdx = i - gCnt;
      auto guideIdx = adjPrevIdx[i];
      auto& rect = rects[guideIdx];
      Rect box = rect.getBBox();
      auto lNum = rect.getLayerNum();
      auto obj = pin2ptr[pinIdx];
      // std::cout <<" pin2 id " <<i <<" prev " <<adjPrevIdx[i] <<std::endl;
      if (pin2GCellMap[obj].find(std::make_pair(box.ll(), lNum))
          != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(std::make_pair(box.ll(), lNum));
      } else if (pin2GCellMap[obj].find(std::make_pair(box.ur(), lNum))
                 != pin2GCellMap[obj].end()) {
        pinIdx2GCellUpdated[pinIdx].push_back(std::make_pair(box.ur(), lNum));
      } else {
        logger_->warn(
            DRT, 221, "genGuides_final net {} error 2.", net->getName());
      }
      guideIdx2Pins[guideIdx].push_back(pinIdx);
    }
  }
  for (auto& guides : pinIdx2GCellUpdated) {
    if (guides.empty()) {
      logger_->warn(DRT,
                    222,
                    "genGuides_final net {} pin not in any guide.",
                    net->getName());
    }
  }

  std::map<std::pair<Point, frLayerNum>, std::set<int>> updatedNodeMap;
  // pinIdx2GCellUpdated tells pin residency in gcell
  for (int i = 0; i < nCnt - gCnt; i++) {
    auto obj = pin2ptr[i];
    for (auto& [pt, lNum] : pinIdx2GCellUpdated[i]) {
      Point absPt = design_->getTopBlock()->getGCellCenter(pt);
      tmpGRPins_.emplace_back(obj, absPt);
      updatedNodeMap[std::make_pair(pt, lNum)].insert(i + gCnt);
    }
  }
  for (int i = 0; i < gCnt; i++) {
    if (!adjVisited[i]) {
      continue;
    }
    auto& rect = rects[i];
    Rect box = rect.getBBox();
    updatedNodeMap[std::make_pair(Point(box.xMin(), box.yMin()),
                                  rect.getLayerNum())]
        .insert(i);
    updatedNodeMap[std::make_pair(Point(box.xMax(), box.yMax()),
                                  rect.getLayerNum())]
        .insert(i);
    // std::cout <<"add guide " <<i <<" to " <<Point(box.xMin(),  box.yMin())
    // <<" " <<rect.getLayerNum() <<std::endl; std::cout <<"add guide " <<i <<"
    // to "
    // <<Point(box.xMax(), box.yMax())    <<" " <<rect.getLayerNum()
    // <<std::endl;
  }
  for (auto& [pr, idxS] : updatedNodeMap) {
    auto& [pt, lNum] = pr;
    if ((int) idxS.size() == 1) {
      auto idx = *(idxS.begin());
      if (idx < gCnt) {
        // no upper/lower guide
        if (updatedNodeMap.find(std::make_pair(pt, lNum + 2))
                == updatedNodeMap.end()
            && updatedNodeMap.find(std::make_pair(pt, lNum - 2))
                   == updatedNodeMap.end()) {
          auto& rect = rects[idx];
          Rect box = rect.getBBox();
          if (box.ll() == pt) {
            rect.setBBox(Rect(box.xMax(), box.yMax(), box.xMax(), box.yMax()));
          } else {
            rect.setBBox(Rect(box.xMin(), box.yMin(), box.xMin(), box.yMin()));
          }
        }
      } else {
        logger_->error(DRT,
                       223,
                       "Pin dangling id {} ({},{}) {}.",
                       idx,
                       pt.x(),
                       pt.y(),
                       lNum);
      }
    }
  }
  // guideIdx2Pins enables finding from guide to pin
  // adjVisited tells guide to write back
  for (int i = 0; i < gCnt; i++) {
    if (!adjVisited[i]) {
      continue;
    }
    auto& rect = rects[i];
    Rect box = rect.getBBox();
    auto guide = std::make_unique<frGuide>();
    Point begin = design_->getTopBlock()->getGCellCenter(box.ll());
    Point end = design_->getTopBlock()->getGCellCenter(box.ur());
    guide->setPoints(begin, end);
    guide->setBeginLayerNum(rect.getLayerNum());
    guide->setEndLayerNum(rect.getLayerNum());
    guide->addToNet(net);
    net->addGuide(std::move(guide));
    // std::cout <<"add guide " <<begin <<" " <<end <<" " <<rect.getLayerNum()
    // <<std::endl;
  }
}

void io::Parser::genGuides_buildNodeMap(
    std::map<std::pair<Point, frLayerNum>, std::set<int>>& nodeMap,
    int& gCnt,
    int& nCnt,
    std::vector<frRect>& rects,
    std::map<frBlockObject*,
             std::set<std::pair<Point, frLayerNum>>,
             frBlockObjectComp>& pin2GCellMap)
{
  for (int i = 0; i < (int) rects.size(); i++) {
    auto& rect = rects[i];
    Rect box = rect.getBBox();
    nodeMap[std::make_pair(box.ll(), rect.getLayerNum())].insert(i);
    nodeMap[std::make_pair(box.ur(), rect.getLayerNum())].insert(i);
  }
  gCnt = rects.size();  // total guide cnt
  int nodeIdx = rects.size();
  for (auto& [obj, locS] : pin2GCellMap) {
    for (auto& loc : locS) {
      nodeMap[loc].insert(nodeIdx);
    }
    nodeIdx++;
  }
  nCnt = nodeIdx;  // total node cnt
}

bool io::Parser::genGuides_astar(
    frNet* net,
    std::vector<bool>& adjVisited,
    std::vector<int>& adjPrevIdx,
    std::map<std::pair<Point, frLayerNum>, std::set<int>>& nodeMap,
    int& gCnt,
    int& nCnt,
    bool forceFeedThrough,
    bool retry)
{
  // a star search

  // node index, node visited
  std::vector<std::vector<int>> adjVec(nCnt, std::vector<int>());
  std::vector<bool> onPathIdx(nCnt, false);
  adjVisited.clear();
  adjPrevIdx.clear();
  adjVisited.resize(nCnt, false);
  adjPrevIdx.resize(nCnt, -1);
  for (auto& [pr, idxS] : nodeMap) {
    auto& [pt, lNum] = pr;
    for (auto it1 = idxS.begin(); it1 != idxS.end(); it1++) {
      auto it2 = it1;
      it2++;
      auto idx1 = *it1;
      for (; it2 != idxS.end(); it2++) {
        auto idx2 = *it2;
        // two pins, no edge
        if (idx1 >= gCnt && idx2 >= gCnt) {
          continue;
          // two gcells, has edge
        }
        if (idx1 < gCnt && idx2 < gCnt) {
          // no M1 cross-gcell routing allowed
          // BX200307: in general VIA_ACCESS_LAYER should not be used (instead
          // of 0)
          if (lNum != VIA_ACCESS_LAYERNUM) {
            adjVec[idx1].push_back(idx2);
            adjVec[idx2].push_back(idx1);
          }
          // one pin, one gcell
        } else {
          auto gIdx = std::min(idx1, idx2);
          auto pIdx = std::max(idx1, idx2);
          // only out edge
          if (ALLOW_PIN_AS_FEEDTHROUGH || forceFeedThrough) {
            adjVec[pIdx].push_back(gIdx);
            adjVec[gIdx].push_back(pIdx);
          } else {
            if (pIdx == gCnt) {
              adjVec[pIdx].push_back(gIdx);
              // std::cout <<"add edge2 " <<pIdx <<" " <<gIdx <<std::endl;
              // only in edge
            } else {
              adjVec[gIdx].push_back(pIdx);
              // std::cout <<"add edge3 " <<gIdx <<" " <<pIdx <<std::endl;
            }
          }
        }
      }
      // add intersecting guide2guide edge excludes pin
      if (idx1 < gCnt
          && nodeMap.find(std::make_pair(pt, lNum + 2)) != nodeMap.end()) {
        for (auto nbrIdx : nodeMap[std::make_pair(pt, lNum + 2)]) {
          if (nbrIdx < gCnt) {
            adjVec[idx1].push_back(nbrIdx);
            adjVec[nbrIdx].push_back(idx1);
            // std::cout <<"add edge4 " <<idx1 <<" " <<nbrIdx <<std::endl;
            // std::cout <<"add edge5 " <<nbrIdx <<" " <<idx1 <<std::endl;
          }
        }
      }
    }
  }

  struct wf
  {
    int nodeIdx;
    int prevIdx;
    int cost;
    bool operator<(const wf& b) const
    {
      if (cost == b.cost) {
        return nodeIdx > b.nodeIdx;
      }
      return cost > b.cost;
    }
  };
  for (int findNode = gCnt; findNode < nCnt - 1; findNode++) {
    // std::cout <<"finished " <<findNode <<" nodes" <<std::endl;
    std::priority_queue<wf> pq;
    if (findNode == gCnt) {
      // push only first pin into pq
      pq.push({gCnt, -1, 0});
    } else {
      // push every visited node into pq
      for (int i = 0; i < nCnt; i++) {
        if (onPathIdx[i]) {
          // penalize feedthrough in normal mode
          if (ALLOW_PIN_AS_FEEDTHROUGH && i >= gCnt) {
            pq.push({i, adjPrevIdx[i], 2});
            // penalize feedthrough in fallback mode
          } else if (forceFeedThrough && i >= gCnt) {
            pq.push({i, adjPrevIdx[i], 10});
          } else {
            pq.push({i, adjPrevIdx[i], 0});
          }
        }
      }
    }
    int lastNodeIdx = -1;
    while (!pq.empty()) {
      auto wfront = pq.top();
      pq.pop();
      if (!onPathIdx[wfront.nodeIdx] && adjVisited[wfront.nodeIdx]) {
        continue;
      }
      if (wfront.nodeIdx > gCnt && wfront.nodeIdx < nCnt
          && adjVisited[wfront.nodeIdx] == false) {
        adjVisited[wfront.nodeIdx] = true;
        adjPrevIdx[wfront.nodeIdx] = wfront.prevIdx;
        lastNodeIdx = wfront.nodeIdx;
        break;
      }
      adjVisited[wfront.nodeIdx] = true;
      adjPrevIdx[wfront.nodeIdx] = wfront.prevIdx;
      // visit other nodes
      for (auto nbrIdx : adjVec[wfront.nodeIdx]) {
        if (!adjVisited[nbrIdx]) {
          pq.push({nbrIdx, wfront.nodeIdx, wfront.cost + 1});
        }
      }
    }
    // trace back path
    while ((lastNodeIdx != -1) && (!onPathIdx[lastNodeIdx])) {
      onPathIdx[lastNodeIdx] = true;
      lastNodeIdx = adjPrevIdx[lastNodeIdx];
    }
    adjVisited = onPathIdx;
  }
  // skip one-pin net
  if (nCnt == gCnt + 1) {
    return true;
  }
  int pinVisited = count(adjVisited.begin() + gCnt, adjVisited.end(), true);
  // true error when allowing feedthrough
  if (pinVisited != nCnt - gCnt
      && (ALLOW_PIN_AS_FEEDTHROUGH || forceFeedThrough) && retry) {
    logger_->warn(DRT,
                  224,
                  "{} {} pin not visited, number of guides = {}.",
                  net->getName(),
                  nCnt - gCnt - pinVisited,
                  gCnt);
  }
  // fallback to feedthrough in next iter
  if (pinVisited != nCnt - gCnt && !ALLOW_PIN_AS_FEEDTHROUGH
      && !forceFeedThrough && retry) {
    logger_->warn(DRT,
                  225,
                  "{} {} pin not visited, fall back to feedthrough mode.",
                  net->getName(),
                  nCnt - gCnt - pinVisited);
  }
  if (pinVisited == nCnt - gCnt) {
    return true;
  }
  return false;
}

<<<<<<< HEAD:src/drt/src/io/GuideProcessor.cpp
void GuideProcessor::processGuides()
{
  if (tmp_guides_.empty()) {
    return;
  }
  ProfileTask profile("IO:postProcessGuide");
  if (VERBOSE > 0) {
    logger_->info(DRT, 169, "Post process guides.");
  }
  buildGCellPatterns();

  getDesign()->getRegionQuery()->initOrigGuide(tmp_guides_);
  int cnt = 0;
  for (auto& [net, rects] : tmp_guides_) {
    net->setOrigGuides(rects);
    genGuides(net, rects);
    cnt++;
    if (VERBOSE > 0) {
      if (cnt < 1000000) {
        if (cnt % 100000 == 0) {
          logger_->report("  complete {} nets.", cnt);
        }
      } else {
        if (cnt % 1000000 == 0) {
          logger_->report("  complete {} nets.", cnt);
        }
      }
    }
  }

  // global unique id for guides
  int currId = 0;
  for (auto& net : getDesign()->getTopBlock()->getNets()) {
    for (auto& guide : net->getGuides()) {
      guide->setId(currId);
      currId++;
    }
  }

  logger_->info(DRT, 178, "Init guide query.");
  getDesign()->getRegionQuery()->initGuide();
  getDesign()->getRegionQuery()->printGuide();
  logger_->info(DRT, 179, "Init gr pin query.");
  getDesign()->getRegionQuery()->initGRPin(tmpGRPins_);

  if (!SAVE_GUIDE_UPDATES) {
    if (VERBOSE > 0) {
      logger_->info(DRT, 245, "skipped writing guide updates to database.");
    }
  } else {
    saveGuidesUpdates();
  }
}

}  // namespace drt::io
=======
}  // namespace drt
>>>>>>> writer:src/drt/src/io/io_guide.cpp
