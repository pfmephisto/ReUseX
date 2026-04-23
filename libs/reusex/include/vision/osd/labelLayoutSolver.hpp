#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <random>
#include <vector>

namespace reusex::vision::osd {

/// @brief Axis-aligned 2-D bounding box used internally by the label layout
/// solver.
struct LayoutBox {
  float left;   ///< Left edge x-coordinate.
  float top;    ///< Top edge y-coordinate.
  float right;  ///< Right edge x-coordinate.
  float bottom; ///< Bottom edge y-coordinate.

  /// @brief Returns the width of the box.
  inline float width() const { return right - left; }

  /// @brief Returns the height of the box.
  inline float height() const { return bottom - top; }

  /// @brief Returns the area of the box (clamped to 0 for inverted boxes).
  inline float area() const {
    return std::max(0.0f, right - left) * std::max(0.0f, bottom - top);
  }

  /// @brief Computes the intersection area of two LayoutBoxes.
  static inline float intersectArea(const LayoutBox &box1,
                                    const LayoutBox &box2) {
    float l = std::max(box1.left, box2.left);
    float r = std::min(box1.right, box2.right);
    float t = std::max(box1.top, box2.top);
    float b = std::min(box1.bottom, box2.bottom);
    float w = std::max(0.0f, r - l);
    float h = std::max(0.0f, b - t);
    return w * h;
  }

  /// @brief Returns true when two LayoutBoxes overlap.
  static inline bool intersects(const LayoutBox &box1, const LayoutBox &box2) {
    return (box1.left < box2.right && box1.right > box2.left &&
            box1.top < box2.bottom && box1.bottom > box2.top);
  }
};

/// @brief Measured bounding box of a rendered text string.
struct TextSize {
  int width;    ///< Rendered string width in pixels.
  int height;   ///< Rendered string height (ascent) in pixels.
  int baseline; ///< Descent below the baseline in pixels.
};

/// @brief Final placement result for a single label.
struct LayoutResult {
  float x;       ///< Left edge of the placed label box.
  float y;       ///< Top edge of the placed label box.
  int fontSize;  ///< Font size chosen for this label (may be scaled down).
  int width;     ///< Width of the label box in pixels.
  int height;    ///< Height of the label box in pixels.
  int textAscent; ///< Ascent value to use when calling putText.
};

/// @brief Tunable parameters controlling the label placement algorithm.
struct LayoutConfig {
  int gridSize = 100; ///< Spatial-index cell size in pixels.
  int spatialIndexThreshold =
      20; ///< Minimum number of items before a grid index is used.
  int maxIterations =
      30; ///< Maximum refinement iterations (higher = better quality).
  int paddingX = 2; ///< Horizontal padding around each label in pixels.
  int paddingY = 2; ///< Vertical padding around each label in pixels.

  /// Base geometric cost for anchor position 1 (top of object — preferred).
  float costPos1_Top = 0.0f;
  /// Base geometric cost for anchor position 2 (right of object).
  float costPos2_Right = 10.0f;
  /// Base geometric cost for anchor position 3 (bottom of object).
  float costPos3_Bottom = 20.0f;
  /// Base geometric cost for anchor position 4 (left of object).
  float costPos4_Left = 30.0f;

  /// Extra cost for a sliding (non-anchor) candidate.  Must be > costPos4_Left.
  float costSlidingPenalty = 100.0f;
  /// Cost multiplier per font-size scale tier (keep large to prefer position
  /// changes over size reduction).
  float costScaleTier = 10000.0f;
  /// Penalty per unit overlap with an object bounding box.
  float costOccludeObj = 100000.0f;
  /// Base penalty per unit overlap with another label.
  float costOverlapBase = 100000.0f;
};

/// @brief Compact spatial hash-grid for fast overlap queries during layout.
///
/// All object and label boxes are inserted into a uniform grid so that
/// candidate overlap checks only need to visit a small neighbourhood rather
/// than the full item list.
class FlatUniformGrid {
  public:
  int rows = 0, cols = 0;
  float cellW = 100.0f, cellH = 100.0f;
  float invCellW = 0.01f, invCellH = 0.01f;

  std::vector<int> gridHead;
  struct Node {
    int id;
    int next;
  };
  std::vector<Node> nodes;

  FlatUniformGrid() { nodes.reserve(4096); }

  /// @brief Resize the grid to cover a canvas of w×h pixels using cells of
  /// gridSize.
  void resize(int w, int h, int gridSize) {
    if (gridSize <= 0)
      gridSize = 100;
    int newCols = (w + gridSize - 1) / gridSize;
    int newRows = (h + gridSize - 1) / gridSize;
    cellW = (float)gridSize;
    cellH = (float)gridSize;
    invCellW = 1.0f / cellW;
    invCellH = 1.0f / cellH;

    if (newCols * newRows > (int)gridHead.size()) {
      gridHead.resize(newCols * newRows, -1);
    }
    cols = newCols;
    rows = newRows;
  }

  /// @brief Reset all cells, keeping allocated memory.
  void clear() {
    if (!gridHead.empty()) {
      std::fill(gridHead.begin(), gridHead.begin() + (rows * cols), -1);
    }
    nodes.clear();
  }

  /// @brief Insert item @p id with the given box into the grid.
  inline void insert(int id, const LayoutBox &box) {
    int c1 = std::max(0, std::min(cols - 1, (int)(box.left * invCellW)));
    int r1 = std::max(0, std::min(rows - 1, (int)(box.top * invCellH)));
    int c2 = std::max(0, std::min(cols - 1, (int)(box.right * invCellW)));
    int r2 = std::max(0, std::min(rows - 1, (int)(box.bottom * invCellH)));

    for (int r = r1; r <= r2; ++r) {
      int rowOffset = r * cols;
      for (int c = c1; c <= c2; ++c) {
        int idx = rowOffset + c;
        nodes.push_back({id, gridHead[idx]});
        gridHead[idx] = (int)nodes.size() - 1;
      }
    }
  }

  /// @brief Invoke @p visitor for each unique item overlapping @p box.
  /// @p visitedToken and @p cookie are used to avoid visiting the same item
  /// twice per query.
  template <typename Visitor>
  inline void query(const LayoutBox &box, std::vector<int> &visitedToken,
                    int cookie, Visitor &&visitor) {
    int c1 = std::max(0, std::min(cols - 1, (int)(box.left * invCellW)));
    int r1 = std::max(0, std::min(rows - 1, (int)(box.top * invCellH)));
    int c2 = std::max(0, std::min(cols - 1, (int)(box.right * invCellW)));
    int r2 = std::max(0, std::min(rows - 1, (int)(box.bottom * invCellH)));

    for (int r = r1; r <= r2; ++r) {
      int rowOffset = r * cols;
      for (int c = c1; c <= c2; ++c) {
        int nodeIdx = gridHead[rowOffset + c];
        while (nodeIdx != -1) {
          const auto &node = nodes[nodeIdx];
          if (visitedToken[node.id] != cookie) {
            visitedToken[node.id] = cookie;
            visitor(node.id);
          }
          nodeIdx = node.next;
        }
      }
    }
  }
};

/// @brief Greedy iterative label placement solver.
///
/// Given a set of object bounding boxes and their text labels, LabelLayoutSolver
/// finds a non-overlapping placement for each label by minimising a cost
/// function that penalises:
///   - departure from the preferred anchor position (top of the object),
///   - overlap with other object boxes, and
///   - overlap with other already-placed labels.
///
/// The solver generates a set of discrete placement candidates for each label
/// (four anchor positions × several font-scale tiers + sliding variants) and
/// then iteratively re-assigns labels to reduce total cost.
///
/// Usage example:
/// @code
///   LabelLayoutSolver solver(imgW, imgH, [&](const std::string &t, int sz) {
///       int w, h, base;
///       renderer.getTextSize(t, sz, &w, &h, &base);
///       return TextSize{w, h, base};
///   });
///   solver.add(box.left, box.top, box.right, box.bottom, label, fontSize);
///   solver.solve();
///   for (const auto &res : solver.getResults()) { /* draw */ }
/// @endcode
class LabelLayoutSolver {
  public:
  /// @brief Internal representation of a single placement candidate for a
  /// label.
  struct Candidate {
    LayoutBox box;        ///< Candidate label box on the canvas.
    float geometricCost;  ///< Cost based on anchor preference and font scale.
    float staticCost;     ///< Cost from overlap with object boxes (computed
                          ///< once).
    float area;           ///< Box area (cached for performance).
    float invArea;        ///< Reciprocal of area (cached for performance).
    int16_t fontSize;     ///< Font size for this candidate.
    int16_t textAscent;   ///< Text ascent (for putText offset calculation).
  };

  private:
  struct LayoutItem {
    int id;
    LayoutBox objectBox;
    uint32_t candStart;
    uint16_t candCount;
    int selectedRelIndex;
    LayoutBox currentBox;
    float currentArea;
    float currentTotalCost;
  };

  LayoutConfig config;
  int canvasWidth, canvasHeight;
  std::function<TextSize(const std::string &, int)> measureFunc;

  std::vector<LayoutItem> items;
  std::vector<Candidate> candidatePool;
  std::vector<int> processOrder;
  FlatUniformGrid grid;
  std::vector<int> visitedCookie;
  int currentCookie = 0;
  std::mt19937 rng;

  public:
  /// @brief Construct a solver for a canvas of the given dimensions.
  /// @param w    Canvas width in pixels.
  /// @param h    Canvas height in pixels.
  /// @param func Callable that measures a text string at a given font size and
  ///             returns a TextSize.  Signature:
  ///             `TextSize(const std::string &text, int fontSize)`.
  /// @param cfg  Optional tuning parameters.
  template <typename Func>
  LabelLayoutSolver(int w, int h, Func &&func,
                    const LayoutConfig &cfg = LayoutConfig())
      : config(cfg), canvasWidth(w), canvasHeight(h),
        measureFunc(std::forward<Func>(func)), rng(12345) {
    items.reserve(128);
    candidatePool.reserve(4096);
    visitedCookie.reserve(128);
  }

  /// @brief Update the layout configuration.
  void setConfig(const LayoutConfig &cfg) { config = cfg; }

  /// @brief Change the canvas dimensions (e.g. after a resize).
  void setCanvasSize(int w, int h) {
    canvasWidth = w;
    canvasHeight = h;
  }

  /// @brief Remove all items from the solver (allows reuse across frames).
  void clear() {
    items.clear();
    candidatePool.clear();
    processOrder.clear();
  }

  /// @brief Register a new label to be placed.
  /// @param l           Left edge of the associated object box.
  /// @param t           Top edge.
  /// @param r           Right edge.
  /// @param b           Bottom edge.
  /// @param text        Label string (UTF-8).
  /// @param baseFontSize Initial (maximum) font size in pixels.
  void add(float l, float t, float r, float b, const std::string &text,
           int baseFontSize) {
    if (r - l < 2.0f) {
      float cx = (l + r) * 0.5f;
      l = cx - 1;
      r = cx + 1;
    }
    if (b - t < 2.0f) {
      float cy = (t + b) * 0.5f;
      t = cy - 1;
      b = cy + 1;
    }

    LayoutItem item;
    item.id = (int)items.size();
    item.objectBox = {std::floor(l), std::floor(t), std::ceil(r), std::ceil(b)};
    item.candStart = (uint32_t)candidatePool.size();

    generateCandidatesInternal(item, text, baseFontSize);
    item.candCount = (uint16_t)(candidatePool.size() - item.candStart);

    if (item.candCount > 0) {
      item.selectedRelIndex = 0;
      const auto &c = candidatePool[item.candStart];
      item.currentBox = c.box;
      item.currentArea = c.area;
      item.currentTotalCost = c.geometricCost;
    } else {
      Candidate dummy;
      dummy.box = {0, 0, 0, 0};
      dummy.geometricCost = 1e9f;
      dummy.staticCost = 0;
      dummy.area = 0.1f;
      dummy.invArea = 10.0f;
      dummy.fontSize = (int16_t)baseFontSize;
      dummy.textAscent = 0;
      candidatePool.push_back(dummy);
      item.candCount = 1;
      item.selectedRelIndex = 0;
      item.currentBox = dummy.box;
      item.currentArea = 0.1f;
      item.currentTotalCost = 1e9f;
    }
    items.push_back(std::move(item));
  }

  /// @brief Run the iterative placement optimisation.
  /// Call this after adding all items and before calling getResults().
  void solve() {
    if (items.empty())
      return;
    const size_t N = items.size();

    if (visitedCookie.size() < N)
      visitedCookie.resize(N, 0);
    bool useGrid = (N >= (size_t)config.spatialIndexThreshold);

    if (useGrid) {
      grid.resize(canvasWidth, canvasHeight, config.gridSize);
      grid.clear();
      for (const auto &item : items)
        grid.insert(item.id, item.objectBox);
    }

    for (auto &item : items) {
      float minCost = std::numeric_limits<float>::max();
      int bestIdx = 0;

      for (uint32_t i = 0; i < item.candCount; ++i) {
        Candidate &cand = candidatePool[item.candStart + i];
        float penalty = 0.0f;

        auto checkStaticConflict = [&](int otherId) {
          const auto &other = items[otherId];
          if (LayoutBox::intersects(cand.box, other.objectBox)) {
            float inter = LayoutBox::intersectArea(cand.box, other.objectBox);
            penalty += (inter * cand.invArea) * config.costOccludeObj;
          }
        };

        if (useGrid) {
          currentCookie++;
          grid.query(cand.box, visitedCookie, currentCookie,
                     checkStaticConflict);
        } else {
          for (const auto &other : items)
            checkStaticConflict(other.id);
        }
        cand.staticCost = penalty;

        float total = cand.geometricCost + cand.staticCost;
        if (total < minCost) {
          minCost = total;
          bestIdx = (int)i;
        }
      }
      item.selectedRelIndex = bestIdx;
      const auto &bestCand = candidatePool[item.candStart + bestIdx];
      item.currentBox = bestCand.box;
      item.currentArea = bestCand.area;
      item.currentTotalCost = minCost;
    }

    processOrder.resize(N);
    for (size_t i = 0; i < N; ++i)
      processOrder[i] = (int)i;

    for (int iter = 0; iter < config.maxIterations; ++iter) {
      std::shuffle(processOrder.begin(), processOrder.end(), rng);
      int changeCount = 0;

      if (useGrid) {
        grid.clear();
        for (const auto &item : items)
          grid.insert(item.id, item.currentBox);
      }

      for (int idx : processOrder) {
        auto &item = items[idx];

        auto calculateDynamicCost = [&](const LayoutBox &box, float invBoxArea,
                                        int selfId) -> float {
          float overlapCost = 0.0f;
          auto visitor = [&](int otherId) {
            if (selfId == otherId)
              return;
            const auto &otherBox = items[otherId].currentBox;
            if (LayoutBox::intersects(box, otherBox)) {
              float inter = LayoutBox::intersectArea(box, otherBox);
              overlapCost += (inter * invBoxArea) * config.costOverlapBase;
            }
          };
          if (useGrid) {
            currentCookie++;
            grid.query(box, visitedCookie, currentCookie, visitor);
          } else {
            for (size_t j = 0; j < N; ++j)
              visitor((int)j);
          }
          return overlapCost;
        };

        const auto &curCand =
            candidatePool[item.candStart + item.selectedRelIndex];
        float curDyn =
            calculateDynamicCost(item.currentBox, curCand.invArea, item.id);
        float currentRealTotal =
            curCand.geometricCost + curCand.staticCost + curDyn;

        if (currentRealTotal < 1.0f)
          continue;

        float bestIterCost = currentRealTotal;
        int bestRelIdx = -1;

        for (int i = 0; i < (int)item.candCount; ++i) {
          if (i == item.selectedRelIndex)
            continue;
          const auto &cand = candidatePool[item.candStart + i];

          float baseCost = cand.geometricCost + cand.staticCost;
          if (baseCost >= bestIterCost)
            continue;

          float newOverlap =
              calculateDynamicCost(cand.box, cand.invArea, item.id);
          float newTotal = baseCost + newOverlap;

          if (newTotal < bestIterCost) {
            bestIterCost = newTotal;
            bestRelIdx = i;
          }
        }

        if (bestRelIdx != -1) {
          item.selectedRelIndex = bestRelIdx;
          const auto &newCand = candidatePool[item.candStart + bestRelIdx];
          item.currentBox = newCand.box;
          item.currentArea = newCand.area;
          changeCount++;
        }
      }
      if (changeCount == 0)
        break;
    }
  }

  /// @brief Retrieve the placement result for each label in insertion order.
  /// @return Vector of LayoutResult, one per call to add().
  std::vector<LayoutResult> getResults() const {
    std::vector<LayoutResult> results;
    results.reserve(items.size());
    for (const auto &item : items) {
      const auto &cand = candidatePool[item.candStart + item.selectedRelIndex];
      results.push_back({cand.box.left, cand.box.top, (int)cand.fontSize,
                         (int)cand.box.width(), (int)cand.box.height(),
                         (int)cand.textAscent});
    }
    return results;
  }

  private:
  void generateCandidatesInternal(LayoutItem &item, const std::string &text,
                                  int baseFontSize) {
    static const struct {
      float scale;
      int tier;
    } levels[] = {{1.0f, 0}, {0.9f, 1}, {0.8f, 2}, {0.75f, 3}};

    const auto &obj = item.objectBox;
    for (const auto &lvl : levels) {
      int fontSize = (int)(baseFontSize * lvl.scale);
      if (fontSize < 9)
        break;

      TextSize ts = measureFunc(text, fontSize);
      float fW = std::ceil((float)ts.width + config.paddingX * 2);
      float fH =
          std::ceil((float)(ts.height + ts.baseline + config.paddingY * 2));
      float scalePenalty = lvl.tier * config.costScaleTier;
      float area = fW * fH;
      float invArea = 1.0f / (area > 0.1f ? area : 1.0f);

      auto addCand = [&](float x, float y, float posCost) {
        if (x < 0 || y < 0 || x + fW > canvasWidth || y + fH > canvasHeight)
          return;
        candidatePool.emplace_back();
        auto &c = candidatePool.back();
        c.box = {x, y, x + fW, y + fH};
        c.geometricCost = posCost + scalePenalty;
        c.staticCost = 0;
        c.area = area;
        c.invArea = invArea;
        c.fontSize = (int16_t)fontSize;
        c.textAscent = (int16_t)ts.height;
      };

      // Priority 1: Top (aligned left above)
      addCand(obj.left, obj.top - fH, config.costPos1_Top);
      // Priority 2: Right-Top (aligned top right)
      addCand(obj.right, obj.top, config.costPos2_Right);
      // Priority 3: Bottom (aligned left below)
      addCand(obj.left, obj.bottom, config.costPos3_Bottom);
      // Priority 4: Left-Top (aligned top left)
      addCand(obj.left - fW, obj.top, config.costPos4_Left);

      const float baseSlidePenalty = config.costSlidingPenalty;

      auto getDynamicSteps = [](float rangeSize) {
        return std::clamp((int)(rangeSize / 40.0f), 3, 15);
      };

      float rangeX = std::max(0.0f, obj.right - fW - obj.left);
      if (rangeX > 1.0f) {
        int stepsX = getDynamicSteps(rangeX);
        float invStepsX = 1.0f / (float)stepsX;
        for (int i = 1; i < stepsX; ++i) {
          float r = i * invStepsX;
          float x = obj.left + rangeX * r;
          float penalty = baseSlidePenalty + (r * 10.0f);
          addCand(x, obj.top - fH, config.costPos1_Top + penalty);
          addCand(x, obj.bottom, config.costPos3_Bottom + penalty);
        }
      }

      float rangeY = std::max(0.0f, obj.bottom - fH - obj.top);
      if (rangeY > 1.0f) {
        int stepsY = getDynamicSteps(rangeY);
        float invStepsY = 1.0f / (float)stepsY;
        for (int i = 1; i < stepsY; ++i) {
          float r = i * invStepsY;
          float y = obj.top + rangeY * r;
          float penalty = baseSlidePenalty + (r * 10.0f);
          addCand(obj.right, y, config.costPos2_Right + penalty);
          addCand(obj.left - fW, y, config.costPos4_Left + penalty);
        }
      }
    }
  }
};

} // namespace reusex::vision::osd
