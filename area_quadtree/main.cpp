#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <list>
#include <memory>
#include <optional>
#include <random>
#include <vector>

// Random device seed
constexpr auto kSeed = 1234;

constexpr float kEpsilon = 1e-6;

constexpr float kXmin = -1.0;
constexpr float kXmax = 1.0;
constexpr float kYmin = -1.0;
constexpr float kYmax = 1.0;

constexpr float kMinDistanceX = 1e-5;
constexpr float kMinDistanceY = 1e-5;

constexpr int kPointsNumber = 100000;

class TimeBench {
 public:
  TimeBench(std::string aLabel) : mLabel{std::move(aLabel)} {}

  ~TimeBench() {
    const auto endPoint = std::chrono::high_resolution_clock::now();
    const auto processTime =
        std::chrono::duration_cast<std::chrono::microseconds>(endPoint -
                                                              mStartPoint)
            .count();
    std::cout << mLabel << " : " << processTime << " microseconds\n";
  }

 private:
  const std::string mLabel;
  const std::chrono::high_resolution_clock::time_point mStartPoint =
      std::chrono::high_resolution_clock::now();
};

struct Point {
  Point(float aX, float aY) : x{aX}, y{aY} {}
  float x{};
  float y{};
};

bool operator==(const Point& lhs, const Point& rhs) {
  return (std::abs(lhs.x - rhs.x) < kEpsilon &&
          std::abs(lhs.y - rhs.y) < kEpsilon);
}

bool operator!=(const Point& lhs, const Point& rhs) { return !(lhs == rhs); }

std::ostream& operator<<(std::ostream& os, const Point& data) {
  os << "{" << data.x << "," << data.y << "}";
  return os;
}

struct Rectangle {
  Rectangle(float aXmin, float aXmax, float aYmin, float aYmax)
      : mXmin{std::min(aXmin, aXmax)},
        mXmax{std::max(aXmin, aXmax)},
        mYmin{std::min(aYmin, aYmax)},
        mYmax{std::max(aYmin, aYmax)} {
    mCenterX = 0.5f * (mXmin + mXmax);
    mCenterY = 0.5f * (mYmin + mYmax);
  }

  [[nodiscard]] bool isPointInside(const Point& aPoint) const {
    if (aPoint.x > mXmax || aPoint.x < mXmin || aPoint.y > mYmax ||
        aPoint.y < mYmin) {
      // std::cout << "Out of borders" << '\n';
      return false;
    }
    return true;
  }

  operator bool() const {
    return !(std::isfinite(mXmin) && std::isfinite(mXmax) &&
             std::isfinite(mYmin) && std::isfinite(mYmax));
  }

  [[nodiscard]] float Xmin() const { return mXmin; }

  [[nodiscard]] float Xmax() const { return mXmax; }

  [[nodiscard]] float Ymin() const { return mYmin; }

  [[nodiscard]] float Ymax() const { return mYmax; }

  [[nodiscard]] float Xcenter() const { return mCenterX; }

  [[nodiscard]] float Ycenter() const { return mCenterY; }

  [[nodiscard]] Rectangle createTopRight() const {
    return {mCenterX, mXmax, mCenterY, mYmax};
  }

  [[nodiscard]] Rectangle createBottomRight() const {
    return {mCenterX, mXmax, mYmin, mCenterY};
  }

  [[nodiscard]] Rectangle createTopLeft() const {
    return {mXmin, mCenterX, mCenterY, mYmax};
  }

  [[nodiscard]] Rectangle createBottomLeft() const {
    return {mXmin, mCenterX, mYmin, mCenterY};
  }

 private:
  // Borders
  float mXmax;
  float mXmin;
  float mYmax;
  float mYmin;

  // Center point
  float mCenterX;
  float mCenterY;
};

std::ostream& operator<<(std::ostream& os, const Rectangle& data) {
  os << "{" << data.Xmin() << "," << data.Xmax() << "}{" << data.Ymin() << ","
     << data.Ymax() << "}";
  return os;
}

enum class Overlap { YES, NO, PARTIAL };

Overlap calculateOverlap(const Rectangle& lhs, const Rectangle& rhs) {
  const auto xLeft = std::max(lhs.Xmin(), rhs.Xmin());
  const auto yTop = std::max(lhs.Ymin(), rhs.Ymin());
  const auto xRight = std::min(lhs.Xmax(), rhs.Xmax());
  const auto yBottom = std::min(lhs.Ymax(), rhs.Ymax());

  if (xRight < xLeft || yBottom < yTop) {
    return Overlap::NO;
  }

  const auto intersectionArea = (xRight - xLeft) * (yBottom - yTop);

  const auto lhsArea = (lhs.Xmax() - lhs.Xmin()) * (lhs.Ymax() - lhs.Ymin());
  const auto rhsArea = (rhs.Xmax() - rhs.Xmin()) * (rhs.Ymax() - rhs.Ymin());
  const auto minaArea = std::min(lhsArea, rhsArea);

  // const auto iou =
  //     intersectionArea / float(lhsArea + rhsArea - intersectionArea);

  if (std::abs(intersectionArea - minaArea) < kEpsilon) {
    return Overlap::YES;
  }

  return Overlap::PARTIAL;
}

class Node {
 public:
  static std::unique_ptr<Node> createNode(const Rectangle& aBorder) {
    if (std::abs(aBorder.Xmax() - aBorder.Xmin()) < kMinDistanceX ||
        std::abs(aBorder.Ymax() - aBorder.Ymin()) < kMinDistanceY) {
      return nullptr;
    }

    std::unique_ptr<Node> result(new Node(aBorder));

    return result;
  }

  bool insertPoint(const Point& aPoint) {
    // std::cout << "Insert " << aPoint << '\n';
    // Point out of borders
    if (!mBorder.isPointInside(aPoint)) {
      // std::cout << "Out of borders" << '\n';
      return false;
    }

    // No previously stored point and no child nodes
    if (!mPoint && !(mTopRight || mTopLeft || mBottomRight || mBottomLeft)) {
      mPoint = aPoint;
      // std::cout << "Point inserted" << '\n';
      return true;
    }

    bool isInserted = false;
    if (aPoint.x > mBorder.Xcenter()) {
      if (aPoint.y > mBorder.Ycenter()) {
        // std::cout << "TopRight" << '\n';
        if (mTopRight) {
          isInserted = mTopRight->insertPoint(aPoint);
        } else {
          mTopRight = createNode(mBorder.createTopRight());
          if (mTopRight) {
            isInserted = mTopRight->insertPoint(aPoint);
          }
        }
      } else {
        // std::cout << "BottomRight" << '\n';
        if (mBottomRight) {
          isInserted = mBottomRight->insertPoint(aPoint);
        } else {
          mBottomRight = createNode(mBorder.createBottomRight());
          if (mBottomRight) {
            isInserted = mBottomRight->insertPoint(aPoint);
          }
        }
      }

    } else {
      if (aPoint.y > mBorder.Ycenter()) {
        // std::cout << "TopLeft" << '\n';
        if (mTopLeft) {
          isInserted = mTopLeft->insertPoint(aPoint);
        } else {
          mTopLeft = createNode(mBorder.createTopLeft());
          if (mTopLeft) {
            isInserted = mTopLeft->insertPoint(aPoint);
          }
        }
      } else {
        // std::cout << "BottomLeft" << '\n';
        if (mBottomLeft) {
          isInserted = mBottomLeft->insertPoint(aPoint);
        } else {
          mBottomLeft = createNode(mBorder.createBottomLeft());
          if (mBottomLeft) {
            isInserted = mBottomLeft->insertPoint(aPoint);
          }
        }
      }
    }

    // If new node is created then move the stored point
    if (isInserted && mPoint) {
      // std::cout << "Rebalance point" << '\n';
      const auto rebalancedPoint = std::move(mPoint.value());
      mPoint.reset();
      return insertPoint(rebalancedPoint);
    }

    return isInserted;
  }

  std::optional<Point> findPoint(const Point& aPoint) {
    // std::cout << "Find " << aPoint << '\n';
    // std::cout << "Find in " << mBorder << '\n';

    // Point out of borders
    if (!mBorder.isPointInside(aPoint)) {
      // std::cout << "Find out of borders" << '\n';
      return std::nullopt;
    }

    // No more points to search
    if (mPoint) {
      // Check if point found is close enough
      if (*mPoint != aPoint) {
        // std::cout << "Not close enough " << std::abs(mPoint->x - aPoint.x)
        //           << " " << std::abs(mPoint->y - aPoint.y) << "\n";
        return std::nullopt;
      }
      // std::cout << "Found\n";
      return mPoint;
    }

    // TODO: Parallelization point
    if (mTopRight) {
      // std::cout << "Search TopRight\n";
      if (auto pointFound = mTopRight->findPoint(aPoint)) {
        return pointFound;
      }
    }

    if (mTopLeft) {
      // std::cout << "Search TopLeft\n";
      if (auto pointFound = mTopLeft->findPoint(aPoint)) {
        return pointFound;
      }
    }

    if (mBottomRight) {
      // std::cout << "Search BottomRight\n";
      if (auto pointFound = mBottomRight->findPoint(aPoint)) {
        return pointFound;
      }
    }

    if (mBottomLeft) {
      // std::cout << "Search BottomLeft\n";
      if (auto pointFound = mBottomLeft->findPoint(aPoint)) {
        return pointFound;
      }
    }

    // std::cout << "Not Found\n";
    return std::nullopt;
  }

  [[nodiscard]] std::list<Point> findPointsInArea(
      const Rectangle& aArea) const {
    const auto overlap = calculateOverlap(mBorder, aArea);

    std::list<Point> result;
    if (overlap == Overlap::NO) {
      // std::cout << "No overlap between " << mBorder << " and " << aArea <<
      // '\n';
      return {};
    }

    // std::cout << "Overlap between " << mBorder << " and " << aArea
    // << '\n'; Leaf node
    if (mPoint) {
      // std::cout << "Point found in area\n";
      return {*mPoint};
    }

    // TODO: Parallelization point
    if (mTopRight) {
      // std::cout << "Search area TopRight\n";
      auto&& pointsFoundInArea = mTopRight->findPointsInArea(aArea);
      result.splice(std::end(result), std::move(pointsFoundInArea));
    }

    if (mTopLeft) {
      // std::cout << "Search area TopLeft\n";
      auto&& pointsFoundInArea = mTopLeft->findPointsInArea(aArea);
      result.splice(std::end(result), std::move(pointsFoundInArea));
    }

    if (mBottomRight) {
      // std::cout << "Search area BottomRight\n";
      auto&& pointsFoundInArea = mBottomRight->findPointsInArea(aArea);
      result.splice(std::end(result), std::move(pointsFoundInArea));
    }

    if (mBottomLeft) {
      // std::cout << "Search area BottomLeft\n";
      auto&& pointsFoundInArea = mBottomLeft->findPointsInArea(aArea);
      result.splice(std::end(result), std::move(pointsFoundInArea));
    }
    return result;
  }

  bool deletePoint(const Point& aPoint) {
    // std::cout << "Delete " << aPoint << '\n';
    // std::cout << "Delete in " << mBorder << '\n';

    // Point out of borders
    if (!mBorder.isPointInside(aPoint)) {
      // std::cout << "Delete out of borders" << '\n';
      return false;
    }

    // Leaf - one point to be deleted
    if (mPoint) {
      // Check if point found is close enough
      if (*mPoint != aPoint) {
        // std::cout << "Not close enough to delete"
        //           << std::abs(mPoint->x - aPoint.x) << " "
        //           << std::abs(mPoint->y - aPoint.y) << "\n";
        return false;
      }
      // std::cout << "Deleted\n";
      mPoint = std::nullopt;
      return true;
    }

    bool isDeleted = false;
    // TODO: Parallelization point
    if (mTopRight) {
      // std::cout << "Delete in TopRight\n";
      if (mTopRight->deletePoint(aPoint)) {
        isDeleted = true;
        if (mTopRight->isEmpty()) {
          mTopRight.reset();
        }
      }
    }

    if (mTopLeft) {
      // std::cout << "Delete in TopLeft\n";
      if (mTopLeft->deletePoint(aPoint)) {
        isDeleted = true;
        if (mTopLeft->isEmpty()) {
          mTopLeft.reset();
        }
      }
    }

    if (mBottomRight) {
      // std::cout << "Delete in BottomRight\n";
      if (mBottomRight->deletePoint(aPoint)) {
        isDeleted = true;
        if (mBottomRight->isEmpty()) {
          mBottomRight.reset();
        }
      }
    }

    if (mBottomLeft) {
      // std::cout << "Delete in BottomLeft\n";
      if (mBottomLeft->deletePoint(aPoint)) {
        isDeleted = true;
        if (mBottomLeft->isEmpty()) {
          mBottomLeft.reset();
        }
      }
    }

    // Rebalance last remaining leaf
    const int activeTopRight = static_cast<bool>(mTopRight);
    const int activeTopLeft = static_cast<bool>(mTopLeft);
    const int activeBottomRight = static_cast<bool>(mBottomRight);
    const int activeBottomLeft = static_cast<bool>(mBottomLeft);
    if (activeTopRight + activeTopLeft + activeBottomRight + activeBottomLeft ==
        1) {
      if (mTopRight && mTopRight->mPoint) {
        // std::cout << "Rebalance after delete in TopRight\n";
        mPoint = mTopRight->mPoint;
        mTopRight.reset();
      }

      if (mTopLeft && mTopLeft->mPoint) {
        // std::cout << "Rebalance after delete in TopLeft\n";
        mPoint = mTopLeft->mPoint;
        mTopLeft.reset();
      }

      if (mBottomRight && mBottomRight->mPoint) {
        // std::cout << "Rebalance after delete in BottomRight\n";
        mPoint = mBottomRight->mPoint;
        mBottomRight.reset();
      }

      if (mBottomLeft && mBottomLeft->mPoint) {
        // std::cout << "Rebalance after delete in BottomLeft\n";
        mPoint = mBottomLeft->mPoint;
        mBottomLeft.reset();
      }
    }

    return isDeleted;
  }

  [[nodiscard]] bool isEmpty() const {
    return !(mPoint || mTopRight || mTopLeft || mBottomRight || mBottomLeft);
  }

  [[nodiscard]] Rectangle getArea() const { return mBorder; }

  [[nodiscard]] std::list<Point> getAllPoints() const {
    return findPointsInArea(getArea());
  }

  std::vector<std::pair<Point, Rectangle>> getAreaInfo() {
    if (mPoint) {
      return {{*mPoint, mBorder}};
    }

    std::vector<std::pair<Point, Rectangle>> result;
    // TODO: Parallelization point
    if (mTopRight) {
      // std::cout << "Area info TopRight\n";
      const auto areaInfo = mTopRight->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }

    if (mTopLeft) {
      // std::cout << "Area info TopLeft\n";
      const auto areaInfo = mTopLeft->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }

    if (mBottomRight) {
      // std::cout << "Area info BottomRight\n";
      const auto areaInfo = mBottomRight->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }

    if (mBottomLeft) {
      // std::cout << "Area info BottomLeft\n";
      const auto areaInfo = mBottomLeft->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }
    return result;
  }

 private:
  Node(const Rectangle& aBorder) : mBorder{aBorder} {
    // std::cout << "Node border " << mBorder << '\n';
  }

  // Node borders
  Rectangle mBorder;

  // Stored point
  std::optional<Point> mPoint;

  // Node quads
  std::unique_ptr<Node> mTopRight;
  std::unique_ptr<Node> mTopLeft;
  std::unique_ptr<Node> mBottomRight;
  std::unique_ptr<Node> mBottomLeft;
};

int main(int /*argc*/, char* /*argv*/[]) {
  std::vector<Point> testPoints;
  testPoints.reserve(kPointsNumber);

  // Random generator of points
  std::random_device rd{};
  std::mt19937 gen{rd()};
  gen.seed(kSeed);

  std::uniform_real_distribution<float> distX{kXmin, kXmax};
  std::uniform_real_distribution<float> distY{kYmin, kYmax};

  std::cout << "Points count: " << kPointsNumber << '\n';

  // Insert points in vector
  {
    TimeBench bench{"Insert points in vector"};
    for (int i = 0; i < kPointsNumber; i++) {
      testPoints.emplace_back(distX(gen), distY(gen));
    }
  }

  // Root node
  auto root = Node::createNode({kXmin, kXmax, kYmin, kYmax});
  if (!root) {
    std::cout << "Node area too small!\n";
    return EXIT_FAILURE;
  }

  // Insert points in Quad Tree
  int insertedPoints = 0;
  {
    TimeBench bench{"Insert points in Quad Tree"};
    for (const auto& point : testPoints) {
      // std::cout << "===\n";
      if (root->insertPoint(point)) {
        insertedPoints++;
      } else {
        // std::cout << "Failed to insert point!" << '\n';
      }
      // std::cout << "===\n";
    }
  }
  if (insertedPoints != testPoints.size()) {
    std::cout << "Expected points: " << testPoints.size()
              << "  Inserted points: " << insertedPoints << '\n';
  }

  // Find points in vector
  int pointsFound = 0;
  {
    TimeBench bench{"Find points in vector"};
    for (const auto& point : testPoints) {
      auto itFoundPoint =
          std::find(std::begin(testPoints), std::end(testPoints), point);
      if (std::end(testPoints) != itFoundPoint) {
        pointsFound++;
      }
    }
  }
  if (pointsFound != testPoints.size()) {
    std::cout << "Expected points: " << testPoints.size()
              << "  Found points: " << pointsFound << '\n';
  }

  // Find points in Quad Tree
  pointsFound = 0;
  {
    TimeBench bench{"Find points in Quad Tree"};
    for (const auto& point : testPoints) {
      // std::cout << "=====\n";
      const auto pointFound = root->findPoint(point);
      // std::cout << "Find point: " << pointFound.has_value() << '\n';
      if (pointFound) {
        pointsFound++;
        // std::cout << "Found point coordinates: {" << pointFound->x << "," <<
        // pointFound->y << "}\n";
      }
      // std::cout << "=====\n";
    }
  }
  if (pointsFound != testPoints.size()) {
    std::cout << "Expected points: " << testPoints.size()
              << "  Found points: " << pointsFound << '\n';
  }

  // Missing point?
  // const auto pointFound = root->findPoint(Point{0.1, 0.1});
  // std::cout << "Find point: " << pointFound.has_value() << '\n';
  // if (pointFound) {
  //   std::cout << "Found point coordinates: {" << pointFound->x << "," <<
  //   pointFound->y << "}\n";
  // }

  // Search points in area
  const Rectangle searchArea{-1.0, 0.0, -1.0, 0.0};
  std::vector<Point> pointsFoundInArea;

  // Search points in area in vector
  {
    TimeBench bench{"Find points in area in vector"};
    std::copy_if(std::begin(testPoints), std::end(testPoints),
                 std::back_inserter(pointsFoundInArea),
                 [&searchArea](const auto& aPoint) {
                   return searchArea.isPointInside(aPoint);
                 });
  }
  std::cout << "Points found in area: " << pointsFoundInArea.size() << '\n';

  // Search points in area in Quad Tree
  std::list<Point> pointsFoundInAreaQT;
  {
    TimeBench bench{"Find points in area in Quad Tree"};
    pointsFoundInAreaQT = root->findPointsInArea(searchArea);
  }
  std::cout << "Points found in area: " << pointsFoundInAreaQT.size() << '\n';

  // Area info for each stored point
  // for (const auto& areaInfo : root->getAreaInfo()) {
  //   std::cout << "=====\n";
  //   std::cout << "Area info: " << areaInfo.first << " : " << areaInfo.second
  //   << '\n'; std::cout << "=====\n";
  // }

  // Delete points in Quad Tree
  int pointsDeleted = 0;
  {
    TimeBench bench{"Delete points in Quad Tree"};
    for (const auto& point : testPoints) {
      // std::cout << "=====\n";
      // std::cout << "Points in root:\n";
      // for (const auto& currentPoint : root->getAllPoints()) {
      //   std::cout << currentPoint << '\n';
      // }
      // std::cout << "=====\n";
      const auto isPointDeleted = root->deletePoint(point);
      if (isPointDeleted) {
        pointsDeleted++;
      }
      // std::cout << "Delete point: " << point << " " << isPointDeleted <<
      // '\n'; std::cout << "=====\n";
    }
  }

  if (pointsDeleted != testPoints.size()) {
    std::cout << "Expected deleted points: " << testPoints.size()
              << "  Deleted points: " << pointsDeleted << '\n';
  }

  // std::cout << "Root empty: " << root->isEmpty() << '\n';

  std::cout << "Done.\n";

  return EXIT_SUCCESS;
}