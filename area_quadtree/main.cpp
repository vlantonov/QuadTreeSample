#include <cmath>
#include <cstdlib>
#include <iostream>
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

constexpr float kMinDistanceX = 0.001;
constexpr float kMinDistanceY = 0.001;

constexpr int kPointsNumber = 10;

struct Point {
  float x{};
  float y{};
};

bool operator==(const Point& lhs, const Point& rhs) {
  return (std::abs(lhs.x - rhs.x) < kEpsilon &&
          std::abs(lhs.y - lhs.y) < kEpsilon);
}

bool operator!=(const Point& lhs, const Point& rhs) { return !(lhs == rhs); }

struct Rectangle {
  Rectangle(float aXmin, float aXmax, float aYmin, float aYmax)
      : mXmin{std::min(aXmin, aXmax)},
        mXmax{std::max(aXmin, aXmax)},
        mYmin{std::min(aYmin, aYmax)},
        mYmax{std::max(aYmin, aYmax)} {
    mCenterX = 0.5 * (mXmin + mXmax);
    mCenterY = 0.5 * (mYmin + mYmax);
  }

  bool isPointInside(const Point& aPoint) const {
    if (aPoint.x > mXmax || aPoint.x < mXmin || aPoint.y > mYmax ||
        aPoint.y < mYmin) {
      std::cout << "Out of borders" << '\n';
      return false;
    }
    return true;
  }

  operator bool() const {
    return !(std::isfinite(mXmin) && std::isfinite(mXmax) &&
             std::isfinite(mYmin) && std::isfinite(mYmax));
  }

  float Xmin() const { return mXmin; }

  float Xmax() const { return mXmax; }

  float Ymin() const { return mYmin; }

  float Ymax() const { return mYmax; }

  float Xcenter() const { return mCenterX; }

  float Ycenter() const { return mCenterY; }

  Rectangle createTopRight() const {
    return {mCenterX, mXmax, mCenterY, mYmax};
  }

  Rectangle createBottomRight() const {
    return {mCenterX, mXmax, mYmin, mCenterY};
  }

  Rectangle createTopLeft() const { return {mXmin, mCenterX, mCenterY, mYmax}; }

  Rectangle createBottomLeft() const {
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
  static std::shared_ptr<Node> createNode(const Rectangle& aBorder) {
    if (std::abs(aBorder.Xmax() - aBorder.Xmin()) < kMinDistanceX ||
        std::abs(aBorder.Ymax() - aBorder.Ymin()) < kMinDistanceY) {
      return nullptr;
    }

    std::shared_ptr<Node> result(new Node(aBorder));

    return result;
  }

  bool insertPoint(const Point& aPoint) {
    std::cout << "{" << aPoint.x << "," << aPoint.y << "}" << '\n';
    // Point out of borders
    if (!mBorder.isPointInside(aPoint)) {
      std::cout << "Out of borders" << '\n';
      return false;
    }

    // No previously stored point and no child nodes
    if (!mPoint && !(mTopRight || mTopLeft || mBottomRight || mBottomLeft)) {
      mPoint = aPoint;
      std::cout << "Point inserted" << '\n';
      return true;
    }

    bool isInserted = false;
    if (aPoint.x > mBorder.Xcenter()) {
      if (aPoint.y > mBorder.Ycenter()) {
        std::cout << "TopRight" << '\n';
        if (mTopRight) {
          isInserted = mTopRight->insertPoint(aPoint);
        } else {
          mTopRight = createNode(mBorder.createTopRight());
          if (mTopRight) {
            isInserted = mTopRight->insertPoint(aPoint);
          }
        }
      } else {
        std::cout << "BottomRight" << '\n';
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
        std::cout << "TopLeft" << '\n';
        if (mTopLeft) {
          isInserted = mTopLeft->insertPoint(aPoint);
        } else {
          mTopLeft = createNode(mBorder.createTopLeft());
          if (mTopLeft) {
            isInserted = mTopLeft->insertPoint(aPoint);
          }
        }
      } else {
        std::cout << "BottomLeft" << '\n';
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
      std::cout << "Rebalance point" << '\n';
      const auto rebalancedPoint = std::move(mPoint.value());
      mPoint.reset();
      return insertPoint(rebalancedPoint);
    }

    return isInserted;
  }

  std::optional<Point> findPoint(const Point& aPoint) {
    std::cout << "Find {" << aPoint.x << "," << aPoint.y << "}" << '\n';
    std::cout << "Find in " << mBorder << '\n';

    // Point out of borders
    if (!mBorder.isPointInside(aPoint)) {
      std::cout << "Find out of borders" << '\n';
      return std::nullopt;
    }

    // No more points to search
    if (mPoint) {
      // Check if point found is close enough
      if (*mPoint != aPoint) {
        std::cout << "Not close enough " << std::abs(mPoint->x - aPoint.x)
                  << " " << std::abs(mPoint->y - aPoint.y) << "\n";
        return std::nullopt;
      }
      std::cout << "Found\n";
      return mPoint;
    }

    // TODO: Parallelization point
    if (mTopRight) {
      std::cout << "Search TopRight\n";
      if (auto pointFound = mTopRight->findPoint(aPoint)) {
        return pointFound;
      }
    }

    if (mTopLeft) {
      std::cout << "Search TopLeft\n";
      if (auto pointFound = mTopLeft->findPoint(aPoint)) {
        return pointFound;
      }
    }

    if (mBottomRight) {
      std::cout << "Search BottomRight\n";
      if (auto pointFound = mBottomRight->findPoint(aPoint)) {
        return pointFound;
      }
    }

    if (mBottomLeft) {
      std::cout << "Search BottomLeft\n";
      if (auto pointFound = mBottomLeft->findPoint(aPoint)) {
        return pointFound;
      }
    }

    std::cout << "Not Found\n";
    return std::nullopt;
  }

  std::vector<Point> findPointsInArea(const Rectangle& aArea) const {
    const auto overlap = calculateOverlap(mBorder, aArea);

    std::vector<Point> result;
    switch (overlap) {
      case Overlap::NO:
        std::cout << "No overlap between " << mBorder << " and " << aArea
                  << '\n';
        return {};
      case Overlap::YES:
        std::cout << "Full overlap between " << mBorder << " and " << aArea
                  << '\n';
        // Leaf node
        if (mPoint) {
          std::cout << "Point found in area\n";
          return {*mPoint};
        }

        // TODO: Parallelization point
        if (mTopRight) {
          std::cout << "Search area TopRight\n";
          const auto pointsFoundInArea = mTopRight->findPointsInArea(aArea);
          result.insert(std::end(result), std::cbegin(pointsFoundInArea),
                        std::cend(pointsFoundInArea));
        }

        if (mTopLeft) {
          std::cout << "Search area TopLeft\n";
          const auto pointsFoundInArea = mTopLeft->findPointsInArea(aArea);
          result.insert(std::end(result), std::cbegin(pointsFoundInArea),
                        std::cend(pointsFoundInArea));
        }

        if (mBottomRight) {
          std::cout << "Search area BottomRight\n";
          const auto pointsFoundInArea = mBottomRight->findPointsInArea(aArea);
          result.insert(std::end(result), std::cbegin(pointsFoundInArea),
                        std::cend(pointsFoundInArea));
        }

        if (mBottomLeft) {
          std::cout << "Search area BottomLeft\n";
          const auto pointsFoundInArea = mBottomLeft->findPointsInArea(aArea);
          result.insert(std::end(result), std::cbegin(pointsFoundInArea),
                        std::cend(pointsFoundInArea));
        }
        return result;

      case Overlap::PARTIAL:
        // TODO: Parallelization point
        std::cout << "Partial overlap between " << mBorder << " and " << aArea
                  << '\n';
        if (mTopRight) {
          std::cout << "Search area TopRight\n";
          const auto pointsFoundInArea = mTopRight->findPointsInArea(aArea);
          std::copy_if(std::cbegin(pointsFoundInArea),
                       std::cend(pointsFoundInArea), std::back_inserter(result),
                       [&aArea](const auto& aPoint) {
                         return aArea.isPointInside(aPoint);
                       });
        }

        if (mTopLeft) {
          std::cout << "Search area TopLeft\n";
          const auto pointsFoundInArea = mTopLeft->findPointsInArea(aArea);
          std::copy_if(std::cbegin(pointsFoundInArea),
                       std::cend(pointsFoundInArea), std::back_inserter(result),
                       [&aArea](const auto& aPoint) {
                         return aArea.isPointInside(aPoint);
                       });
        }

        if (mBottomRight) {
          std::cout << "Search area BottomRight\n";
          const auto pointsFoundInArea = mBottomRight->findPointsInArea(aArea);
          std::copy_if(std::cbegin(pointsFoundInArea),
                       std::cend(pointsFoundInArea), std::back_inserter(result),
                       [&aArea](const auto& aPoint) {
                         return aArea.isPointInside(aPoint);
                       });
        }

        if (mBottomLeft) {
          std::cout << "Search area BottomLeft\n";
          const auto pointsFoundInArea = mBottomLeft->findPointsInArea(aArea);
          std::copy_if(std::cbegin(pointsFoundInArea),
                       std::cend(pointsFoundInArea), std::back_inserter(result),
                       [&aArea](const auto& aPoint) {
                         return aArea.isPointInside(aPoint);
                       });
        }
        return result;
    }

    std::cout << "Unhandled overlap value: " << static_cast<int>(overlap)
              << "\n";
    return {};
  }

  bool deletePoint(const Point& aPoint) {
    std::cout << "Delete {" << aPoint.x << "," << aPoint.y << "}" << '\n';
    std::cout << "Delete in " << mBorder << '\n';

    // Point out of borders
    if (!mBorder.isPointInside(aPoint)) {
      std::cout << "Delete out of borders" << '\n';
      return false;
    }

    // Leaf - one point to be deleted
    if (mPoint) {
      // Check if point found is close enough
      if (*mPoint != aPoint) {
        std::cout << "Not close enough to delete"
                  << std::abs(mPoint->x - aPoint.x) << " "
                  << std::abs(mPoint->y - aPoint.y) << "\n";
        return false;
      }
      std::cout << "Deleted\n";
      mPoint = std::nullopt;
      return true;
    }

    bool isDeleted = false;
    // TODO: Parallelization point
    if (mTopRight) {
      std::cout << "Delete in TopRight\n";
      if (mTopRight->deletePoint(aPoint)) {
        isDeleted = true;
        if (mTopRight->isEmpty()) {
          mTopRight.reset();
        }
      }
    }

    if (mTopLeft) {
      std::cout << "Delete in TopLeft\n";
      if (mTopLeft->deletePoint(aPoint)) {
        isDeleted = true;
        if (mTopLeft->isEmpty()) {
          mTopLeft.reset();
        }
      }
    }

    if (mBottomRight) {
      std::cout << "Delete in BottomRight\n";
      if (mBottomRight->deletePoint(aPoint)) {
        isDeleted = true;
        if (mBottomRight->isEmpty()) {
          mBottomRight.reset();
        }
      }
    }

    if (mBottomLeft) {
      std::cout << "Delete in BottomLeft\n";
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
        std::cout << "Rebalance after delete in TopRight\n";
        mPoint = mTopRight->mPoint;
        mTopRight.reset();
      }

      if (mTopLeft && mTopLeft->mPoint) {
        std::cout << "Rebalance after delete in TopLeft\n";
        mPoint = mTopLeft->mPoint;
        mTopLeft.reset();
      }

      if (mBottomRight && mBottomRight->mPoint) {
        std::cout << "Rebalance after delete in BottomRight\n";
        mPoint = mBottomRight->mPoint;
        mBottomRight.reset();
      }

      if (mBottomLeft && mBottomLeft->mPoint) {
        std::cout << "Rebalance after delete in BottomLeft\n";
        mPoint = mBottomLeft->mPoint;
        mBottomLeft.reset();
      }
    }

    return isDeleted;
  }

  bool isEmpty() const {
    return !(mPoint || mTopRight || mTopLeft || mBottomRight || mBottomLeft);
  }

  Rectangle getArea() const { return mBorder; }

  std::vector<Point> getAllPoints() const {
    return findPointsInArea(getArea());
  }

  std::vector<std::pair<Point, Rectangle>> getAreaInfo() {
    if (mPoint) {
      return {{*mPoint, mBorder}};
    }

    std::vector<std::pair<Point, Rectangle>> result;
    // TODO: Parallelization point
    if (mTopRight) {
      std::cout << "Area info TopRight\n";
      const auto areaInfo = mTopRight->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }

    if (mTopLeft) {
      std::cout << "Area info TopLeft\n";
      const auto areaInfo = mTopLeft->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }

    if (mBottomRight) {
      std::cout << "Area info BottomRight\n";
      const auto areaInfo = mBottomRight->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }

    if (mBottomLeft) {
      std::cout << "Area info BottomLeft\n";
      const auto areaInfo = mBottomLeft->getAreaInfo();
      result.insert(std::end(result), std::cbegin(areaInfo),
                    std::cend(areaInfo));
    }
    return result;
  }

 private:
  Node(const Rectangle& aBorder) : mBorder{aBorder} {
    std::cout << "Node border " << mBorder << '\n';
  }

  // Node borders
  Rectangle mBorder;

  // Stored point
  std::optional<Point> mPoint;

  // Node quads
  std::shared_ptr<Node> mTopRight;
  std::shared_ptr<Node> mTopLeft;
  std::shared_ptr<Node> mBottomRight;
  std::shared_ptr<Node> mBottomLeft;
};

int main(int arcg, char* argv[]) {
  std::vector<Point> testPoints;
  testPoints.reserve(kPointsNumber);

  // Random generator of points
  std::random_device rd{};
  std::mt19937 gen{rd()};
  gen.seed(kSeed);

  std::uniform_real_distribution<float> distX{kXmin, kXmax};
  std::uniform_real_distribution<float> distY{kYmin, kYmax};

  // Insert points in vector
  for (int i = 0; i < kPointsNumber; i++) {
    testPoints.emplace_back(distX(gen), distY(gen));
  }

  // Root node
  auto root = Node::createNode({kXmin, kXmax, kYmin, kYmax});
  if (!root) {
    std::cout << "Node area too small!\n";
    return EXIT_FAILURE;
  }

  // Insert points in Quad Tree
  for (const auto& point : testPoints) {
    std::cout << "===\n";
    if (!root->insertPoint(point)) {
      std::cout << "Failed to insert point!" << '\n';
    }
    std::cout << "===\n";
  }

  // Find points in Quad Tree
  for (const auto& point : testPoints) {
    std::cout << "=====\n";
    const auto pointFound = root->findPoint(point);
    std::cout << "Find point: " << pointFound.has_value() << '\n';
    if (pointFound) {
      std::cout << "Found point coordinates: {" << pointFound->x << ","
                << pointFound->y << "}\n";
    }
    std::cout << "=====\n";
  }

  // Missing point?
  const auto pointFound = root->findPoint(Point{0.1, 0.1});
  std::cout << "Find point: " << pointFound.has_value() << '\n';
  if (pointFound) {
    std::cout << "Found point coordinates: {" << pointFound->x << ","
              << pointFound->y << "}\n";
  }

  // Search points in area
  const Rectangle searchArea{-1.0, 0.0, -1.0, 0.0};

  const auto pointsFoundInArea = root->findPointsInArea(searchArea);
  std::cout << "Points found in area " << searchArea << " : "
            << pointsFoundInArea.size() << '\n';
  for (const auto& point : pointsFoundInArea) {
    std::cout << "Point in area: {" << point.x << "," << point.y << "}\n";
  }

  // Area info for each stored point
  for (const auto& areaInfo : root->getAreaInfo()) {
    std::cout << "=====\n";
    std::cout << "Area info: {" << areaInfo.first.x << "," << areaInfo.first.y
              << "} : " << areaInfo.second << '\n';
    std::cout << "=====\n";
  }

  // Delete points in Quad Tree
  for (const auto& point : testPoints) {
    std::cout << "=====\n";
    std::cout << "Points in root:\n";
    for (const auto& currentPoint : root->getAllPoints()) {
      std::cout << "{" << currentPoint.x << "," << currentPoint.y << "}\n";
    }
    std::cout << "=====\n";
    const auto isPointDeleted = root->deletePoint(point);
    std::cout << "Delete point: {" << point.x << "," << point.y << "} "
              << isPointDeleted << '\n';
    std::cout << "=====\n";
  }

  std::cout << "Root empty: " << root->isEmpty() << '\n';

  std::cout << "Done.\n";

  return EXIT_SUCCESS;
}