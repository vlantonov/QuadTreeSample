#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <random>
#include <vector>

// Random device seed
constexpr auto kSeed = 1234;

constexpr float epsilon = 1e-6;

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
  }

  operator bool() const {
    return !(std::isfinite(mXmin) && std::isfinite(mXmax) &&
             std::isfinite(mYmin) && std::isfinite(mYmax));
  }

  float Xmin() const { return mXmin; }

  float Xmax() const { return mXmax; }

  float Ymin() const { return mYmin; }

  float Ymax() const { return mYmax; }

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

  if (std::abs(intersectionArea - minaArea) < epsilon) {
    return Overlap::YES;
  }

  return Overlap::PARTIAL;
}

class Node {
 public:
  static std::shared_ptr<Node> createNode(float aBorderXmin, float aBorderXmax,
                                          float aBorderYmin,
                                          float aBorderYmax) {
    if (std::abs(aBorderXmax - aBorderXmin) < kMinDistanceX ||
        std::abs(aBorderYmax - aBorderYmin) < kMinDistanceY) {
      return nullptr;
    }

    std::shared_ptr<Node> result(
        new Node(aBorderXmin, aBorderXmax, aBorderYmin, aBorderYmax));

    return result;
  }

  bool insertPoint(const Point& aPoint) {
    std::cout << "{" << aPoint.x << "," << aPoint.y << "}" << '\n';
    // Point out of borders
    // TODO: Rectangle method
    if (aPoint.x > mBorderXmax || aPoint.x < mBorderXmin ||
        aPoint.y > mBorderYmax || aPoint.y < mBorderYmin) {
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
    if (aPoint.x > mCenterX) {
      if (aPoint.y > mCenterY) {
        std::cout << "TopRight" << '\n';
        if (mTopRight) {
          isInserted = mTopRight->insertPoint(aPoint);
        } else {
          mTopRight = createNode(mCenterX, mBorderXmax, mCenterY, mBorderYmax);
          if (mTopRight) {
            isInserted = mTopRight->insertPoint(aPoint);
          }
        }
      } else {
        std::cout << "BottomRight" << '\n';
        if (mBottomRight) {
          isInserted = mBottomRight->insertPoint(aPoint);
        } else {
          mBottomRight =
              createNode(mCenterX, mBorderXmax, mBorderYmin, mCenterY);
          if (mBottomRight) {
            isInserted = mBottomRight->insertPoint(aPoint);
          }
        }
      }

    } else {
      if (aPoint.y > mCenterY) {
        std::cout << "TopLeft" << '\n';
        if (mTopLeft) {
          isInserted = mTopLeft->insertPoint(aPoint);
        } else {
          mTopLeft = createNode(mBorderXmin, mCenterX, mCenterY, mBorderYmax);
          if (mTopLeft) {
            isInserted = mTopLeft->insertPoint(aPoint);
          }
        }
      } else {
        std::cout << "BottomLeft" << '\n';
        if (mBottomLeft) {
          isInserted = mBottomLeft->insertPoint(aPoint);
        } else {
          mBottomLeft =
              createNode(mBorderXmin, mCenterX, mBorderYmin, mCenterY);
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
    std::cout << "Find in {" << mBorderXmin << "," << mBorderXmax << "}{"
              << mBorderYmin << "," << mBorderYmax << "}" << '\n';
    // Point out of borders
    // TODO: Rectangle method
    if (aPoint.x > mBorderXmax || aPoint.x < mBorderXmin ||
        aPoint.y > mBorderYmax || aPoint.y < mBorderYmin) {
      std::cout << "Find out of borders" << '\n';
      return std::nullopt;
    }

    // No more points to search
    if (mPoint) {
      std::cout << "Found\n";
      return mPoint;
    }

    // TODO: Replace recursion with iteration
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

 private:
  Node(float aBorderXmin, float aBorderXmax, float aBorderYmin,
       float aBorderYmax)
      : mBorderXmin{aBorderXmin},
        mBorderXmax{aBorderXmax},
        mBorderYmin{aBorderYmin},
        mBorderYmax{aBorderYmax} {
    mCenterX = 0.5 * (mBorderXmin + mBorderXmax);
    mCenterY = 0.5 * (mBorderYmin + mBorderYmax);
    std::cout << "{" << mBorderXmin << "," << mBorderXmax << "}{" << mBorderYmin
              << "," << mBorderYmax << "}" << '\n';
  }

  // Node borders
  // TODO: This should be rectangle
  float mBorderXmax;
  float mBorderXmin;
  float mBorderYmax;
  float mBorderYmin;

  // Center point
  float mCenterX;
  float mCenterY;

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
  auto root = Node::createNode(kXmin, kXmax, kYmin, kYmax);
  if (!root) {
    std::cout << "Node range too small!\n";
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
    auto pointFound = root->findPoint(point);
    std::cout << "Find point: " << pointFound.has_value() << '\n';
    if (pointFound) {
      std::cout << "Found point coordinates: {" << pointFound->x << ","
                << pointFound->y << "}\n";
    }
    std::cout << "=====\n";
  }

  // Missing point?
  auto pointFound = root->findPoint(Point{0.1, 0.1});
  std::cout << "Find point: " << pointFound.has_value() << '\n';
  if (pointFound) {
    std::cout << "Found point coordinates: {" << pointFound->x << ","
              << pointFound->y << "}\n";
  }

  std::cout << "Done.\n";

  return EXIT_SUCCESS;
}