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
        std::chrono::duration_cast<std::chrono::milliseconds>(endPoint -
                                                              mStartPoint)
            .count();
    std::cout << mLabel << " : " << processTime << " milliseconds\n";
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
        mYmax{std::max(aYmin, aYmax)} {}

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

 private:
  // Borders
  float mXmax;
  float mXmin;
  float mYmax;
  float mYmin;
};

std::ostream& operator<<(std::ostream& os, const Rectangle& data) {
  os << "{" << data.Xmin() << "," << data.Xmax() << "}{" << data.Ymin() << ","
     << data.Ymax() << "}";
  return os;
}

class Node {
 public:
  static std::unique_ptr<Node> createNode(const Point& aPoint) {
    std::unique_ptr<Node> result(new Node(aPoint));
    return result;
  }

  bool insertPoint(const Point& aPoint) {
    // std::cout << "Insert " << aPoint << '\n';

    // Check if point is previously inserted
    if (aPoint == mPoint) {
      // std::cout << "Point already exists!\n";
      return false;
    }

    bool isInserted = true;
    if (aPoint.x > mPoint.x) {
      if (aPoint.y > mPoint.y) {
        // std::cout << "TopRight" << '\n';
        if (mTopRight) {
          isInserted = mTopRight->insertPoint(aPoint);
        } else {
          mTopRight = createNode(aPoint);
          isInserted = static_cast<bool>(mTopRight);
        }
      } else {
        // std::cout << "BottomRight" << '\n';
        if (mBottomRight) {
          isInserted = mBottomRight->insertPoint(aPoint);
        } else {
          mBottomRight = createNode(aPoint);
          isInserted = static_cast<bool>(mBottomRight);
        }
      }

    } else {
      if (aPoint.y > mPoint.y) {
        // std::cout << "TopLeft" << '\n';
        if (mTopLeft) {
          isInserted = mTopLeft->insertPoint(aPoint);
        } else {
          mTopLeft = createNode(aPoint);
          isInserted = static_cast<bool>(mTopLeft);
        }
      } else {
        // std::cout << "BottomLeft" << '\n';
        if (mBottomLeft) {
          isInserted = mBottomLeft->insertPoint(aPoint);
        } else {
          mBottomLeft = createNode(aPoint);
          isInserted = static_cast<bool>(mBottomLeft);
        }
      }
    }

    updateDepth();

    return isInserted;
  }

  bool findPoint(const Point& aPoint) {
    // std::cout << "Find " << aPoint << '\n';

    // Check if Node point found is close enough
    if (mPoint == aPoint) {
      // std::cout << "Found\n";
      return true;
    }

    // Search children
    if (aPoint.x > mPoint.x) {
      if (aPoint.y > mPoint.y) {
        // std::cout << "Find TopRight" << '\n';
        if (mTopRight) {
          return mTopRight->findPoint(aPoint);
        }
      } else {
        // std::cout << "Find BottomRight" << '\n';
        if (mBottomRight) {
          return mBottomRight->findPoint(aPoint);
        }
      }

    } else {
      if (aPoint.y > mPoint.y) {
        // std::cout << "Find TopLeft" << '\n';
        if (mTopLeft) {
          return mTopLeft->findPoint(aPoint);
        }
      } else {
        // std::cout << "Find BottomLeft" << '\n';
        if (mBottomLeft) {
          return mBottomLeft->findPoint(aPoint);
        }
      }
    }

    // std::cout << "Not Found\n";
    return false;
  }

  [[nodiscard]] std::list<Point> findPointsInArea(
      const Rectangle& aArea) const {
    // std::cout << "Find in " << aArea << '\n';

    std::list<Point> result;

    if (aArea.isPointInside(mPoint)) {
      result.push_back(mPoint);
    }

    // Search children
    // TODO: Parallelization point
    if (mPoint.x <= aArea.Xmax() && mPoint.y <= aArea.Ymax() && mTopRight) {
      // std::cout << "Search Area TopRight\n";
      auto&& pointsFoundInArea = mTopRight->findPointsInArea(aArea);
      result.splice(std::end(result), pointsFoundInArea);
    }

    if (mPoint.x >= aArea.Xmin() && mPoint.y <= aArea.Ymax() && mTopLeft) {
      // std::cout << "Search Area TopLeft\n";
      auto&& pointsFoundInArea = mTopLeft->findPointsInArea(aArea);
      result.splice(std::end(result), pointsFoundInArea);
    }

    if (mPoint.x <= aArea.Xmax() && mPoint.y >= aArea.Ymin() && mBottomRight) {
      // std::cout << "Search Area BottomRight\n";
      auto&& pointsFoundInArea = mBottomRight->findPointsInArea(aArea);
      result.splice(std::end(result), pointsFoundInArea);
    }

    if (mPoint.x >= aArea.Xmin() && mPoint.y >= aArea.Ymin() && mBottomLeft) {
      // std::cout << "Search Area BottomLeft\n";
      auto&& pointsFoundInArea = mBottomLeft->findPointsInArea(aArea);
      result.splice(std::end(result), pointsFoundInArea);
    }

    return result;
  }

  bool deletePoint(const Point& aPoint) {
    // std::cout << "Delete " << aPoint << '\n';

    // Check if Node point to delete is close enough
    if (mPoint == aPoint) {
      mIsdeleted = true;
      // Current node point value is invalid - update depth
      const auto largestDepth =
          std::max({getDepth(mTopRight), getDepth(mBottomRight),
                    getDepth(mTopLeft), getDepth(mBottomLeft)});
      mDepth = largestDepth;

      // std::cout << (mDepth ? "Deleted current node"
      //                      : "Current node invalidated")
      //           << '\n';

      if (mTopRight && !mBottomRight && !mTopLeft && !mBottomLeft) {
        // std::cout << "Replace with TopRight node\n";

        mBottomRight = std::move(mTopRight->mBottomRight);
        mTopLeft = std::move(mTopRight->mTopLeft);
        mBottomLeft = std::move(mTopRight->mBottomLeft);

        mPoint = mTopRight->mPoint;
        mDepth = mTopRight->mDepth;
        mTopRight = std::move(mTopRight->mTopRight);
        return true;
      }

      if (!mTopRight && mBottomRight && !mTopLeft && !mBottomLeft) {
        // std::cout << "Replace with BottomRight node\n";

        mTopRight = std::move(mBottomRight->mTopRight);
        mTopLeft = std::move(mBottomRight->mTopLeft);
        mBottomLeft = std::move(mBottomRight->mBottomLeft);

        mPoint = mBottomRight->mPoint;
        mDepth = mBottomRight->mDepth;
        mBottomRight = std::move(mBottomRight->mBottomRight);
        return true;
      }

      if (!mTopRight && !mBottomRight && mTopLeft && !mBottomLeft) {
        // std::cout << "Replace with TopLeft node\n";
        mTopRight = std::move(mTopLeft->mTopRight);
        mBottomRight = std::move(mTopLeft->mBottomRight);
        mBottomLeft = std::move(mTopLeft->mBottomLeft);

        mPoint = mTopLeft->mPoint;
        mDepth = mTopLeft->mDepth;
        mTopLeft = std::move(mTopLeft->mTopLeft);
        return true;
      }

      if (!mTopRight && !mBottomRight && !mTopLeft && mBottomLeft) {
        // std::cout << "Replace with BottomLeft node\n";
        mTopRight = std::move(mBottomLeft->mTopRight);
        mBottomRight = std::move(mBottomLeft->mBottomRight);
        mTopLeft = std::move(mBottomLeft->mTopLeft);

        mPoint = mBottomLeft->mPoint;
        mDepth = mBottomLeft->mDepth;
        mBottomLeft = std::move(mBottomLeft->mBottomLeft);
        return true;
      }

      // TODO: Proceed children of deleted node

      return true;
    }

    // Delete in children
    if (aPoint.x > mPoint.x) {
      if (aPoint.y > mPoint.y) {
        // std::cout << "Delete in TopRight" << '\n';
        if (mTopRight) {
          const auto isDeleted = mTopRight->deletePoint(aPoint);
          if (mTopRight->isEmpty()) {
            mTopRight.reset();
            // std::cout << "Deleted Node TopRight" << '\n';
          }
          if (isDeleted) {
            updateDepth();
          }
          return isDeleted;
        }
      } else {
        // std::cout << "Delete in BottomRight" << '\n';
        if (mBottomRight) {
          const auto isDeleted = mBottomRight->deletePoint(aPoint);
          if (mBottomRight->isEmpty()) {
            // std::cout << "Deleted Node BottomRight" << '\n';
            mBottomRight.reset();
          }
          if (isDeleted) {
            updateDepth();
          }
          return isDeleted;
        }
      }

    } else {
      if (aPoint.y > mPoint.y) {
        // std::cout << "Delete in TopLeft" << '\n';
        if (mTopLeft) {
          const auto isDeleted = mTopLeft->deletePoint(aPoint);
          if (mTopLeft->isEmpty()) {
            // std::cout << "Deleted Node TopLeft" << '\n';
            mTopLeft.reset();
          }
          if (isDeleted) {
            updateDepth();
          }
          return isDeleted;
        }
      } else {
        // std::cout << "Delete in BottomLeft" << '\n';
        if (mBottomLeft) {
          const auto isDeleted = mBottomLeft->deletePoint(aPoint);
          if (mBottomLeft->isEmpty()) {
            // std::cout << "Deleted Node BottomLeft" << '\n';
            mBottomLeft.reset();
          }
          if (isDeleted) {
            updateDepth();
          }
          return isDeleted;
        }
      }
    }

    // std::cout << "Point " << aPoint << " not deleted\n";
    return false;
  }

  [[nodiscard]] Point getPoint() const { return mPoint; }

  [[nodiscard]] std::list<Point> getAllPoints() const {
    std::list<Point> points;

    if (!mIsdeleted) {
      points.push_back(mPoint);
    }

    // TODO: Parallelization point
    if (mTopRight) {
      // std::cout << "Get points from TopRight\n";
      auto&& pointsFound = mTopRight->getAllPoints();
      points.splice(std::end(points), pointsFound);
    }

    if (mTopLeft) {
      // std::cout << "Get points from TopLeft\n";
      auto&& pointsFound = mTopLeft->getAllPoints();
      points.splice(std::end(points), pointsFound);
    }

    if (mBottomRight) {
      // std::cout << "Get points from BottomRight\n";
      auto&& pointsFound = mBottomRight->getAllPoints();
      points.splice(std::end(points), pointsFound);
    }

    if (mBottomLeft) {
      // std::cout << "Get points from BottomLeft\n";
      auto&& pointsFound = mBottomLeft->getAllPoints();
      points.splice(std::end(points), pointsFound);
    }

    return points;
  }

  [[nodiscard]] int getDepth() const { return mDepth; }

  [[nodiscard]] bool isEmpty() const { return mDepth == 0; }

 private:
  Node(const Point& aPoint) : mPoint{aPoint} {
    // std::cout << "Node point " << mPoint << '\n';
  }

  static int getDepth(std::unique_ptr<Node>& aNode) {
    if (aNode) {
      return aNode->mDepth;
    }
    return 0;
  }

  void updateDepth() {
    mDepth = std::max({getDepth(mTopRight), getDepth(mBottomRight),
                       getDepth(mTopLeft), getDepth(mBottomLeft)});
    if (!mIsdeleted) {
      mDepth++;
    }
  }

  // Point
  Point mPoint;

  // Node depth
  int mDepth{1};

  // TODO: Refactor using mDepth
  bool mIsdeleted{false};

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
  auto root = Node::createNode(testPoints.at(0));
  if (!root) {
    std::cout << "Failed to create root!\n";
    return EXIT_FAILURE;
  }

  // Insert points in Quad Tree
  int insertedPoints = 1;
  {
    TimeBench bench{"Insert points in Quad Tree"};

    // First point already inserted
    for (auto startIt = std::next(std::begin(testPoints));
         startIt != std::end(testPoints); ++startIt) {
      const auto point = *startIt;
      // std::cout << "===\n";
      const auto isInserted = root->insertPoint(point);
      if (isInserted) {
        insertedPoints++;
        // std::cout << "Root depth after insert: " << root->getDepth() << '\n';
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
      const auto isPointFound = root->findPoint(point);
      // std::cout << "Find point: " << isPointFound << '\n';
      if (isPointFound) {
        pointsFound++;
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
  // std::cout << "Find point: " << pointFound << '\n';

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
      // std::cout << "Delete point: " << point << " " << isPointDeleted
      //           << "  root depth: " << root->getDepth() << '\n';
      // std::cout << "=====\n";
    }
  }

  if (pointsDeleted != testPoints.size()) {
    std::cout << "Expected deleted points: " << testPoints.size()
              << "  Deleted points: " << pointsDeleted << '\n';
  }

  // std::cout << "Root empty: " << root->isEmpty() << '\n';

  // std::cout << "=====\n";
  // std::cout << "Points in root:\n";
  // for (const auto& currentPoint : root->getAllPoints()) {
  //   std::cout << currentPoint << '\n';
  // }
  // std::cout << "=====\n";

  std::cout << "Done.\n";

  return EXIT_SUCCESS;
}