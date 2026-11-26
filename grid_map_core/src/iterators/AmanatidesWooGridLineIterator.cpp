#include "grid_map_core/iterators/AmanatidesWooGridLineIterator.hpp"
#include <iostream>
#include <limits>

#include "grid_map_core/GridMapMath.hpp"

namespace grid_map
{

AmanatidesWooGridLineIterator::AmanatidesWooGridLineIterator(
  const grid_map::GridMap & gridMap, const Position & start,
  const Position & end)
{
  Index startIndex, endIndex;
  if (getIndexLimitedToMapRange(gridMap, start, end, startIndex) &&
    getIndexLimitedToMapRange(gridMap, end, start, endIndex))
  {
    initialize(gridMap, startIndex, endIndex);
  } else {
    throw std::invalid_argument("Failed to construct AmanatidesWooGridLineIterator.");
  }
}

AmanatidesWooGridLineIterator::AmanatidesWooGridLineIterator(
  const grid_map::GridMap & gridMap, const Index & start,
  const Index & end)
{
  initialize(gridMap, start, end);
}

AmanatidesWooGridLineIterator & AmanatidesWooGridLineIterator::operator=(const AmanatidesWooGridLineIterator & other)
{
  index_ = other.index_;
  start_ = other.start_;
  end_ = other.end_;
  mapLength_ = other.mapLength_;
  mapPosition_ = other.mapPosition_;
  resolution_ = other.resolution_;
  bufferSize_ = other.bufferSize_;
  bufferStartIndex_ = other.bufferStartIndex_;
  return *this;
}

bool AmanatidesWooGridLineIterator::operator!=(const AmanatidesWooGridLineIterator & other) const
{
  return (index_ != other.index_).any();
}

const Index & AmanatidesWooGridLineIterator::operator*() const
{
  return index_;
}

AmanatidesWooGridLineIterator & AmanatidesWooGridLineIterator::operator++()
{
  if (tMaxX_ < tMaxY_)
  {
    unwrappedIndex_.x() += indexIncrementDirection_.x();
    tMaxX_ += tDeltaX_;
  }
  else if (tMaxY_ < tMaxX_)
  {
    unwrappedIndex_.y() += indexIncrementDirection_.y();
    tMaxY_ += tDeltaY_;
  }
  else
  {
    // Exact corner crossing.
    unwrappedIndex_.x() += indexIncrementDirection_.x();
    unwrappedIndex_.y() += indexIncrementDirection_.y();
    tMaxX_ += tDeltaX_;
    tMaxY_ += tDeltaY_;
  }
  // Convert the unwrapped/global index back to the actual buffer index.
  index_ =getBufferIndexFromIndex(unwrappedIndex_, bufferSize_, bufferStartIndex_);
  return *this;
}

bool AmanatidesWooGridLineIterator::isPastEnd() const
{
  const bool pastX = (indexIncrementDirection_.x() > 0 && unwrappedIndex_.x() > unwrappedEnd_.x()) || (indexIncrementDirection_.x() < 0 && unwrappedIndex_.x() < unwrappedEnd_.x());
  const bool pastY = (indexIncrementDirection_.y() > 0 && unwrappedIndex_.y() > unwrappedEnd_.y()) || (indexIncrementDirection_.y() < 0 && unwrappedIndex_.y() < unwrappedEnd_.y());
  return pastX || pastY;}

bool AmanatidesWooGridLineIterator::initialize(
  const grid_map::GridMap & gridMap, const Index & start,
  const Index & end)
{
  start_ = start;
  end_ = end;
  mapLength_ = gridMap.getLength();
  mapPosition_ = gridMap.getPosition();
  resolution_ = gridMap.getResolution();
  bufferSize_ = gridMap.getSize();
  bufferStartIndex_ = gridMap.getStartIndex();
  initializeIterationParameters();
  return true;
}

bool AmanatidesWooGridLineIterator::getIndexLimitedToMapRange(
  const grid_map::GridMap & gridMap,
  const Position & start, const Position & end,
  Index & index)
{
  const double eps = 1e-9;
  Vector direction = (end - start).normalized();
  Position newStart = start + eps * direction;
  while (!gridMap.getIndex(newStart, index)) {
    newStart += (gridMap.getResolution() - std::numeric_limits<double>::epsilon()) * direction;
    if ((end - newStart).norm() <
      gridMap.getResolution() - std::numeric_limits<double>::epsilon())
    {
      return false;
    }
  }
  return true;
}

void AmanatidesWooGridLineIterator::initializeIterationParameters()
{
  unwrappedStart_ = getIndexFromBufferIndex(start_, bufferSize_, bufferStartIndex_);
  unwrappedEnd_ = getIndexFromBufferIndex(end_, bufferSize_, bufferStartIndex_);
  unwrappedIndex_ = unwrappedStart_;

  index_ = start_;

  // Extract the grid index increment directions
  const Index deltaIndex = unwrappedEnd_ - unwrappedStart_;
  if (deltaIndex.x() > 0)
  {
    indexIncrementDirection_.x() = 1.0;
  }
  else if (deltaIndex.x() < 0)
  {
    indexIncrementDirection_.x() = - 1.0;
  }
  else
  {
    indexIncrementDirection_.x() = 0.0;
  }
  if (deltaIndex.y() > 0)
  {
    indexIncrementDirection_.y() = 1.0;
  }
  else if (deltaIndex.y() < 0)
  {
    indexIncrementDirection_.y() = - 1.0;
  }
  else
  {
    indexIncrementDirection_.y() = 0.0;
  }

  // Calculate in grid indices the normalised step to reach the next cell
  tDeltaX_ = deltaIndex.x() != 0
            ? 0.5 / std::abs(static_cast<double>(deltaIndex.x()))
            : std::numeric_limits<float>::infinity();
  tDeltaY_ = deltaIndex.y() != 0
            ? 0.5 / std::abs(static_cast<double>(deltaIndex.y()))
            : std::numeric_limits<float>::infinity();

  tMaxX_ = 0.5 * tDeltaX_;
  tMaxY_ = 0.5 * tDeltaY_;
}
}  // namespace grid_map
