/*
 * LineIterator.hpp
 *
 *  Created on: Nov 13, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#ifndef GRID_MAP_CORE__ITERATORS__TRAVERSALGRIDLINEITERATOR_HPP_
#define GRID_MAP_CORE__ITERATORS__TRAVERSALGRIDLINEITERATOR_HPP_

#include <Eigen/Core>
#include <queue>

#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/iterators/SubmapIterator.hpp"

namespace grid_map
{

/*!
 * Iterator class to iterate over a line in the map.
 * Based on Bresenham Line Drawing algorithm.
 */
class TraversalGridLineIterator
{
public:
  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting point of the line.
   * @param end the ending point of the line.
   * @throw std::invalid_argument if start and end impose an ill conditioned line iteration.
   */
  TraversalGridLineIterator(const grid_map::GridMap & gridMap, const Position & start, const Position & end);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   */
  TraversalGridLineIterator(const grid_map::GridMap & gridMap, const Index & start, const Index & end);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  TraversalGridLineIterator & operator=(const TraversalGridLineIterator & other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator!=(const TraversalGridLineIterator & other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index & operator*() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  TraversalGridLineIterator & operator++();

  /*!
   * Indicates if iterator is past end.
   * @return true if iterator is out of scope, false if end has not been reached.
   */
  bool isPastEnd() const;

private:
  /*!
   * Construct function.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   * @return true if successful, false otherwise.
   */
  bool initialize(const grid_map::GridMap & gridMap, const Index & start, const Index & end);

  /*!
   * Computes the parameters requires for the line drawing algorithm.
   */
  void initializeIterationParameters();

  /*!
   * Finds the index of a position on a line within the limits of the map.
   * @param[in] gridMap the grid map that defines the map boundaries.
   * @param[in] start the position that will be limited to the map range.
   * @param[in] end the ending position of the line.
   * @param[out] index the index of the moved start position.
   * @return true if successful, false otherwise.
   */
  bool getIndexLimitedToMapRange(
    const grid_map::GridMap & gridMap, const Position & start,
    const Position & end, Index & index);

  //! Current index.
  Index index_;

  //! Starting index of the line.
  Index start_;

  //! Ending index of the line.
  Index end_;

  // ! Unwrapped indices
  Index unwrappedIndex_;
  Index unwrappedStart_;
  Index unwrappedEnd_;

  //! Helper variables for iterator.
  Size indexIncrementDirection_;
  Length incrementWorldCoordinates_;
  double tMaxX_;
  double tMaxY_;

  //! Map information needed to get position from iterator.
  Length mapLength_;
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;
  Index bufferStartIndex_;
  grid_map::Position startPosition_;
  grid_map::Position endPosition_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace grid_map
#endif  // GRID_MAP_CORE__ITERATORS__TRAVERSALGRIDLINEITERATOR_HPP_
