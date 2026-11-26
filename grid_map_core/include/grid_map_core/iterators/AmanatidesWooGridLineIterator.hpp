/*
 * Copyright (c) 2026 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 * All rights reserved.
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential.
 *
 * Name:       Amanatides-Woo Grid Line Iterator
 * Author:     Natalia Melo Fernandez
 * Date:       26.11.2026
 * Abstract:   Implements the Amanatides-Woo line iterator
 */

#ifndef GRID_MAP_CORE__ITERATORS__AMANATIDESWOOGRIDLINEITERATOR_HPP_
#define GRID_MAP_CORE__ITERATORS__AMANATIDESWOOGRIDLINEITERATOR_HPP_

#include <Eigen/Core>

#include "grid_map_core/GridMap.hpp"

namespace grid_map
{

/*!
 * Iterator class to iterate over all cells a line passes through, skipping cells touched only at a corner or boundary
 * Based on Amanatides-Woo line iterator algorithms.
 */
class AmanatidesWooGridLineIterator
{
public:
  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting point of the line.
   * @param end the ending point of the line.
   * @throw std::invalid_argument if start and end impose an ill conditioned line iteration.
   */
  AmanatidesWooGridLineIterator(const grid_map::GridMap & gridMap, const Position & start, const Position & end);

  /*!
   * Constructor.
   * @param gridMap the grid map to iterate on.
   * @param start the starting index of the line.
   * @param end the ending index of the line.
   */
  AmanatidesWooGridLineIterator(const grid_map::GridMap & gridMap, const Index & start, const Index & end);

  /*!
   * Assignment operator.
   * @param iterator the iterator to copy data from.
   * @return a reference to *this.
   */
  AmanatidesWooGridLineIterator & operator=(const AmanatidesWooGridLineIterator & other);

  /*!
   * Compare to another iterator.
   * @return whether the current iterator points to a different address than the other one.
   */
  bool operator!=(const AmanatidesWooGridLineIterator & other) const;

  /*!
   * Dereference the iterator with const.
   * @return the value to which the iterator is pointing.
   */
  const Index & operator*() const;

  /*!
   * Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  AmanatidesWooGridLineIterator & operator++();

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
  double tDeltaX_;
  double tDeltaY_;
  double tMaxX_;
  double tMaxY_;

  //! Map information needed to get position from iterator.
  Length mapLength_;
  Position mapPosition_;
  double resolution_;
  Size bufferSize_;
  Index bufferStartIndex_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace grid_map
#endif  // GRID_MAP_CORE__ITERATORS__AMANATIDESWOOGRIDLINEITERATOR_HPP_
