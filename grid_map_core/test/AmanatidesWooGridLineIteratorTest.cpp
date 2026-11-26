/*
 * LineIteratorTest.cpp
 *
 *  Created on: Sep 14, 2016
 *      Author: Dominic Jud
 *	 Institute: ETH Zurich, ANYbotics
 */

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

#include "grid_map_core/iterators/AmanatidesWooGridLineIterator.hpp"
#include "grid_map_core/GridMap.hpp"


TEST(AmanatidesWooGridLineIterator, DiagonalLineTest)
{
// Line traversal from (2.0, 2.5) to (1.0, -3.5).
// S = start cell; E = end cell; * = other traversed cells.
// y-direction increases to the left 
// x-direction increases upwards
//
// Iterate over cells only when the line passes through their interior. 
// 
//             0   1   2   3   4   5   6   7
//           +---+---+---+---+---+---+---+---+
//         0 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         1 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         2 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         3 |   | S | * | * | * |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         4 |   |   |   |   | * | * | * | E |
//           +---+---+---+---+---+---+---+---+
//         5 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         6 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         7 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         8 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         9 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//        10 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(11.0, 8.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(2.0, 2.5), grid_map::Position(1.0, -3.5)));
  grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(2.0, 2.5), grid_map::Position(1.0, -3.5));
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(5, (*iterator)(1));

  ++iterator; 
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(6, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));  
  EXPECT_EQ(7, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(AmanatidesWooGridLineIterator, PurelyHorizontalLine)
{
// Line traversal from (-4.0, 0.5) to (4.0, 0.5).
// S = start cell; E = end cell; * = other traversed cells.
// y-direction increases to the left 
// x-direction increases upwards
// 
//             0   1   2   3   4   5   6   7
//           +---+---+---+---+---+---+---+---+
//         0 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         1 |   |   |   | E |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         2 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         3 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         4 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         5 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         6 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         7 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         8 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         9 |   |   |   | S |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//        10 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(11.0, 8.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(-4.0, 0.5), grid_map::Position(4.0, 0.5)));
  grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(-4.0, 0.5), grid_map::Position(4.0, 0.5));
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(9, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(8, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(7, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator; 
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(2, (*iterator)(0));  
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(1, (*iterator)(0));  
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(AmanatidesWooGridLineIterator, PurelyVerticalLine)
{
// Line traversal from (0.0, -1.5) to (0.0, 1.5).
// S = start cell; E = end cell; * = other traversed cells.
// y-direction increases to the left 
// x-direction increases upwards
//
//             0   1   2   3   4   5   6   7
//           +---+---+---+---+---+---+---+---+
//         0 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         1 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         2 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         3 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         4 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         5 |   |   | E | * | * | S |   |   |
//           +---+---+---+---+---+---+---+---+
//         6 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         7 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         8 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         9 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//        10 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(11.0, 8.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(0.0, -1.5), grid_map::Position(0.0, 1.5)));
  grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(0.0, -1.5), grid_map::Position(0.0, 1.5));
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(5, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(AmanatidesWooGridLineIterator, TraversalLineThroughCorners)
{
// Line traversal from (1.5, 1.5) to (-1.5, -1.5).
// S = start cell; E = end cell; * = other traversed cells.
// y-direction increases to the left 
// x-direction increases upwards
// 
// Cells touched only at the corners are not iterated over.
// i.e. (4,2), (3,3), (5,3), (4,4), (6,4), (5,5) are skipped
//
//             0   1   2   3   4   5   6   7
//           +---+---+---+---+---+---+---+---+
//         0 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         1 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         2 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         3 |   |   | S |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         4 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         5 |   |   |   |   | * |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         6 |   |   |   |   |   | E |   |   |
//           +---+---+---+---+---+---+---+---+
//         7 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         8 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         9 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(10.0, 8.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(1.5, 1.5), grid_map::Position(-1.5, -1.5)));
  grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(1.5, 1.5), grid_map::Position(-1.5, -1.5));
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(5, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(AmanatidesWooGridLineIterator, TraversalLineStartsAndEndsInTheBoundaryBetweenMultipleCells)
{
// Line traversal from (2.0, 2.0) to (-2.0, -2.0).
// S = start cell; E = end cell; * = other traversed cells.
// y-direction increases to the left 
// x-direction increases upwards
// 
//  Both endpoints lie on corners:
// - Start (2, 2): upper-left corner of cell (3,2).
// - End (-2,-2): lower-right corner of cell (6,5).
//
// Cells touched only at the corners or boundaries are not iterated over.
// Cells that touch the start point, i.e. (2,1), (3,1) and (2,2) are skipped.
// Cells that touch the end point, i.e. (6,6), (7,6), (7,5) are skipped
//
//             0   1   2   3   4   5   6   7
//           +---+---+---+---+---+---+---+---+
//         0 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         1 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         2 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         3 |   |   | S |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         4 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         5 |   |   |   |   | * |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         6 |   |   |   |   |   | E |   |   |
//           +---+---+---+---+---+---+---+---+
//         7 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         8 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         9 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//

  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(10.0, 8.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(2.0, 2.0), grid_map::Position(-2.0, -2.0)));
  grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(2.0, 2.0), grid_map::Position(-2.0, -2.0));
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(5, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(AmanatidesWooGridLineIterator, TraversalLineStartsAndEndsInTheBoundaryBetweenTwoCells)
{
// Line traversal from (1.5, 2.0) to (-1.5, -2.0).
// S = start cell; E = end cell; * = other traversed cells.
// y-direction increases to the left 
// x-direction increases upwards
// 
//  Both endpoints lie on corners:
// - Start (1.5, 2): upper-left corner of cell (3,2).
// - End (-1.5,-2): lower-right corner of cell (6,5).
//
// Cells touched only at the corners or boundaries are not iterated over.
// Cell that touches the start point, i.e. (3,1) is skipped.
// Cell that touches the end point, i.e. (6,6) is skipped
//
//             0   1   2   3   4   5   6   7
//           +---+---+---+---+---+---+---+---+
//         0 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         1 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         2 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         3 |   |   | S |   |    |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         4 |   |   |   | * |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         5 |   |   |   |   | * |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         6 |   |   |   |   |   | E |   |   |
//           +---+---+---+---+---+---+---+---+
//         7 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         8 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//         9 |   |   |   |   |   |   |   |   |
//           +---+---+---+---+---+---+---+---+
//

  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(10.0, 8.0), 1.0, grid_map::Position(0.0, 0.0));

  EXPECT_NO_THROW(grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(1.5, 2.0), grid_map::Position(-1.5, -2.0)));
  grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(1.5, 2.0), grid_map::Position(-1.5, -2.0));
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(5, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(6, (*iterator)(0));
  EXPECT_EQ(5, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}

TEST(AmanatidesWooGridLineIterator, DiagonalLineTestWithDifferentResolution)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(5.5, 4.0), 0.5, grid_map::Position(0.0, 0.0));


  EXPECT_NO_THROW(grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(1.0, 1.25), grid_map::Position(0.5, -1.75)));
  grid_map::AmanatidesWooGridLineIterator iterator(map, grid_map::Position(1.0, 1.25), grid_map::Position(0.5, -1.75));
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(1, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(2, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(3, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(3, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(4, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(5, (*iterator)(1));

  ++iterator; 
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));
  EXPECT_EQ(6, (*iterator)(1));

  ++iterator;
  EXPECT_FALSE(iterator.isPastEnd());
  EXPECT_EQ(4, (*iterator)(0));  
  EXPECT_EQ(7, (*iterator)(1));

  ++iterator;
  EXPECT_TRUE(iterator.isPastEnd());
}
