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

#include "grid_map_core/iterators/TraversalGridLineIterator.hpp"
#include "grid_map_core/GridMap.hpp"


TEST(TraversalGridLineIterator, LineTest1)
{
  grid_map::GridMap map({"types"});
  map.setGeometry(grid_map::Length(11.0, 8.0), 1.0, grid_map::Position(0.0, 0.0));


  EXPECT_NO_THROW(grid_map::TraversalGridLineIterator iterator(map, grid_map::Position(2.0, 2.5), grid_map::Position(1.0, -3.5)));
  grid_map::TraversalGridLineIterator iterator(map, grid_map::Position(2.0, 2.5), grid_map::Position(1.0, -3.5));
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