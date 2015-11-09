//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <array>
#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::GridDatabase2D * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		// As of now, I've implemented an expand method which returns a list of traversable neighbors,
		// and computes the g, h, and f values.

		// The following example prints out the neighbors of the start node.
		int startIndex = gSpatialDatabase->getCellIndexFromLocation(start);
		SearchNode startNode(startIndex, 0, 0);
		std::vector<SearchNode> neighbors = _expand(startNode, goal);
		std::cout << "Neighbors to start node are as follows:\n";
		for (const SearchNode& node: neighbors) {
			node.printDebug();
		}

		// TODO: The next step is to make open and closed sets, initialize the open set with 
		// startNode then iteratively expand, choosing to expand on the node in the open set with
		// lowest f value.
		return false;
	}

	// Helper method which attempts to add a SearchNode to the output vector if it is traversable.
	void AStarPlanner::_tryToAdd(unsigned int x, unsigned int z, const SearchNode& from, float cost, Util::Point goal, std::vector<SearchNode>& out) {
		int index = gSpatialDatabase->getCellIndexFromGridCoords(x, z);
		if (!canBeTraversed(index)) return;
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(index, p);
		float h = distanceBetween(p, goal);
		out.push_back(SearchNode(index, from.g() + cost, h));
	}

	// Returns a list of neighboring traversable cells.
	std::vector<SearchNode> AStarPlanner::_expand(const SearchNode& node, const Util::Point goal) {
		unsigned int x, z;
		int index;
		std::vector<SearchNode> out;
		gSpatialDatabase->getGridCoordinatesFromIndex(node.index(), x, z);

		// Try to add the four cardinal directions.
		_tryToAdd(x - 1, z, node, 1, goal, out);
		_tryToAdd(x + 1, z, node, 1, goal, out);
		_tryToAdd(x, z - 1, node, 1, goal, out);
		_tryToAdd(x, z + 1, node, 1, goal, out);

		// Try the diagonals, with cost approximately sqrt(2).
		_tryToAdd(x - 1, z - 1, node, 1.414f, goal, out);
		_tryToAdd(x - 1, z + 1, node, 1.414f, goal, out);
		_tryToAdd(x + 1, z - 1, node, 1.414f, goal, out);
		_tryToAdd(x + 1, z + 1, node, 1.414f, goal, out);

		return std::move(out);
	}
}