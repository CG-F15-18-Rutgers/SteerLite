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
#include <limits>
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

		int goalIndex = gSpatialDatabase->getCellIndexFromLocation(goal);

		std::vector<SearchNode> open;
		std::vector<SearchNode> closed;
		SearchNode* goalNode = nullptr;
		open.push_back(startNode);
		while (!open.empty()) {

			// Get the node with the minimum f.
			std::vector<SearchNode>::iterator minIter;
			float minF = std::numeric_limits<float>::max();
			for (std::vector<SearchNode>::iterator iter = open.begin(); iter != open.end(); iter++) {
				if (iter->f() < minF) {
					minF = iter->f();
					minIter = iter;
				}
			}

			SearchNode minNode = *minIter;
			open.erase(minIter);

			if (minNode.index() == goalIndex) {
				std::cout << "Visiting goal \n";
				goalNode = &minNode;
				break;
			}

			// Expand this node.
			std::cout << "Expanding node \n";
			minNode.printDebug();
			std::vector<SearchNode> expanded = _expand(minNode, goal);
			for (SearchNode& node : expanded) {
				// If this cell is already in the open set, check if this is a cheaper path.
				std::vector<SearchNode>::iterator iter = std::find(open.begin(), open.end(), node);
				if (iter != open.end()) {
					std::cout << "Node is in the open set\n";
					if (node.g() < iter->g()) {
						iter->g(node.g());
						iter->prev(&minNode);
						assert(node.index() != minNode.index());
					}
				} else {
					if (std::find(closed.begin(), closed.end(), node) == closed.end()) {
						node.prev(&minNode);
						assert(node.index() != minNode.index());
						open.push_back(node);
					}
				}
			}
			closed.push_back(minNode);
		}

		if (goalNode == nullptr) {
			return false;
		}

		// Go back from goal node to start.
		SearchNode* ptr = goalNode;
		while (ptr != nullptr) {
			Util::Point pt;
			gSpatialDatabase->getLocationFromIndex(ptr->index(), pt);
			agent_path.insert(agent_path.begin(), pt);
			ptr->printDebug();
			ptr = ptr->prev();
		}

		// TODO: fix the previous links.
		return true;
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