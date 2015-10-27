    /*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"
#include <limits>

SteerLib::GJK_EPA::GJK_EPA()
{
}


/**
 * Name: GJK
 * Parameters: std::vector<Util::Vector>& _shapeA, std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex
 * Description: Main GJK algorithm logic. Given two shapes _shapeA and _shapeB, determines whether a collision occurs, determines points of collision if any and populates them into _simplex
 * Returns: bool result of collision checking
 */
bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex) {

    // The choice of initial direction matters. Making it centerB - centerA leads to a degenerate case
    // for GJK where the simplex ends up touching the origin.
    Util::Vector direction = Util::Vector(1, 0, 0);

	Util::Vector A = support(_shapeA, direction) - support(_shapeB, -direction);
	_simplex.push_back(A);

	direction = -A;
	while(true) {
		A = support(_shapeA, direction) - support(_shapeB, -direction);

		//means there cannot be a collision because the furthest point is not even past the origin
		if(dot(A, direction) < 0) {
			return false;
		}

		//add new point to simplex
		_simplex.push_back(A);

		if(nearest_symplex(_simplex, direction)) {
			return true;
		}
	}
}


/**
 * Name: nearest_symplex
 * Parameters: std::vector<Util::Vector>& _simplex, Util::Vector& direction
 * Description: Finds the simplex closest to the origin, associates new direction vector corresponding to new simplex, and determines whether symplex contains origin
 * Returns: bool result of checking symplex includes origin
 *
 */
bool SteerLib::GJK_EPA::nearest_symplex(std::vector<Util::Vector>& _simplex, Util::Vector& _direction) {
	//which part of the simplex is closest to origin
	//most recently added point in simplex, will be closest to origin among points in simplex
	Util::Vector A = _simplex.back();
	Util::Vector A0 = -A;

	//Since implementing GJK on 2D space, only need to work with 3 points for check of encompassing origin
    if (_simplex.size() == 3) {
        //Get other points  
        Util::Vector C = _simplex[0];
        Util::Vector B = _simplex[1];

        //Edges to check
        Util::Vector AB = B - A;
        Util::Vector AC = C - A;

        Util::Vector BC = C - B; 
        Util::Vector BA = A - B;

        //(AC x AB x AB)
        Util::Vector AB_perp = cross(cross(AC, AB), AB);
        //(AB x AC x AC)
        Util::Vector AC_perp = cross(cross(AB, AC), AC);
        Util::Vector BC_perp = cross(cross(BA, BC), BC);

        if (dot(AB_perp, A0) > 0) {
            //if the origin beyond AB, remove point C
            _simplex.erase(_simplex.begin());

            //set new direction
            _direction = AB_perp;

        }
        else if (dot(AC_perp, A0) > 0) {
            //if the origin beyond AC, remove point B
            _simplex.erase(_simplex.begin() + 1);
            //set new direction
            _direction = AC_perp;

        }
        else if (dot(BC_perp, -B) > 0) {
            // Remove point A
            _simplex.erase(_simplex.begin() + 2);
            _direction = BC_perp;
        }
        else {
            //origin is within or touching simplex, collision
            return true;
        }
    }
	else {
		//there are only 2 points in the symplex

		Util::Vector B = _simplex.back();

		Util::Vector AB = B - A;
		
		//get the perpendicular to AB in direction of origin
		//Triple product evaluation (AB x A0 x AB)
        Util::Vector AB_perp = cross(cross(AB, A0), AB);
		//set new direction
		_direction = AB_perp;
	}

	return false;
}


/**
 * Name: support
 * Parameters: std::vector<Util::Vector> _shape, Util::Vector _direction
 * Description: Find the point on a given shape _shape that has the highest dot product with given direction _direction.
 * 		In other words, the farthest point on the shape in the direction given.
 * Returns: Util::Vector representing farthest point on shape in direction of _direction
 */
Util::Vector SteerLib::GJK_EPA::support(const std::vector<Util::Vector> _shape, Util::Vector _direction) {
	int numPoints = _shape.size();
	Util::Vector furthest;
    float furthestDot = std::numeric_limits<float>::lowest();

	//go through each of the points and test if the dot product with _direction is higher
	for(int i = 0; i < numPoints; i++) {
		float tempDot = dot(_shape[i], _direction);
		if(tempDot > furthestDot) {
			//if new dot product is higher than furthestDot, set furthest to current point and furthestDot to correspond to that point's dot product
			furthest = _shape[i];
			furthestDot = tempDot;
		}
	}

	return furthest;
}


/**
 * Name: shape_center
 * Parameters: std::vector<Util::Vector> _shape
 * Description: Finds the center of the shape _shape by averaging all points that lie in the shape
 * Returns: Util::Vector representing center of _shape
 */
Util::Vector SteerLib::GJK_EPA::shape_center(const std::vector<Util::Vector>& _shape) {
	Util::Vector origin;

	int numPoints = _shape.size();

	//average the points in the shape to get the center
	for(int i = 0; i < numPoints; i++) {
		//Since only doing 2D GJK, get the projection of each vector onto the x-z plane
		origin += _shape[i];
		origin[1] = 0;
	}
	origin /= (float)numPoints;

	return origin;
}

void SteerLib::GJK_EPA::EPA(
    const std::vector<Util::Vector>& shapeA,
    const std::vector<Util::Vector>& shapeB,
    std::vector<Util::Vector>& simplex,
    float& penetration_depth,
    Util::Vector& penetration_vector) {
    for (int i = 0; i < shapeA.size() + shapeB.size(); i++) {
        Util::Vector normal;
        int indexA, indexB;

        // Get the closest edge, along with it's normal and distance to origin along normal.
        float distance = findClosestEdge(simplex, indexA, indexB, normal);
        Util::Vector A = simplex[indexA];
        Util::Vector B = simplex[indexB];
        Util::Vector p = support(shapeA, normal) - support(shapeB, -normal);
        double supportDistance = p * normal;

        // Check if the distance to the support point along the normal isn't much more
        // then the distance to the closest edge. If so, then we're close enough to
        // the closest penetration distance.
        if (supportDistance - distance < .001) {
            penetration_vector = normal;
            penetration_depth = supportDistance;
            return;
        }
        else {
            simplex.insert(simplex.begin() + indexB, p);
        }
    }

}

float SteerLib::GJK_EPA::findClosestEdge(const std::vector<Util::Vector>& simplex, int& indexA, int& indexB, Util::Vector& normal) {
    float closestDistance = std::numeric_limits<float>::max();
    for (int i = 0; i < simplex.size(); i++) {
        int j = i + 1 == simplex.size() ? 0 : i + 1;
        Util::Vector a = simplex[i];
        Util::Vector b = simplex[j];
        // The edge is defined by the points a -> b
        Util::Vector edge = b - a;
        
        // Get the normal vector from the edge to the origin.
        Util::Vector n = normalize(cross(cross(edge, a), edge));

        // Now project a (or b) onto n to get the distance
        double distance = n * a;
        if (distance < closestDistance) {
            closestDistance = distance;
            indexA = i;
            indexB = j;
            normal = n;
        }
    }
    return closestDistance;
}


//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	//Collision simplex
	std::vector<Util::Vector> simplex;

	if(GJK(_shapeA, _shapeB, simplex)) {
		//Use collision simplex to compute penetration depth and vector for direction

        // Uncomment EPA to test. It works on polygons_test.xml, but not on polygons1.xml. I made a subset
        // of polygons1.xml named polygons_degenerate.xml which contains only two polygons which cause
        // an infinite loop.
        //
        // Using a tempSimplex with points actually covering the origin seems to work, leading me to believe the issue is in GJK

        EPA(_shapeA, _shapeB, simplex, return_penetration_depth, return_penetration_vector);
		

		//There's a collision, so must return true regardless of penetration_depth/vector
		return true;
	}

	// There is no collision
    return false;
}
