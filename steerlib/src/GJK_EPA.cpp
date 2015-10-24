/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}


/**
 *
 *
 *
 */
bool SteerLib::GJK_EPA::GJK(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex) {

	Util::Vector centerA = shape_center(_shapeA);
	Util::Vector centerB = shape_center(_shapeB);

	Util::Vector direction = centerB - centerA;

	Util::Vector A = support(_shapeA, direction) - support(_shapeB, -direction);
	_simplex.push_back(A);

	direction = -A;

	while(true) {
		A = support(_shapeA, direction) - support(_shapeB, -direction);

		if(dot(A, direction) < 0) {
			return false;
		}
		_simplex.push_back(A);

		if(nearest_symplex(_simplex, direction)) {

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
bool SteerLib::GJK_EPA::nearest_symplex(std::vector<Util::Vector>& simplex, Util::Vector& direction) {

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
	float furthestDot = dot(furthest, _direction);

	//go through each of the points and test if the dot product with _direction is higher
	for(int i = 0; i < numPoints; i++) {
		float tempDot = dot(shape[i], _direction);
		if(tempDot > furthestDot) {
			//if new dot product is higher than furthestDot, set furthest to current point and furthestDot to correspond to that point's dot product
			furthest = shape[i];
			furthestDot = tempDot;
		}
	}

	return furthest;
}


/**
 * Name: dot
 * Parameters: Util::Vector _vect1, Util::Vector _vect2
 * Description: Computes the dot product of two vectors _vect1 and _vect2
 * Returns: float result of dot product
 */
float SteerLib::GJK_EPA::dot(Util::Vector _vect1, Util::Vector _vect2) {
	//(x3 = x1 * x2, y3 = y1 * y2, z3 = z1 * z2)
	Util::Vector dotVect = _vect1 * _vect2;
	float sum  = 0.0f;

	//0 + x3 + y3 + z3
	for(int i = 0; i < 3; i++) {
		sum += dotVect[i];
	}
	return sum;
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



//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	//Collision simplex
	std::vector<Util::Vector> simplex;

	if(GJK(_shapeA, _shapeB, simplex)) {
		//Use collision simplex to compute penetration depth and vector for direction
		EPA(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& _simplex, float& return_penetration_depth, Util::Vector& return_penetration_vector);

		//There's a collision, so must return true regardless of penetration_depth/vector
		return true;
	}

	// There is no collision
    return false;
}
