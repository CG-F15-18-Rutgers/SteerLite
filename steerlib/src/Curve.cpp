//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

#include <string>

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI
	// Robustness: make sure there is at least two control point: start and end points
	// Note: I'm using assertions because this should never happen. The robustCheck should
	// handle user input error.
	assert(controlPoints.size() >= 2);
	assert(window > 0);

	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	for (int i = 1; i < controlPoints.size(); i++) {
		CurvePoint c0 = controlPoints[i - 1];
		CurvePoint c1 = controlPoints[i];
		float timeDifference = c1.time - c0.time;
		Point previousPoint = c0.position;
		for (int step = 1; step <= window; step++) {
			// ratio ranges from 0 (starting point) and to 1 (final point).
			float ratio = (float)(step) / window;
			float time = c0.time + timeDifference * ratio;
			Point curvePoint; 
			calculatePoint(curvePoint, time);
			DrawLib::drawLine(previousPoint, curvePoint, curveColor);
			previousPoint = curvePoint;
		}
	}

	
#endif
}

bool sortHelper(CurvePoint i, CurvePoint j){

    return (i.time < j.time) ? true : false;
}
// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
    /*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function sortControlPoints is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================
    */
    std::sort (controlPoints.begin(),controlPoints.end(), sortHelper);
    return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
    //make sure type of curve is specified and there are at least two points
<<<<<<< HEAD
    if (controlPoints.size() < 2 || type != 0 && type != 1)
        return false;

    //make sure each control point is in ascending order
    if (controlPoints[0].time < 0)
        return false;

    for (int i = 1; i < controlPoints.size() - 1; i++){
        if (controlPoints[i - 1].time > controlPoints[i].time)
            return false; 
        if (controlPoints[i].time < 0)
            return false;
=======
	if (controlPoints.size() < 2 || type != 0 && type != 1)
		return false;
		
		//make sure each control point is in ascending order
	if (controlPoints[0].time < 0)
		return false;

    for (int i = 1; i < controlPoints.size() - 1; i++){
		if (controlPoints[i - 1].time > controlPoints[i].time)
			return false;
		if (controlPoints[i].time < 0)
			return false;
>>>>>>> 95415bc073eb48811e9729417aea73403f02b24c
    }
    /*
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function checkRobust is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

    
	return true;
    */
<<<<<<< HEAD
    return true;
=======
	return true;
>>>>>>> 95415bc073eb48811e9729417aea73403f02b24c
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
    //Iterate through the control points
	for(int i=0; i < controlPoints.size(); i++) {
        //Check to see if any of the control points are beyond current time.
        //Already sorted by the point of calling function so just have to find first point
        if(controlPoints[i].time >= time) {
            //Found a point
            nextPoint = i;
            return true;
        }
    }

    //Could not find any points to pursue
    return false;

}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	assert(nextPoint > 0);
	Point newPosition;

	CurvePoint c1 = controlPoints[nextPoint - 1];
	CurvePoint c2 = controlPoints[nextPoint];

    // Point Vectors
    Point p1 = c1.position;
    Point p2 = c2.position;

    // Tangent Vectors
    Vector s1 = c1.tangent;
    Vector s2 = c2.tangent;

    // Times
    float t1 = c1.time;
    float t2 = c2.time; 
        
    // If input time isn't in interval, return 0 vector
    if (!(time >= t1 && time <= t2)) {
        newPosition.x = 0;
        newPosition.y = 0;
        newPosition.z = 0;
        return newPosition; 
    }

    // Time values
    float interval = t2 - t1;      
    float itSq   = interval*interval;
    float itCub  = itSq*interval;

    float t_ti    = time - t1;
    float t_tiSq  = t_ti*t_ti; 
    float t_tiCub = t_tiSq*t_ti;

    // Basis functions
    float f1 = 2*(t_tiCub/itCub) - 3*(t_tiSq/itSq) + 1;
    float f2 = -2*(t_tiCub/itCub) + 3*(t_tiSq/itSq);
    float f3 = (t_tiCub/itSq) - 2*(t_tiSq/interval) + t_ti;
    float f4 = (t_tiCub/itSq) - (t_tiSq/interval); 

	// Points can be multiplied by scalars, they also play well with Vectors.
	Point value = f1 * p1 + f2 * p2 + f3 * s1 + f4 * s2;
	return value;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
        unsigned int previousPoint;
        unsigned int nextPoint2;

        // If nextPoint is 0, we look ahead in the array  
        if (nextPoint == 0) {
            previousPoint = 0;
            nextPoint2 = 1;
        }
        // Otherwise, we look behind in the array
        else {
            previousPoint = nextPoint - 1;
            nextPoint2 = 1;
        }

        // Point Vectors
        Point p1 = controlPoints[previousPoint].position;
        Point p2 = controlPoints[nextPoint2].position;

         // Tangent Vectors
        Vector s1 = controlPoints[previousPoint].tangent;
        Vector s2 = controlPoints[nextPoint2].tangent;

        // Times
        float t1 = controlPoints[previousPoint].time;
        float t2 = controlPoints[nextPoint2].time;

        // If input time isn't in interval, return 0 vector
        if (!(time >= t1 && time <= t2)) {
            newPosition.x = 0;
            newPosition.y = 0;
            newPosition.z = 0;
            return newPosition;
        }

        // Time values
        float interval = t2 - t1;
        float itSq   = interval*interval;
        float itCub  = itSq*interval;

        float t_ti    = time - t1;
        float t_tiSq  = t_ti*t_ti;
        float t_tiCub = t_tiSq*t_ti;

        // Basis functions
        float f1 = 2*(t_tiCub/itCub) - 3*(t_tiSq/itSq) + 1;
        float f2 = -2*(t_tiCub/itCub) + 3*(t_tiSq/itSq);
        float f3 = (t_tiCub/itSq) - 2*(t_tiSq/interval) + t_ti;
        float f4 = (t_tiCub/itSq) - (t_tiSq/interval);

        // Position components
        float x1 = p1.x;
        float x2 = p2.x;

        float y1 = p1.y;
        float y2 = p2.y;

        float z1 = p1.z;
        float z2 = p2.z;

        // Tangent components
        float s_x1;
        float s_x2;

        float s_y1;
        float s_y2;

        float s_z1;
        float s_z2;

        // Extra values for Catmull-Rom 
        float x0;
        float y0;
        float z0;

        float x3;  
        float y3;
        float z3;

        // Catmull-Rom tangent components
        if (previousPoint == 0) {
            x3 = controlPoints[nextPoint2 + 1].position.x;
            y3 = controlPoints[nextPoint2 + 1].position.y;
            z3 = controlPoints[nextPoint2 + 1].position.z;
  
            s_x1 = 2*(x2 - x1) - (x3 - x1)/2;
            s_x2 = (x3 - x1)/2;

            s_y1 = 2*(y2 - y1) - (y3 - y1)/2;
            s_y2 = (y3 - y1)/2;

            s_z1 = (z2 - z0)/2;
            s_z2 = 2*(z2 - z1) - (z3 - z1)/2;
               
        }
        else if (nextPoint2 == controlPoints.size() - 1) {
            x0 = controlPoints[previousPoint-1].position.x;
            y0 = controlPoints[previousPoint-1].position.y;
            z0 = controlPoints[previousPoint-1].position.z;

            s_x1 = (x2 - x0)/2;
            s_x2 = 2*(x2 - x1) - (x2 - x0)/2;

            s_y1 = (y2 - y0)/2;
            s_y2 = 2*(y2 - y1) - (y2 - y0)/2;

            s_z1 = (z2 - z0)/2;
            s_z2 = 2*(z2 - z1) - (z2 - z0)/2;
        }
        else {
            x0 = controlPoints[previousPoint-1].position.x;
            y0 = controlPoints[previousPoint-1].position.y;
            z0 = controlPoints[previousPoint-1].position.z;
           
            x3 = controlPoints[nextPoint2 + 1].position.x;
            y3 = controlPoints[nextPoint2 + 1].position.y;
            z3 = controlPoints[nextPoint2 + 1].position.z;
            
            s_x1 = (x2 - x0)/2;
            s_x2 = (x3 - x1)/2;

            s_y1 = (y2 - y0)/2;
            s_y2 = (y3 - y1)/2;
        
            s_z1 = (z2 - z0)/2;
            s_z2 = (z3 - z1)/2;
        }
      

        // Final Values
        float x_value = x1*f1 + x2*f2 + s_x1*f3 + s_x2*f4;
        float y_value = y1*f1 + y2*f2 + s_y1*f3 + s_y2*f4;
        float z_value = z1*f1 + z2*f2 + s_z1*f3 + s_z2*f4;

        // Initialize newPosition, rleturn
        newPosition.x = x_value;
        newPosition.y = y_value;
        newPosition.z = z_value;
        return newPosition;
}
