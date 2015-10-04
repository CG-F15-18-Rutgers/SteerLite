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
	float startTime = controlPoints.front().time;
	float endTime = controlPoints.back().time;
	float diff = endTime - startTime;
	unsigned int previousPointIndex = 0;
	unsigned int currentPointIndex = 0;

	assert(diff > 0);

	for (int i = 0; i < window; i++) {
		// The ratio goes from 0 (starting point) and to 1 (final point).
		float ratio = (float)(i) / window;
		// Note: Renish mentioned something about the time being normalized for this method.
		// It isn't clear to me whether I should pass the exact time (as I am now) or
		// the normalized time to findTimeInterval. Regardless, it should be easy to switch.
		findTimeInterval(currentPointIndex, ratio * diff + startTime);
		Point from = controlPoints[previousPointIndex].position;
		Point to = controlPoints[currentPointIndex].position;
		DrawLib::drawLine(from, to, curveColor);
		previousPointIndex = currentPointIndex;
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
    if (controlPoints.size() < 2 || type != 0 && type != 1)
        return false

    //make sure each control point is in ascending order
    if (controlPoints[0].time < 0)
        return false

    for (int i = 1; i < controlPoints.size() - 1; i++){
        if (controlPoints[i - 1].time > controlPoints[i].time)
            return false 
        if (controlPoints[i].time < 0)
            return false
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
    return true
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function findTimeInterval is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
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
            nextPoint2 = nextPoint;
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
        float s_x1 = s1.x;
        float s_x2 = s2.x;

        float s_y1 = s1.y;
        float s_y2 = s2.y;

        float s_z1 = s1.z;
        float s_z2 = s2.z;
   	
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





// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
<<<<<<< HEAD
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
        float t0;
        float x0;
        float y0;
        float z0;

        float t3;
        float x3;  
        float y3;
        float z3;

        // Catmull-Rom tangent components
        if (previousPoint == 0) {
            t3 = controlPoints[nextPoint2 + 1].time;
            x3 = controlPoints[nextPoint2 + 1].position.x;
            y3 = controlPoints[nextPoint2 + 1].position.y;
            z3 = controlPoints[nextPoint2 + 1].position.z;

            s_x1 = ((t3-t1)/(t3-t2))*((x2-x1)/(t2-t1)) - ((t2-t1)/(t3-t2))*((x3-x1)/(t3-t1));
            s_x2 = ((t2 - t1)/(t3 - t1))*((x3 - x2)/(t3 - t2)) + ((t3 - t2)/(t3 - t1))*((x2 - x1)/(t2 - t1));

            s_y1 = ((t3-t1)/(t3-t2))*((y2-y1)/(t2-t1)) - ((t2-t1)/(t3-t2))*((y3-y1)/(t3-t1));
            s_y2 = ((t2 - t1)/(t3 - t1))*((y3 - y2)/(t3 - t2)) + ((t3 - t2)/(t3 - t1))*((y2 - y1)/(t2 - t1));

            s_z1 = ((t3-t1)/(t3-t2))*((z2-z1)/(t2-t1)) - ((t2-t1)/(t3-t2))*((z3-z1)/(t3-t1));
            s_z2 = ((t2 - t1)/(t3 - t1))*((z3 - z2)/(t3 - t2)) + ((t3 - t2)/(t3 - t1))*((z2 - z1)/(t2 - t1));
            
        }
        else if (nextPoint2 == controlPoints.size() - 1) {
            t0 = controlPoints[previousPoint - 1].time;
            x0 = controlPoints[previousPoint-1].position.x;
            y0 = controlPoints[previousPoint-1].position.y;
            z0 = controlPoints[previousPoint-1].position.z;

            s_x1 = ((t1-t0)/(t2-t0))*((x2-x1)/(t2-t1)) + ((t2-t1)/(t2-t0))*((x1-x0)/(t1-t0));
            s_x2 = ((t2-t0)/(t2-t1))*((x1-x0)/(t1-t0)) - ((t1-t0)/(t2-t1))*((x2-x0)/(t2-t0));
 

            s_y1 = ((t1-t0)/(t2-t0))*((y2-y1)/(t2-t1)) + ((t2-t1)/(t2-t0))*((y1-y0)/(t1-t0));
            s_y2 = ((t2-t0)/(t2-t1))*((y1-y0)/(t1-t0)) - ((t1-t0)/(t2-t1))*((y2-y0)/(t2-t0));


            s_z1 = ((t1-t0)/(t2-t0))*((z2-z1)/(t2-t1)) + ((t2-t1)/(t2-t0))*((z1-z0)/(t1-t0));
            s_z2 = ((t2-t0)/(t2-t1))*((z1-z0)/(t1-t0)) - ((t1-t0)/(t2-t1))*((z2-z0)/(t2-t0));

        }
        else {
            t0 = controlPoints[previousPoint - 1].time;
            x0 = controlPoints[previousPoint-1].position.x;
            y0 = controlPoints[previousPoint-1].position.y;
            z0 = controlPoints[previousPoint-1].position.z;
           

            t3 = controlPoints[nextPoint2 + 1].time;
            x3 = controlPoints[nextPoint2 + 1].position.x;
            y3 = controlPoints[nextPoint2 + 1].position.y;
            z3 = controlPoints[nextPoint2 + 1].position.z;

            s_x1 = ((t1-t0)/(t2-t0))*((x2-x1)/(t2-t1)) + ((t2-t1)/(t2-t0))*((x1-x0)/(t1-t0));
            s_x2 = ((t2 - t1)/(t3 - t1))*((x3 - x2)/(t3 - t2)) + ((t3 - t2)/(t3 - t1))*((x2 - x1)/(t2 - t1));

            s_y1 = ((t1-t0)/(t2-t0))*((y2-y1)/(t2-t1)) + ((t2-t1)/(t2-t0))*((y1-y0)/(t1-t0));
            s_y2 = ((t2 - t1)/(t3 - t1))*((y3 - y2)/(t3 - t2)) + ((t3 - t2)/(t3 - t1))*((y2 - y1)/(t2 - t1));
 
            s_z1 = ((t1-t0)/(t2-t0))*((z2-z1)/(t2-t1)) + ((t2-t1)/(t2-t0))*((z1-z0)/(t1-t0));
            s_z2 = ((t2 - t1)/(t3 - t1))*((z3 - z2)/(t3 - t2)) + ((t3 - t2)/(t3 - t1))*((z2 - z1)/(t2 - t1));
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
=======

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function useCatmullCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================


	// Calculate time interval, and normal time required for later curve calculations

	// Calculate position at t = time on Catmull-Rom curve
	
	// Return result
	return newPosition;
>>>>>>> 1ee022bf669ad1ff9aa3f4e170939687170e07e3
}
