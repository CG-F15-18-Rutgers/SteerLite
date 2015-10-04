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
            float time = min(c1.time, c0.time + timeDifference * ratio);
			Point curvePoint; 
			calculatePoint(curvePoint, time);
			DrawLib::drawLine(previousPoint, curvePoint, curveColor);
			previousPoint = curvePoint;
		}
	}

	
#endif
}

bool sortHelper(CurvePoint i, CurvePoint j){

    return (i.time <= j.time) ? true : false;
}
// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
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
        return false;

    //make sure each control point is in ascending order
    if (controlPoints[0].time < 0)
        return false;

    for (int i = 1; i < controlPoints.size() - 1; i++) {
        if (controlPoints[i - 1].time > controlPoints[i].time)
            return false;
        if (controlPoints[i].time < 0)
            return false;
    }

	if (controlPoints.size() < 2 || type != 0 && type != 1)
        return false;

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
    //Iterate through the control points
	for(int i=0; i < controlPoints.size(); i++) {
        //Check to see if any of the control points are beyond current time.
        //Already sorted by the point of calling function so just have to find first point
        if(controlPoints[i].time > time) {
            //Found a point
            nextPoint = i;
            return true;
        }
    }

    //Could not find any points to pursue
    return false;

}

// Given two CurvePoints (including tangents) uses the hermite formula to compute the interpolation.
// Note: this is called by the catmull rom implementation since catmull rom just precomputes the tangents.
Point hermiteInterpolateInternal(const CurvePoint& c1, const CurvePoint& c2, float time) {
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
        return Point();
    }

    // Time values
    float interval = t2 - t1;
    float itSq = interval*interval;
    float itCub = itSq*interval;

    float t_ti = time - t1;
    float t_tiSq = t_ti*t_ti;
    float t_tiCub = t_tiSq*t_ti;

    // Basis functions
    float f1 = 2 * (t_tiCub / itCub) - 3 * (t_tiSq / itSq) + 1;
    float f2 = -2 * (t_tiCub / itCub) + 3 * (t_tiSq / itSq);
    float f3 = (t_tiCub / itSq) - 2 * (t_tiSq / interval) + t_ti;
    float f4 = (t_tiCub / itSq) - (t_tiSq / interval);

    // Points can be multiplied by scalars, they also play well with Vectors.
    return f1 * p1 + f2 * p2 + f3 * s1 + f4 * s2;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	assert(nextPoint > 0);

	CurvePoint c1 = controlPoints[nextPoint - 1];
	CurvePoint c2 = controlPoints[nextPoint];

    return hermiteInterpolateInternal(c1, c2, time);
}


// Given a point index, it returns the tangent as computed by the catmull rom formulas
Vector Curve::getCatmullTangent(const unsigned int pointIndex) {
    // The boundaries are special cases.
    if (pointIndex == 0) {
        CurvePoint c0 = controlPoints[0];
        CurvePoint c1 = controlPoints[1];
        CurvePoint c2 = controlPoints[2];
        float t0 = c0.time;
        float t1 = c1.time;
        float t2 = c2.time;
        Point y0 = c0.position;
        Point y1 = c1.position;
        Point y2 = c2.position;
        return (t2 - t0) / (t2 - t1) * (y1 - y0) / (t1 - t0) - (t1 - t0) / (t2 - t1) * (y2 - y0) / (t2 - t0);
    }
    else if (pointIndex == controlPoints.size() - 1) {
        // m and p stand for minus and plus.
        CurvePoint c_im2 = controlPoints[pointIndex - 2];
        CurvePoint c_im1 = controlPoints[pointIndex - 1];
        CurvePoint c_i = controlPoints[pointIndex];
        float t_im2 = c_im2.time;
        float t_im1 = c_im1.time;
        float t_i = c_i.time;
        Point y_im2 = c_im2.position;
        Point y_im1 = c_im1.position;
        Point y_i = c_i.position;
        return ((t_im1 - t_im2) / (t_i - t_im2))*((y_i - y_im1) / (t_i - t_im1)) + ((t_i - t_im1) / (t_i - t_im2))*((y_im1 - y_im2) / (t_im1 - t_im2));
    }
    else {
        CurvePoint c_im1 = controlPoints[pointIndex-1];
        CurvePoint c_i = controlPoints[pointIndex];
        CurvePoint c_ip1 = controlPoints[pointIndex+1];
        float t_im1 = c_im1.time;
        float t_i = c_i.time;
        float t_ip1 = c_ip1.time;
        Point y_im1 = c_im1.position;
        Point y_i = c_i.position;
        Point y_ip1 = c_ip1.position;
        return (t_i - t_im1) / (t_ip1 - t_im1) * (y_ip1 - y_i) / (t_ip1 - t_i) + (t_ip1 - t_i) / (t_ip1 - t_im1) * (y_i - y_im1) / (t_i - t_im1);
    }
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
    assert(nextPoint > 0);

    CurvePoint c0 = controlPoints[nextPoint - 1];
    CurvePoint c1 = controlPoints[nextPoint];

    c0.tangent = getCatmullTangent(nextPoint - 1);
    c1.tangent = getCatmullTangent(nextPoint);
    
    return hermiteInterpolateInternal(c0, c1, time);
}
