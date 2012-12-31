#include <iostream>
#include <math.h>
using namespace std;

class Point {
public:
	double x;
	double y;
	double theta;
	
	Point(void){
		x = 0;
		y = 0;
		theta = 0;
	}
	Point(double a, double b, double t){
		x = a;
		y = b;
		theta = t;
	}
	~Point(void){}
    
    void coutPoint(void){
        cout<<"("<<x<<","<<y<<","<<theta<<")"<<" ";
    }
    
    bool equal(Point p){
        if (x==p.x && y==p.y && theta==p.theta) {
            return true;
        } else {
            return false;
        }
    }
    
    double distTo(Point p){
        return sqrt(pow(x-p.x,2)+pow(y-p.y,2));
    }
};