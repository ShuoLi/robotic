/**
 *@file MotionPlanner.hpp
 *@brief Interface for the motion-planning algorithms
 */

#ifndef MOTION_PLANNER_HPP_
#define MOTION_PLANNER_HPP_

#define DEBUG 0							//Debug flag
#define CLOCKWISE 0						//State for turn mode to be CW
#define COUNTER_CLOCKWISE 1				//State for turn mode to be CCW
#define PI M_PI
#define PIHALF 1.5707964				//90 degrees in radian

#include "Simulator.hpp"
#include <iostream>
#include <Point.hpp>
#include <math.h>
#include <Utils.hpp>
#include <Graph.hpp>

using namespace std;


/**
 *@brief Interface for the motion-planning algorithms
 *
 *@remark
 *  Feel free to add additional functions/data members as you see fit
 */
class MotionPlanner
{
public:
	/**
     *@brief Set simulator
     *@param simulator pointer to simulator
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator
     */
	MotionPlanner(Simulator * const simulator);
    
    
	/**
     *@brief Free memory and delete objects allocated by this instance
     *
     *@remark
     *  Constructor performs only pointer assignment. Therefore,
     *  destructor should not delete the simulator.
     */
	~MotionPlanner(void);
    
	/**
     *@brief Run motion-planning algorithm for at most tmax seconds
     *
     *@param tmax upper bound on the computation time for the motion-planning algorithm
     *@remark
     *  The graphical interface will call this function and pass to it a certain
     *  value for the time upper bound. Your implementation (which will be in
     *  different classes that extend MotionPlanner, e.g., RRT, PRM) should
     *  not run for much longer than this upper bound.
     */
	void Solve(const double tmax);
    
	/**
     *@brief Get the highest-reward path computed by the motion planner so far
     *
     *@param path vector where you can store your path
     *@remark
     *  After running your motion-planning algorithm for tmax seconds, the graphical
     *  interface will call this function to obtain the best solution that your
     *  algorithm has found. The best solution is the collision-free path with
     *  the highest accumulated total reward.
     *@remark
     *  The path will consist of a sequence of configurations (orientations + positions), i.e.,
     *  <CENTER>(theta_1, x_1, y_1), (theta_2, x_2, y_2), ...., (theta_n, x_n, y_n)</CENTER>
     *  You can push these values into the vector by using the push_back function, i.e.,
     * <CENTER>
     *   path->push_back(theta_1);
     *   path->push_back(x_1);
     *   path->push_back(y_1);
     *   ...and so on
     * </CENTER>
     *  
     */
	void GetHighestRewardCollisionFreePath(std::vector<double> * path);
    
	/**
     *@brief Draw motion planner
     *@remark
     *  You can use this function to draw your motion planner. It may help you during
     *  debugging to figure out how the motion planner is performing.
     *  You can use the functions from Graphics to do the drawing of points,
     *  edges, and so on.
     */
	void Draw();
    
protected:
    
	/**
     *@brief Pointer to simulator, which provides access to robot, obstacles, reward regions
     */
    Simulator  *m_simulator;
	friend class Graphics; 
    
	//reward areas
	int nReward;// number of rewards
    Point *rewards;//array to store reward centers
    
    //robot
    Point robot;
    
    //sample Point
    Point sampleP;
	Point qNew;//temp test111
    Point center;
    Node* rt;
    
    //T graph
    Graph T;
    double step_size;
    
    //velocity of the car
    double velocity;
    
    //the phi of the car
    double phi;
    
    //the radius of dubins
    double dubinsR;
    
    //the diagonal line of this box
    double sqrtDist;
    
    //temp using **********************************************
    Point centerA;
    Point centerB;
    vector<Point*> nextTurnP;
    
    //store the Free path
    vector<Node*> pathRRT;
    vector<Node*> newPathRRT;
    Node* startNode;//store the current startNode
    vector<int> pathSize;// record the sizes of path segments
    
    //reward Node to store node in reward region
    Node* rewardNode;
    bool rewardReached;// flag if reward region reached by T
    
    //flag if game if over
    bool ifover;
    
    // dubins curve free or not
    bool ifFreeCurve;
    
    //the center of the Goal
    Point goal_center;
    
    //the minium distance of path
    double minDist;
    
    //the path length for a new path
    double pathDist;
    
    //Dubins R array
    double* dubinsRs;

    //time to adjust the dubinsRs base
    double timeToAdjust;
    
    
    
	////////////////////////////// obstacles
	// number of obstacles
	int numObst(void){
		return m_simulator->GetNrObstacles();
	}
	// number of vertices of ith obst
	int numVerticeObst(int i){
		return m_simulator->GetObstacleNrVertices(i);
	}
	// vertices of ith obst
	const double* verticeObst(int i){
		return m_simulator->GetObstacleVertices(i);
	}
    
	////////////////////////////// reward area
	// number of reward regions
	int numRewardArea(void){
		return m_simulator->GetNrRewardRegions();
	}
	// value of ith reward area
	int valueRewardArea(int i){
		return m_simulator->GetRewardRegionValue(i);
	}
	// x of ith reward center
	double xReward(int i){
		return m_simulator->GetRewardRegionCenterX(i);
	}
	// y of ith reward center
	double yReward(int i){
		return m_simulator->GetRewardRegionCenterY(i);
	}
	// radius of ith reward area
	double radiusReward(int i){
		return m_simulator->GetRewardRegionRadius(i);
	}
    
	////////////////////////////// robot
	// length of robot
	double lengthRobot(void){
		return m_simulator->GetRobotLength();
	}
	// width of robot
	double widthRobot(void){
		return m_simulator->GetRobotWidth();
	}
	// x of robot center
	double xRobot(void){
		return m_simulator->GetRobotCenterX();
	}
	// y of robot center
	double yRobot(void){
		return m_simulator->GetRobotCenterY();
	}
	// angle of robot in radians
	double angleRobot(void){
		return m_simulator->GetRobotAngleInRadians();
	}
	// set robot orientation and position
	void setRobotOriPos(double theta,double x,double y){
		m_simulator->SetRobotOrientationAndPosition(theta,x,y);
	}
	// if robot in collision 
	bool ifRobotCollision(void){
		return m_simulator->IsRobotInCollision();
	}
	// if robotcenter in the ith reward area
	bool ifRobotInReward(int i){
		return m_simulator->IsRobotCenterInsideRewardRegion(i);
	}
	// if point in the ith reward area
	bool ifPointInReward(double x, double y, int i){
		return m_simulator->IsPointInsideRewardRegion(x,y,i);
	}
    
	/////////////////////////////// bounding box
	// min coordinate(x,y) of bounding box
	const double* minPointBox(void){
		return m_simulator->GetBoundingBoxMin();
	}
	// max coordinate(x,y) of bounding box
	const double* maxPointBox(void){
		return m_simulator->GetBoundingBoxMax();
	}
    
	////////////////////////self defined
    //  initial rewards with points
    void initialRewards(Point* rewards, int nReward){
        for (int i=0; i<nReward; i++) {
            rewards[i].x = xReward(i);
            rewards[i].y = yReward(i);
        }
    }
    //distance between a and b
    double dist(Point a, Point b){
        return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    }
    //get the four vertics of robot
    double* verticeRobotAtPoint(Point p){
        double rr = sqrt(pow(lengthRobot(),2)+pow(widthRobot(),2))/2;//rRobot
        double xc = p.x;
        double yc = p.y;
        double theta = p.theta;
        double alpha = atan2(widthRobot(),lengthRobot());
        double* vs = new double[8];
        vs[0] = xc + rr*cos(theta+alpha);//1.x
        vs[1] = yc + rr*sin(theta+alpha);//1.y
        vs[2] = xc + rr*cos(theta-alpha);//2.x
        vs[3] = yc + rr*sin(theta-alpha);//2.y
        vs[4] = xc - rr*cos(theta+alpha);//3.x
        vs[5] = yc - rr*sin(theta+alpha);//3.y
        vs[6] = xc - rr*cos(theta-alpha);//4.x
        vs[7] = yc - rr*sin(theta-alpha);//4.y
        return vs;
    }
    // check it point in free configuration space
    bool ifFreeSpace(Point p){//after boundary inflated
        bool inBox = false;
        bool outObst = true;
        double robotR = sqrt(pow(lengthRobot(),2)+pow(widthRobot(),2))/2;
        if(minPointBox()[0]+robotR<p.x && minPointBox()[1]+robotR<p.y){//check out boundary
            if(maxPointBox()[0]-robotR>p.x && maxPointBox()[1]-robotR>p.y){
                inBox = true;
            }
        }
        //check obstacle
        for (int i=0; i<numObst(); i++) {
            int nvPolyRobot = 4;
            double* vRobot = verticeRobotAtPoint(p);//
            int nvPolyObst = numVerticeObst(i);
            const double* vObst = verticeObst(i);
            bool polypolyCollision = PolygonPolygonCollision(nvPolyObst,vObst,nvPolyRobot, vRobot);
            if(polypolyCollision){
                outObst = false;
                break;
            }
        }
        if(inBox && outObst){
            return true;
        }else{
            return false;
        }
    }
    
    bool ifSafeMove(Point start, Point end){//check if move from start to end safe(no collision)
        bool outObst = true;
        //        double robotR = sqrt(pow(lengthRobot(),2)+pow(widthRobot(),2))/2;
        //check obstacle
        for (int i=0; i<numObst(); i++) {
            //            int nvPolyRobot = 4;
            //            double* vRobot = verticeRobotAtPoint(end);//
            int nvPolyObst = numVerticeObst(i);
            const double* vObst = verticeObst(i);
            //            bool polypolyCollision = PolygonPolygonCollision(nvPolyObst,vObst,nvPolyRobot, vRobot);// check end point safe or not
            //            if(polypolyCollision){
            //                outObst = false;
            //                break;
            //            }
            double p0[2] = {start.x,start.y}; //check move path safe or not
            double p1[2] = {end.x,end.y};
            bool movePathCollision = SegmentPolygonIntersection(p0,p1,nvPolyObst,vObst);
            if(movePathCollision){
                outObst = false;
                break;
            }
        }
        if(outObst){
            return true;
        }else{
            return false;
        }
    }
    
	//move one step ahead toward to sample point
    Point getNextStepPoint(Point start, Point end){
        double delta = end.theta-start.theta;
        double cosPhi = getCosPhi(delta);
//        cout<<"cosPhi"<<cosPhi<<endl;
        
        double dx = velocity*cosPhi*cos(start.theta);
        double dy = velocity*cosPhi*sin(start.theta);
        
        double nextX = start.x + dx;
        double nextY = start.y + dy;
        
        double append = delta*step_size/velocity;
        cout<<"append "<<append<<endl;
        
        double nextTheta = start.theta + append;
        cout<<"/n"<<nextX<<endl;
        Point qtemp = Point(nextX,nextY,nextTheta);
        
        return qtemp;
    }
    
    double getCosPhi(double delta){
        double tmp = lengthRobot()*delta/velocity;
        cout<<"tmp:"<<tmp<<endl;
        return sqrt(1-pow(tmp,2));
    }
    
    //the new method to compute the x' and y'
    Point getNextCirclePoint(Point qn, Point qr){
        double delta = qr.theta-qn.theta;
        
        //calculate the center of the circle
        //initial xc and yc
        double xc;
        double yc;
        double tan_theta0=tan(qn.theta+PI/2);
		double tan_theta1=tan(qr.theta+PI/2);
        //1.check if theta0 and theta1 =PI or 0
        if(ifequal(qn.theta,PI)||ifequal(qn.theta,0)){
            xc=qn.x;
            yc = tan_theta1*xc - tan_theta1*qn.x +qn.y;
        }else if(qr.theta==PI||qr.theta==0){
            xc=qr.x;
            yc=tan_theta0*xc-tan_theta0*qn.x+qn.y;
        }else{
            xc=(qr.y-tan_theta1*qr.x-qn.y+tan_theta0*qn.x)/(tan_theta0-tan_theta1);
            yc=tan_theta0*xc-tan_theta0*qn.x+qn.y;
        }
        center.x=xc;
        center.y=yc;
        center.theta=0;
        //        center.coutPoint();
        //calculate the radius
        double radius=sqrt(pow(qn.x-xc,2)+pow(qn.y-yc,2));
        //check if radius is larger than minRadius
        double minRadius = lengthRobot()/tan(0.43);//max angle 25
        if(radius < minRadius){
            return Point(10,10,1);
        }
        
        //make sure the radius is not too large
        if(radius > 9999){
            return Point(10,10,1);
        }
        //initial the solve point 
        double x_sol1;
        double y_sol1;
        //check which side the center is
        if(ifequal(qn.theta-phi, PI/2)){
            y_sol1 = yc;
            x_sol1 = xc - radius;}
        else if(ifequal(qn.theta-phi, 3*PI/2)){
            y_sol1 = yc;
            x_sol1 = xc + radius;
        }else{
            double b=-2*(qn.x+tan(qn.theta-phi/2)*qn.y);
            double a=1+pow(tan(qn.theta-phi/2),2);
            double c=pow(qn.x,2)+pow(qn.y,2)-pow(radius,2);
            x_sol1 = (double)(-b+sqrt(pow(b,2)-4*a*c))/2*a;
            if(ifequal(x_sol1,qn.x)){
                x_sol1 = (double)(-b-sqrt(pow(b,2)-4*a*c))/2*a;
            }
            y_sol1=tan(qn.theta-phi/2)*x_sol1;
        }
        Point qnew = Point(x_sol1,y_sol1,qn.theta+phi);
        return qnew;
    };
    
    //---------------------THE NEW VERSION--------------------------//    
    //dubins curve function
    //function turn 
    // return true if curve from qn to qr exists, return false if obstacle on curve
    bool Dubins(Point qn, Point qr){
        // draw the circles tangent with A, B
        // A first
        // get the two circle tangent with A 
        double tan_vitual=tan(qn.theta+PI/2);
        double x_no_sign_squre =pow(dubinsR,2)/(1+(pow(tan_vitual,2)));
        double x_no_sign =sqrt(x_no_sign_squre);
        double xc_a_1 = qn.x+x_no_sign;
        double xc_a_2 = qn.x-x_no_sign;
        double yc_a_1 = x_no_sign*tan_vitual+qn.y;
        double yc_a_2 = (-x_no_sign)*tan_vitual+qn.y;
        Point center1=Point(xc_a_1,yc_a_1,0);
        Point center2=Point(xc_a_2,yc_a_2,0);
         
        double distCenterPoint1 = dist(center1,qr);
        double distCenterPoint2 = dist(center2,qr);
        
//        centerA = distCenterPoint1<distCenterPoint2?center1:center2;
        if(distCenterPoint1<distCenterPoint2)
            centerA=center1;
        else
            centerA=center2;
        
        double thetaCenter = atan2(qn.y-centerA.y, qn.x-centerA.x);
        //test111
        double tmpDeltaTheta = qn.theta - thetaCenter;
        if (tmpDeltaTheta>PI) {
            tmpDeltaTheta -= 2*PI;
        }
        
//        cout<<endl<<"thetaCenter angle: "<< thetaCenter<<" qn.theta: "<< qn.theta <<"==> \n------------------- theta-center.theta for A:"<<tmpDeltaTheta<<endl;

        if(ifequal(tmpDeltaTheta, PI/2)){
            centerA.theta = 1;//anti-clockwise
        }else{
            centerA.theta = -1;//clockwise
        }
//        cout<<"centerA.theta "<<qn.theta/PI*180<<":"<<centerA.theta<<endl;
        
        //Point B 
        tan_vitual=tan(qr.theta+PI/2);
        x_no_sign_squre =pow(dubinsR,2)/(1+(pow(tan_vitual,2)));
        x_no_sign =sqrt(x_no_sign_squre);
        double xc_b_1 = qr.x+x_no_sign;
        double xc_b_2 = qr.x-x_no_sign;
        double yc_b_1 = x_no_sign*tan_vitual+qr.y;
        double yc_b_2 = (-x_no_sign)*tan_vitual+qr.y;
        center1=Point(xc_b_1,yc_b_1,0);
        center2=Point(xc_b_2,yc_b_2,0);
//        cout<<"center1: ";center1.coutPoint();//
//        cout<<"center2: ";center2.coutPoint();//
        distCenterPoint1 = dist(center1,qn);
        distCenterPoint2 = dist(center2,qn);
//        centerB = distCenterPoint1<distCenterPoint2?center1:center2;
        if(distCenterPoint1<distCenterPoint2)
            centerB=center1;
        else
            centerB=center2;
        
        thetaCenter = atan2(qr.y-centerB.y, qr.x-centerB.x);
        
        //test111
        tmpDeltaTheta = qr.theta - thetaCenter;
        if (tmpDeltaTheta>PI) {
            tmpDeltaTheta -= 2*PI;
        }
//        cout<<endl<<"thetaCenter angle: "<<thetaCenter<<" qr.theta: "<<qr.theta<<" ==>"<<"\n----------------- theta-center.theta for B: "<<tmpDeltaTheta<<endl;
        if(ifequal(tmpDeltaTheta, PI/2)){
            centerB.theta = 1;//anti-clockwise
        }else{
            centerB.theta = -1;//clockwise
        }
//        cout<<"centerB.theta "<<qr.theta/PI*180<<":"<<centerB.theta<<endl;
        //finish initial B
        
        // calculate the angle theta, distance of segment(centerA, centerB)
        double tmpAngleCenters = atan2(centerB.y-centerA.y, centerB.x-centerA.x);
//        cout<<"tmpAngleCenters: "<<tmpAngleCenters<<endl;
        double angleCenters = tmpAngleCenters > 0? tmpAngleCenters : 2*PI+tmpAngleCenters;//adjust the range from [-PI,PI] to [0,2PI]
        double distCenters = dist(centerA,centerB);
//        cout<<"distA->B "<<distCenters<<endl;
        //if the distance between two centers is too small, then the curve doesn't exist, break out. 
        if(distCenters<2*dubinsR)
            return false;
//        cout<<endl<<"centerA: ";
//        centerA.coutPoint();
//        cout<<endl<<"centerB: ";
//        centerB.coutPoint();
//        cout<<"\nfinish centerA and centerB \n";
        
        
        // check the direction of circles at A side, if same or diff, and then turn around the circle to the leaving point
        // if same (centerA.theta == centerB.theta)
        
        // clear path vector, prepared for new pash push
        nextTurnP.clear();
        //push the first point into the vector
        Point* new_point = new Point(qn.x,qn.y,qn.theta);
        nextTurnP.push_back(new_point);
        //-------------------------this is the begin of moving--------------------------//
        if(ifequal(centerA.theta ,centerB.theta)){
            
            //same circle direction strategy: tangent line not seperate 2 circles, circles on the same side of line
            // the whole motion path composed of turning C1, going streight S, turning C2.
            // path of turning C1 
            //keep turning until (A.theta == angleCenters), then leave this circle
            //make sure A.theta will wrap up at 0-2PI range.
            // angle step size is phi*directionOfCircle
            
            double tempAthetaC1 = qn.theta*centerA.theta;
//            cout<<"angleCenters: "<<angleCenters<<endl;
            double angleLeave = angleCenters*centerA.theta;
            if(tempAthetaC1 > angleLeave){//handle case: tempAthetaC1 > angleLeave 
                angleLeave += 2*PI;
            }
            
            while(true){
//                cout<<"tempAthetaC1: "<<tempAthetaC1<<" angleLeave: "<<angleLeave<<" phi: "<<phi<<endl;
                if( tempAthetaC1 + phi > angleLeave){
                    Point* tmpp = turn(*nextTurnP.back(), (angleLeave-tempAthetaC1)*centerA.theta, centerA);// turn A to angle angleCenters arond centerA, last move, ready to leave
                    if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                        cout<<"obstacle, and NULL"<<endl;
                        return false; 
                    }
                    //                    tmpp->coutPoint();
                    nextTurnP.push_back(tmpp);
                    break;
                }
                Point* tmpp = turn(*nextTurnP.back(), phi*centerA.theta, centerA);// turn A phi angle around centerA
                if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                    cout<<"obstacle, and NULL"<<endl;
                    return false; 
                }
                //tmpp->coutPoint();
                nextTurnP.push_back(tmpp);
                // update tempAthetaC1
                tempAthetaC1 = tempAthetaC1+phi;
            }
//            cout<<endl<<"leaving C1 at : ";
            //            nextTurnP.back()->coutPoint();
            // path of going streight S
            // keep going streight for distance of distCenters, 
            // then it reach the tangent point from streight line to circle B.
            double currentDist=0;
            while(true){
                // move Point A one stepSizeStreight forward
                if(currentDist+step_size > distCenters){
                    Point* tmpp = moveStreight(*nextTurnP.back(), distCenters-currentDist); // last move
                    if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                        cout<<"obstacle, and NULL"<<endl;
                        return false; 
                    }
                    //                    tmpp->coutPoint();
                    nextTurnP.push_back(tmpp);
                    break;
                }
                Point* tmpp=moveStreight(*nextTurnP.back(), step_size);// normal move
                if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                    cout<<"obstacle, and NULL"<<endl;
                    return false; 
                }
                //                tmpp->coutPoint();
                nextTurnP.push_back(tmpp);
                //update currentDist
                currentDist +=step_size;
            }
            
            
            // path of turing C2
            //keep turning until (A.theta == B.theta), then path done, reach destination
            //make sure A.theta will wrap up at 0-2PI range.
            // angle step size is phi*directionOfCircle
            double tempAthetaC2 = nextTurnP.back()->theta*centerB.theta;
            double angleDone = qr.theta*centerB.theta;
            if(tempAthetaC2 > angleDone){//handle case: tempAthetaC2 > angleDone 
                angleDone += 2*PI;
            }
            while(true){
                if( tempAthetaC2 + phi > angleDone){
                    Point* tmpp = turn(*nextTurnP.back(), (angleDone-tempAthetaC2)*centerB.theta, centerB);// turn A to angle angleCenters around centerB, last move, path done
                    if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                        cout<<"obstacle, and NULL"<<endl;
                        return false; 
                    }
                    //                    tmpp->coutPoint();
                    nextTurnP.push_back(tmpp);
                    break;
                }
                Point* tmpp = turn(*nextTurnP.back(), phi*centerB.theta,centerB);// turn A phi angle around centerB
                if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                    cout<<"obstacle, and NULL"<<endl;
                    return false; 
                }
                // update tempAthetaC2
                nextTurnP.push_back(tmpp);
                //                tmpp->coutPoint();
                tempAthetaC2 = tempAthetaC2+phi;
            }
        }//end of dubins curve for same circle direction case
        else{
            // the whole motion path composed of turning C1, going streight S, turning C2.
            // path of turning C1 
            //keep turning until (A.theta == angleTangentLine), then leave this first circle
            double absAngleCenterLine2TangentLine = asin((double)dubinsR/(distCenters/2));// abs(angle from centerLine to tangentLine)
            double angleTangentLine = angleCenters + absAngleCenterLine2TangentLine*centerA.theta;
            //make sure A.theta will wrap up at 0-2PI range.
            // angle step size is phi*directionOfCircle
            double tempAthetaC1 = qn.theta*centerA.theta;
            double angleLeave = angleTangentLine*centerA.theta;
            if(tempAthetaC1 > angleLeave){//handle case: tempAthetaC1 > angleLeave 
                angleLeave += 2*PI;
            }
            while(true){
                if( tempAthetaC1 + phi > angleLeave){
                    Point* tmpp=turn(*nextTurnP.back(), (angleLeave-tempAthetaC1)*centerA.theta, centerA);// turn A to angle angleCenters around centerA, last move, ready to leave
                    if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                        cout<<"obstacle, and NULL"<<endl;
                        return false; 
                    }
                    nextTurnP.push_back(tmpp);
                    //                    tmpp->coutPoint();
                    break;
                }
                Point* tmpp =turn(*nextTurnP.back(), phi*centerA.theta, centerA);// turn phi angle around centerA
                if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                    cout<<"obstacle, and NULL"<<endl;
                    return false; 
                }
                // update tempAthetaC1
                nextTurnP.push_back(tmpp);
                //                tmpp->coutPoint();
                tempAthetaC1 = tempAthetaC1+phi;
            }
            // path of going streight S
            // keep going streight for distance of distTangentLine, 
            // 	then it reach the tangent point from streight line to circle B.
            double distTangentLine = 2*sqrt(pow(distCenters/2,2)-pow(dubinsR,2));
            double currentDist = 0;// record the current distance on the streight line
            while(true){
                // move Point A one stepSizeStreight forward
                if(currentDist+step_size > distTangentLine){
                    Point* tmpp=moveStreight(*nextTurnP.back(), distTangentLine-currentDist); // last move
                    if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                        cout<<"obstacle, and NULL"<<endl;
                        return false; 
                    }
                    //                    tmpp->coutPoint();
                    nextTurnP.push_back(tmpp);
                    break;
                }
                Point* tmpp = moveStreight(*nextTurnP.back(), step_size);// normal move			
                if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                    cout<<"obstacle, and NULL"<<endl;
                    return false; 
                }
                //update currentDist
                //                 tmpp->coutPoint();
                nextTurnP.push_back(tmpp);
                currentDist += step_size;
            }
            
            // path of turing C2
            //keep turning until (A.theta == B.theta), then path done, reach destination
            //make sure A.theta will wrap up at 0-2PI range.
            // angle step size is phi*directionOfCircle
            double tempAthetaC2 = nextTurnP.back()->theta*centerB.theta;
            double angleDone = qr.theta*centerB.theta;
            if(tempAthetaC2 > angleDone){//handle case: tempAthetaC2 > angleDone 
                angleDone += 2*PI;
            }
            while(true){
                if( tempAthetaC2 + phi > angleDone){
                    Point* tmpp=turn(*nextTurnP.back(), (angleDone-tempAthetaC2)*centerB.theta, centerB);// turn A to angle angleCenters around centerB, last move, path done
                    if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                        cout<<"obstacle, and NULL"<<endl;
                        return false; 
                    }
                    //                    tmpp->coutPoint();
                    nextTurnP.push_back(tmpp);
                    break;
                }
                Point* tmpp = turn(*nextTurnP.back(), phi*centerB.theta, centerB);// turn A phi angle around centerB
                if (tmpp == NULL) {// collision happen, stop curve and return dubins()
//                    cout<<"obstacle, and NULL"<<endl;
                    return false; 
                }
                // update tempAthetaC2
                //                tmpp->coutPoint();
                nextTurnP.push_back(tmpp);
                tempAthetaC2 = tempAthetaC2+phi;
            }
        }//end of dubins curve for diff circle direction case
        return true;
    }
    
    // turn Point A by deltaAngle around circle center, and check collision, return new Point , null for collisionOnPath.
    Point* turn(Point A, double deltaAngle, Point center){
        bool ifCollision = false;
        //update position and orientation
        double tmpNewTheta = A.theta + deltaAngle;
        double newTheta;
        if (tmpNewTheta >= 2*PI) {// adjust the range of new theta into [0,2PI)
            newTheta = tmpNewTheta - 2*PI;
        }else if(tmpNewTheta < 0){
            newTheta = tmpNewTheta + 2*PI;
        }else{
            newTheta = tmpNewTheta;
        }
        //        cout<<"newTheta: "<<newTheta<<endl;//
        double deltaX2center = dubinsR*cos(newTheta-center.theta*PI/2);
        double deltaY2center = dubinsR*sin(newTheta-center.theta*PI/2);
        double newX = center.x + deltaX2center;
        double newY = center.y + deltaY2center;
        //create Point at new position and orientation
        Point* newP = new Point(newX, newY, newTheta);
        //check collistion status
        if(!ifFreeSpace(*newP)){
            ifCollision = true;
        }
        if(ifCollision == true){
            return NULL;		
        }else{
            return newP;
        }
    }
    
    // if a == b at two digit after digital point.
    bool ifequal(double a, double b){
        if(floor(a*1000) == floor(b*1000)){
            return true; 
        }
        return false;
    }
    
    // move Point A streight by distance, and check collision, return new Point, null for collisionOnPath
    Point* moveStreight(Point A, double distance){
        bool ifCollision = false;
        //update postion
        double newX = A.x + distance*cos(A.theta);
        double newY = A.y + distance*sin(A.theta);
        //create Point for new position and orientation
        Point* newP = new Point(newX, newY, A.theta);
        //check collision status
        if(!ifFreeSpace(*newP)){
            ifCollision = true;
        }
        if(ifCollision == true){
            return NULL;		
        }else{
            return newP;
        }
    }
    
    //check if the qnew can connect with the goal
    bool checkToTheGoal(Point qnew,double radom_theta){
        goal_center.theta=radom_theta;
        return Dubins(qnew, goal_center);
        cout<<"Arrival the goal"<<endl;
    }
    
};



#endif
