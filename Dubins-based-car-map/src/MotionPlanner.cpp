#include "MotionPlanner.hpp"
#include "Graphics.hpp"

MotionPlanner::MotionPlanner(Simulator * const simulator)
{
	m_simulator = simulator;
	//initialize your data structures/variables here
	nReward = numRewardArea();
    rewards = new Point[nReward];
    initialRewards(rewards, nReward);
    // initial robot position
	robot = Point(xRobot(),yRobot(),angleRobot());
    //initialize T tree, set root of tree
    rt= new Node(robot);
    T.addRoot(rt);
    startNode = rt;
    //setup step_size
    step_size = 0.1;//
    // check if reward region reached
    rewardNode = NULL;  
    rewardReached = false;
    //initial game parameter
    ifover = false;
    //initial verlocity
    velocity = 2*lengthRobot();//velocity must larger than PI*lengthRobot
    //phi=-25/180*PI;
    phi=0.2;
    dubinsR=1.3;    
    sqrtDist=sqrt(pow(maxPointBox()[0]-minPointBox()[0],2)+pow(maxPointBox()[1]-minPointBox()[1],2));
    goal_center = Point(m_simulator->GetRewardRegionCenterX(0),m_simulator->GetRewardRegionCenterY(0),0);
    minDist=9999999;
    dubinsRs=new double[3];
    dubinsRs[0]=1;
    for(int i=1;i<3;i+=1){
        dubinsRs[i]=dubinsRs[i-1]+0.1;
    }
    timeToAdjust = 5;
}

MotionPlanner::~MotionPlanner(void)
{
}

void MotionPlanner::Solve(const double tmax)
{
    /*
     * you can use the functionality provided in Utils.hpp to measure
     * running time, e.g.,
     *  Clock clk;
     *  ClockStart(&clk);
     *  while(ClockElapsed(&clk) < tmax)
     *  {
     *  }
     * If your algorithm obtains a solution earlier than the maximum time,
     * then you can break out of the while loop (using return or break statement)
     */   
    Clock clk;
    ClockStart(&clk);
    //    for(int nr=nReward-1; nr>=0; nr--){
    while (ClockElapsed(&clk) < tmax) {
        
        //guided sample:
        // sample around qgoal with uniform distribution 
        // round area:center- qgoal, radius- dist(qgoal,q closest to qgoal in T)
        //            cout<<"obst:"<<numObst()<<endl;//
        //            cout<<"start point: ";
        //            robot.coutPoint();
        //            cout<<"reward area: ";
        //            rewards[nr].coutPoint();
        
        //p center sampling
        //            double deltaLeft = abs(rewards[nr].x-minPointBox()[0]);
        //            double deltaRight = abs(rewards[nr].x-maxPointBox()[0]);        
        //            double deltaH = deltaLeft>deltaRight? deltaLeft : deltaRight;
        //            
        //            double randomP = pow(PseudoRandomUniformReal(0,1),1.5);
        //            
        //            double randomR = randomP*deltaH;
        //            cout<<"random dist in R:"<<randomR<<endl;
        //            double randomTheta = PseudoRandomUniformReal(0,2)*PI;
        //            cout<<"random theta in 2PI:"<<randomTheta<<endl;
        //            sampleP = Point(rewards[nr].x+randomR*cos(randomTheta),rewards[nr].y+randomR*sin(randomTheta));
        
        //globel sampling better
        //    double maxSampleAngle = sin(0.17)*velocity/lengthRobot();
        //    double minSampleAngle = sin(-0.17)*velocity/lengthRobot();
        //    cout<<"max "<<maxSampleAngle<<"min "<<minSampleAngle<<endl;
        //        sampleP = Point(PseudoRandomUniformReal(minPointBox()[0],maxPointBox()[0]),PseudoRandomUniformReal(minPointBox()[1],maxPointBox()[1]),PseudoRandomUniformReal(0,2*PI-0.001));
        sampleP = Point(PseudoRandomUniformReal(minPointBox()[0],maxPointBox()[0]),PseudoRandomUniformReal(minPointBox()[1],maxPointBox()[1]),0);
        //    sampleP = Point(PseudoRandomUniformReal(minPointBox()[0],maxPointBox()[0]),PseudoRandomUniformReal(minPointBox()[1],maxPointBox()[1]),PI);
        //    sampleP = Point(-10,3,PI);
        
        //extend RRT Algorithm
        Node* closeNode = T.getClosestNeighbor(sampleP,sqrtDist);
        //        cout<<"closeNode: ";
        //        closeNode->getLoc().coutPoint();
        //        cout<<"sample: ";
        //        sampleP.coutPoint();
        //    qNew = getNextStepPoint(closeNode->getLoc(), sampleP);// get the qNew toward sample point
        // ifFreeCurve
        ifFreeCurve = false;
        Point qnear=closeNode->getLoc();
        sampleP.theta=qnear.theta+PseudoRandomUniformReal(-PI/2,PI/2);
        double bestCurveLength=999999;
        vector<Point*> tempNextTurn=nextTurnP;
        //invoke of dubins
        for(int i=0;i<3;i++){
            dubinsR=dubinsRs[i];
            ifFreeCurve = Dubins(qnear, sampleP);
            if(ifFreeCurve){
                double pathlength=0;
                for(int j=0;j<nextTurnP.size()-1;j++){
                    pathlength+=dist(*nextTurnP[j],*nextTurnP[j+1]);
                }
                if(pathlength<bestCurveLength){
                    tempNextTurn=nextTurnP;
                    bestCurveLength=pathlength;
                }
            }
        }
        nextTurnP=tempNextTurn;
        
        //qNew =getNextCirclePoint(closeNode->getLoc(), sampleP);
        if (ifFreeCurve) {
            //        cout<<endl<<"qNew: ";
            //        qNew.coutPoint();
            //    if(ifSafeMove(closeNode->getLoc(),qNew)){
            //    if(ifFreeSpace(qNew)){
            if(true){
                //                cout<<"SAFE STEP"<<endl;
                Node* samplePNode = T.addNodeAndEdge(closeNode, sampleP,nextTurnP);//add node and edge into tree
                //             //check if get into reward region 0
                //             if (ifPointInReward(qNew.x,qNew.y,nr)){
                //                 rewardNode = tmp;
                //                 //reached reward region break;
                //                 cout<<"REACHED REWARD REGION";
                //                rewardReached = true;
                //                 break;
                //             }
                //               double random_theta=sampleP.theta+PseudoRandomUniformReal(-PI/3,PI/3);//genterate a random theta for the goal point
                double random_theta=sampleP.theta+PseudoRandomUniformReal(-PI,PI);
                //                if(dist(sampleP, goal_center)<5){
                if(checkToTheGoal(sampleP,random_theta)){
                    Node* goalNode = T.addNodeAndEdge(samplePNode,goal_center,nextTurnP);
                    rewardReached = true;
                    rewardNode = goalNode;
                    cout<<"arrival at the goal"<<endl;
                }
                //                }
            }else{
                cout<<"non-safe step"<<endl;
            }
            // T.displayGraph();
            if (rewardReached == true && rewardNode != NULL) {
                cout<<" reachGoal ";
                //                int prePathLength = pathRRT.size();
                //push to pathRRT
                newPathRRT.push_back(rewardNode);
                Node* pNode = rewardNode->getParentNode();
                //                cout<<"enter the reward check area"<<endl;
                while (pNode != startNode) {
                    newPathRRT.push_back(pNode);
                    pNode = pNode->getParentNode();
                }
                newPathRRT.push_back(startNode);
                //        cout<<newPathRRT.size()<<endl;
                
                pathDist=0;
                for(int i=0;i<newPathRRT.size()-1;i++){
                    pathDist+=dist(newPathRRT[i]->getLoc(), newPathRRT[i+1]->getLoc());
                }
                cout<<"path distance:"<<pathDist<<endl;
                if(pathDist<minDist){
                    cout<<"met a shorter path"<<endl;
                    pathRRT=newPathRRT;
                    minDist=pathDist;
                }
                newPathRRT.clear();
                rewardReached=false;
            }else{//adjust dubinsRs
                if(ClockElapsed(&clk) > timeToAdjust){
                    if (dubinsRs[0]>1) {
                        dubinsRs[0]=PseudoRandomUniformReal(0.5,1);///test111
                    }else{
                        dubinsRs[0]=PseudoRandomUniformReal(1,2);///test111
                    }
                    cout<<"adjustment of dubinsRs------------------------------------------- "<< dubinsRs[0] <<endl;
                    for(int k=1;k<3;k++){
                        dubinsRs[k]=dubinsRs[k-1]+0.2;
                    }
                    timeToAdjust = ClockElapsed(&clk) + 5;
                }
            }
            
            
        }
    } //end of loop for maxt time
    //         //if reward region reached
    //    cout<<endl<<"rewardReached"<<rewardReached<<endl;
    //    cout<<"rewardNode:"<<endl;
    //    rewardNode->getLoc().coutPoint();
    //check whether the new PathRRT is shorter than the pathRRT
    
    //    }//end of loop for reward region
    ifover=true;
}




void MotionPlanner::GetHighestRewardCollisionFreePath(std::vector<double> * path)
{
    
    /*
     * CS336: since there is only one goal region in your case, this function needs to simply 
     *        get the path to the goal region with index 0
     */
    
    /*
     * CS436: since you are dealing with multiple goal, you should try to get the path
     *        with the highest accumulated reward
     */
	//shuffle pathes]
    
    //    cout<<"enter the highest Function"<<endl;
    //    cout<<pathRRT.size()<<endl;
    cout<<"the final minDist is:"<<minDist<<endl;
    for (int i=int(pathRRT.size()-1); i>0; i--) {
        Edge* edgeP=T.getEdge(pathRRT[i], pathRRT[i-1]);
        //         for(int i
        vector<Point *> edgeV = edgeP->getEdgeV();
        
        for (int j=0; j<edgeV.size()-1; j++) {
            path->push_back(edgeV[j]->theta);
            path->push_back(edgeV[j]->x);
            path->push_back(edgeV[j]->y);
        }
        //        cout<<path->size()<<endl;
    }
}

void MotionPlanner::Draw(void)
{
	//you can use the draw functions to draw your roadmap or tree
	//e.g.,
	// DrawColor(r, g, b)
	// DrawPoint2D(x, y)
	//DrawSegment2D(x1,y1,x2,y2);
    
    //    if(ifFreeSpace(sampleP)){
    //	DrawColor(0,0,0);
    //	DrawPoint2D(sampleP.x,sampleP.y);
    //    
    //    DrawColor(255,0,0);
    //    DrawPoint2D(centerA.x,centerA.y);
    //    //    DrawColor(0,0,255);
    //    DrawPoint2D(centerB.x,centerB.y);
    //	DrawColor(0,255,0);
    //	DrawPoint2D(qNew.x, qNew.y);
    //    DrawColor(0,0,255);
    //    DrawPoint2D(center.x, center.y);
    //draw nextTurnP vector path
    
    //    DrawColor(0,255,0);
    //    for (int i=0; i<nextTurnP.size(); i++) {
    //        DrawPoint2D(nextTurnP[i]->x, nextTurnP[i]->y);
    //    }
    
    //    }
    
    //    for(int i=0 ; i < T.nodeList.size() ; i++)
    //    {
    //        DrawColor(0,0,0);
    //        for (int j=0; j< T.nodeList[i]->getAdjNodeList().size(); j++) {
    //            DrawSegment2D(T.nodeList[i]->getLoc().x,
    //                          T.nodeList[i]->getLoc().y, 
    //                          T.nodeList[i]->getAdjNodeList()[j].getDstNode()->getLoc().x, 
    //                          T.nodeList[i]->getAdjNodeList()[j].getDstNode()->getLoc().y);            
    //            
    //        }
    //    }
    //    if (ifFreeCurve) {
    //    for(int i=0;i<T.edgeList.size();i++){
    //        DrawColor(0,0,255);
    //        vector<Point*> v= T.edgeList[i]->getEdgeV();
    //        for (int j=0; j<v.size(); j++) {
    //            DrawPoint2D(v[j]->x, v[j]->y);
    //        }
    //    }
    
    //    }
    
    
    
    if(ifover){
        DrawColor(0,255,0);         
        
        for (int i=int(pathRRT.size()-1); i>0; i--) {
            Edge* edgeP=T.getEdge(pathRRT[i], pathRRT[i-1]);
            //         for(int i
            vector<Point *> edgeV = edgeP->getEdgeV();
            
            for (int j=0; j<edgeV.size()-1; j++) {
                DrawSegment2D(edgeV[j]->x, edgeV[j]->y,edgeV[j+1]->x, edgeV[j+1]->y);
            }
            //        cout<<path->size()<<endl;
        }
    }
    
    
    
}

