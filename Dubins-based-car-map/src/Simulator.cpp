#include "Simulator.hpp"

Simulator::Simulator(const char fname[])
{
//default values

//default minimum/maximum values of scene bounding box
    m_min[0] = -22;
    m_min[1] = -14;
    m_max[0] =  22;
    m_max[1] =  14;

//default dimensions of the robot
    m_robotLength  = 2;
    m_robotWidth   = 0.5 * m_robotLength;

//default position and orientation of the robot
    m_robotCenterX = m_robotCenterY = m_robotAngle = 0;
    
    m_robotOrig[0] = -0.5 * m_robotLength;    m_robotOrig[1] = -0.5 * m_robotWidth;
    m_robotOrig[2] =  0.5 * m_robotLength;    m_robotOrig[3] = -0.5 * m_robotWidth;
    m_robotOrig[4] =  0.5 * m_robotLength;    m_robotOrig[5] =  0.5 * m_robotWidth;
    m_robotOrig[6] = -0.5 * m_robotLength;    m_robotOrig[7] =  0.5 * m_robotWidth;

    SetRobotOrientationAndPosition(m_robotAngle, m_robotCenterX, m_robotCenterY);    

    SetupFromFile(fname);    
}

Simulator::~Simulator(void)
{
//delete the obstacles
    const int n = m_obstacles.size();
    for(int i = 0; i < n; ++i)
	if(m_obstacles[i])
	    delete m_obstacles[i];    

//delete the reward regions
    const int r = m_rewards.size();
    for(int i = 0; i < r; ++i)
	if(m_rewards[i])
	    delete m_rewards[i];    
}


void Simulator::SetRobotOrientationAndPosition(const double theta, const double x, const double y)
{
    m_robotAngle   = theta;
    m_robotCenterX = x;
    m_robotCenterY = y;

    const double c = cos(theta);
    const double s = sin(theta);

//first rotate and the translate each robot vertex from its original placement 
//to the current placement
    for(int i = 0; i < 4; ++i)
    {
	m_robotCurr[2 * i    ] = m_robotCenterX + m_robotOrig[2 * i] * c - m_robotOrig[2 * i + 1] * s;
	m_robotCurr[2 * i + 1] = m_robotCenterY + m_robotOrig[2 * i] * s + m_robotOrig[2 * i + 1] * c;
    }
}

bool Simulator::IsRobotInCollision(void) const
{
    for(int i = 0; i < (int) m_obstacles.size(); ++i)
    {
	const int     nv   = m_obstacles[i]->m_vertices.size() / 2;
	const double *poly = &(m_obstacles[i]->m_vertices[0]);
	
	if(PolygonPolygonCollision(4, m_robotCurr, nv, poly))
	    return true;
    }
    return false;    
}

bool Simulator::IsRobotCenterInsideRewardRegion(const int i) const
{
    return IsPointInsideRewardRegion(m_robotCenterX, m_robotCenterY, i);
}

bool Simulator::IsPointInsideRewardRegion(const double x, const double y, const int i) const
{
    return
	(x - m_rewards[i]->m_centerX) * (x - m_rewards[i]->m_centerX) +
	(y - m_rewards[i]->m_centerY) * (y - m_rewards[i]->m_centerY) 
	<= m_rewards[i]->m_radius * m_rewards[i]->m_radius;
}

void Simulator::SetupFromFile(const char fname[])
{
//file with obstacles and reward regions
//
//file format is just a sequence of polygons

    FILE *in = fopen(fname, "r");
    if(in)
    {
	int           nrObstacles;
	int           nrRewardRegions;	
	int           nrVertices;	
	Obstacle     *obst;
	RewardRegion *reward;
	
	if(fscanf(in, "%d", &nrObstacles) != 1)
	{
	    printf("error: expecting number of obstacles\n");
	    fclose(in);
	    return;	    
	}

	for(int i = 0; i < nrObstacles; ++i)
	{
	    if(fscanf(in, "%d", &nrVertices) != 1)
	    {
		printf("error: expecting number of vertices for obstacle %d\n", i);
		fclose(in);
		return;
	    }
	    obst = new Obstacle();
	    obst->m_vertices.resize(2 * nrVertices);	    
	    for(int i = 0; i < 2 * nrVertices; ++i)
		fscanf(in, "%lf", &(obst->m_vertices[i]));
	    obst->m_triangles.resize(3 * (nrVertices - 2));
	    for(int i = 0; i < (int) obst->m_triangles.size(); ++i)
		fscanf(in, "%d", &(obst->m_triangles[i]));
	    m_obstacles.push_back(obst);	    
	}

	if(fscanf(in, "%d", &nrRewardRegions) != 1)
	{
	    printf("error: expecting number of reward regions\n");
	    fclose(in);
	    return;	    
	}

	for(int i = 0; i < nrRewardRegions; ++i)
	{
	    reward = new RewardRegion();
	    if(fscanf(in, "%d %lf %lf %lf", 
		      &(reward->m_value), 
		      &(reward->m_centerX),
		      &(reward->m_centerY),
		      &(reward->m_radius)) != 4)
	    {
		printf("expecting value, centerX, centerY, radius for reward region %d\n", i);
		delete reward;
		fclose(in);
		return;
	    }
	    m_rewards.push_back(reward);	    
	}
	double theta, x, y;
	
	if(fscanf(in, "%lf %lf %lf", &theta, &x, &y) != 3)
	{
	    printf("expecting robot's initial placement (theta, x, y)\n");
	    fclose(in);
	    return;	    
	}
	SetRobotOrientationAndPosition(theta, x, y);

	fclose(in);	    


    }	
    else
	printf("..could not open file <%s>\n", fname);
}
