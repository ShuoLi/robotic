/**
 *@file Simulator.hpp
 *@author Erion Plaku 
 *@brief Problem information
 */

#ifndef SIMULATOR_HPP_
#define SIMULATOR_HPP_

#include "Utils.hpp"

/**
 *@author Erion Plaku 
 *@brief Problem information
 *@remark
 * Input
 * - n polygonal obstacles O_1, O_2, ..., O_n
 * - m reward regions R_1, R_2, ..., R_m, where
 *   each reward region is a circle and has an associated value R_i.value
 * - one robot that can translate and rotate (geometry of robot corresponds to a rectangle)
 * - tmax: upper bound on computation time
 * .
 * Objective
 * - Compute as quickly as possible (within the time bound) a collision-free path
 *   that maximizes the total rewards collected by the robot. A robot collects the i-th reward
 *   iff the robot center enters the circle of the i-th reward region along the path.
 *
 *@remark
 *  Simulator provides functionality to access information about the obstacles, 
 *  the reward regions, and the robot. The simulator also allows the user to 
 *  - set the current orientation and position of the robot
 *  - check whether the robot in its current placement is in collision with the obstacles
 *  - check whether the robot center in its current placement is inside the i-th reward region
 *  .
 */
class Simulator
{
public:    
    /**
     *@brief Initialize variables
     *@param fname name of file with obstacles and reward regions
     */
    Simulator(const char fname[]);
    
    /**
     *@brief Delete allocated memory
     */
    ~Simulator(void);


/**
 *@name Information about the obstacles
 *@{
 */
    /**
     *@brief Get number of obstacles
     */ 	
    int GetNrObstacles(void) const
    {
	return m_obstacles.size();	
    }

    /**
     *@brief Get number of vertices of the i-th obstacle
     */ 	
    int GetObstacleNrVertices(const int i) const
    {
	return m_obstacles[i]->m_vertices.size() / 2;	
    }

    /**
     *@brief Get vertices of the i-th obstacle
     *@remark
     *  Each obstacle is represented as a polygon.
     *  You can obtain the vertices of the i-th obstacle
     *  by calling this function as follows
     *  <CENTER>
     *   const double *vertices = m_simulator->GetObstacleVertices(i);
     *  </CENTER>
     *  Then, the x-coordinate of the j-th vertex is given by
     *  <CENTER>
     *         vertices[2 * j]
     *  </CENTER>
     *  and the y-coordinate of the j-th vertex is given by
     *  <CENTER>
     *         vertices[2 * j + 1]
     *  </CENTER>
     * 
     */ 	
    const double* GetObstacleVertices(const int i) const
    {
	return &(m_obstacles[i]->m_vertices[0]);
    }

/**
 *@}
 */

/**
 *@name Information about the reward regions
 *@{
 */
    /**
     *@brief Get number of reward regions
     */ 	
    int GetNrRewardRegions(void) const
    {
	return m_rewards.size();	
    }

    /**
     *@brief Get reward value of the i-th reward region
     *@param i index of reward region
     */ 	
    int GetRewardRegionValue(const int i) const
    {
	return m_rewards[i]->m_value;	
    }

    /**
     *@brief Get x-coordinate of the center of the i-th reward region (which is a circle)
     *@param i index of reward region
     */ 	
    double GetRewardRegionCenterX(const int i) const
    {
	return m_rewards[i]->m_centerX;	
    }

    /**
     *@brief Get y-coordinate of the center of the i-th reward region (which is a circle)
     *@param i index of reward region
     */ 	
    double GetRewardRegionCenterY(const int i) const
    {
	return m_rewards[i]->m_centerY;	
    }

    /**
     *@brief Get radius of the i-th reward region (which is a circle)
     *@param i index of reward region
     */ 	
    double GetRewardRegionRadius(const int i) const
    {
	return m_rewards[i]->m_radius;	
    }

/**
 *@}
 */


/**
 *@name Information about the robot
 *@{
 */
    /**
     *@brief Get robot length (robot is a rectangle)
     */ 	
    double GetRobotLength(void) const
    {
	return m_robotLength;
    }
    
    /**
     *@brief Get robot width (robot is a rectangle)
     */ 	
    double GetRobotWidth(void) const
    {
	return m_robotWidth;
    }
    
    /**
     *@brief Get x-coordinate of robot's center
     */ 	
    double GetRobotCenterX(void) const
    {
	return m_robotCenterX;
    }
    
    /**
     *@brief Get y-coordinate of robot's center
     */ 	
    double GetRobotCenterY(void) const
    {
	return m_robotCenterY;
    }
    
    /**
     *@brief Get robot's counterclockwise orientation with respect to x-axis
     */ 	
    double GetRobotAngleInRadians(void) const
    {
	return m_robotAngle;
    }

    /**
     *@brief Set robot orientation and position
     *@param theta new orientation of the robot (theta is in radians)
     *@param x new x-coordinate of robot's center
     *@param y new y-coordinate of robot's center
     *
     *@par Description
     *  In its default frame the robot is placed so that the center is at (0, 0), i.e.,
     *  geometry of the robot corresponds to the following rectangle
     *  <CENTER>
     *   (-length/2, -width/2) (length/2, -width/2) (length/2, width/2) (-length/2, width/2)
     *  </CENTER>
     *  This function has the effect of first rotating the robot by an angle theta and then
     *  translating the robot by (x, y)
     */ 	
    void SetRobotOrientationAndPosition(const double theta, const double x, const double y);
    
    /**
     *@brief Return true iff the robot in its current placement is in collision with an obstacle
     */
    bool IsRobotInCollision(void) const;

    /**
     *@brief Return true iff the robot center is inside the i-th reward region
     *@remark The robot accumulates the i-th reward iff
     * its center is inside the i-th reward region
     *@param i index of reward region
     */
    bool IsRobotCenterInsideRewardRegion(const int i) const;

    /**
     *@brief Return true iff the point (x, y) is inside the i-th reward region
     *@param x x-coordinate of point
     *@param y y-coordinate of point
     *@param i index of reward region
     */
    bool IsPointInsideRewardRegion(const double x, const double y, const int i) const;
    
    
/**
 *@}
 */

/**
 *@name Information about the bounding box
 *@{
 */
    /**
     *@brief Get minimum (x, y) coordinates of bounding box
     */ 	
    const double* GetBoundingBoxMin(void) const
    {
	return m_min;
    }
    
    /**
     *@brief Get maximum (x, y) coordinates of bounding box
     */ 	
    const double* GetBoundingBoxMax(void) const
    {
	return m_max;	
    }
    

/**
 *@}
 */

    
protected:
    /**
     *@brief Read polygonal obstacles and reward regions from input file
     *
     *@param fname name of file with obstacles and reward regions
     */ 	
    void SetupFromFile(const char fname[]);
    
    /**
     *@brief Obstacles: each obstacle corresponds to a polygon
     */
    struct Obstacle
    {
	std::vector<double> m_vertices;
	std::vector<int>    m_triangles;
    };

    /**
     *@brief Reward regions: each reward region corresponds to a circle;
     * there is also a reward value associated with each reward region
     */
    struct RewardRegion
    {
	int    m_value;	
	double m_centerX;
	double m_centerY;
	double m_radius;	
    };
    
    /**
     *@brief All the obstacles
     */
    std::vector<Obstacle *>  m_obstacles;

    /**
     *@brief All the reward regions
     */
    std::vector<RewardRegion *> m_rewards;

    /**
     *@brief Robot information: robot is a rectangle
     */
    double m_robotLength;
    double m_robotWidth;
    double m_robotCenterX;
    double m_robotCenterY;
    double m_robotAngle;    
    double m_robotOrig[8];    
    double m_robotCurr[8];

    /**
     *@brief Bounding box: minimum and maximum scene values
     */
    double m_min[2];
    double m_max[2];

    friend class Graphics;
};

#endif
