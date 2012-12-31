/**
 *@file Graphics.hpp
 *@author Erion Plaku 
 *@brief Graphics for running simulation and setting up problem
 */

#ifndef  GRAPHICS_HPP_
#define  GRAPHICS_HPP_

#include "MotionPlanner.hpp"
#include "Simulator.hpp"

/**
 *@name Functions for drawing
 *@{
 */

/**
 *@brief Set drawing color
 *
 *@param r red component (value [0, 1])
 *@param g green component (value [0, 1])
 *@param b blue component (value [0, 1])
 */
void DrawColor(const double r, const double g, const double b);

/**
 *@brief Draw point
 *
 *@param x x coordinate
 *@param y y coordinate
 */
void DrawPoint2D(const double x, const double y);

/**
 *@brief Draw segment from (x1, y1) to (x2, y2)
 *
 *@param x1 x coordinate of first point
 *@param y1 y coordinate of first point
 *@param x2 x coordinate of second point
 *@param y2 y coordinate of secondt point
 */
void DrawSegment2D(const double x1, const double y1,
		   const double x2, const double y2);

/**
 *@brief Draw circle
 *
 *@param cx x position of circle center
 *@param cy y position of circle center
 *@param r circle radius
 */
void DrawCircle2D(const double cx, const double cy, const double r);

/**
 *@brief Draw string
 *
 *@param x x position of where to draw the string
 *@param y y position of where to draw the string
 *@param str string
 */
void DrawString2D(const double x, const double y, const char str[]);

/**
 *@}
 */

/**
 *@author Erion Plaku 
 *@brief  Graphics for running simulation and setting up problem
 */
class Graphics
{   
public:
   /**
    *@brief Initialize data and variables
    *
    *@param motionPlanner pointer to motion planner
    */
    Graphics(MotionPlanner  * const motionPlanner);
    
   /**
    *@brief Destroy window
    */
    ~Graphics(void);

   /**
    *@brief Print help information
    */
    void HandleEventOnHelp(void);
    
   /**
    *@brief Main event loop
    */
    void MainLoop(void);

   /**
    *@brief Maximum time to run motion planner
    *@param tmax maximum run time
    */
    void SetMotionPlannerMaxTime(const double tmax)
    {
	m_motionPlannerMaxTime = tmax;
    }
    
    
protected:
   /**
    *@brief Perform simulation step
    */
    void HandleEventOnTimer(void);
    
   /**
    *@brief Main rendering function
    */
    void HandleEventOnDisplay(void);
    
   /**
    *@brief Respond to event when left button is clicked
    *
    *@param mousePosX x-position
    *@param mousePosY y-position
    */
    void HandleEventOnMouseLeftBtnDown(const double mousePosX, const double mousePosY);
    
   /**
    *@brief Respond to key presses
    *
    *@param key key pressed
    */
    void HandleEventOnKeyPress(const int key);
    
   /**
    *@brief Respond to menu-item selections
    *
    *@param item selected item
    */
    void HandleEventOnMenu(const int item);



   /**
    *@name GLUT callback functions
    *@{
    */

    static void CallbackEventOnDisplay(void);
    static void CallbackEventOnMouse(int button, int state, int x, int y);
    static void CallbackEventOnTimer(int id);
    static void CallbackEventOnMenu(int item);
    static void CallbackEventOnKeyPress(unsigned char key, int x, int y);
    static void CallbackEventOnSpecialKeyPress(int key, int x, int y);	
    static void MousePosition(const int x, const int y, double *posX, double *posY);

   /**
    *@}
    */

   /**
    *@name Motion planner data
    *@{
    */
    
   /**
    *@brief Pointer to an instance of the motion planner
    */
    MotionPlanner *m_motionPlanner;
    
    /**
    *@brief Maximum time to run motion planner
    */
    double m_motionPlannerMaxTime;

   /**
    *@brief Total time motion planner has been running
    */
    double m_motionPlannerTotalTime;

   /**
    *@brief Boolean variable to indicate whether or not motion planner should be drawn
    */
    bool m_motionPlannerDraw;

	/**
    *@brief Boolean variable to indicate whether or not motion planner should start with virtual Goals
    */
    bool m_motionPlannerVirtual;
    
    /**
     *@}
     */

    /**
     *@name Best path data
     *@{
     */
    
    /**
     *@brief Sequence of configurations contistuting the 
     *       highest reward path computed by the motion planner     
     */
    std::vector<double> m_bestPath;
    
    /**
     *@brief Boolean variable to indicate whether the best path 
     *       should be displayed (in animation) or not
     */
    bool m_bestPathAnimate;    
    
    /**
     *@brief Redisplay rate (in milliseconds) for playing back the best path
     */
    int m_bestPathAnimateTimer;

    /**
     *@brief The total reward accumulated by the best path
     */
    int m_bestPathTotalReward;

    /**
     *@brief Current index in the best path animation, i.e.,
     *       which configuration of the best path should be currently displayed
     */
    int m_bestPathCurrIndex;

    /**
     *@brief Reward accumulated by the best path from beginning until (and including)
     *       the current index
     */
    int m_bestPathCurrReward;

    /**
     *@brief Reward regions that have not yet been reached by the best path from beginning
     *       until (and including) the current index
     */
    std::vector<int> m_bestPathCurrRemainingRewards;

    /**
     *@}
     */

    
   /**
    *@name Different menu options
    *@{
    */

    int MENU_RUN_MOTION_PLANNER;
    int MENU_SET_MAX_TIME_MOTION_PLANNER;    
    int MENU_GET_BEST_PATH;
    int MENU_SET_TIMER_ANIMATE_BEST_PATH;
    int MENU_DRAW_MOTION_PLANNER;
	int MENU_RUN_MOTION_PLANNER_WITH_VIRTUAL;
    
    /**
     *@}
     */
};

#endif
