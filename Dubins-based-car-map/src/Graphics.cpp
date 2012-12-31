#include "Graphics.hpp"

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif


void DrawColor(const double r, const double g, const double b)
{
    glColor3d(r, g, b);    
}

void DrawPoint2D(const double x, const double y)
{
    glPointSize(4);    
    glBegin(GL_POINTS);
    glVertex2d(x, y);    
    glEnd();
    glPointSize(1);
}

void DrawSegment2D(const double x1, const double y1,
		   const double x2, const double y2)
{
    glLineWidth(3);    
    glBegin(GL_LINES);
    glVertex2d(x1, y1);
    glVertex2d(x2, y2);    
    glEnd();
    glLineWidth(1);
}




void DrawString2D(const double x, const double y, const char str[])
{
    if(str)
    {
	glRasterPos2d(x, y);
	for(int i = 0; str[i] != '\0'; ++i)
	    glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
    }    
}

void DrawCircle2D(const double cx, const double cy, const double r)
{
    const int    nsides = 50;    
    const double angle  = 2 * M_PI / nsides;
    
    glBegin(GL_POLYGON);
    for(int i = 0; i <= nsides; i++)
	glVertex2d(cx + r * cos(i * angle), cy + r * sin(i * angle));
    glEnd();	
}



Graphics *m_graphics = NULL;

Graphics::Graphics(MotionPlanner * const motionPlanner) 
{
    m_motionPlanner          = motionPlanner;
    m_motionPlannerMaxTime   = 10;  //seconds
    m_motionPlannerTotalTime = 0;
    m_motionPlannerDraw      = true;
    
    m_bestPathAnimate      = false;
    m_bestPathAnimateTimer = 5; //milliseconds
    m_bestPathTotalReward  = 0;
    m_bestPathCurrIndex    = -1;
    m_bestPathCurrReward   = 0;
}

Graphics::~Graphics(void)
{
}

void Graphics::MainLoop(void)
{	
    m_graphics = this;

//create window    
    static int    argc = 1;	
    static char  *args = (char*)"args";
    glutInit(&argc, &args);    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);    
    glutInitWindowSize(1000, 600);
    glutInitWindowPosition(0, 0); 
    glutCreateWindow("Bug Algorithms");	   	


//register callback functions
    glutDisplayFunc(CallbackEventOnDisplay);
    glutMouseFunc(CallbackEventOnMouse);
    glutIdleFunc(NULL);
    glutTimerFunc(0, CallbackEventOnTimer, 0); 
    glutKeyboardFunc(CallbackEventOnKeyPress);
    glutSpecialFunc(CallbackEventOnSpecialKeyPress);

//create menu
    glutCreateMenu(CallbackEventOnMenu);

    MENU_RUN_MOTION_PLANNER          = 1;
    MENU_SET_MAX_TIME_MOTION_PLANNER = 2;    
    MENU_GET_BEST_PATH               = 3;
    MENU_SET_TIMER_ANIMATE_BEST_PATH = 4;
    MENU_DRAW_MOTION_PLANNER         = 5;
 
    glutAddMenuEntry("Run motion planner",                 MENU_RUN_MOTION_PLANNER);
    glutAddMenuEntry("Set max time to run motion planner", MENU_SET_MAX_TIME_MOTION_PLANNER);
    glutAddMenuEntry("Get highest reward path",            MENU_GET_BEST_PATH);
    glutAddMenuEntry("Set timer to animate highest reward path",     MENU_SET_TIMER_ANIMATE_BEST_PATH);
    glutAddMenuEntry("Draw motion planner [yes/no]",       MENU_DRAW_MOTION_PLANNER);

    glutAttachMenu(GLUT_RIGHT_BUTTON);	

//enter main event loop
    glutMainLoop();	
}

void Graphics::HandleEventOnTimer(void)
{	    
    if(m_bestPathAnimate && m_bestPath.size() > 0)
    {	
	++m_bestPathCurrIndex;
	if(3 * m_bestPathCurrIndex >= m_bestPath.size())
	    m_bestPathCurrIndex = (m_bestPath.size()/3) - 1;

	const double *cfg = &m_bestPath[m_bestPathCurrIndex * 3];
	
	m_motionPlanner->m_simulator->SetRobotOrientationAndPosition(cfg[0], cfg[1], cfg[2]);

	int found = -1;
	for(int j = 0; found == -1 && j < (int) m_bestPathCurrRemainingRewards.size(); ++j)
	    if(m_motionPlanner->m_simulator->IsPointInsideRewardRegion(cfg[1], cfg[2],
								       m_bestPathCurrRemainingRewards[j]))
		found = j;
	if(found >= 0)
	{
	    m_bestPathCurrReward += 
		m_motionPlanner->m_simulator->GetRewardRegionValue(m_bestPathCurrRemainingRewards[found]);
	    m_bestPathCurrRemainingRewards[found] = 
		m_bestPathCurrRemainingRewards[m_bestPathCurrRemainingRewards.size() - 1];
	    m_bestPathCurrRemainingRewards.pop_back();		

	    printf("PLAYBACK OF HIGHEST-REWARD PATH: current accumulated reward = %d\n",
		   m_bestPathCurrReward);
	}
    }
} 

void Graphics::HandleEventOnMouseLeftBtnDown(const double mousePosX, const double mousePosY)
{	
}

void Graphics::HandleEventOnMenu(const int item)
{
    if(item == MENU_RUN_MOTION_PLANNER)
    {
	Clock clk;
	ClockStart(&clk);
	m_motionPlanner->Solve(m_motionPlannerMaxTime);
	m_motionPlannerTotalTime += ClockElapsed(&clk);
	printf("Motion planner has been run for %f seconds\n", m_motionPlannerTotalTime);	
    }
    else if(item == MENU_SET_MAX_TIME_MOTION_PLANNER)
    {
	printf(" current max time for motion planner = %f [s]\n", m_motionPlannerMaxTime);
	printf(" enter new value: ");
	scanf("%lf", &m_motionPlannerMaxTime);
    }
    else if(item == MENU_DRAW_MOTION_PLANNER)
    {
	m_motionPlannerDraw = !m_motionPlannerDraw;
	printf("DRAW_MOTION_PLANNER: %s\n", m_motionPlannerDraw ? "YES" : "NO");	
    } 
    else if(item == MENU_GET_BEST_PATH)
    {
	m_bestPath.clear();
	m_motionPlanner->GetHighestRewardCollisionFreePath(&m_bestPath);
	m_bestPathAnimate   = true;
	m_bestPathCurrIndex = -1;
	m_bestPathCurrReward= 0;

	m_bestPathTotalReward = 0;	
	const int nr = m_motionPlanner->m_simulator->GetNrRewardRegions();
	std::vector<int> rem;
	rem.resize(nr);
	for(int i = 0; i < nr; ++i)
	    rem[i] = i;
	for(int i = 0; i < (int) m_bestPath.size(); i += 3) //3 doubles per config in path
	{
	    int found = -1;
	    for(int j = 0; found == -1 && j < (int) rem.size(); ++j)
		if(m_motionPlanner->m_simulator->IsPointInsideRewardRegion(m_bestPath[i + 1],
									   m_bestPath[i + 2],
									   rem[j]))
		    found = j;
	    if(found >= 0)
	    {
		m_bestPathTotalReward += m_motionPlanner->m_simulator->GetRewardRegionValue(rem[found]);
		rem[found] = rem[rem.size() - 1];
		rem.pop_back();		
	    }
	}	

	m_bestPathCurrRemainingRewards.resize(nr);
	for(int i = 0; i < nr; ++i)
	    m_bestPathCurrRemainingRewards[i] = i;
	
	
	printf("Highest reward path has a total reward of %d\n", m_bestPathTotalReward);	
    }
    else if(item == MENU_SET_TIMER_ANIMATE_BEST_PATH)
    {
	printf(" current timer interval to animate best path = %d [ms]\n", m_bestPathAnimateTimer);
	printf(" enter new value (the smaller the value, the faster the redisplay rate): ");
	scanf("%d", &m_bestPathAnimateTimer);
    }    
}

void Graphics::HandleEventOnKeyPress(const int key)
{
//    cout<<key<<endl;//test111
    switch(key)
    {
    case 27: //escape key
	exit(0);
    
        case 114:
            HandleEventOnMenu(MENU_RUN_MOTION_PLANNER);
            break;
	
    case GLUT_KEY_F1: 
	HandleEventOnHelp();	
	break;
    }
}

void Graphics::HandleEventOnHelp(void)
{
    printf("Help: right click to display menu\n");
}


void Graphics::HandleEventOnDisplay(void)
{
    char str[100];
    
    if(m_motionPlannerDraw)
	m_motionPlanner->Draw();
    
//draw robot
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    glColor3f(1, 0, 0);
    glBegin(GL_POLYGON);
    glVertex2dv(&(m_motionPlanner->m_simulator->m_robotCurr[0]));
    glVertex2dv(&(m_motionPlanner->m_simulator->m_robotCurr[2]));
    glVertex2dv(&(m_motionPlanner->m_simulator->m_robotCurr[4]));
    glVertex2dv(&(m_motionPlanner->m_simulator->m_robotCurr[6]));
    glEnd();

    glColor3f(0, 0, 0);
    DrawPoint2D(m_motionPlanner->m_simulator->GetRobotCenterX(),
		m_motionPlanner->m_simulator->GetRobotCenterY());

    if(m_bestPathAnimate && m_bestPath.size() > 0)
    {
	sprintf(str, "%d", m_bestPathCurrReward);
	glColor3f(0, 0, 0);	
	DrawString2D(m_motionPlanner->m_simulator->GetRobotCenterX(),
		     m_motionPlanner->m_simulator->GetRobotCenterY(), str);
    }
    
    
    
//draw obstacles
    glColor3f(0.45, 0.34, 0.76);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);	
    glBegin(GL_TRIANGLES);

    const int no = m_motionPlanner->m_simulator->GetNrObstacles();    
    for(int i = 0; i < no; ++i)
    {
	Simulator::Obstacle *obst = m_motionPlanner->m_simulator->m_obstacles[i];
	const int            ntri = obst->m_triangles.size();
    
	for(int j = 0; j < ntri; j += 3)
	{
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 0]]);
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 1]]);
	    glVertex2dv(&obst->m_vertices[2 * obst->m_triangles[j + 2]]);
	}
    }
    glEnd();

//draw reward regions
    if(m_bestPathAnimate && m_bestPath.size() > 0)
    {
	for(int i = 0; i < m_bestPathCurrRemainingRewards.size(); ++i)
	{
	    const int id = m_bestPathCurrRemainingRewards[i];	    

	    glColor3f(0, 1, 0);	    
	    DrawCircle2D(m_motionPlanner->m_simulator->GetRewardRegionCenterX(id),
			 m_motionPlanner->m_simulator->GetRewardRegionCenterY(id),
			 m_motionPlanner->m_simulator->GetRewardRegionRadius(id));
	    sprintf(str, "%d", m_motionPlanner->m_simulator->GetRewardRegionValue(id));	    
	    glColor3f(0, 0, 0);	    
	    DrawString2D(m_motionPlanner->m_simulator->GetRewardRegionCenterX(id),
			 m_motionPlanner->m_simulator->GetRewardRegionCenterY(id),
			 str);
	}	
    }
    else
    {
	for(int i = 0; i < m_motionPlanner->m_simulator->GetNrRewardRegions(); ++i)
	{
	    const int id = i;	    

	    glColor3f(0, 1, 0);	    
	    DrawCircle2D(m_motionPlanner->m_simulator->GetRewardRegionCenterX(id),
			 m_motionPlanner->m_simulator->GetRewardRegionCenterY(id),
			 m_motionPlanner->m_simulator->GetRewardRegionRadius(id));
	    sprintf(str, "%d", m_motionPlanner->m_simulator->GetRewardRegionValue(id));	    
	    glColor3f(0, 0, 0);	    
	    DrawString2D(m_motionPlanner->m_simulator->GetRewardRegionCenterX(id),
			 m_motionPlanner->m_simulator->GetRewardRegionCenterY(id),
			 str);
	}	
    }    
}

void Graphics::CallbackEventOnDisplay(void)
{
    if(m_graphics)
    {
	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);    
//	glEnable(GL_DEPTH_TEST);
//	glShadeModel(GL_SMOOTH);	
	
	glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	const double x = m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMin()[0];
	
	glOrtho(m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMin()[0], 
		m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMax()[0], 
		m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMin()[1], 
		m_graphics->m_motionPlanner->m_simulator->GetBoundingBoxMax()[1], -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();	    
	
	m_graphics->HandleEventOnDisplay();
	
	glutSwapBuffers();	    
    }
}

void Graphics::CallbackEventOnMouse(int button, int state, int x, int y)
{
    if(m_graphics && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
    {
	double mouseX, mouseY;
	MousePosition(x, y, &mouseX, &mouseY);
	m_graphics->HandleEventOnMouseLeftBtnDown(mouseX , mouseY);
	glutPostRedisplay();
    }	    
}

void Graphics::CallbackEventOnTimer(int id)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnTimer();
	glutTimerFunc(m_graphics->m_bestPathAnimateTimer, CallbackEventOnTimer, id);
	glutPostRedisplay();	    
    }
}

void Graphics::CallbackEventOnMenu(int item)
{
    if(m_graphics)
    {
	m_graphics->HandleEventOnMenu(item);
	glutPostRedisplay();
    }    
}

void Graphics::CallbackEventOnSpecialKeyPress(int key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::CallbackEventOnKeyPress(unsigned char key, int x, int y)
{
    if(m_graphics)
	m_graphics->HandleEventOnKeyPress(key);	
}


void Graphics::MousePosition(const int x, const int y, double *posX, double *posY)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble posZ;
    
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
    
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
    glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
    
    gluUnProject(winX, winY, winZ, modelview, projection, viewport, posX, posY, &posZ);
}

int main(int argc, char **argv)
{
    PseudoRandomSeed();
	
    if(argc < 3)
    {
	printf("missing arguments\n");		
	printf("  MotionPlanner <scene.txt> <time>\n");
	printf("where\n");
	printf("  <scene.txt> is one of the provided scene files\n");
	printf("  <time> is the maximum running time (in seconds) for the motion planner\n");
	
	return 0;		
    }

    Simulator sim(argv[1]);    
    MotionPlanner mp(&sim);    
    Graphics graphics(&mp);
    
    graphics.SetMotionPlannerMaxTime(atof(argv[2]));    
    graphics.HandleEventOnHelp();
    graphics.MainLoop();
    
    return 0;    
}
