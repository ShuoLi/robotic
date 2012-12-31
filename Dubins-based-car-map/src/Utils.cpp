#include "Utils.hpp"

bool IsPointInsidePolygon(const double p[], const int n, const double poly[])
{
    int i, i1, lcross = 0, rcross = 0;
    double px = p[0];
    double py = p[1];
    double x = 0.0;
    bool testr;
    bool testl;
    
    for(i = 0; i < n; i++)
    {
	if(poly[2 * i] == px && poly[2 * i + 1] == py)
	    return true;
	
	i1 = (i + n - 1) % n;
	
	testr = (poly[2 * i + 1] > py) != (poly[2 * i1 + 1] > py);
	testl = (poly[2 * i + 1] < py) != (poly[2 * i1 + 1] < py);
	
	if(testr || testl)
	    x = (poly[2 * i] * poly[2 * i1 + 1] - 
		 poly[2 * i1] * poly[2 * i + 1] + 
		 py * (poly[2 * i1] - poly[2 * i])) /
		(poly[2 * i1 + 1] - poly[2 * i + 1]);
	
	if(testr && x > px)
	    rcross++;
	if(testl && x < px)
	    lcross++;
    }
    
    if( (rcross & 1) != (lcross & 1) )
	return true;
    return (rcross & 1);  
}

bool SegmentSegmentIntersection(const double p0[],
				const double p1[],
				const double p2[],
				const double p3[])
{
    //code from Graphics Gems V
    
    double x1 = p0[0], x2 = p1[0], x3 = p2[0], x4 = p3[0],
	y1 = p0[1], y2 = p1[1], y3 = p2[1], y4 = p3[1];
    
    double Cx,Ay,By,Cy,d,e,f;
    double x1lo,x1hi,y1lo,y1hi;
    double Ax = x2 - x1;
    double Bx = x3 - x4;
    
    if(Ax < 0) { x1lo = x2; x1hi = x1; } 
    else       { x1hi = x2; x1lo = x1; }
    
    if(Bx > 0) { if(x1hi < x4 || x3 < x1lo) return false; } 
    else       { if(x1hi < x3 || x4 < x1lo) return false; }
    
    Ay = y2 - y1;
    By = y3 - y4;
    
    /* Y bound box test*/
    if(Ay < 0)  { y1lo = y2; y1hi = y1; } 
    else        { y1hi = y2; y1lo = y1; }
    
    if(By > 0)  { if(y1hi < y4 || y3 < y1lo) return false; } 
    else          if(y1hi < y3 || y4 < y1lo) return false;
    
    f  = Ay * Bx - Ax * By;					/* both denominator*/
    if(f == 0)
	return false;
    
    Cx = x1 - x3;
    Cy = y1 - y3;
    
    d  = By * Cx - Bx * Cy;					/* alpha numerator*/ 
    /* alpha tests*/
    if(f > 0) { if(d < 0 || d > f) return false; } 
    else if(d > 0 || d < f) return false;
    
    e = Ax * Cy - Ay * Cx;					/* beta numerator*/
    /* beta tests*/
    if(f > 0) { if(e < 0 || e > f) return false; } 
    else if(e > 0 || e < f) return false;
    
    return true;	    
}

bool SegmentPolygonIntersection(const double p0[],
				const double p1[],
				const int    n,
				const double poly[])
{
    for(int i = 0; i < n - 1; ++i)
	if(SegmentSegmentIntersection(p0, p1, &(poly[2 * i]), &(poly[2 * i + 2])))
	    return true;
    return SegmentSegmentIntersection(p0, p1, &(poly[2 * n - 2]), &(poly[0]));
}

bool PolygonPolygonCollision(const int n,
			     const double poly[],
			     const int n2,
			     const double poly2[])
{
    if(IsPointInsidePolygon(poly, n2, poly2) ||
       IsPointInsidePolygon(poly2, n, poly))
	return true;
    for(int i = 0; i < n - 1; ++i)
	if(SegmentPolygonIntersection(&poly[2 * i], &poly[2 * i + 2], n2, poly2))
	    return true;
    return SegmentPolygonIntersection(&poly[2 * n - 2], &poly[0], n2, poly2);
}
