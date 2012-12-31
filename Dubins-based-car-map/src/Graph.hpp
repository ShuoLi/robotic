#ifndef GRAPH_H_
#define GRAPH_H_

/*
 * This file has declarations for classes used to represent the graph
 */

#include <vector>
#include <stack>
#include <string>
#include <iostream>
#include "Utils.hpp"

using namespace std;

//enum for the status of a node
enum Status {
	NOT_VISITED,
    VISITED
};

//forward declaration
class Node;

//An object of this class represents an edge in the graph.
class Edge
{
private:
    Node *orgNode;//the originating vertex
    Node *dstNode;//the destination vertex
    double cost;//cost of the edge
    vector<Point*> edge_path;
    
public:
    Edge(Node *firstNode, Node *secNode, double inCost)
    {
        orgNode = firstNode;
        dstNode = secNode;
        cost = inCost;
    }
    
    Node* getDstNode()
    {
        return dstNode;
    }
    
    Node* getOrgNode()
    {
        return orgNode;
    }
    
    double getCost()
    {
        return cost;
    }
    
    void setEdgeV(vector<Point*> e){
        edge_path=e;
    }
    
    vector<Point*> getEdgeV(){
        return edge_path;
    }
};

//An object of this class holds a vertex of the graph
class Node
{
private:
    Point location;//location of this node
    Node* parentNode;// store the parent node
    vector<Edge> adjNodeList;//list of outgoing edges for this vertex
    enum Status status;//used in dfs to mark the node visited
    double depth;
    
public:
    Node(Point loc)
    {
        location = Point(loc.x,loc.y,loc.theta);
        status = NOT_VISITED;
        depth = 0;
    }
    
    //do not del the adj nodes here...they will be deleted by graph destructor
    ~Node()
    {
        adjNodeList.clear();
    }
    
    enum Status getStatus()
    {
        return status;
    }
    
    void setStatus(enum Status st)
    {
        status = st;
    }
        
    Point getLoc(){
        return location;
    }
    
    void setParentNode(Node* parent){
        parentNode = parent;
    }
    
    void setDepth(double tdepth){
//        cout<<" "<<tdepth<<" ";
        depth = tdepth;
    }
    
    Node* getParentNode(){
        return parentNode;
    }
    
    void addAdjNode(Node *adj, double cost)
    {
        //create an edge with 'this' as the originating node and adj as the destination node
        Edge newEdge(this, adj, cost);
        adjNodeList.push_back(newEdge);
    }
    
    vector<Edge>& getAdjNodeList()
    {
        return adjNodeList;
    }
    
    //displays all adjacent verticies of this vertex
    void displayList()
    {
        string edgeOp = " -> " ;
        for(int i=0 ; i < adjNodeList.size() ; i++)
        {
            Edge edg = adjNodeList[i];
            cout << "("<<location.x<<","<<location.y<<")" << " -> (" << edg.getDstNode()->getLoc().x << ","<<edg.getDstNode()->getLoc().y << ")" <<endl ;
        }
        
    }
    double getDepth(){
        return depth;
    }
};

//An object of class graph holds a directed graph
class Graph
{
public:
    Node* root;// root of this tree/graph
    vector<Node*> nodeList;//list of verticies
    vector<Edge*> edgeList;
    bool foundCycle;//true if a cycle is found, false otherwise
    int desiredCycSize;
    
    void clearVisited() 
    {
        for(int i = 0; i < nodeList.size() && !foundCycle ; i++)
        {
            nodeList[i]->setStatus(NOT_VISITED);
        }
    }
    
    void addNewNode(Node *nNode)
    {
        nodeList.push_back(nNode);
    }
    
    Graph(){
        root = NULL;
        foundCycle = false;
    }    
    ~Graph()
    {
        //free mem allocated to verticies
        for(int i=0 ; i < nodeList.size() ; i++)
            delete nodeList[i];
        nodeList.clear();
    }
    
    void addRoot(Node* rt)
    {
        root = rt;
        addNewNode(rt);
    }

    void displayGraph()
    {
        for(int i=0 ; i < nodeList.size() ; i++)
        {
            nodeList[i]->displayList();	
        }
    }
    
    Node* getClosestNeighbor(Point q,double sqrtDist){ // return the cloest node to point q in T.
        double minDist = 9999;
        Node* closetNode = NULL;
        for (int i=0; i<nodeList.size(); i++) {
            double treeDepth = nodeList[i]->getDepth();
            double dist = nodeList[i]->getLoc().distTo(q);
            
//            dist = (double)dist+treeDepth+4*pow((q.theta-nodeList[i]->getLoc().theta),2);
            double randomDepth = PseudoRandomUniformReal(0,treeDepth);
//            dist = (double)dist+randomDepth;
                        dist = (double)dist+randomDepth+2.5*abs((q.theta-nodeList[i]->getLoc().theta));

            //            cout<<"treeDepth:"<<treeDepth<<"randomDepth: "<<randomDepth<<":"<<dist<<endl;//":"<<15*pow((q.theta-nodeList[i]->getLoc().theta),2)<<":"<<dist<<endl;
            if(minDist > dist){
                closetNode = nodeList[i];
                minDist = dist;
            }
        }
        return closetNode;
    }
    
    Node* findNodeByLoc(Point loc)
    {
        for(int i = 0 ; i < nodeList.size() ; i++)
        {
            if(nodeList[i]->getLoc().equal(loc))
                return nodeList[i];
        }
//        cout<<"no node in this location"<<endl;
        return NULL;
    }
    
    Node* addNodeAndEdge(Node* nNear,Point qNew,vector<Point*> toSample){// add node and edge, add node by location Point qNew, add edge by nNear->Node(qNew)
        Node *v = findNodeByLoc(qNew);
        if (v==NULL) {//only add node when this node not in T
            v = new Node(qNew);
            v->setParentNode(nNear);
            double pathLength_qNear2qNew = nNear->getDepth();
            for (int i=0; i<toSample.size()-1; i++) {
                pathLength_qNear2qNew +=  dist(*toSample[i], *toSample[i+1]);
            }
            v->setDepth(pathLength_qNear2qNew);
            addNewNode(v);
            double cost = nNear->getLoc().distTo(qNew);
            Edge* e = new Edge(nNear, v, cost);
            e->setEdgeV(toSample);
            edgeList.push_back(e);

//            nNear->addAdjNode(v,cost);
        }
        return v;
    }
    
    vector<Edge*> getEdgeList(){
        return edgeList;
    }
    
    Edge* getEdge(Node* pNode, Node* currentNode){
        for (int i=0; i<edgeList.size(); i++) {
            if(edgeList[i]->getOrgNode()->getLoc().equal(pNode->getLoc()) && 
               edgeList[i]->getDstNode()->getLoc().equal(currentNode->getLoc())){
                return edgeList[i];
            }
        }
    }
    
    double dist(Point a, Point b){
        return sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    }
};
#endif