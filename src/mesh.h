#ifndef __MESH_H__
#define __MESH_H__

#include <cstdlib>
#include <vector>
#include "vector3.h"

// classes
class HEdge;
class Vertex;
class Face;
class Mesh;
class OneRingHEdge;
class OneRingVertex;
class FaceEnumeration;

// types
typedef std::vector<HEdge*> HEdgeList;
typedef std::vector<Vertex*> VertexList;
typedef std::vector<Face*> FaceList;

// other helper functions
inline void SetPrevNext(HEdge *e1, HEdge *e2);
inline void SetTwin(HEdge *e1, HEdge *e2);
inline void SetFace(Face *f, HEdge *e);

////////// class HEdge //////////
class HEdge {
private:
	HEdge *twin, *prev, *next;	// twin/previous/next half edges
	Vertex *start;//, *off_pos;	// start vertex, //20110827
	Face *face;					// left face
	bool boundary;				// flag for boundary edge

    bool valid;
    
	// it's for count boundary loop use, 
	// you can use it freely for marking in boundary loop counting 
	// and connected component counting
	bool flag;	  

public:
	/////////////////////////////////////
	// constructor
	double dist;// for marking the propagation distance
	double angle;//the angle with respect to which it subtend to the current edge
	double left_angle;
	HEdge(bool b=false) { 
		boundary = b;
		twin = prev = next = NULL; 
		start = NULL;
		face = NULL;
		flag = false;
		valid = true;  
	}
	/////////////////////////////////////
	// access functions  
	HEdge*  Twin() const { return twin; }
	HEdge*  Prev() const { return prev; }
	HEdge*  Next() const { return next; }
	Vertex* Start() const { return start; }
	//Vertex* Off_pos() const { return off_pos; } //20110827
	Vertex* End() const { return next->start; } // for convenience
	Face*   LeftFace() const { return face; }
	bool    Flag() const { return flag; }

	HEdge*  SetTwin(HEdge* e) { return twin = e; }
	HEdge*  SetPrev(HEdge* e) { return prev = e; }
	HEdge*  SetNext(HEdge* e) { return next = e; }
	Vertex* SetStart(Vertex* v) { return start = v; }
	//Vertex* SetOff_pos(Vertex* v) { return off_pos= v; }//20110827
	Face*   SetFace(Face* f) { return face = f; }
	bool    SetFlag(bool b) { return flag = b; }
	bool    SetValid(bool b) { return valid = b; }
	bool    IsBoundary() const { return boundary; }
    bool    IsValid() const { return valid; }
	//zyk
	void SetBoundary(bool b){ boundary = b; }
};

////////// class OneRingHEdge //////////
// this class is use for access the neighbor HALF EDGES
// of a given vertex, please see Vertex::IsBoundary() for its usage
class OneRingHEdge {
private:
	HEdge *start, *next;
public:
	OneRingHEdge(const Vertex * v);	// constructor
	HEdge * NextHEdge();			// iterator
};

////////// class OneRingVertex //////////
// this class is use for access the neighbor VERTICES 
// of a given vertex, please see Vertex::Valence() for its usage
class OneRingVertex {
private:
	OneRingHEdge ring;
public:
	OneRingVertex(const Vertex * v) : ring(v) { }	// constructor
	Vertex * NextVertex() { HEdge *he=ring.NextHEdge(); return (he)?he->End():NULL; } // iterator
};

////////// class Vertex //////////
class Vertex {
private:
	Vector3d position;	// position (x,y,z) in space
	Vector3d normal;	// normal vector for smooth shading rendering
	Vector3d color;		// color value for curvature displaying

    Vector3d cur_pos;   // for bisector: the position the current vertex is to be moved to
	Vector3d cur_pos1;   // for vertex: the position the current vertex is to be moved to
	Vector3d cur_pos001;  //for split event
	Vector3d cur_pos002;  //for edge event
	Vector3d cur_pos007;  //for marking the bisectors
    
    Vertex* leftv;
	Vertex* rightv;
	HEdge *l_edge;//0915
	HEdge *r_edge;//0915


	HEdge *he;			// one of half edge starts with this vertex
    HEdge *pass_edge; // mark the edge that the front of the vertex just crosses
    HEdge *split_edge; //20110827


	int index;			// index in the Mesh::vList, DO NOT UPDATE IT
    int flag;           // 0 for unselected, 1 for selected
    bool valid;
	
public:
	int visit;           // indicator for CountBoundaryLoops(), 2 if the vertex is unvisited and 3 otherwise
	short dfs_visit = 0;//for DFS visit, 0 for not visited, 1 for detected, 2 for done
	int mark;
	int event_type; //10 for bisector-edge; 0 for edge-edge; 1 for split event; 2 for edge event
	double sum_angle;//mark the total degree of angles (on te unburned side) incident to the vertex
	double dist;// for marking the propagation distance
    double H; //mean curvature
	double G; //Gaussian curvature
	double K1; //max principal curvature
	double K2; //min principal curvature

	//zyk
	Vector3d LaplacianCoordinate;
	vector<HEdge*> adjHEdges; // for reading object only, do not use it in other place

	// constructors
	Vertex() : he(NULL), flag(0), valid(true) { }
	Vertex(const Vector3d & v) : he(NULL), position(v), flag(0), valid(true) { }
	Vertex(double x, double y, double z) : he(NULL), position(x,y,z), flag(0), valid(true) { }

	// access functions
	const Vector3d & Position() const { return position; }

	const Vector3d & Cur_pos() const { return cur_pos; } //anton 20110810

	const Vector3d & Cur_pos1() const { return cur_pos1; } //anton 20110810

	const Vector3d & Cur_pos001() const { return cur_pos001; } //anton 20110822

	const Vector3d & Cur_pos002() const { return cur_pos002; } //anton 20110822

	const Vector3d & Cur_pos007() const { return cur_pos007; } //anton 20110822

	Vertex*  Left_v() const { return leftv; }

	Vertex*  Right_v() const { return rightv; }

	const Vector3d & Normal() const { return normal; }
	const Vector3d & Color() const { return color; }
	HEdge *HalfEdge() const { return he; }
	HEdge *L_Edge() const { return l_edge; }
	HEdge *R_Edge() const { return r_edge; }
	HEdge *Split_Edge() const { return split_edge; }; //20110827
	//HEdge *HalfEdge() const { return he; }
	int Index() const { return index; }
	int Flag() const { return flag; }
	const Vector3d & SetPosition(const Vector3d & p) { return position = p; }

    const Vector3d & SetCur_pos(const Vector3d & pp) { return cur_pos = pp; } //anton 20110810

	const Vector3d & SetCur_pos1(const Vector3d & pp) { return cur_pos1 = pp; } //anton 20110810

	const Vector3d & SetCur_pos001(const Vector3d & pp) { return cur_pos001 = pp; } //anton 20110822

	const Vector3d & SetCur_pos002(const Vector3d & pp) { return cur_pos002 = pp; } //anton 20110822

	const Vector3d & SetCur_pos007(const Vector3d & pp) { return cur_pos007 = pp; } //anton 20110822

	Vertex*   SetLeft_v(Vertex* ll) { return leftv = ll; }

	Vertex*   SetRight_v(Vertex* rr) { return rightv = rr; }


	const Vector3d & SetNormal(const Vector3d & n) { return normal = n; }
	const Vector3d & SetColor(const Vector3d & c) { return color = c; }
	HEdge * SetHalfEdge(HEdge * he) { return Vertex::he = he; }
    HEdge * SetL_Edge(HEdge * le) { return Vertex::l_edge = le; }//0915
	HEdge * SetR_Edge(HEdge * re) { return Vertex::r_edge = re; }//0915
	HEdge * SetSplit_Edge(HEdge * he) { return Vertex::split_edge = he; }//999
	int SetIndex(int index) { return Vertex::index = index; }

    int SetFlag(int value) { return Vertex::flag = value; }
    
    bool IsValid() const { return valid; }
    bool SetValid(bool b) { return valid = b; }
    
	// check for boundary vertex
	bool IsBoundary() const {
		OneRingHEdge ring(this);
		HEdge *curr = NULL;
		while (curr=ring.NextHEdge()) if (curr->IsBoundary()) return true;
		return false;
	}

	// compute the valence (# of neighbor vertices)
	int Valence() const {
		int count = 0;
		OneRingVertex ring(this);
		Vertex *curr = NULL;
		while (curr=ring.NextVertex()) count++;
		return count;
	}
	
};

////////// class Face //////////
class Face {
private:
	HEdge * he;
    bool valid;
	Vector3d normal_f;//1007
public:
	// constructor
	Face() : he(NULL), valid(true) { }
    
	const Vector3d & Normal_f() const { return normal_f; }
    const Vector3d & SetNormal_f(const Vector3d & n) { return normal_f = n; }
    //Vector3d normal_f; //1006

    
	// access function
	HEdge * HalfEdge() const { return he; }
	HEdge * SetHalfEdge(HEdge * he) { return Face::he = he; }
	
	// check for boundary face
	bool IsBoundary() {
		HEdge *curr = he;
		do {
			if (curr->Twin()->IsBoundary()) return true;
			curr = curr->Next();
		} while (curr != he);
		return false;
	}
	bool SetValid(bool b) { return valid = b; }
	bool IsValid() const { return valid;}
};

////////// class Mesh //////////
class Mesh {
public:
	HEdgeList heList;		// list of NON-boundary half edges
	HEdgeList bheList;		// list of boundary half egdes
	VertexList vList;		// list of vertices
	FaceList fList;			// list of faces

	// constructor & destructors
	Mesh() { }
	~Mesh() { Clear(); }

	// access functions
	const HEdgeList Edges() const { return heList; }
	const HEdgeList BoundaryEdges() const { return bheList; }
	const VertexList Vertices() const { return vList; }
	const FaceList Faces() const { return fList; }

	// functions for loading obj files,
	// you DO NOT need to understand and use them
	void AddVertex(Vertex *v) { vList.push_back(v); }
	void AddFace(int v1, int v2, int v3);
   
	void Clear() {

		//cout<< "CANNOT USE THE ORIGINAL CODE IF WE NEED TO GUARANTEE CONSTANT TIME DELETIONG OPERATION (MORE PRECISELY, IN TIME PROPORTIONAL TO THE DEGREE OF THE SELECTED VERTEX"<<endl;
        
		size_t i;
		for (i=0; i<heList.size(); i++) if(heList[i]!=NULL) delete heList[i];
		for (i=0; i<bheList.size(); i++) if(bheList[i]!=NULL) delete bheList[i];
		for (i=0; i<vList.size(); i++) if(vList[i]!=NULL) delete vList[i];
		for (i=0; i<fList.size(); i++) if(fList[i]!=NULL) delete fList[i];
		heList.clear();
		bheList.clear();
		vList.clear();
		fList.clear();
		
	}
	

	Vector3d MinCoord() const {
		Vector3d minCoord;
		for (size_t i=0; i<vList.size(); i++)
			minCoord = minCoord.Min(vList[i]->Position());
		return minCoord;
	}

	Vector3d MaxCoord() const {
		Vector3d maxCoord;
		for (size_t i=0; i<vList.size(); i++)
			maxCoord = maxCoord.Max(vList[i]->Position());
		return maxCoord;
	}

	bool LoadObjFile(const char * filename);

	/************************************************************************/
	/* please implement the following functions */
	void DisplayMeshInfo();
	void ComputeVertexNormals();
	void ComputeVertexCurvatures();
	void UmbrellaSmooth();
	void ImplicitUmbrellaSmooth();



	//void ComputeSS(); //20110816



	/************************************************************************/

	// additional helper functions
	// implement them if in needed
	int CountBoundaryLoops();
	int CountConnectedComponents();
	void DFSVisit(Vertex * v);

	//void DeleteSelectedVertex(int vertex);
	//void Deletefaces(HEdge *v_del);
	bool check_DFS_finished(){
		size_t i;
		for (i = 0; i <vList.size(); i++)
		{
			if (vList[i]->dfs_visit != 2)
			{
				return false;
			}
		}
		return true;
	}
};

// other helper functions
inline void SetPrevNext(HEdge *e1, HEdge *e2) { e1->SetNext(e2); e2->SetPrev(e1); }
inline void SetTwin(HEdge *e1, HEdge *e2) { e1->SetTwin(e2); e2->SetTwin(e1); }
inline void SetFace(Face *f, HEdge *e) { f->SetHalfEdge(e); e->SetFace(f); }

#endif // __MESH_H__