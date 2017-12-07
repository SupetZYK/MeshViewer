#ifdef WIN32
	#include <windows.h>
	
#include "glut.h"
#else
#include <GL/glut.h>
#endif
#include "mesh.h"
#include "matrix.h"
#include "OpenGLProjector.h"
#include <fstream>//20110819


// Enumeration
enum EnumDisplayMode { HIDDENLINE, FLATSHADED, SMOOTHSHADED, COLORSMOOTHSHADED,DELETESELECTEDVERTEX,WIREFRAME};

enum Mode 
{ 
	Viewing, 
	Selection 
};
Mode currentMode = Viewing;


// variables
int displayMode = FLATSHADED;	// current display mode
int mainMenu, displayMenu;		// glut menu handlers
int winWidth, winHeight;		// window width and height
double winAspect;				// winWidth / winHeight;
int lastX, lastY;				// last mouse motion position
int currSelectedVertex = -1;         // current selected vertex
bool leftDown,leftUp, rightUp, rightDown, middleDown, middleUp, shiftDown;		// mouse down and shift down flags
double sphi = 90.0, stheta = 45.0, sdepth = 10;	// for simple trackball
double xpan = 0.0, ypan = 0.0;				// for simple trackball
double zNear = 1.0, zFar = 100.0;
double g_fov = 45.0;
Vector3d g_center;
double g_sdepth;
Mesh mesh;	// our mesh

// functions
void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax);
void InitGL();
void InitMenu();
void InitGeometry();
void MenuCallback(int value);
void ReshapeFunc(int width, int height);
void DisplayFunc();
void DrawWireframe();
void DrawHiddenLine();
void DrawFlatShaded();
void DrawSmoothShaded();
void DrawColorSmoothShaded();
void DrawSelectedVertices();
//void Partition();

void KeyboardFunc(unsigned char ch, int x, int y);
void MouseFunc(int button, int state, int x, int y);
void MotionFunc(int x, int y);
void SelectVertexByPoint();
void DeleteSelectedVertex(int vertex);


void SetBoundaryBox(const Vector3d & bmin, const Vector3d & bmax) {
	double PI = 3.14159265358979323846;
	double radius = bmax.Distance(bmin);
	g_center = 0.5 * (bmin+bmax);
	zNear    = 0.2 * radius / sin(0.5 * g_fov * PI / 180.0);
	zFar     = zNear + 2.0 * radius;
	g_sdepth = zNear + radius;
	zNear *= 0.1;
	zFar *= 10;
	sdepth = g_sdepth;
}

// init openGL environment
void InitGL() {
	GLfloat light0Position0[] = { 0, 1, 0, 0.0 }; 
	GLfloat light0Position1[] = { 0, -1, 0, 0.0 };
	GLfloat light0Position2[] = { 1, 0, 0, 0.0 };
	GLfloat light0Position3[] = { -1, 0, 0, 0.0 };
	GLfloat light0Position4[] = { 0, 0, -1, 0.0 };
	GLfloat light0Position5[] = { 0, 0, -1, 0.0 };
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(500, 500);
	glutCreateWindow("Mesh Viewer");
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glPolygonOffset(1.0, 1.0);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_DEPTH_TEST);
	//glEnable(GL_CULL_FACE);
	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glLightfv (GL_LIGHT0, GL_POSITION, light0Position0);
	GLfloat lightambient[] = { 0.5,0.5,0.5,1 };
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lightambient);
	//glEnable(GL_LIGHT0);

	glutReshapeFunc(ReshapeFunc);
	glutDisplayFunc(DisplayFunc);
	glutKeyboardFunc(KeyboardFunc);
	glutMouseFunc(MouseFunc);
	glutMotionFunc(MotionFunc);
}

// init right-click menu
void InitMenu() {
	displayMenu = glutCreateMenu(MenuCallback);
	glutAddMenuEntry("Wireframe", WIREFRAME);
	glutAddMenuEntry("Hidden Line", HIDDENLINE);
	glutAddMenuEntry("Flat Shaded", FLATSHADED);
	glutAddMenuEntry("Smooth Shaded", SMOOTHSHADED);
	glutAddMenuEntry("Color Smooth Shaded", COLORSMOOTHSHADED);
    glutAddMenuEntry("Delete Selected Vertex", DELETESELECTEDVERTEX);
	mainMenu = glutCreateMenu(MenuCallback);
	glutAddSubMenu("Display", displayMenu);
	glutAddMenuEntry("Exit", 99);
	glutAttachMenu(GLUT_RIGHT_BUTTON);//glutAttachMenu(GLUT_MIDDLE_BUTTON);
}

// init geometry (if no input argument is provided)
void InitGeometry() {
	const int VSIZE = 4;
	const int HESIZE = 12;
	const int FSIZE = 4;
	int i;
	Vertex *v[VSIZE];
	HEdge *he[HESIZE];
	Face *f[FSIZE];
	
	for (i=0; i<VSIZE; i++) {
		v[i] = new Vertex();
		mesh.vList.push_back(v[i]);
	}
	v[0]->SetPosition(Vector3d(0.0, 0.0, 0.0));
	v[1]->SetPosition(Vector3d(10.0, 0.0, 0.0));
	v[2]->SetPosition(Vector3d(0.0, 10.0, 0.0));
	v[3]->SetPosition(Vector3d(0.0, 0.0, 10.0));

	v[0]->SetNormal(Vector3d(-0.577, -0.577, -0.577));
	v[1]->SetNormal(Vector3d(0.0, -0.7, -0.7));
	v[2]->SetNormal(Vector3d(-0.7, 0.0, -0.7));
	v[3]->SetNormal(Vector3d(-0.7, -0.7, 0.0));

	for (i=0; i<FSIZE; i++) {
		f[i] = new Face();
		mesh.fList.push_back(f[i]);
	}

	for (i=0; i<HESIZE; i++) {
		he[i] = new HEdge();
		mesh.heList.push_back(he[i]);
	}
	for (i=0; i<FSIZE; i++) {
		int base = i*3;
		SetPrevNext(he[base], he[base+1]);
		SetPrevNext(he[base+1], he[base+2]);
		SetPrevNext(he[base+2], he[base]);
		SetFace(f[i], he[base]);
	}
	SetTwin(he[0], he[4]);
	SetTwin(he[1], he[7]);
	SetTwin(he[2], he[10]);
	SetTwin(he[3], he[8]);
	SetTwin(he[5], he[9]);
	SetTwin(he[6], he[11]);
	he[0]->SetStart(v[1]); he[1]->SetStart(v[2]); he[2]->SetStart(v[3]);
	he[3]->SetStart(v[0]); he[4]->SetStart(v[2]); he[5]->SetStart(v[1]);
	he[6]->SetStart(v[0]); he[7]->SetStart(v[3]); he[8]->SetStart(v[2]);
	he[9]->SetStart(v[0]); he[10]->SetStart(v[1]); he[11]->SetStart(v[3]);
	v[0]->SetHalfEdge(he[3]);
	v[1]->SetHalfEdge(he[0]);
	v[2]->SetHalfEdge(he[1]);
	v[3]->SetHalfEdge(he[2]);
}

// GLUT menu callback function
void MenuCallback(int value) {
	switch (value) {
	case DELETESELECTEDVERTEX:DeleteSelectedVertex(currSelectedVertex); break;
	case 99: exit(0); break;
	default: 
		displayMode = value;
		glutPostRedisplay();
		break;
	}
}

// GLUT reshape callback function
void ReshapeFunc(int width, int height) {
	winWidth = width;
	winHeight = height;
	winAspect = (double)width/(double)height;
	glViewport(0, 0, width, height);
	glutPostRedisplay();
}

// GLUT display callback function
void DisplayFunc() {
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(g_fov, winAspect, zNear, zFar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity(); 
	glTranslatef(xpan, ypan, -sdepth);
	glRotatef(-stheta, 1.0, 0.0, 0.0);
	glRotatef(sphi, 0.0, 1.0, 0.0);
	glTranslatef(-g_center[0], -g_center[1], -g_center[2]);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	switch (displayMode) {
	case WIREFRAME: DrawWireframe(); DrawSelectedVertices(); break;
	case HIDDENLINE: DrawHiddenLine(); DrawSelectedVertices(); break;
	case FLATSHADED: DrawFlatShaded(); DrawSelectedVertices(); break;
	case SMOOTHSHADED: DrawSmoothShaded(); DrawSelectedVertices(); break;
	case COLORSMOOTHSHADED: DrawColorSmoothShaded(); DrawSelectedVertices(); break;
	//case DELETESELECTEDVERTEX: DeleteSelectedVertex(currSelectedVertex); break;
	}
	glutSwapBuffers();
}

// Wireframe render function
void DrawWireframe() {
	HEdgeList heList = mesh.Edges();
	HEdgeList bheList = mesh.BoundaryEdges();
	glColor3f(0.3, 0.3, 1.0);
	glBegin(GL_LINES);
	size_t i;
	for (i=0; i<heList.size(); i++) {
		glVertex3dv(heList[i]->Start()->Position().ToArray());
		glVertex3dv(heList[i]->End()->Position().ToArray());
	}
	glColor3f(1, 0, 0);
	for (i=0; i<bheList.size(); i++) {
		glVertex3dv(bheList[i]->Start()->Position().ToArray());
		glVertex3dv(bheList[i]->End()->Position().ToArray());
	}
	
	glEnd();
}

// Hidden Line render function
void DrawHiddenLine() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT); 
	glEnable(GL_POLYGON_OFFSET_FILL);
	glColor3f(0, 0, 0);
	glBegin(GL_TRIANGLES);
	for (size_t i=0; i<fList.size(); i++) {
		Face *f = fList[i];
		const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
		const Vector3d & pos2 = f->HalfEdge()->End()->Position();
		const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
		glVertex3dv(pos1.ToArray());
		glVertex3dv(pos2.ToArray());
		glVertex3dv(pos3.ToArray());
	}
	glEnd();
	glDisable(GL_POLYGON_OFFSET_FILL);

	DrawWireframe();
}

// Flat Shaded render function
void DrawFlatShaded() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_FLAT); 
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i=0; i<fList.size(); i++) {
		if(fList[i]!=NULL && fList[i]->HalfEdge()->LeftFace()!=NULL )
		{
			Face *f = fList[i];
			const Vector3d & pos1 = f->HalfEdge()->Start()->Position();
			const Vector3d & pos2 = f->HalfEdge()->End()->Position();
			const Vector3d & pos3 = f->HalfEdge()->Next()->End()->Position();
			Vector3d normal = f->Normal_f();
			glColor3f((normal[0] + 1) / 2, (normal[1] + 1) / 2, (normal[2] + 1) / 2);
			glNormal3dv(normal.ToArray());
			glVertex3dv(pos1.ToArray());
			glVertex3dv(pos2.ToArray());
			glVertex3dv(pos3.ToArray());
		}
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

// Smooth Shaded render function
void DrawSmoothShaded() { 
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH); 
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES) ;
	for (size_t i=0; i<fList.size(); i++) {
		Face *f = fList[i];
		Vertex * v1 = f->HalfEdge()->Start();
		Vertex * v2 = f->HalfEdge()->End();
		Vertex * v3 = f->HalfEdge()->Next()->End();
		glNormal3dv(v1->Normal().ToArray());
		glVertex3dv(v1->Position().ToArray());
		glNormal3dv(v2->Normal().ToArray());
		glVertex3dv(v2->Position().ToArray());
		glNormal3dv(v3->Normal().ToArray());
		glVertex3dv(v3->Position().ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

void DrawColorSmoothShaded() {
	FaceList fList = mesh.Faces();
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glColor3f(0.4f, 0.4f, 1.0f);
	glBegin(GL_TRIANGLES);
	for (size_t i = 0; i<fList.size(); i++) {
		Face *f = fList[i];
		Vertex * v1 = f->HalfEdge()->Start();
		Vertex * v2 = f->HalfEdge()->End();
		Vertex * v3 = f->HalfEdge()->Next()->End();
		glColor3f(v1->Color()[0],v1->Color()[1],v1->Color()[2]);
		glNormal3dv(v1->Normal().ToArray());
		glVertex3dv(v1->Position().ToArray());
		glColor3f(v2->Color()[0], v2->Color()[1], v2->Color()[2]);
		glNormal3dv(v2->Normal().ToArray());
		glVertex3dv(v2->Position().ToArray());
		glColor3f(v3->Color()[0], v3->Color()[1], v3->Color()[2]);
		glNormal3dv(v3->Normal().ToArray());
		glVertex3dv(v3->Position().ToArray());
	}
	glEnd();
	glDisable(GL_LIGHTING);
}

 

// draw the selected ROI vertices on the mesh
void DrawSelectedVertices()
{
	if (currSelectedVertex == -1)return;
	glColor3f(1.0, 0.0, 0.0);
	glPointSize(10.0f);
	glEnable(GL_POINT_SMOOTH);
	glBegin(GL_POINTS);
	glVertex3dv(mesh.vList[currSelectedVertex]->Position().ToArray());
	glEnd();
	//glFlush();
	//cout<< "the selected vertex is drawn "<<endl;
}
//zyk
void SetBoundary(HEdgeList& hl)
{
	for (int i = 0; i < hl.size(); i++)
	{
		if (hl[i]->IsBoundary())continue;
		hl[i]->SetBoundary(true);
		hl[i]->Twin()->SetBoundary(true);
		for (short count = 0; count < 2; count++)
		{
			for (vector<HEdge*>::iterator it = mesh.heList.begin(); it != mesh.heList.end(); it++)
			{
				if (*it == hl[i] || *it == hl[i]->Twin())
				{
					mesh.bheList.push_back(*it);
					mesh.heList.erase(it);
					break;
				}
			}
		}
	}
}

//zyk
void DeleteHedgeList(HEdgeList& hl)
{
	for (int i = 0; i < hl.size(); i++)
	{
		if (!hl[i]->IsBoundary())
		{
			for (short count = 0; count < 2; count++)
			{
				for (vector<HEdge*>::iterator it = mesh.heList.begin(); it != mesh.heList.end(); it++)
				{
					if (*it == hl[i] || *it == hl[i]->Twin())
					{
						mesh.heList.erase(it);
						break;
					}
				}
			}

		}
		else
		{
			for (short count = 0; count < 2; count++)
			{
				for (vector<HEdge*>::iterator it = mesh.bheList.begin(); it != mesh.bheList.end(); it++)
				{
					if (*it == hl[i] || *it == hl[i]->Twin())
					{
						mesh.bheList.erase(it);
						break;
					}
				}
			}
		}
	}
}
//delete selected vertex and its incident faces and half-edges

void DeleteSelectedVertex(int vertex)
{
	if (vertex == -1)return;
	cout<< "the selected vertex is gone, its index is "<<vertex<<endl;
	OneRingHEdge ring(mesh.vList[vertex]);
	HEdge* curr = NULL;
	Face* pf = NULL;
	HEdgeList hl1,hl2;//hedge to be deleted
	while (curr = ring.NextHEdge())
	{
		pf = curr->LeftFace();
		hl1.push_back(curr);
		hl2.push_back(curr->Next());
		SetPrevNext(curr->Twin()->Prev(), curr->Next());
		//face member of class hedge
		curr->SetFace(NULL);
		curr->Twin()->SetFace(NULL);
		curr->Next()->SetFace(NULL);
		//hedge member of class vertex
		curr->End()->SetHalfEdge(curr->Next());

		if (pf != NULL)//left face exist
		{
			//now delete the face
			for (vector<Face*>::iterator it = mesh.fList.begin(); it != mesh.fList.end(); it++)
			{
				if (*it == pf)
				{
					mesh.fList.erase(it);
					break;
				}
			}
		}

	}
	DeleteHedgeList(hl1);
	SetBoundary(hl2);
	vector<Vertex*>::iterator itv = mesh.vList.begin() + vertex;
	mesh.vList.erase(itv);
	currSelectedVertex = -1;
	glutPostRedisplay();
} 


// GLUT keyboard callback function
void KeyboardFunc(unsigned char ch, int x, int y) { 
	switch (ch) { 
	case '3':
		cout<<"Delete Selected Vertex"<<endl;
		DeleteSelectedVertex(currSelectedVertex);
		break; 
	case '4':
		cout<<"Umbrella Smooth"<<endl;;
		mesh.UmbrellaSmooth();
		break;
	case '5':
        //cout<<"Implicit Umbrealla Smooth"<<endl;
		mesh.ImplicitUmbrellaSmooth();
        break;
	case '6':
        cout<<"0006"<<endl;
        break;

	case '1':	// key '1'
		currentMode = Viewing;
		cout << "Viewing mode" << endl;
		break;
	case '2':	// key '2'
		currentMode = Selection;
		cout << "Selection mode" << endl;
		break;
	case '9': 
		cout<<"0009"<<endl;
		break;
		//cout<<"test000 "<<currSelectedVertex<<endl;
	case 27:
		exit(0);
		break;
	}
	glutPostRedisplay();
}

// GLUT mouse callback function
void MouseFunc(int button, int state, int x, int y) {
	
	lastX = x;
	lastY = y;
	leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
	leftUp = (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP);
    rightDown = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN);
	rightUp = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_UP);
	middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
	middleUp = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_UP);
	shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);

	if (currentMode == Selection && state == GLUT_UP)
	{
	    if (middleUp)
		{
		    if (currSelectedVertex != -1)
		    {
		        mesh.Vertices()[currSelectedVertex]->SetFlag(0);
		        currSelectedVertex = -1;
		    }
		}
		else
		{
			SelectVertexByPoint();
			//DrawSelectedVertices();
		}
		
		//if (leftUp)
	    //{
		  //  SelectVertexByPoint(); 
			
		//}
        //if(middleUp)
		//{DeleteSelectedVertex(currSelectedVertex);}
        
		lastX = lastY = 0;
		glutPostRedisplay();
	}

}

// GLUT mouse motion callback function
void MotionFunc(int x, int y) {
	if (leftDown)
		if(!shiftDown) { // rotate
			sphi += (double)(x - lastX) / 4.0;
			stheta += (double)(lastY - y) / 4.0;
		} else { // pan
			xpan += (double)(x - lastX)*sdepth/zNear/winWidth;
			ypan += (double)(lastY - y)*sdepth/zNear/winHeight;
		}
	// scale
	if (middleDown) sdepth += (double)(lastY - y) / 10.0;

	lastX = x;
	lastY = y;
	glutPostRedisplay();
}


// select a mesh point
void SelectVertexByPoint()
{
	// get the selection position
	int x = lastX, y = winHeight - lastY;
	Vector3d u(x,y,0);
	OpenGLProjector projector; 
	//by zyk!
	//find point that z!=1
	u[2] = projector.GetDepthValue(u.X(), u.Y());
	if (u.Z() == 1)
	{
		u[0] = x + 10;
		u[1] = y + 10;
		u[2] = projector.GetDepthValue(u.X(), u.Y());
		if (u.Z() == 1)
		{
			u[0] = x + 10;
			u[1] = y - 10;
			u[2] = projector.GetDepthValue(u.X(), u.Y());
			if (u.Z() == 1)
			{
				u[0] = x - 10;
				u[1] = y - 10;
				u[2] = projector.GetDepthValue(u.X(), u.Y());
				if (u.Z() == 1)
				{
					u[0] = x - 10;
					u[1] = y + 10;
					u[2] = projector.GetDepthValue(u.X(), u.Y());
					if (u.Z() == 1)
					{
						cout << "No vertex!" << endl;
						return;
					}
				}
			}
		}
	}
	Vector3d objPnt = projector.UnProject(u);
	double dis = 999999;
	for (size_t i = 0; i < mesh.vList.size(); i++)
	{
		double dis1 = abs(mesh.vList[i]->Position().X() - objPnt.X()) + abs(mesh.vList[i]->Position().Y() - objPnt.Y()) + abs(mesh.vList[i]->Position().Z() - objPnt.Z());
		if (dis1 < dis)
		{
			dis = dis1;
			currSelectedVertex = i;
		}
	}
	mesh.vList[currSelectedVertex]->SetFlag(1);
	//cout << u << endl;
	//cout << objPnt << endl;
	cout << mesh.vList[currSelectedVertex]->Position() << endl;
}


// main function
int main(int argc, char **argv) {
	glutInit(&argc, argv);
	InitGL();
	InitMenu();
	if (argc>=2) mesh.LoadObjFile(argv[1]);
	else InitGeometry();
	SetBoundaryBox(mesh.MinCoord(), mesh.MaxCoord());
	/************************************************************************/
	/* activate the following code if you finish the corresponding functions*/
 	mesh.DisplayMeshInfo();
	/************************************************************************/
	glutMainLoop();
	return 0;
}


