#include "mesh.h"
#include "matrix.h"
#include <cstring>
#include <iostream>
#include <strstream>
#include <fstream>
#include <cmath>
#include <float.h>
using namespace std;

/////////////////////////////////////////
// helping inline functions
inline double Cot(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
	Vector3d v1 = p1 - p2;
	Vector3d v2 = p3 - p2;

	v1 /= v1.L2Norm();
	v2 /= v2.L2Norm();
	double tmp = v1.Dot(v2);
	return 1.0 / tan(acos(tmp));
}

inline double Area(const Vector3d & p1, const Vector3d & p2, const Vector3d & p3) {
	Vector3d v1 = p2 - p1;
	Vector3d v2 = p3 - p1;
	return v1.Cross(v2).L2Norm() / 2.0;
}


/////////////////////////////////////////
// implementation of OneRingHEdge class
OneRingHEdge::OneRingHEdge(const Vertex * v) {
	if (v == NULL) start = next = NULL;
	else start = next = v->HalfEdge();
}

HEdge * OneRingHEdge::NextHEdge() {
	HEdge *ret = next;
	if (next && next->Prev()->Twin() != start)
		next = next->Prev()->Twin();
	else
		next = NULL;
	return ret;
}

/////////////////////////////////////////
// implementation of Mesh class
//
// function AddFace
// it's only for loading obj model, you do not need to understand it
void Mesh::AddFace(int v1, int v2, int v3) {
	int i;
	HEdge *he[3], *bhe[3];
	Vertex *v[3];
	Face *f;

	// obtain objects
	for (i=0; i<3; i++) he[i] = new HEdge();
	for (i=0; i<3; i++) bhe[i] = new HEdge(true);
	v[0] = vList[v1];
	v[1] = vList[v2];
	v[2] = vList[v3];
	f = new Face();

	// connect prev-next pointers
	SetPrevNext(he[0], he[1]);
	SetPrevNext(he[1], he[2]);
	SetPrevNext(he[2], he[0]);
	SetPrevNext(bhe[0], bhe[1]);
	SetPrevNext(bhe[1], bhe[2]);
	SetPrevNext(bhe[2], bhe[0]);

	// connect twin pointers
	SetTwin(he[0], bhe[0]);
	SetTwin(he[1], bhe[2]);
	SetTwin(he[2], bhe[1]);

	// connect start pointers for bhe
	bhe[0]->SetStart(v[1]);
	bhe[1]->SetStart(v[0]);
	bhe[2]->SetStart(v[2]);
	for (i=0; i<3; i++) he[i]->SetStart(v[i]);

	// connect start pointers
	// connect face-hedge pointers
	for (i=0; i<3; i++) {
		v[i]->SetHalfEdge(he[i]);
		v[i]->adjHEdges.push_back(he[i]);
		SetFace(f, he[i]);
	}
	v[0]->adjHEdges.push_back(bhe[1]);
	v[1]->adjHEdges.push_back(bhe[0]);
	v[2]->adjHEdges.push_back(bhe[2]);

	// merge boundary if needed
	for (i=0; i<3; i++) {
		Vertex *start = bhe[i]->Start();
		Vertex *end   = bhe[i]->End();
		for (size_t j=0; j<end->adjHEdges.size(); j++) {
			HEdge *curr = end->adjHEdges[j];
			if (curr->IsBoundary() && curr->End()==start) {
				SetPrevNext(bhe[i]->Prev(), curr->Next());
				SetPrevNext(curr->Prev(), bhe[i]->Next());
				SetTwin(bhe[i]->Twin(), curr->Twin());
				bhe[i]->SetStart(NULL);	// mark as unused
				curr->SetStart(NULL);	// mark as unused
				break;
			}
		}
	}

	// finally add hedges and faces to list
	for (i=0; i<3; i++) heList.push_back(he[i]);
	for (i=0; i<3; i++) bheList.push_back(bhe[i]);
	fList.push_back(f);
}

// function LoadObjFile
// it's only for loading obj model, you do not need to understand it
bool Mesh::LoadObjFile(const char *filename) {
	if (filename==NULL || strlen(filename)==0) return false;
	ifstream ifs(filename);
	if (ifs.fail()) return false;

	Clear();

	char buf[1024], type[1024];
	do {
		ifs.getline(buf, 1024);
		istrstream iss(buf);
		iss >> type;

		// vertex
		if (strcmp(type, "v") == 0) {
			double x, y, z;
			iss >> x >> y >> z;			
            AddVertex(new Vertex(x,y,z));

		}
		// face
		else if (strcmp(type, "f") == 0) {
			int index[3];
			iss >> index[0] >> index[1] >> index[2];
			AddFace(index[0]-1, index[1]-1, index[2]-1);
		}
	} while (!ifs.eof());
	ifs.close();

	size_t i;
	Vector3d box = this->MaxCoord() - this->MinCoord();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() / box.X());

	Vector3d tot;
	for (i=0; i<vList.size(); i++) tot += vList[i]->Position();
	Vector3d avg = tot / vList.size();
	for (i=0; i<vList.size(); i++) vList[i]->SetPosition(vList[i]->Position() - avg);

	HEdgeList list;
	for (i=0; i<bheList.size(); i++)
		if (bheList[i]->Start()) list.push_back(bheList[i]);
	bheList = list;

	for (i=0; i<vList.size(); i++) 
	{
		vList[i]->adjHEdges.clear(); 
		vList[i]->SetIndex((int)i);
		vList[i]->SetFlag(0);
	}

	return true;
}

void Mesh::DisplayMeshInfo()
{
	int NO_VERTICES = (int)vList.size();
	int NO_FACES = (int)fList.size();
	int NO_HEDGES = (int)heList.size()+(int)bheList.size();
	int NO_B_LOOPS = CountBoundaryLoops();
	int NO_COMPONENTS = CountConnectedComponents();
	int NO_GENUS = NO_COMPONENTS - (NO_VERTICES - NO_HEDGES/2 +  NO_FACES + NO_B_LOOPS)/2;
	ComputeVertexNormals();
	ComputeVertexCurvatures();
	cout << "Number Vertices " << NO_VERTICES << endl;
	cout << "Number faces " << NO_FACES << endl;
	cout << "Number half edges " << NO_HEDGES << endl;
	cout << "Number edges" << NO_HEDGES / 2 << endl;
	cout << "Number Boundary Loops " << NO_B_LOOPS << endl;
	cout << "Number components " << NO_COMPONENTS << endl;
	cout << "is dfs visit finished? " << check_DFS_finished() << endl;
	//cout << "NO_GENUS" << NO_GENUS << endl;
}

int Mesh::CountBoundaryLoops()
{
    int no_loop =0;//count the number of boundary loops
	size_t i;
    for (i=0; i< bheList.size(); i++)
	{
	   HEdge *cur=bheList[i];
       HEdge *nex=cur;
	   while(nex->Start()->visit!=1)
	   {
	     nex->Start()->visit=1;
         nex=nex->Next();
		 if (nex==cur)
		 {no_loop++;break;} 
	   }
	}
	return no_loop;
}

int Mesh::CountConnectedComponents()
{
	int no_component =0;
	//cout<<"number of Connected components"<<endl;
	size_t i;
	for (i = 0; i < vList.size(); i++)
	{
		Vertex*curr = vList[i];
		if (curr->dfs_visit == 0)
		{
			DFSVisit(curr);
			no_component += 1;
		}
	}
	return no_component;
}

void Mesh::DFSVisit(Vertex * v)
{
	//cout<<"DFS"<<endl;
	if (v->dfs_visit != 0)return;
	v->dfs_visit = 1;
	OneRingVertex ring(v);
	Vertex* curr = NULL;
	while (curr=ring.NextVertex())
	{
		if (curr->dfs_visit == 0)
		{
			DFSVisit(curr);
		}
	}
	v->dfs_visit = 2;
}


// -------------------------------------------------------
// DO NOT TOUCH THE FOLLOWING FOR NOW
// -------------------------------------------------------
//zyk
void Mesh::ComputeVertexNormals()
{
	//calculate face normals first and add it to each vertex
	for (size_t i = 0; i < fList.size(); i++)
	{
		if (fList[i] != NULL && fList[i]->HalfEdge()->LeftFace() != NULL)
		{
			Face *f = fList[i];
			Vertex*v1, *v2, *v3;
			v1 = f->HalfEdge()->Start();
			v2 = f->HalfEdge()->End();
			v3 = f->HalfEdge()->Next()->End();
			const Vector3d & pos1 = v1->Position();
			const Vector3d & pos2 = v2->Position();
			const Vector3d & pos3 = v3->Position();
			Vector3d normal = (pos2 - pos1).Cross(pos3 - pos1);
			v1->SetNormal(v1->Normal() + normal);
			v2->SetNormal(v2->Normal() + normal);
			v3->SetNormal(v3->Normal() + normal);
			normal /= normal.L2Norm();
			f->SetNormal_f(normal);//1007
		}
	}
	for (size_t i = 0; i < vList.size(); i++)
	{
		if (!vList[i]->IsBoundary()) //for interior vertices only
		{
			Vector3d n = vList[i]->Normal();
			vList[i]->SetNormal(n / n.L1Norm());
		}
	}
}

void Mesh::UmbrellaSmooth() 
{

	//cout<<"Umbrella Smooth starts..."<<endl;
	/*for (size_t i = 0; i < vList.size(); i++)
	{
		if (vList[i]->IsBoundary())continue;
		vList[i]->SetPosition(vList[i]->Position() + 0.2*vList[i]->LaplacianCoordinate);
	}
	ComputeVertexNormals();
	ComputeVertexCurvatures();*/
	for (size_t i = 0; i < vList.size(); i++)
	{
		Vertex *v= vList[i];
		if (v->IsBoundary())continue;
		OneRingHEdge ring(v);
		HEdge*curr = NULL;
		double cot_sum = 0;
		double cot_all_sum = 0;
		double v_A = 0;
		double v_A_1 = 0;
		Vector3d color_v(0.0,0.0,0.0);
		while (curr = ring.NextHEdge())
		{
			if (!curr->IsBoundary())
			{
				const Vector3d & pos1 = v->Position();//p1
				const Vector3d & pos2 = curr->End()->Position();//p2
				const Vector3d & pos3 = curr->Prev()->Start()->Position();//p3
				const Vector3d & pos4 = curr->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
				cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
				cot_all_sum += cot_sum;
				color_v = color_v + cot_sum*(pos3 - pos1);
				v_A_1 = Area(pos1, pos2, pos3);
				v_A = v_A + v_A_1;//SUM
			}
		}

		v->SetPosition(v->Position() + 0.00005*color_v / v_A);
	}
	
	//cout << "Umbrella Smooth Ends!" << endl;
}
//void Mesh::ImplicitUmbrellaSmooth()
//{
//    //cout<< "Implicit Umbrella Smooth starts..."<<endl;
//	//first construct a empty sparse matrix
//	size_t n = vList.size();
//	//buffer of vertex x,y,and, z value
//	double *X, *Y, *Z, *X_out,*Y_out,*Z_out;
//	X = new double[n];
//	Y = new double[n];
//	Z = new double[n];
//	X_out = new double[n];
//	Y_out = new double[n];
//	Z_out = new double[n];
//	Matrix A(n, n);
//	//give a lamda
//	double lamda = 0.95;
//	//Iterite and assign value
//	for (size_t i = 0; i < n; i++)
//	{
//		Vertex *v = vList[i];
//		X[i] = v->Position()[0];
//		Y[i] = v->Position()[1];
//		Z[i] = v->Position()[2];
//		X_out[i] = v->Position()[0];
//		Y_out[i] = v->Position()[1];
//		Z_out[i] = v->Position()[2];
//		if (v->IsBoundary())
//		{
//			A.AddElement(i, i, 1);
//			continue;
//		}
//		OneRingHEdge ring(v);
//		HEdge*curr = NULL;
//		double cot_sum = 0;
//		double cot_all_sum = 0;
//		double v_A = 0;
//		double v_A_1 = 0;
//		//Vector3d color_v(0.0, 0.0, 0.0);
//		HEdge *cur = vList[i]->HalfEdge();
//		HEdge *nex = cur;
//		while (nex && nex->Prev()->Twin() != cur)
//		{
//			if (nex && !nex->IsBoundary())//exclude the holes formed by boundary loops
//			{
//				const Vector3d & pos1 = vList[i]->Position();//p1
//				const Vector3d & pos2 = nex->End()->Position();//p2
//				const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
//				if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
//				{
//					const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
//					cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
//					cot_all_sum += cot_sum;
//
//				}
//				v_A_1 = Area(pos1, pos2, pos3);
//				v_A = v_A + v_A_1;//SUM
//				A.AddElement(i, nex->End()->Index(), -lamda*cot_sum);
//			}
//			
//			nex = nex->Prev()->Twin();
//		}
//
//		//process the last nex
//		if (nex && !nex->IsBoundary())
//		{
//			const Vector3d & pos1 = vList[i]->Position();//p1
//			const Vector3d & pos2 = nex->End()->Position();//p2
//			const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
//			if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
//			{
//				const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
//				cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
//				cot_all_sum += cot_sum;
//			}
//			v_A_1 = Area(pos1, pos2, pos3);
//			v_A = v_A + v_A_1;//SUM
//
//			A.AddElement(i, nex->End()->Index(), -lamda*cot_sum);
//		}//end if
//		
//
//		for (int j = 0; j < A.elements.size(); j++)
//		{
//			if (A.elements[j].row == i)
//			{
//				A.elements[j].value /= cot_all_sum;
//			}
//		}
//		A.AddElement(i, i, 1 + lamda);
//	}
//
//	A.SortMatrix();
//	/*A.Multiply(X, X_out);
//	A.Multiply(Y, Y_out);
//	A.Multiply(Z, Z_out);*/
//	A.BCG(X, X_out, 2, 0.01);
//	A.BCG(Y, Y_out, 2, 0.01);
//	A.BCG(Z, Z_out, 2, 0.01);
//	for (size_t i = 0; i < n; i++)
//	{
//		Vertex *v = vList[i];
//		if (v->IsBoundary())
//		{
//			continue;
//		}
//		v->SetPosition(Vector3d(X_out[i], Y_out[i], Z_out[i]));
//	}
//	delete X;
//	delete Y;
//	delete Z;
//	delete X_out;
//	delete Y_out;
//	delete Z_out;
//}

void Mesh::ImplicitUmbrellaSmooth()
{
    //cout<< "Implicit Umbrella Smooth starts..."<<endl;
	//first construct a empty sparse matrix
	size_t n = vList.size();
	//buffer of vertex x,y,and, z value
	double *X, *Y, *Z, *X_out,*Y_out,*Z_out;
	X = new double[n];
	Y = new double[n];
	Z = new double[n];
	X_out = new double[n];
	Y_out = new double[n];
	Z_out = new double[n];
	Matrix A(n, n);
	//give a lamda
	double lamda = 0.9;
	//Iterite and assign value
	for (size_t i = 0; i < n; i++)
	{
		Vertex *v = vList[i];
		X[i] = v->Position()[0];
		Y[i] = v->Position()[1];
		Z[i] = v->Position()[2];
		X_out[i] = v->Position()[0];
		Y_out[i] = v->Position()[1];
		Z_out[i] = v->Position()[2];
		if (v->IsBoundary())
		{
			A.AddElement(i, i, 1);
			continue;
		}
		OneRingHEdge ring(v);
		HEdge*curr = NULL;
		double cot_sum = 0;
		double cot_all_sum = 0;
		double v_A = 0;
		double v_A_1 = 0;
		//Vector3d color_v(0.0, 0.0, 0.0);
		while (curr = ring.NextHEdge())
		{
			if (!curr->IsBoundary())
			{
				const Vector3d & pos1 = v->Position();//p1
				const Vector3d & pos2 = curr->End()->Position();//p2
				const Vector3d & pos3 = curr->Prev()->Start()->Position();//p3
				const Vector3d & pos4 = curr->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
				cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
				cot_all_sum += cot_sum;
				//color_v = color_v + cot_sum*(pos3 - pos1);
				v_A_1 = Area(pos1, pos2, pos3);
				v_A = v_A + v_A_1;//SUM

				A.AddElement(i, curr->End()->Index(), -lamda*cot_sum);
			}
		}
		//end of iterating a row
		for (int j = 0; j < A.Elements().size(); j++)
		{
			if (A.elements[j].row == i )
			{
				A.elements[j].value /= cot_all_sum;
			}
		}
		A.AddElement(i, i, 1 + lamda*cot_all_sum / cot_all_sum);
		
	}

	A.SortMatrix();
	/*A.Multiply(X, X_out);
	A.Multiply(Y, Y_out);
	A.Multiply(Z, Z_out);*/
	A.BCG(X, X_out, 1, 0.01);
	A.BCG(Y, Y_out, 1, 0.01);
	A.BCG(Z, Z_out, 1, 0.01);
	for (size_t i = 0; i < n; i++)
	{
		Vertex *v = vList[i];
		if (v->IsBoundary())
		{
			continue;
		}
		v->SetPosition(Vector3d(X_out[i], Y_out[i], Z_out[i]));
	}
	delete X;
	delete Y;
	delete Z;
	delete X_out;
	delete Y_out;
	delete Z_out;
}
void Mesh::ComputeVertexCurvatures()
{
	//Gaussian curvature
	size_t i;
	double min_c = 200000;//initialize the maximum value of curvature
	double max_c = -200000;//initialize the minimum value of curvature

	for (i = 0; i < vList.size(); i++)
	{
		if (!vList[i]->IsBoundary()) //for interior vertices only
		{
			double  v_A_1 = 0.0; //initialize the area of any face incident to each vertex v
			double  v_A = 0.0;//the area of all face incident to vertex v
			double  Angle_v_1 = 0.0; //initialize any angle incident on each vertex v
			double  Angle_v = 0.0;//the angles incident on vertex v

			double cot_sum = 0.0;//coefficient of (vj - vi) in the mean curvature function
			double cot_all_sum = 0.0;//zyk
			Vector3d color_v(0.0, 0.0, 0.0);

			HEdge *cur = vList[i]->HalfEdge();
			HEdge *nex = cur;
			while (nex && nex->Prev()->Twin() != cur)
			{
				if (nex && !nex->IsBoundary())//exclude the holes formed by boundary loops
				{
					const Vector3d & pos1 = vList[i]->Position();//p1
					const Vector3d & pos2 = nex->End()->Position();//p2
					const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
					if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
					{
						const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
						cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
						cot_all_sum += cot_sum;
						color_v = color_v + cot_sum*(pos3 - pos1);

					}
					v_A_1 = Area(pos1, pos2, pos3);
					v_A = v_A + v_A_1;//SUM

					Vector3d v1 = pos2 - pos1;
					Vector3d v2 = pos3 - pos1;
					v1 /= v1.L2Norm();
					v2 /= v2.L2Norm();
					double temp = v1.Dot(v2);
					Angle_v_1 = acos(temp);
					Angle_v = Angle_v + Angle_v_1;
				}

				nex = nex->Prev()->Twin();

			}

			//process the last nex
			if (nex && !nex->IsBoundary())
			{
				const Vector3d & pos1 = vList[i]->Position();//p1
				const Vector3d & pos2 = nex->End()->Position();//p2
				const Vector3d & pos3 = nex->Prev()->Start()->Position();//p3
				if (!vList[i]->IsBoundary())//if the current vertex is an interior vertex
				{
					const Vector3d & pos4 = nex->Prev()->Twin()->Prev()->Start()->Position();//p4, for computing the mean curvature
					cot_sum = Cot(pos1, pos2, pos3) + Cot(pos1, pos4, pos3);
					cot_all_sum += cot_sum;
					color_v = color_v + cot_sum*(pos3 - pos1);

				}

				v_A_1 = Area(pos1, pos2, pos3);
				v_A = v_A + v_A_1;//SUM

				//computing the sum of angles for Gaussian curvature
				Vector3d v1 = pos2 - pos1;
				Vector3d v2 = pos3 - pos1;
				v1 /= v1.L2Norm();
				v2 /= v2.L2Norm();
				double temp = v1.Dot(v2);
				Angle_v_1 = acos(temp);
				Angle_v = Angle_v + Angle_v_1;
			}//end if

			double PI = 3.14159265358979323846;
			double G = (2.0 / v_A)*(2 * PI - Angle_v);//Gaussian curvature
			double color_vn = color_v.L2Norm();
			vList[i]->LaplacianCoordinate = color_v/cot_all_sum;
			double H = -color_vn / (2 * v_A); //mean curvature   

			//compute principle curvatures
			double K1 = H + sqrt(fabs(H*H - G));//max principle curvature

			double K2 = H - sqrt(fabs(H*H - G));//min principle curvature
			//remember different types of curvatures for each 
			vList[i]->G = G; //record gaussian curvature at vList[i]
			vList[i]->H = H; //record mean curvature at vList[i]
			vList[i]->K1 = K1; //record max principle curvature at vList[i]
			vList[i]->K2 = K2; //record min principle curvature at vList[i]
			/*
			//"max principal curvature" scheme
			if(K1 < min_c) // reset min curvatures
			min_c = K1;
			if(K1 > max_c) //reset max curvatures
			max_c = K1;

			*/
			//"min principal curvature" scheme
			if (K2 < min_c) // reset min curvatures
				min_c = K2;
			if (K2 > max_c) //reset max curvatures
				max_c = K2;


			/*
			//"mean curvature" scheme
			if(H < min_c) // reset min curvatures
			min_c = H;
			if(H > max_c) //reset max curvatures
			max_c = H;
			*/

			/*
			//"Gaussian curvature" scheme
			if(G < min_c) // reset min curvatures
			min_c = G;
			if(G > max_c) //reset max curvatures
			max_c = G;

			*/
		}//end 'if' condition for interior vertices

	}//end the first 'for' loop

	/**********interpolation based on the scheme of [0->1] equals [blue->red]**********/
	//"max principal curvature" scheme
	/*
	for (i=0; i< vList.size(); i++)
	{
	if(!vList[i]->IsBoundary())//process interior vertices
	{
	Vector3d color_max((vList[i]->K1 - min_c)/(max_c - min_c),0.0,1-(vList[i]->K1 - min_c)/(max_c - min_c));
	vList[i]->SetColor(color_max);
	}
	else //process boundary vertices
	{
	Vector3d color_max(0.0,0.0,0.0);
	vList[i]->SetColor(color_max);
	}
	}//end this for

	*/
	//"min principal curvature" scheme
	for (i = 0; i < vList.size(); i++)
	{
		if (!vList[i]->IsBoundary())//process interior vertices
		{
			Vector3d color_min;
			float r = (vList[i]->K2 - min_c) / (max_c - min_c);
			if (r < 0.8)
				color_min = Vector3d(0, 1, 0);
			else color_min = Vector3d(r, 0, 1 - r);
			vList[i]->SetColor(color_min);
		}
		else //process boundary vertices
		{
			Vector3d color_max(0.0, 0.0, 0.0);
			vList[i]->SetColor(color_max);
		}
	}//end this for



	//"mean curvature" scheme
	// 	for (i=0; i< vList.size(); i++) 
	// 	{ 
	// 	  if(!vList[i]->IsBoundary())//process interior vertices
	// 	  {
	// 	    Vector3d color_mean((vList[i]->H - min_c)/(max_c - min_c),0.0,1-(vList[i]->H - min_c)/(max_c - min_c));
	// 	    vList[i]->SetColor(color_mean); 
	// 	  }
	// 	  else //process boundary vertices
	// 	  {
	// 		Vector3d color_mean(0.0,0.0,0.0);
	// 	    vList[i]->SetColor(color_mean); 
	// 	  }
	// 	}//end this for
	//    
	//  

	// 	//"Gaussian curvature" scheme
	// 	for (i=0; i< vList.size(); i++) 
	// 	{ 
	// 	  if(!vList[i]->IsBoundary())//process interior vertices
	// 	  {
	// 	    Vector3d color_Gau((vList[i]->G - min_c)/(max_c - min_c),0.0,1-(vList[i]->G - min_c)/(max_c - min_c));
	// 	    vList[i]->SetColor(color_Gau); 
	// 	  }
	// 	  else //process boundary vertices
	// 	  {
	// 		Vector3d color_max(0.0,0.0,0.0);
	// 	    vList[i]->SetColor(color_max); 
	// 	  }
	// 
	// 	}//end this for
}