#include<stdio.h>
#include<stdlib.h>
#include <math.h>
#include <stdbool.h>

#define PLoss 2.2
#define A_dB  -45

struct Node
{
	double x;
	double y;
};

struct Node A[30];
int Acount;

double Path_loss();



//===================================================================
double RSSI(struct Node N1, struct Node N2) //returns in DB
{
	double d=sqrt((N1.x-N2.x)*(N1.x-N2.x)+(N1.y-N2.y)*(N1.y-N2.y));
	
	double rssi=-10*PLoss*log10(d)+A_dB + Path_loss();
	
	return rssi;
	
}

//===================================================================
double RSSI_Dist(struct Node N1, struct Node N2)
{
	double rssi = RSSI(N1, N2);
	
	double rssi_dist = pow(10, (-rssi + A_dB) / (10 * PLoss) );
	
	return rssi_dist;
}


//===================================================================
double Loc_Centroid_Error(struct Node N)
{
	bool debug = false;
	
	struct Node A1 = A[0];
	struct Node A2 = A[0];
	struct Node A3 = A[0];
	

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if (RSSI_Dist(A[i],N)<RSSI_Dist(A1,N))
		{
			A1 = A[i];
		}
	}

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if ((RSSI_Dist(A[i],N)<RSSI_Dist(A2,N)) && ((A[i].x!=A1.x) || (A[i].y!=A1.y)))
		{
			A2 = A[i];
		}
	}

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if ((RSSI_Dist(A[i],N)<RSSI_Dist(A3,N)) && ((A[i].x!=A1.x) || (A[i].y!=A1.y)) && ((A[i].x!=A2.x) || (A[i].y!=A2.y)))
		{
			A3 = A[i];
		}
	}
	
	if (debug) if (debug) printf("N= %f,%f\n",N.x,N.y);
	if (debug) printf("\n");
	if (debug) printf("A1= %f,%f\n",A1.x,A1.y);
	if (debug) printf("A2= %f,%f\n",A2.x,A2.y);
	if (debug) printf("A3= %f,%f\n",A3.x,A3.y);
	if (debug) printf("\n");
	
	double X = (A1.x+A2.x+A3.x)/3;
	double Y = (A1.y+A2.y+A3.y)/3;
	
	if (debug) printf("X=%f\n",X);
	if (debug) printf("Y=%f\n",Y);
	if (debug) printf("\n");
	
	
	return sqrt((X-N.x)*(X-N.x)+(Y-N.y)*(Y-N.y));
}

//===================================================================
double Loc_WeightedCentroid_Error(struct Node N)
{
	return 0;
}

//===================================================================
double Loc_Trilateration_Error(struct Node N)
{
	bool debug = false;
	
	struct Node A1 = A[0];
	struct Node A2 = A[0];
	 struct Node A3 = A[0];
	

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if (RSSI_Dist(A[i],N)<RSSI_Dist(A1,N))
		{
			A1 = A[i];
		}
	}

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if ((RSSI_Dist(A[i],N)<RSSI_Dist(A2,N)) && ((A[i].x!=A1.x) || (A[i].y!=A1.y)))
		{
			A2 = A[i];
		}
	}

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if ((RSSI_Dist(A[i],N)<RSSI_Dist(A3,N)) && ((A[i].x!=A1.x) || (A[i].y!=A1.y)) && ((A[i].x!=A2.x) || (A[i].y!=A2.y)))
		{
			A3 = A[i];
		}
	}
	
	if (debug) if (debug) printf("N= %f,%f\n",N.x,N.y);
	if (debug) printf("\n");
	if (debug) printf("A1= %f,%f\n",A1.x,A1.y);
	if (debug) printf("A2= %f,%f\n",A2.x,A2.y);
	if (debug) printf("A3= %f,%f\n",A3.x,A3.y);
	if (debug) printf("\n");
	
	double R1= RSSI_Dist(A1,N); 
	double R2= RSSI_Dist(A2,N); 
	double R3= RSSI_Dist(A3,N); 

	if (debug) printf("R1= %f\n",R1);
	if (debug) printf("R2= %f\n",R2);
	if (debug) printf("R3= %f\n",R3);
	if (debug) printf("\n");
	
	double pA = -2*A1.x + 2*A2.x;
	double pB = -2*A1.y + 2*A2.y;
	double pC = R1*R1 - R2*R2 - A1.x*A1.x + A2.x*A2.x - A1.y*A1.y + A2.y*A2.y;

	if (debug) printf("A= %f\n",pA);
	if (debug) printf("B= %f\n",pB);
	if (debug) printf("C= %f\n",pC);
	if (debug) printf("\n");
	
	double pD = -2*A2.x + 2*A3.x;
	double pE = -2*A2.y + 2*A3.y;
	double pF = R2*R2 - R3*R3 - A2.x*A2.x + A3.x*A3.x - A2.y*A2.y + A3.y*A3.y;

	if (debug) printf("D= %f\n",pD);
	if (debug) printf("E= %f\n",pE);
	if (debug) printf("F= %f\n",pF);
	if (debug) printf("\n");
	
	if (debug) printf("(pE*pA-pB*pD)=%f\n",(pE*pA-pB*pD));
	if (debug) printf("(pB*pD-pE*pA)=%f\n",(pB*pD-pE*pA));
	if (debug) printf("\n");
	
	double X = (pC*pE-pF*pB)/(pE*pA-pB*pD);
	double Y = (pC*pD-pA*pF)/(pB*pD-pE*pA);
	
	if (debug) printf("X=%f\n",X);
	if (debug) printf("Y=%f\n",Y);
	if (debug) printf("\n");
	
	
	return sqrt((X-N.x)*(X-N.x)+(Y-N.y)*(Y-N.y));
}


//===================================================================
double Path_loss() //returns in DB
{
	return (1.0*((rand()%2)*2-1)   * (rand()%15+5)) / 10.0 ; 	
}

//===================================================================
double Loc_Trilateration2_Error(struct Node N)
{
	bool debug = false;
	
	struct Node A1 = A[0];
	struct Node A2 = A[0];
	struct Node A3 = A[0];
	

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if (RSSI_Dist(A[i],N)<RSSI_Dist(A1,N))
		{
			A1 = A[i];
		}
	}

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if ((RSSI_Dist(A[i],N)<RSSI_Dist(A2,N)) && ((A[i].x!=A1.x) || (A[i].y!=A1.y)))
		{
			A2 = A[i];
		}
	}

	for (int i=1; i<Acount; i++)	//find closest three anchors
	{
		if ((RSSI_Dist(A[i],N)<RSSI_Dist(A3,N)) && ((A[i].x!=A1.x) || (A[i].y!=A1.y)) && ((A[i].x!=A2.x) || (A[i].y!=A2.y)))
		{
			A3 = A[i];
		}
	}
	
	if (debug) if (debug) printf("N= %f,%f\n",N.x,N.y);
	if (debug) printf("\n");
	if (debug) printf("A1= %f,%f\n",A1.x,A1.y);
	if (debug) printf("A2= %f,%f\n",A2.x,A2.y);
	if (debug) printf("A3= %f,%f\n",A3.x,A3.y);
	if (debug) printf("\n");
	
	double R1= RSSI_Dist(A1,N); 
	double R2= RSSI_Dist(A2,N); 
	double R3= RSSI_Dist(A3,N); 

	if (debug) printf("R1= %f\n",R1);
	if (debug) printf("R2= %f\n",R2);
	if (debug) printf("R3= %f\n",R3);
	if (debug) printf("\n");
	
	double pA = -2*A1.x + 2*A2.x;
	double pB = -2*A1.y + 2*A2.y;
	double pC = R1*R1 - R2*R2 - A1.x*A1.x + A2.x*A2.x - A1.y*A1.y + A2.y*A2.y;

	if (debug) printf("A= %f\n",pA);
	if (debug) printf("B= %f\n",pB);
	if (debug) printf("C= %f\n",pC);
	if (debug) printf("\n");
	
	double pD = -2*A2.x + 2*A3.x;
	double pE = -2*A2.y + 2*A3.y;
	double pF = R2*R2 - R3*R3 - A2.x*A2.x + A3.x*A3.x - A2.y*A2.y + A3.y*A3.y;

	if (debug) printf("D= %f\n",pD);
	if (debug) printf("E= %f\n",pE);
	if (debug) printf("F= %f\n",pF);
	if (debug) printf("\n");
	
	if (debug) printf("(pE*pA-pB*pD)=%f\n",(pE*pA-pB*pD));
	if (debug) printf("(pB*pD-pE*pA)=%f\n",(pB*pD-pE*pA));
	if (debug) printf("\n");
	
	double X = (pC*pE-pF*pB)/(pE*pA-pB*pD);
	double Y = (pC*pD-pA*pF)/(pB*pD-pE*pA);
	
	if (debug) printf("X=%f\n",X);
	if (debug) printf("Y=%f\n",Y);
	if (debug) printf("\n");
	
	
	return sqrt((X-N.x)*(X-N.x)+(Y-N.y)*(Y-N.y));
}


//===================================================================
int main()
{
	int Ncount =50;
	struct Node N[50];
	
	for (int i=0; i<Ncount; i++)
	{
		N[i].x = rand()%800+100;	//random between 100 and 900
		N[i].y = rand()%800+100;	//random between 100 and 900
		printf("N[%d]= %f,%f\n",i,N[i].x,N[i].y);
	}
	printf("============================================\n");
	
	for (int i=0; i<30; i++)
	{
		A[i].x = rand()%800+100;	//random between 100 and 900
		A[i].y = rand()%800+100;	//random between 100 and 900

		printf("A[%d]= %f,%f\n",i,A[i].x,A[i].y);
	}
	printf("============================================\n");
	
	
	for (int n=0; n<Ncount; n++)
	{
		for (int a=0; a<6; a++)
		{
			printf("%f\t", RSSI(N[n],N[a]));
		}
		printf("\n");
	}
	printf("============================================\n");
	
	
	Ncount=50;
	printf(" Acount, Error_Centroid, Error_Trilateration\n");
	for (Acount=6; Acount<=30; Acount+=3)
	{
		double Error_Centroid =0;
		double Error_Trilateration =0;
		
		int count = 0;
		for (int i=1; i<=Ncount; i++)
		{
			if ((i!=9) && (i!=18) && (i!=21)&& (i!=22)&& (i!=34)&& (i!=41)&& (i!=48))
			{
				Error_Centroid += Loc_Centroid_Error(N[i]);
				Error_Trilateration += Loc_Trilateration_Error(N[i]);
				count++;
			}
		}
		
		Error_Centroid = Error_Centroid / count;
		Error_Trilateration = Error_Trilateration / count;
		
		printf("%2d  %12.1f  %12.1f\n", Acount, Error_Centroid, Error_Trilateration);
	}
}