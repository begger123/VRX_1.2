#ifndef USV16_GLOBAL_HEADER_INCLUDED
#define USV16_GLOBAL_HEADER_INCLUDED

////////////
// Macros //
////////////

#define PI 3.14159265358979323846 // double precision pi

////////////////////////////
// Actuator Limits Macros //
////////////////////////////

#define ACTMAXFWDTHRUST 210.0
#define ACTMAXREVTHRUST -150.5
#define ACTMAXAZI 45.0
#define ACTMINAZI -45.0

#define MAXFWDTHR 420.0
#define MAXREVTHR -300.0
#define MAXSTBTHR 300.0
#define MAXPORTHR -300.0
#define MAXCWTORQ 360.0
#define MAXCCWTORQ -360.0

/////////////////////////////
// Helpful Macro Functions //
/////////////////////////////

#define DEG2RAD(x)	((x)*PI/180.0)
#define RAD2DEG(x)	((x)*180.0/PI)

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y)) 
#define MAX(X, Y) (((X) < (Y)) ? (Y) : (X))

#define SAT(X,BOT,TOP) ( MIN( (MAX((X),(BOT))) , (TOP) ) ) 

#define GOT_HERE cout << "At " << __FILE__ << ":" << __LINE__ << endl
#define GEN_ERR cerr << "ERROR->"<< __FILE__<< "->" << __LINE__ << ":" << endl

//////////////////////
// Global Functions //
//////////////////////

inline double wrap(double psi)
{
	if 		(psi > PI)  psi-=2*PI;
	else if (psi < -PI) psi+=2*PI;

	return psi;
}

#endif