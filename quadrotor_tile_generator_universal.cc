// need to adjust constants VAR_NO (number of variables), if ODE system changes, 
// TILED_VAR_NO (number of variables to be tiled)
// entries in A (array that holds interval initial conditions for variables to be tiled)
// y[.] initial coniditons in the main ODE loop
// Input argument: fctr (number of tiles that each tiled interval will be subdivided into)

/*61:*/
#line 74 "./setintegp.w"

#include <fstream> 
#include "vnode.h"
#include <iomanip> 
#include <sstream> 
#include <iostream> // needed for cin and cout
#include <cstdlib> // for atoi
#include <math.h> // for pow
#include <stdio.h> // for printf
#include <bitset>
#include "tiler.h"

// update this if ODE/PDE system changes
#define VAR_NO 6
#define TILED_VAR_NO 2
// input argument: fctr
unsigned int TILE_NO = 0;

// static tiler from singleton Tiler Class
Tiler* Tiler::tlr;
// global tiler declaration
Tiler* tiler;

using namespace std;
using namespace vnodelp;

/*19:*/
#line 76 "./usage.w"

void usage(const char* program);

template<typename var_type> 
void Quadrotor(int n,var_type*yp,
const var_type*y,
var_type t,void*param){

interval K(0.89),m(1.4),g(9.81);
interval n0(55.0),d0(70.0),d1(17.0);
//nonzero elements of the gain matrix 
//obtained from lqr procedure in MATLAB
interval k12(0.25),k14(0.954733917870621);
interval k21(0.125),k23(0.315828119014695);
interval k25(0.435969745268800), k26(0.027574741614871);

// system with LQR gain matrix incorporated
yp[0] = y[2];
yp[1] = y[3];
yp[2] = (-K*(k12*y[1]+k14*y[3])/m+g)*sin(y[4]);
yp[3] = -g+(g-K*(k12*y[1]+k14*y[3])/m)*cos(y[4]);
yp[4] = y[5];
yp[5] = -n0*(k21*y[0]+k23*y[2]+k25*y[4]+k26*y[5])-d0*y[4]-d1*y[5];
}

#line 1 "./basic.w"

/*:19*/
#line 81 "./setintegp.w"

int main(int argc, char *argv[])
{
     
        if (argc != 2) {
            usage(argv[0]);
            return -1;
        } else {
            TILE_NO = std::atoi(argv[1]);
        }
            // initialize the tiler
            // total number of variables in the system, e.g. 6
            // number of state variables to be tiled, e.g. x, y gives n = 2
            // interval scaling factor, e.g. 4 tiles per interval
            tiler = Tiler::getInstance(VAR_NO, TILED_VAR_NO, TILE_NO);

            /*21:*/
            #line 24 "./basic.w"

            // original state constraints
            interval A[tiler->getNTile()];

            // quadrocopter is approaching an obstacle perpendicular to the x-axis head on
            *A = interval(0, 0.8);
            *(A+1) = interval(0, 1.0);

            //*A = interval(-0.5, 0.5);
            //*(A+1) = interval(-0.8, 0.8);

            // number of tiles, if original intervals are cut in fctr pieces
            unsigned int sz = pow(tiler->getFctr(),tiler->getNTile());

            //initialize array to hold permutations of subinterval tiles for initial conditions
            interval intvls[sz*tiler->getNTile()];

            //the last step in generating subinterval tiles for the initial condition ranges supplied in A
            tiler->tile(A, intvls);


            for (int i=0; i<sz; i++) {

                //increment the output file names
                stringstream ss;
                ss<<"quadrotor.tile.generator.tight"<<i;
                
                ofstream outFile1(ss.str().c_str(),ios::out);
                interval t= 0.0,tend= 5.0;

                iVector y(VAR_NO);

                // need to modify the initial conditions if we change the set up
                //x, y
                y[0] = interval(0.0);
                y[1] = interval(0.0);
                //x-vel, y-vel
                y[2] = *(intvls+tiler->getNTile()*i);
                y[3] = *(intvls+tiler->getNTile()*i + 1);
                //roll, roll-vel
                //y[4] = interval(-0.15, 0.15);
                y[4] = interval(0.0);
                y[5] = interval(-1.5, 1.5);


                /*:21*/
                #line 85 "./setintegp.w"

                /*22:*/
                #line 41 "./basic.w"

                AD*ad= new FADBAD_AD(VAR_NO,Quadrotor,Quadrotor);

                /*:22*/
                #line 86 "./setintegp.w"

                /*23:*/
                #line 48 "./basic.w"

                VNODE*Solver= new VNODE(ad);

                Solver->setFirstEntry();

                /*:23*/
                #line 87 "./setintegp.w"

                /*60:*/
                #line 66 "./setintegp.w"


                /*:60*/
                #line 88 "./setintegp.w"

                /*62:*/
                #line 139 "./setintegp.w"

                /*:62*/
                #line 89 "./setintegp.w"

                /*43:*/
                #line 58 "./basici.w"

                //apparently need this to access intermediate results
                Solver->setOneStep(on);

                /*:43*/
                #line 90 "./setintegp.w"

                /*64:*/
                #line 161 "./setintegp.w"

                /*:64*/
                #line 91 "./setintegp.w"

                    //initial coniditions
                    outFile1<<std::setprecision(20)<<midpoint(t)<<"\t"
                    <<inf(y[0])<<"\t"<<sup(y[0])<<"\t"
                    <<inf(y[1])<<"\t"<<sup(y[1])<<"\t"
                    <<inf(y[2])<<"\t"<<sup(y[2])<<"\t"
                    <<inf(y[3])<<"\t"<<sup(y[3])<<"\t"
                    <<inf(y[4])<<"\t"<<sup(y[4])<<"\t"
                    <<inf(y[5])<<"\t"<<sup(y[5])<<"\t"
                    <<endl;


                while(t!=tend)
                {
                Solver->integrate(t,y,tend);
                /*65:*/
                #line 167 "./setintegp.w"

                    outFile1<<std::setprecision(20)<<midpoint(t)<<"\t"
                    <<inf(y[0])<<"\t"<<sup(y[0])<<"\t"
                    <<inf(y[1])<<"\t"<<sup(y[1])<<"\t"
                    <<inf(y[2])<<"\t"<<sup(y[2])<<"\t"
                    <<inf(y[3])<<"\t"<<sup(y[3])<<"\t"
                    <<inf(y[4])<<"\t"<<sup(y[4])<<"\t"
                    <<inf(y[5])<<"\t"<<sup(y[5])<<"\t"
                    <<endl;

                    //if (inf(y[0])<-1.7) cout<<"x exceeds lower bound"<<endl;
                    //if (sup(y[0])>1.7) cout<<"x exceeds upper bound"<<endl;
                    //if (inf(y[1])<0.3) cout<<"y exceeds lower bound"<<endl;
                    //if (sup(y[1])>2) cout<<"y exceeds upper bound"<<endl;
                    if (sup(y[2])<-0.8) {
                        cout<<i<<" "<<"x-velocity exceeds lower bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl;
                    }
                    if (sup(y[2])>0.8) {
                        cout<<i<<" "<<"x-velocity exceeds upper bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl; 
                    }
                    if (sup(y[3])<-1.0) {
                        cout<<i<<" "<<"y-velocity exceeds lower bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl;
                    }
                    if (sup(y[3])>1.0) {
                        cout<<i<<" "<<"y-velocity exceeds upper bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl;
                    }
                    if (sup(y[4])<-0.15) {
                        cout<<i<<" "<<"roll exceeds lower bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl;
                    }
                    if (sup(y[4])>0.15) {
                        cout<<i<<" "<<"roll exceeds upper bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl;
                    }
                    if (sup(y[5])<-1.5) {
                        cout<<i<<" "<<"roll-velocity exceeds lower bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl;
                    }
                    if (sup(y[5])>1.5) {
                        cout<<i<<" "<<"roll-velocity exceeds upper bound"<<endl;
                        cout<<"x-vel interval: "<<*(intvls+tiler->getNTile()*i)<<endl;
                        cout<<"y-vel interval: "<<*(intvls+tiler->getNTile()*i + 1)<<endl;
                    }
                }

                /*:65*/
                #line 95 "./setintegp.w"

                outFile1.close();
            }

            /*63:*/
            #line 145 "./setintegp.w"

            /*:63*/
            #line 97 "./setintegp.w"

            return 0;
}

// Print a message on stdout about how to use this program.
void usage(const char* program) {
  std::cout << "Usage: " << program << " 4 "  << std::endl;
  std::cout << "Enter the name of the executable followed by a number of tiles per state variable interval, e.g., 4"  << std::endl;
}