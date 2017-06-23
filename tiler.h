#ifndef __TILER
#define __TILER

#include <string>
#include "vnode.h"

class Tiler {
        
    private:
            // number of variables
            unsigned int n;
            // number of variables to be tiled
            unsigned int n_tile;
            // number of tiles that each tiled interval will be subdivided into
            unsigned int fctr;
            // singleton
            static Tiler* tlr;
            // private constructor
            Tiler(unsigned int n, unsigned int n_tile, unsigned int fctr): n(n), n_tile(n_tile), fctr(fctr) {}
           
    public:
            // singleton initialization
            static Tiler* getInstance(unsigned int n, unsigned int n_tile, unsigned int fctr);
            // destructor
            ~Tiler() {
                    delete tlr;
            }
            
            // getters
            unsigned int getN();
            unsigned int getNTile();
            unsigned int getFctr();

            void fill_array(unsigned int* nmbrs); // generate consecutive integers, one for each tile
            std::string convertToBase(unsigned int k); // can convert a decimal int to any other base, in this case equal to fctr
            void generateBaseString(std::string* s); // produce a set of digits legal for a given base
            std::string intToString(int number); // convert integer to string
            int countLeftParan(std::string s); // count the number of left parenthesis in a string
            // populate base_nmbrs with base strings representing numbers from 0 to sz = pow(fctr,n) = 2^n
            void fillBaseStrings(unsigned int sz, unsigned int* nmbrs, std::string* base_nmbrs);
            // generate tiles for each state variable
            void tilingHelper(interval* A, interval* tile_A);
            // the last step in generating subinterval tiles for the initial condition ranges supplied in A
            void tilingGenerator(unsigned int sz, std::string* base_nmbrs, interval* tile_A, std::string* Dgts, interval* intvls);
            // populate the intvls array with interval tiles
            void tile(interval* A, interval* intvls);
};

#endif