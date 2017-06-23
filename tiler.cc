#include "tiler.h"
#include <string>
#include <fstream> 
#include <iomanip> 
#include <sstream> 
#include <iostream> // needed for cin and cout
#include <cstdlib> // for atoi
#include <math.h> // for pow
#include <stdio.h> // for printf
#include <bitset>
#include <vector>
#include "vnode.h"


Tiler* Tiler::getInstance(unsigned int n, unsigned int n_tile, unsigned int fctr) {
                if (!tlr) tlr = new Tiler(n, n_tile, fctr);
                return tlr;
            }

unsigned int Tiler::getN() {
	return n;
}
unsigned int Tiler::getNTile() {
	return n_tile;
}            
unsigned int Tiler::getFctr() {
	return fctr;
}

//need to construct the array to populate
void Tiler::fill_array(unsigned int* nmbrs) {
	for (unsigned int i=0; i<pow(fctr, n_tile); i++) {
		*(nmbrs+i)=i;
	}
}

std::string Tiler::convertToBase(unsigned int k) {
	int dividend_new  = k;
	int dividend_old = k;
	int remainder  = 0;
	int nOf0 = 0;
	std::vector<int> remainders;
	while (dividend_new/fctr != 0 || nOf0 < 1) {
		dividend_new = dividend_old/fctr;
		if (dividend_new == 0) nOf0++;
		remainder = dividend_old - dividend_new*fctr;
		remainders.push_back(remainder);
		dividend_old = dividend_new;
	}

    // buffer to hold all numbers up to 64-bits
	char numstr[21];
	std::string conversion="";
    // surround permuted digits with parenthesis to make sure that not only single digits
    // in base representations
    std::string leftParan="(";
    std::string rightParan=")";
	for (unsigned int i=0; i<remainders.size() ; i++) {
        conversion+=leftParan;
		conversion+=itoa(remainders[i], numstr, 10);
        conversion+=rightParan;
	}
    remainders.clear();
	return conversion;
}

void Tiler::generateBaseString(std::string* s) {
    for (unsigned int i=0; i<fctr; i++) {
        *(s+i) = intToString(i);
    }
}

std::string Tiler::intToString(int number) {
    std::ostringstream oss;
    oss<<number;
    return oss.str();
}

int Tiler::countLeftParan(std::string s){
 int count = 0;
  for (unsigned int i = 0; i < s.length(); i++)
    if (s[i] == '(') count++;
  return count;
}

// populate base_nmbrs with base strings representing numbers from 0 to sz = pow(fctr,n) = 2^n
void Tiler::fillBaseStrings(unsigned int sz, unsigned int* nmbrs, std::string* base_nmbrs) {
        for (unsigned int i=0; i<sz; i++) {
                *(base_nmbrs+i)=convertToBase(*(nmbrs+i));
                // make up the trailing 0s lost at conversion
                while (countLeftParan(*(base_nmbrs+i))<n_tile) *(base_nmbrs+i) += "(0)";
                }
 }

void Tiler::tilingHelper(interval* A, interval* tile_A) {
        double delta;
        // tiles are numbered in order of interval shift to the right, i.e., index(left) < index(right)
        for (unsigned int i=0; i<n_tile; i++) {
                delta = width(*(A+i))/fctr;
                for (unsigned int j=0; j<fctr; j++) {
                        *(tile_A + i*fctr+j) = interval(inf(*(A+i)) + j*delta,  inf(*(A+i)) + (j+1)*delta);
                }
        }
 }

void Tiler::tilingGenerator(unsigned int sz, std::string* base_nmbrs, interval* tile_A, std::string* Dgts, interval* intvls) {
//the last step in generating subinterval tiles for initial conditions
	for (unsigned int i=0; i<sz; i++) {
    	string s = *(base_nmbrs+i);
            for (unsigned int j=0; j<n_tile; j++) {
                string leftParan = "(";
                string rightParan = ")";
                int first = s.find(leftParan);
                int last = s.find(rightParan);
                string s_temp = s.substr(first, last-first+1);
				//std::cout<<j<<std::endl;
                for (unsigned int k=0; k<fctr; k++) {
                	string temp = intToString(k);
                    temp = leftParan + temp + rightParan;
                    if (s_temp == temp) *(intvls + i*n_tile+j) = *(tile_A+fctr*j + k);       					
                }
                s = s.substr(last+1);
            }
	}
}

void Tiler::tile(interval* A, interval* intvls) {
		// number of tiles, if original intervals are cut in fctr pieces
		unsigned int sz = pow(fctr,n_tile);

		// initialize arrays for doing the change of base trick for generating permutation of intervals
		unsigned int nmbrs[sz];
		string base_nmbrs[sz];

		// generate consecutive integers, one for each tile
		fill_array(nmbrs);

		// populate base_nmbrs with base strings representing numbers from 0 to sz = pow(fctr,n) = 2^n
		fillBaseStrings(sz, nmbrs, base_nmbrs);

		// generate tiles for each state variable
		interval tile_A[fctr*n_tile];
		tilingHelper(A, tile_A);

		//hold base digits
		std::string Dgts[fctr];
		generateBaseString(Dgts);

		//the last step in generating subinterval tiles for the initial condition ranges supplied in A
		tilingGenerator(sz, base_nmbrs, tile_A, Dgts, intvls);
}