#include "utils.hpp"

int gcd (int a, int b) {

    if (b == 0)
        return a;

    else 
        return gcd(b, a % b);
}

int max(int a, int b) {

    if (a > b)
        return a;
    
    else
        return b;
}


int min(int a, int b) {

    if (a < b)
        return a;
    
    else
        return b;
}