
For using the CppAD sparse2eigen() function
-------------------------------------------

CppAD Documentation says: 
"If include_eigen is specified on the cmake command line, the file `cppad/utility/sparse2eigen.hpp` is included by `cppad/cppad.hpp`. 
In any case, it can also be included separately with out the rest of the CppAD routines. 
Including this file defines this version of the sparse2eigen within the CppAD namespace."

What I did instead:
The above didn't work for me, so I just changed the `CPPAD_HAS_EIGEN` macro in `configure.hpp` to 1