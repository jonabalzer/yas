/*////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2013, Jonathan Balzer
//
// All rights reserved.
//
// This file is part of the R4R library.
//
// The R4R library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The R4R library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the R4R library. If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////*/

#include "iter.h"
#include "darray.h"

#include <math.h>
#include <stdio.h>
#include <assert.h>
#include <iostream>


using namespace std;

template<class Matrix,class Vector,class Scalar>
CIterativeSolver<Matrix,Vector,Scalar>::CIterativeSolver(CPreconditioner<Matrix,Vector,Scalar>& M, size_t n, Scalar eps, bool silent):
	m_M(M),
	m_n(n),
	m_eps(eps),
	m_silent(silent) {

}

template<class Matrix,class Vector,class Scalar>
Scalar CIterativeSolver<Matrix,Vector,Scalar>::CG(Matrix& A, Vector& b, Vector& x) {

	// check dimensions
	if(!(A.NCols()==x.NRows() && x.NRows()==b.NRows() && x.NCols()==b.NCols() && x.NCols()==1)) {

		cout << "ERROR: Check matrix dimensions!" << endl;
		return -1;

	}

	// init
	size_t k = 0;

    Vector r = b - A*x;

	Scalar normr = r.Norm2();

	if(!m_silent)
		cout << "k=" << k << ": " << normr << endl;

	Vector z(r);

	m_M.Solve(z,r);

	Scalar deltan = CDenseArray<Scalar>::InnerProduct(z,r);

	Vector p = z;

	while(k<x.NRows() && k<m_n) {

		Vector q = A*p;

		Scalar alpha = deltan/CDenseArray<Scalar>::InnerProduct(p,q);

		x = x + p*alpha;

		q.Scale(alpha);
		r = r - q;

		normr = r.Norm2();

		k++;

		if(!m_silent)
			cout << "k=" << k << ": " << normr << endl;

		if(normr<m_eps)
			break;

		m_M.Solve(z,r);

		Scalar deltao = deltan;

		deltan = CDenseArray<Scalar>::InnerProduct(z,r);

		Scalar beta = deltan/deltao;

		p.Scale(beta);
		p = z + p;

	}

	return deltan;

}

template<class Matrix,class Vector,class Scalar>
Scalar CIterativeSolver<Matrix,Vector,Scalar>::CGLS(Matrix& A, Vector& b, Vector& x) {

	if(!(A.NCols()==x.NRows() && A.NRows()==b.NRows() && x.NCols()==1)) {

		cout << "ERROR: Check matrix dimensions!" << endl;
		return -1;

	}

	// init
	size_t k = 0;

	// residual of the non-square system
	Vector r = b - A*x;

	Scalar normrt, normr;
	normr = r.Norm2();

	// transpose in place to save computation time
	A.Transpose();
	Vector rnormal = A*r;   // residual of the normal equation
	A.Transpose();


	// preconditioning
	Vector z(rnormal);
	m_M.Solve(z,rnormal);

	// descent direction
	Vector p = z;

	Scalar deltao = CDenseArray<Scalar>::InnerProduct(z,rnormal);  // numerator: z\dot rnormal

	if(!m_silent)
		cout << "k=" << k << ": " << normr << endl;

	//size_t mind = min(A.NRows(),A.NCols());

	//while(k<mind && k<m_n) {
	while(k<m_n) {

		// need that later
		Vector q = A*p;

		// step size (recycle deltao for computation of beta)
		Scalar alpha = deltao/CDenseArray<Scalar>::InnerProduct(q,q);

		// perform descent step
		x = x + p*alpha;

		q.Scale(alpha);
		r = r - q;				// update residual of non-square system

		normrt = r.Norm2();

		k++;

		if(!m_silent)
			cout << "k=" << k << ": " << normr << endl;

		if(fabs(normr-normrt)<m_eps)
			break;

		normr = normrt;

		A.Transpose();
		rnormal = A*r;			// update residual of normal equation
		A.Transpose();

		// apply preconditioner
		m_M.Solve(z,rnormal);
		//z = rnormal;

		// update beta
		Scalar deltan = CDenseArray<Scalar>::InnerProduct(z,rnormal);
		Scalar beta = deltan/deltao;
		deltao = deltan;

		// update direction
		p.Scale(beta);
		p = z + p;

	}

	return deltao;

}

template class  CIterativeSolver<CDenseArray<double>,CDenseVector<double>,double>;
template class  CIterativeSolver<CDenseArray<float>,CDenseVector<float>,float>;


