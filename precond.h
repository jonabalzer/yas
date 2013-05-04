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

#ifndef R4RPRECOND_H_
#define R4RPRECOND_H_

/*! \brief preconditioning of iterative linear solvers
 *
 *
 */
template<class Matrix,class Vector,class Scalar>
class CPreconditioner {

public:

	//! Constructor.
    CPreconditioner(Matrix& A):m_A(A) {}

	//! Performs preconditioning.
    virtual void Solve(Vector& x, Vector& y) { x = y; }

protected:

	Matrix& m_A;						//!< input matrix, M is member of inherited classes


};





#endif /* PRECOND_H_ */
