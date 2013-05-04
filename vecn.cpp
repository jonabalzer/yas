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

#include "vecn.h"
#include <algorithm>
#include <assert.h>
#include <iostream>

using namespace std;

template <typename T, u_int n>
CVector<T,n>::CVector() {

    fill_n(m_data,n,0);

}

template <typename T, u_int n>
CVector<T,n>::CVector(T val) {

    fill_n(m_data,n,val);

}

template <typename T, u_int n>
T& CVector<T,n>::operator ()(u_int i) {

    assert(i<n);

    return m_data[i];

}

template <typename T,u_int n>
void CVector<T,n>::operator +=(const CVector<T,n>& x) {

    for(u_int i=0; i<n; i++)
        m_data[i] += x.m_data[i];

}

template <typename T,u_int n>
void CVector<T,n>::operator*=(const CVector<T,n>& x) {

    for(u_int i=0; i<n; i++)
        m_data[i] *= x.m_data[i];

}

template <typename T, u_int n>
T CVector<T,n>::Get(u_int i) const {

    assert(i<n);

    return m_data[i];

}

template <typename T, u_int n>
double CVector<T,n>::Norm(double p) const {

    assert(p>0);

    double norm = 0;

    for(u_int i=0; i<n; i++)
        norm += pow(fabs((double)m_data[i]),p);

    return pow(norm,1/p);

}


template <typename T, u_int n>
double CVector<T,n>::Norm2() const {

    double norm = 0;

    for(u_int i=0; i<n; i++)
        norm += (double)m_data[i]*(double)m_data[i];

    return sqrt(norm);

}

template<typename T,u_int n>
bool CVector<T,n>::Normalize() {

    double norm = Norm2();

    T normc = (T)norm;

    cout << normc << endl;
    if(norm>0) {

        for(u_int i=0; i<n; i++)
            m_data[i] /= normc;

        return 0;

    }

    return 1;

}

template <class U, u_int m>
ostream& operator << (ostream& os, const CVector<U,m>& x) {

    for(u_int i=0; i<m-1; i++)
        os << (double)x.Get(i) << " ";

    os << (double)x.Get(m-1);

    return os;

}

template class CVector<float,3>;
template class CVector<double,3>;
template class CVector<float,2>;
template class CVector<double,2>;
template ostream& operator << (ostream& os, const CVector<float,3>& x);
template ostream& operator << (ostream& os, const CVector<double,3>& x);
template ostream& operator << (ostream& os, const CVector<float,2>& x);
template ostream& operator << (ostream& os, const CVector<double,2>& x);

