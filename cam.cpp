/*
 * cam.cpp
 *
 *  Created on: May 30, 2012
 *      Author: jbalzer
 */


#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include "cam.h"

using namespace std;
using namespace cv;

CCam::CCam() {

	m_size[0] = 0;
	m_size[1] = 0;
	m_f[0] = 0;
	m_f[1] = 0;
	m_c[0] = 0;
	m_c[1] = 0;

	for(size_t i=0;i<5;i++)
		m_k[i]=0;

	m_alpha = 0;

    m_F = Mat::eye(4,4,CV_32FC1);

	m_Finv = m_F;

}

ostream& operator<< (ostream& os, const CCam& x) {

	os << "# dims" << endl;
	os << x.m_size[0] << " " << x.m_size[1] << endl;
	os << "# focal lengths" << endl;
	os << x.m_f[0] << " " << x.m_f[1] << endl;
	os << "# principle point" << endl;
	os << x.m_c[0] << " " << x.m_c[1] << endl;
	os << "# radial distortion coefficients" << endl;
	os << x.m_k[0] << " " << x.m_k[1] << " " << x.m_k[2] << " " << x.m_k[3] << " " << x.m_k[4] << endl;
	os << "# skew coefficient" << endl;
	os << x.m_alpha << endl;
	os << "# frame world -> cam" << endl;

    for(size_t i=0; i<4; i++) {

        for(size_t j=0; j<4; j++) {

            os << x.m_F.at<float>(i,j);

            if(j<3)
                os << " ";

        }

        if(i<3)
            os << endl;

    }

	return os;

}

bool CCam::SaveToFile(const char* filename) {

	ofstream out(filename);

	if(!out) {

		cout << "ERROR: Could not open file " << filename << "." << endl;
		return 1;

	 }

	out << *this;

	out.close();

	return 0;

}


bool CCam::OpenFromFile(const char* filename) {

	ifstream in(filename);

	if(!in) {

		cout << "ERROR: Could not open " << filename << "." << endl;
		return 1;

	 }


	string linebuffer;

	getline(in,linebuffer);

	in >> m_size[0];
	in >> m_size[1];
	in.get();

	getline(in,linebuffer);

	in >> m_f[0];
	in >> m_f[1];
	in.get();

	getline(in,linebuffer);

	in >> m_c[0];
	in >> m_c[1];
	in.get();

	getline(in,linebuffer);

	in >> m_k[0];
	in >> m_k[1];
	in >> m_k[2];
	in >> m_k[3];
	in >> m_k[4];
	in.get();

	getline(in,linebuffer);

	in >> m_alpha;
	in.get();

	getline(in,linebuffer);

    for(size_t i=0; i<4; i++) {

        for(size_t j=0; j<4; j++) {

            in >> m_F.at<float>(i,j);

        }

        in.get();

    }

	in.close();

    m_Finv = m_F.inv();

	return 0;

}


Vec2f CCam::Project(const Vec3f& x) const {

	// transform into camera coordinate system
    Vec3f xc;
    for(size_t i=0; i<3; i++) {

        for(size_t j=0; j<3; j++) {

            xc[i] = m_F.at<float>(i,j)*x[j] + m_F.at<float>(i,3);

        }

    }

    return ProjectLocal(xc);
}


Vec2f CCam::ProjectLocal(const Vec3f& xc) const {

    // result
    Vec2f xp;

    if(xc[2]<0) {

        xp[0] = -1000;
        xp[1] = -1000;
        return xp;

    }

    // pinhole projection
    Vec2f xn;
    xn[0] = xc[0]/xc[2];
    xn[1] = xc[1]/xc[2];

    // radial distortion
    float r = cv::norm(xn);

    Vec2f dx;
    dx[0] = 2*m_k[2]*xn[0]*xn[1] + m_k[3]*(r*r + 2*xn[0]*xn[0]);
    dx[1] = m_k[2]*(r*r + 2*xn[1]*xn[1]) + 2*m_k[3]*xn[0]*xn[1];

    float fac = 1 + m_k[0]*r*r + m_k[1]*r*r*r*r + m_k[4]*r*r*r*r*r*r;
    Vec2f xd = xn*fac + dx;

    // transform to pixel coordinates
    xp[0] = m_f[0]*(xd[0] + m_alpha*xd[1]) + m_c[0];
    xp[1] = m_f[1]*xd[1] + m_c[1];

    return xp;

}

Vec3f CCam::UnProject(const Vec2i& u) const {

    Vec3f x, xc;

    xc = UnProjectLocal(u);

    for(size_t i=0; i<3; i++) {

        for(size_t j=0; j<3; j++) {

            x[i] = m_Finv.at<float>(i,j)*xc[j] + m_Finv.at<float>(i,3);

        }

    }

    return x;

}

Vec3f CCam::UnProjectLocal(const Vec2i& u) const {

    /* CAVEAT: Distortion model is usually not invertible. Here we use
     * the forward model but have to make sure that the distortion
     * coefficients are estimated to fit it.
     */
    Vec2f xd;
    xd[0] = (1/m_f[0])*((float)u[0]-m_c[0]);
    xd[1] = (1/m_f[1])*((float)u[1]-m_c[1]);

    float r = cv::norm(xd);

    Vec2f dx;
    dx[0] = 2*m_k[2]*xd[0]*xd[1] + m_k[3]*(r*r + 2*xd[0]*xd[0]);
    dx[1] = m_k[2]*(r*r + 2*xd[1]*xd[1]) + 2*m_k[3]*xd[0]*xd[1];

    float fac = 1 + m_k[0]*r*r + m_k[1]*r*r*r*r + m_k[4]*r*r*r*r*r*r;
    Vec2f xn = xd*fac + dx;

    Vec3f xc;
    xc[0] = xn[0];
    xc[1] = xn[1];
    xc[2] = 1.0;

    return xc;

}

