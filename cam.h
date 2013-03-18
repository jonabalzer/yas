/*
 * cam.h
 *
 *  Created on: May 30, 2012
 *      Author: jbalzer
 */

#ifndef CAM_H_
#define CAM_H_

#include <stdlib.h>
#include <opencv2/opencv.hpp>

/*! \brief camera model
 *
 *
 *
 */
class CCam {

public:

	//! Constructor.
	CCam();

	/*! \brief Projects a point into the image plane.
	 *
	 * \param[in] x point in world coordinates
	 * \returns point in pixel coordinates
	 *
	 */
    cv::Vec2f Project(const cv::Vec3f& x) const;

	/*! \brief Projects a point into the image plane without using the extrinsics.
	 *
	 * \param[in] xc point in camera coordinates
	 * \returns point in pixel coordinates
	 *
	 */
    cv::Vec2f ProjectLocal(const cv::Vec3f& xc) const;

	/*! \brief Converts a pixel into a viewing direction (in world coordinates).
	 *
	 * \param[in] u location w.r.t. the pixel coordinate system
	 * \returns direction vector, normalized s.t. z-component equals 1
	 *
	 */
    cv::Vec3f UnProject(const cv::Vec2i& u) const;

	/*! \brief Converts a pixel into a viewing direction (in camera coordinates).
	 *
	 * \param[in] u location w.r.t. the pixel coordinate system
	 * \returns direction vector, normalized s.t. z-component equals 1
	 *
	 */
    cv::Vec3f UnProjectLocal(const cv::Vec2i& u) const;

	/*! \brief Reads camera parameters from a file.
	 *
	 * \param[in] filename file name
	 *
	 */
	bool OpenFromFile(const char* filename);

	//! Writes camera parameters to file.
	bool SaveToFile(const char* filename);

	//! Writes the camera parameters to a stream.
	friend std::ostream& operator << (std::ostream& os, const CCam& x);



protected:

    size_t m_size[2];			//!< pixel size
    float m_f[2];				//!< focal length
    float m_c[2];				//!< principle point
    float m_alpha;				//!< skew coefficient
    float m_k[5];				//!< distortion coefficients
    cv::Mat m_F;                //!< frame world-to-cam
    cv::Mat m_Finv;				//!< inverse cam-to-world frame

};


#endif /* CAM_H_ */
