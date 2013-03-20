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
#include <vector>

#define Z_MIN_MM 300

class CDepthColorSensor;

/*! \brief camera model
 *
 *
 *
 */
class CCam {

    friend class CDepthColorSensor;

public:



	//! Constructor.
	CCam();

    //! Parametrized constructor.
    CCam(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F);

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

class CDepthCam:public CCam {

    friend class CDepthColorSensor;

public:

    //! Standard constructor.
    CDepthCam();

    //! Parametrized constructor.
    CDepthCam(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a);

    //! Converts the disparity from the depth sensor into a metric depth.
    float DisparityToDepth(size_t i, size_t j, float d);

private:

    float m_d[2];               //! disparity inversion parameters
    cv::Mat m_D;                //! spatial distortion pattern
    float m_a[2];               //! distance weights of distortion pattern

};

#endif /* CAM_H_ */
