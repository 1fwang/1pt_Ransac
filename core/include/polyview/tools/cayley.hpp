/******************************************************************************
 * Author:   Laurent Kneip                                                    *
 * Contact:  kneip.laurent@gmail.com                                          *
 * License:  Copyright (c) 2016 Laurent Kneip. All rights reserved.           *
 *                                                                            *
 * This is proprietary software. Use or redistribution, with or without       *
 * modifications, is not permitted unless an individual licensing agreement   *
 * with the owner has been made. Please contact the owner.                    *
 ******************************************************************************/

/**
 * \file cayley.hpp
 * \brief Functions for back-and-forth transformation between rotation matrices
 *        and Cayley-parameters.
 */

#ifndef POLYVIEW_TOOLS_CAYLEY_HPP_
#define POLYVIEW_TOOLS_CAYLEY_HPP_

#include <stdlib.h>
#include <Eigen/Eigen>

/**
 * \brief The namespace of this library.
 */
namespace polyview
{
/**
 * \brief The namespace of the tools.
 */
namespace tools
{

/**
 * \brief Compute a rotation matrix from Cayley-parameters, following [14].
 *
 * \param[in] cayley The Cayley-parameters of a rotation.
 * \return The 3x3 rotation matrix.
 */
Eigen::Matrix3d cayley2rot( const Eigen::Vector3d & cayley);

/**
 * \brief Compute the Cayley-parameters of a rotation matrix, following [14].
 *
 * \param[in] R The 3x3 rotation matrix.
 * \return The Cayley-parameters.
 */
Eigen::Vector3d rot2cayley( const Eigen::Matrix3d & R );

}
}

#endif /* POLYVIEW_TOOLS_CAYLEY_HPP_ */
