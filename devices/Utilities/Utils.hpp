/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef HUMANDYNAMICSESTIMATION_UTILS_HPP
#define HUMANDYNAMICSESTIMATION_UTILS_HPP

// iDynTree
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Rotation.h>
#include <iDynTree/Core/Transform.h>

/**
 * Helper for iDynTree library.
 */
namespace iDynTreeHelper
{
    namespace Rotation
    {
        /**
         * Transform a 3x3 matrix into a skew-symmetric matrix.
         * @param input is a 3x3 matrix;
         * @return a 3x3 skew-symmetric matrix
         */
        iDynTree::Matrix3x3 skewSymmetric(const iDynTree::Matrix3x3& input);

        iDynTree::Vector3 skewVee(const iDynTree::Matrix3x3& input);


        class rotationDistance;
    }
}


class iDynTreeHelper::Rotation::rotationDistance
{
private:
    iDynTree::Rotation rotation1;
    iDynTree::Rotation rotation2;
public:
	rotationDistance();
    rotationDistance(const rotationDistance& rotationDistance);
    rotationDistance(const iDynTree::Rotation rotation1, const iDynTree::Rotation rotation2);
    rotationDistance(const iDynTree::Transform transform1, const iDynTree::Transform transform2);

    iDynTree::Rotation asRotation();
    iDynTree::Vector3 asRPY();
    iDynTree::Vector4 asQuaternion();
    iDynTree::Vector3 asSkewVee();

    double asEuclideanDistanceOfEulerAngles();
    double asTrace();
};


#endif
