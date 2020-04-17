/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.jumprobotics.robot.math;

/**
 * Add your docs here.
 */
import org.jumprobotics.robot.math.Rotation3;
import org.jumprobotics.robot.math.Vector3;

/**
 * A transform that represents a translation and rotation in 3d space
 */
public class RigidTransform3 {

    /**
     * The translation of the transform
     */
    public final Vector3 translation;

    /**
     * The rotation of the transform
     */
    public final Rotation3 rotation;

    /**
     * Creates a new rigid transform from a translation and a rotation
     * @param translation
     * @param rotation
     */
    public RigidTransform3(Vector3 translation, Rotation3 rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    /**
     * Adds this rigid transform and another one together
     * @param other The rigid transform to add
     * @return
     */
    public RigidTransform3 add(RigidTransform3 other) {
        return new RigidTransform3(translation.add(other.translation), rotation.add(other.rotation));
    }

    /**
     * Gives the inverse of the rigid transform by negating the translation and inverting the rotation matrix of the rotation
     * @return The inverted rigid transform
     */
    public RigidTransform3 inverse() {
        return new RigidTransform3(this.translation.negate(), this.rotation.inverse());
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        return translation.toString() + ", " + rotation.toString();
    }
}