
package org.janelia.geometry3d;

/**
 *
 * @author Christopher Bruns <brunsc at janelia.hhmi.org>
 */
public enum CoordinateAxis {
    X, Y, Z;

    /** 
     * private constructor so there can be only three
     */
    private CoordinateAxis() {}

    /**
     * Return true if neither \a axis2 nor \a axis3 is the same as this
    axis nor each other; that is, (this,axis2,axis3) together cover all three
    axes.
     * @param axis2
     * @param axis3
     * @return 
     */
    public boolean areAllDifferentAxes(CoordinateAxis axis2, CoordinateAxis axis3) {
        return isDifferentAxis(axis2) 
                && isDifferentAxis(axis3) 
                && axis2.isDifferentAxis(axis3);
    }

    public boolean areAllSameAxes(CoordinateAxis axis2, CoordinateAxis axis3) {
        return this.isSameAxis(axis2) && this.isSameAxis(axis3);
    }
    
    public CoordinateAxis fromOrdinal(int ordinal) {
        // circular restriction to range [0,2]
        return CoordinateAxis.values()[ordinal % 3];
    }

    public CoordinateAxis getNextAxis() {
        return fromOrdinal(ordinal() + 1);
    }

    public CoordinateAxis getPreviousAxis() {
        return fromOrdinal(ordinal() + 2);
    }

    public String getName() {
        if (this == CoordinateAxis.X) {
            return "X";
        } else if (this == CoordinateAxis.Y) {
            return "Y";
        }
        return "Z";
    }
    
    /**
     * Given this coordinate axis and one other, return the missing one:
        - XAxis.getThirdAxis(YAxis) returns ZAxis (and vice versa)
        - XAxis.getThirdAxis(ZAxis) returns YAxis (and vice versa)
        - YAxis.getThirdAxis(ZAxis) returns XAxis (and vice versa)
     * @param axis2 A coordinate axis that must be distinct from the
        current one; it is a fatal error to provide the same axis.
     * @return The unmentioned third axis.
     */
    public CoordinateAxis getThirdAxis(CoordinateAxis axis2) {
       assert( isDifferentAxis(axis2) );
       CoordinateAxis nextAxis = getNextAxis();
       return nextAxis.isDifferentAxis(axis2) ? nextAxis : axis2.getNextAxis();
    }

    /**
     * Return true if the given \a axis2 is the one preceding this one as
    would be reported by getPreviousAxis().
     * @param axis2
     * @return true if the given \a axis2 is the one preceding this one as
    would be reported by getPreviousAxis()
     */
    public boolean isPreviousAxis(CoordinateAxis axis2) {
        return getPreviousAxis().ordinal() == axis2.ordinal();
    }

    public boolean isDifferentAxis(CoordinateAxis axis2) {
        return ! isSameAxis(axis2);
    }

    /**
     * Return true if the given \a axis2 is the one following this one in a
    reverse cyclical direction, that is, if \a axis2 is the one that would be
    reported by getPreviousAxis().
     * @param axis2
     * @return 
     */
    public boolean isReverseCyclical(CoordinateAxis axis2) {
        return isPreviousAxis(axis2);
    }

    public boolean isSameAxis(CoordinateAxis rhs) {
        return this.ordinal() == rhs.ordinal();
    }

    public boolean isXAxis() {
        return isSameAxis(CoordinateAxis.X);
    }

    public boolean isYAxis() {
        return isSameAxis(CoordinateAxis.Y);
    }

    public boolean isZAxis() {
        return isSameAxis(CoordinateAxis.Z);
    }

}
