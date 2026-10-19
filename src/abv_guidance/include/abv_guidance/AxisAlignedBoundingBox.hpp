#ifndef AXISALIGNEDBOUNDINGBOX_HPP
#define AXISALIGNEDBOUNDINGBOX_HPP

struct AxisAlignedBoundingBox
{
    double mXMin;
    double mXMax;
    double mYMin;
    double mYMax;

    bool overlaps(const AxisAlignedBoundingBox& aOther) const
    {
        return mXMin <= aOther.mXMax && mXMax >= aOther.mXMin &&
               mYMin <= aOther.mYMax && mYMax >= aOther.mYMin;
    }

    // true iff aOther lies fully within this box
    bool contains(const AxisAlignedBoundingBox& aOther) const
    {
        return aOther.mXMin >= mXMin && aOther.mXMax <= mXMax &&
               aOther.mYMin >= mYMin && aOther.mYMax <= mYMax;
    }

    static AxisAlignedBoundingBox fromCircle(double aX, double aY, double aRadius)
    {
        return AxisAlignedBoundingBox{aX - aRadius, aX + aRadius, aY - aRadius, aY + aRadius};
    }
};

#endif // AXISALIGNEDBOUNDINGBOX_HPP
