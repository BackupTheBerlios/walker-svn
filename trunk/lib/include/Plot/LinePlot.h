#ifndef __WALKER_YARD_PLOT_LINE_PLOT_H__
#define __WALKER_YARD_PLOT_LINE_PLOT_H__

#include <slon/Utility/referenced.hpp>
#include <boost/intrusive_ptr.hpp>
#include <sgl/Math/Matrix.hpp>

namespace plot {

/** Line plotter interface.
 * @tparam ValueType - type of the point coordinates.
 * @tparam Dimension - dimensionality of the point coordinates.
 */
template<typename ValueType, int Dimension>
class LinePlot :
    public slon::Referenced
{
public:
    typedef math::Matrix<ValueType, Dimension, 1>   point_type;

public:
    /** Start new plot */
    virtual void plotBegin() = 0;

    /** End plot */
    virtual void plotEnd() = 0;

    /** Check wether plotter is plotting */
    virtual bool isPlotting() const = 0;

    /** Add point to the line. */
    virtual void plotPoint(const point_type& value) = 0;

    virtual ~LinePlot() {}
};

typedef LinePlot<float, 3>  LinePlot3D;
typedef LinePlot<float, 2>  LinePlot2D;

typedef boost::intrusive_ptr<LinePlot2D>    line_plot_2d_ptr;
typedef boost::intrusive_ptr<LinePlot3D>    line_plot_3d_ptr;
    
} // namespace plot

#endif // __WALKER_YARD_PLOT_LINE_PLOT_H__
