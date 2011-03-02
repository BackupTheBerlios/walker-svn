#ifndef __WALKER_YARD_PLOT_DUMP_LINE_PLOT_H__
#define __WALKER_YARD_PLOT_DUMP_LINE_PLOT_H__

#include "LinePlot.h"
#include <boost/shared_ptr.hpp>
#include <ostream>

namespace plot {

/** Generates matlab script for plotting line. */
template<typename ValueType, int Dimension>
class DumpLinePlot :
    public LinePlot<ValueType, Dimension>
{
public:
    typedef math::Matrix<ValueType, Dimension, 1>   point_type;
    typedef boost::shared_ptr<std::ostream>         ostream_ptr;

public:
    DumpLinePlot( const ostream_ptr& ostream = ostream_ptr() );
    ~DumpLinePlot();

    // Override LinePlot
    void plotBegin();
    void plotEnd();
    bool isPlotting() const { return plotting; }
    void plotPoint(const point_type& value);

private:
    bool        plotting;
    ostream_ptr ostream;
};
    
// typedef
typedef DumpLinePlot<float,  2>   DumpLinePlot2D;
typedef DumpLinePlot<float,  3>   DumpLinePlot3D;

} // namespace plot

#endif // __WALKER_YARD_PLOT_DUMP_LINE_PLOT_H__
