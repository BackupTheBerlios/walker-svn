#include "Plot/DumpLinePlot.h"

namespace plot {

template<typename ValueType, int Dimemsion>
DumpLinePlot<ValueType, Dimemsion>::DumpLinePlot(const ostream_ptr& ostream_) :
    ostream(ostream_),
    plotting(false)
{
}

template<typename ValueType, int Dimemsion>
DumpLinePlot<ValueType, Dimemsion>::~DumpLinePlot()
{
    plotEnd();
}

template<typename ValueType, int Dimemsion>
void DumpLinePlot<ValueType, Dimemsion>::plotBegin()
{
    assert(ostream);
    plotting = true;
}

template<typename ValueType, int Dimemsion>
void DumpLinePlot<ValueType, Dimemsion>::plotEnd()
{
    plotting = false;
}

template<typename ValueType, int Dimemsion>
void DumpLinePlot<ValueType, Dimemsion>::plotPoint(const point_type& value)
{
    assert(plotting);
    (*ostream) << value << std::endl;
}

// explicit instantiation
template class DumpLinePlot<float,  2>;
template class DumpLinePlot<float,  3>;
template class DumpLinePlot<double, 2>;
template class DumpLinePlot<double, 3>;

} // namespace plot
