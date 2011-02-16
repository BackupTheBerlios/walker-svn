#ifndef __WALKER_YARD_PLOT_DRAW_LINE_PLOT_H__
#define __WALKER_YARD_PLOT_DRAW_LINE_PLOT_H__

#include "LinePlot.h"
#include <boost/circular_buffer.hpp>
#define NOMINMAX
#include <boost/thread/mutex.hpp>
#include <sgl/Font.h>
#include <slon/Graphics/Renderable/DebugMesh.h>
#include <slon/Realm/Object.h>

namespace plot {

using namespace slon;

/** Draws plot using debug render. */
template<int Dimension>
class DrawLinePlot :
    public LinePlot<float, Dimension>
{
public:
    typedef DrawLinePlot<Dimension>             this_type;
    typedef math::Matrix<float, Dimension, 1>   point_type;
    typedef math::AABB<float, Dimension>        bound_type;
    typedef boost::circular_buffer<point_type>  point_buffer;

private:
    friend class PlotDebugMesh;
    class PlotDebugMesh :
        public graphics::DebugMesh
    {
    public:
        typedef PlotDebugMesh       this_type;
        typedef graphics::DebugMesh base_type;

    public:
        PlotDebugMesh(DrawLinePlot<Dimension>* plotter);

        // Override Entity
        void accept(scene::TransformVisitor& visitor);
        const math::AABBf& getBounds() const;

    private:
        DrawLinePlot<Dimension>*    plotter;
        size_t                      numPlotted;
    };
    typedef boost::intrusive_ptr<PlotDebugMesh> plot_debug_mesh_ptr;

public:
    struct DESC
    {
        math::Vector4f  axisColor;
        math::Vector4f  lineColor;
        bool            autoBounds;             // determine bounds automatically
        bound_type      bounds;                 // bounds of the plot
        point_type      axisGranularity;        // granularity of axis marks
        std::string     axisName[Dimension];    // name of the axis
        size_t          bufferSize;             // maximum number of points in the line

        DESC() :
            axisColor(1.0f, 1.0f, 1.0f, 1.0f),
            lineColor(1.0f, 1.0f, 1.0f, 1.0f),
            autoBounds(true),
            bufferSize(1000)
        {
            bounds.reset_min();
        }
    };

public:
    DrawLinePlot(const DESC& desc);
    ~DrawLinePlot();

    /** Replot graphic with new plot description */
    void reset(const DESC& desc);

    /** Setup position and size of the plot in window coordinates */
    void setRegion(const math::Vector2i& position, const math::Vector2i& size);

    /** Hide or draw plot */
    void hide(bool toggle = true);

    // Override LinePlot
    void plotBegin();
    void plotEnd();
    bool isPlotting() const { return plotting; }
    void plotPoint(const point_type& value);

private:
    void plot();

private:
    DESC            desc;
    math::Vector2i  position;
    math::Vector2i  size;
    bool            plotting;
    bool            dirty;
    bool            hiding;
    point_buffer    points;

    // mutex for accesing point buffer
    boost::mutex    pointBufferMutex;

    // drawing
    plot_debug_mesh_ptr plotDebugMesh;
    realm::object_ptr   plotObject;
};

typedef DrawLinePlot<3>  DrawLinePlot3D;
typedef DrawLinePlot<2>  DrawLinePlot2D;

typedef boost::intrusive_ptr<DrawLinePlot2D>    draw_line_plot_2d_ptr;
typedef boost::intrusive_ptr<DrawLinePlot3D>    draw_line_plot_3d_ptr;
    
} // namespace plot

#endif // __WALKER_YARD_PLOT_DRAW_LINE_PLOT_H__
