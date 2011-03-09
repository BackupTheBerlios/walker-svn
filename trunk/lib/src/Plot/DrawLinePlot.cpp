#include "Plot/DrawLinePlot.h"
#include <boost/bind.hpp>
#include <sgl/Math/Intersection.hpp>
#include <sgl/Math/Ray.hpp>
#include <slon/Graphics/Common.h>
#include <slon/Graphics/Renderable/Debug/DebugDrawCommon.h>
#include <slon/Realm/World.h>

namespace {

    using namespace slon;

    math::Vector3f make_vec3(const math::Vector3f& point)
    {
        return math::Vector3f(point.x, point.y, point.z);
    }

    math::Vector3f make_vec3(const math::Vector2f& point)
    {
        return math::Vector3f(point.x, point.y, 0.0);
    }

    template<typename T, int n>
    math::Matrix<T, n, 1> transform_into(const math::Matrix<T, n, 1>&  vec,
                                         const math::AABB<T, n>&       to)
    {
        return (vec - to.minVec) / to.size();
    }

    template<typename T, int n>
    math::Matrix<T, n, 1> transform_into(const math::Matrix<T, n, 1>&  vec,
                                         const math::AABB<T, n>&       from,
                                         const math::AABB<T, n>&       to)
    {
        return transform_into(vec, from) * to.size() + to.minVec;
    }

    template<typename T, int n>
    math::AABB<T, n> make_client_area();

    template<>
    math::AABB<float, 2> make_client_area<float, 2>()
    {
        return math::AABB2f(0.05f, 0.05f, 0.95f, 0.95f);
    }

    template<>
    math::AABB<float, 3> make_client_area<float, 3>()
    {
        return math::AABB3f(0.05f, 0.05f, 0.05f, 0.95f, 0.95f, 0.95f);
    }

    void drawAxis(graphics::DebugMesh&  debugMesh, 
                  const math::AABB2f&   sourceRegion, 
                  const math::AABB2f&   clientRegion)
    {
        math::Vector2f center = transform_into( math::Vector2f(0.0f, 0.0f), sourceRegion, clientRegion );
        math::Vector2f xAxis  = transform_into( math::Vector2f(sourceRegion.maxVec.x, 0.0f), sourceRegion, clientRegion );
        math::Vector2f yAxis  = transform_into( math::Vector2f(0.0f, sourceRegion.maxVec.y), sourceRegion, clientRegion );

        center = math::min( math::max(center, clientRegion.minVec), clientRegion.maxVec );
        xAxis  = math::min( math::max(xAxis,  clientRegion.minVec), clientRegion.maxVec );
        yAxis  = math::min( math::max(yAxis,  clientRegion.minVec), clientRegion.maxVec );

        float offsX = xAxis.x * 0.02f;
        float offsY = yAxis.y * 0.02f;

        debugMesh << graphics::debug::line(center, xAxis)
                  << graphics::debug::line( xAxis, math::Vector2f(xAxis.x - offsX, xAxis.y + offsY) )
                  << graphics::debug::line( xAxis, math::Vector2f(xAxis.x - offsX, xAxis.y - offsY) )
                  << graphics::debug::line(center, yAxis)
                  << graphics::debug::line( yAxis, math::Vector2f(yAxis.x - offsX, yAxis.y - offsY) )
                  << graphics::debug::line( yAxis, math::Vector2f(yAxis.x + offsX, yAxis.y - offsY) );
    }

    void drawAxis(graphics::DebugMesh&  debugMesh, 
                  const math::AABB3f&   sourceRegion, 
                  const math::AABB3f&   clientRegion)
    {
        math::Vector3f center = transform_into( math::Vector3f(0.0f, 0.0f, 0.0f), sourceRegion, clientRegion );
        math::Vector3f xAxis  = transform_into( math::Vector3f(sourceRegion.maxVec.x, 0.0f, 0.0f), sourceRegion, clientRegion );
        math::Vector3f yAxis  = transform_into( math::Vector3f(0.0f, sourceRegion.maxVec.y, 0.0f), sourceRegion, clientRegion );
        math::Vector3f zAxis  = transform_into( math::Vector3f(0.0f, 0.0f, sourceRegion.maxVec.z), sourceRegion, clientRegion );

        center = math::min( math::max(center, clientRegion.minVec), clientRegion.maxVec );
        xAxis  = math::min( math::max(xAxis,  clientRegion.minVec), clientRegion.maxVec );
        yAxis  = math::min( math::max(yAxis,  clientRegion.minVec), clientRegion.maxVec );
        zAxis  = math::min( math::max(zAxis,  clientRegion.minVec), clientRegion.maxVec );

        debugMesh << graphics::debug::line(center, xAxis)
                  << graphics::debug::line(center, yAxis)
                  << graphics::debug::line(center, zAxis);
    }

} // anonymous namespace

namespace plot {
    
template<int Dimension>
DrawLinePlot<Dimension>::PlotDebugMesh::PlotDebugMesh(DrawLinePlot<Dimension>* plotter_) :
    plotter(plotter_),
    numPlotted(0)
{
}

template<int Dimension>
void DrawLinePlot<Dimension>::PlotDebugMesh::accept(scene::TransformVisitor& visitor)
{
    if (plotter->dirty) 
    {
        plotter->plot();
        numPlotted = plotter->points.size();
    }
}

template<int Dimemsion>
const math::AABBf& DrawLinePlot<Dimemsion>::PlotDebugMesh::getBounds() const
{
    const_cast<this_type*>(this)->aabb.reset_max();
    return aabb;
}

template<int Dimemsion>
DrawLinePlot<Dimemsion>::DrawLinePlot(const DESC& desc_) :
    desc(desc_),
    position(0, 0),
    size(100, 100),
    plotting(false),
    dirty(false),
    hiding(false),
    points(desc_.bufferSize)
{
    plotDebugMesh.reset( new PlotDebugMesh(this) );
    realm::currentWorld().add(plotDebugMesh.get(), false);
}

template<int Dimemsion>
DrawLinePlot<Dimemsion>::~DrawLinePlot()
{
    plotEnd();
    realm::currentWorld().remove( plotObject.get() );
}

template<int Dimension>
void DrawLinePlot<Dimension>::hide(bool toggle)
{
    if (hiding && !toggle)
    {
        realm::currentWorld().add( plotObject.get() );
        hiding = false;
    }
    else if (!hiding && toggle)
    {
        realm::currentWorld().remove( plotObject.get() );
        hiding = true;
    }
}

template<int Dimension>
void DrawLinePlot<Dimension>::plot()
{
    // perform in main thread
    graphics::DebugMesh& debugMesh = *plotDebugMesh;
    debugMesh.clear();
    debugMesh << graphics::debug::depth_test(false);

    // calculate bounds if needed
    if (desc.autoBounds)
    {
        desc.bounds.reset_min();
        std::for_each( points.begin(), points.end(), boost::bind(&bound_type::extend, &desc.bounds, _1) );
    }

    // setup transform
    {
        sgl::rectangle viewport   = graphics::currentDevice()->Viewport();
        math::AABB3f   viewBounds( float(viewport.x), 
                                   float(viewport.y), 
                                   -1.0f,
                                   float(viewport.x + viewport.width), 
                                   float(viewport.y + viewport.height),
                                   1.0f );
        
        math::Vector3f scaling      = math::Vector3f( float(size.x), float(size.y), 1.0f );
        math::Vector3f translation  = math::Vector3f( float(position.x), float(position.y), 1.0f );
        math::Matrix4f transform    = math::make_translation(translation.x, translation.y, translation.z)
                                      * math::make_scaling(scaling.x, scaling.y, scaling.z);

        debugMesh << graphics::debug::projection( math::make_ortho(viewBounds.minVec.x, 
                                                                   viewBounds.maxVec.x, 
                                                                   viewBounds.minVec.y, 
                                                                   viewBounds.maxVec.y, 
                                                                   viewBounds.minVec.z, 
                                                                   viewBounds.maxVec.z) )
                  << graphics::debug::transform(transform);
    }

    // draw axises
    bound_type identBounds = make_client_area<float, Dimension>();
    debugMesh << graphics::debug::color(desc.axisColor);
    drawAxis(debugMesh, desc.bounds, identBounds);

    // plot line
    pointBufferMutex.lock();
    {
        debugMesh << graphics::debug::color(desc.lineColor);
        for (int i = 0; i<int(points.size() - 1); ++i)
        {
            if ( math::contains(desc.bounds, points[i]) )
            {
                if ( math::contains(desc.bounds, points[i+1]) ) 
                {
                    debugMesh << graphics::debug::line( transform_into(points[i],   desc.bounds, identBounds), 
                                                        transform_into(points[i+1], desc.bounds, identBounds) );
                }
                else
                {
                    // find intersection with rectangle region               
                    point_type intPoint;
                    math::find_intersection(desc.bounds, points[i], points[i+1], intPoint);
                    debugMesh << graphics::debug::line( transform_into(points[i], desc.bounds, identBounds), 
                                                        transform_into(intPoint,  desc.bounds, identBounds) );
                }
            }
            else if ( math::contains(desc.bounds, points[i+1]) )
            {
                // find intersection with rectangle region               
                point_type intPoint;
                math::find_intersection(desc.bounds, points[i], points[i+1], intPoint);
                debugMesh << graphics::debug::line( transform_into(intPoint,    desc.bounds, identBounds), 
                                                    transform_into(points[i+1], desc.bounds, identBounds) );
            }
        }

        dirty = false;
    }
    pointBufferMutex.unlock();
}

template<int Dimemsion>
void DrawLinePlot<Dimemsion>::reset(const DESC& desc_)
{
    desc = desc_;
}

template<int Dimemsion>
void DrawLinePlot<Dimemsion>::setRegion(const math::Vector2i& position_, const math::Vector2i& size_)
{
    position = position_;
    size     = size_;
}

template<int Dimemsion>
void DrawLinePlot<Dimemsion>::plotBegin()
{
    plotting = true;
}

template<int Dimemsion>
void DrawLinePlot<Dimemsion>::plotEnd()
{
    plotting = false;
}

template<int Dimemsion>
void DrawLinePlot<Dimemsion>::plotPoint(const point_type& value)
{
    assert(plotting);
    pointBufferMutex.lock();
    {
        points.push_back(value);
        dirty = true;
    }
    pointBufferMutex.unlock();
}

// explicit instantiation
template class DrawLinePlot<2>;
template class DrawLinePlot<3>;

} // namespace plot
