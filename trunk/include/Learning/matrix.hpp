#ifndef __WALKER_YARD_MATRIX_HPP__
#define __WALKER_YARD_MATRIX_HPP__

#include <vector>


namespace learn {
template< typename T,
          typename Container = std::vector<T> >
struct matrix
{
public:
    typedef T           value_type;
    typedef Container   container;

public:
    matrix(){}
    matrix( unsigned    _width,
            unsigned    _height ) :
        values(_width*_height),
        width(_width),
        height(_height)
    {}

    matrix( Container   _values,
            unsigned    _width,
            unsigned    _height ) :
        values(_values),
        width(_width),
        height(_height)
    {}

    template<typename Iterator>
    matrix( Iterator    _values,
            unsigned    _width,
            unsigned    _height ) :
        values(_values, _values + _width * _height),
        width(_width),
        height(_height)
    {}

    template<typename MatrixType>
    matrix(unsigned _width,
           unsigned _height,
           const MatrixType& _matrix ) :
        width(_width),
        height(_height),
        values(_width*_height)
    {
        for (size_t i = 0; i < height; ++i)
        {
            for (size_t j = 0; j < width; ++j)
            {
                values[i*width + j] = _matrix[i][j];
            }
        }
    }

    const T& operator() (unsigned i, unsigned j) const
    {
        assert(i < height && j < width);
        return values[i*width + j];
    }

    T& operator () (unsigned i, unsigned j)
    {
        assert(i < height && j < width);
        return values[i*width + j];
    }

public:
    Container   values;
    unsigned    width;
    unsigned    height;
};

}

#endif // __WALKER_YARD_MATRIX_HPP__