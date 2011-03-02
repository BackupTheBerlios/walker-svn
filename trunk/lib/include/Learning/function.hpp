#ifndef __WALKER_YARD_LEARNING_FUNCTION_HPP__
#define __WALKER_YARD_LEARNING_FUNCTION_HPP__

#include <boost/shared_ptr.hpp>
#include <cmath>
#include <functional>
#include <vector>

namespace learn {

/** Derivative of the function */
template<typename Function>
class derivative;

/** Function with derivative. Simultaneous calculation of the function
 * value and derivative may be faster.
 */
template<typename Function>
class function_with_derivative
{
public:
    typedef Function                                function_type;
    typedef derivative<function_type>               derivative_type;

    typedef typename function_type::argument_type   argument_type;
    typedef std::pair<argument_type, argument_type> result_type;
    
    function_with_derivative( function_type     func_  = function_type(),
                              derivative_type   deriv_ = derivative_type() ) :
        func(func_),
        deriv(deriv_)
    {
    }

    result_type operator () (argument_type x) const
    {
        return result_type( func(x), deriv(x) );
    }

    bool operator == (const function_with_derivative& func_) const { return func == func_.func && deriv == func_.deriv; }
    bool operator != (const function_with_derivative& func_) const { return func != func_.func || deriv != func_.deriv; }

public:
    function_type   func;
    derivative_type deriv;
};

/** Linear function */
template<typename T>
class linear :
    public std::unary_function<T, T>
{
public:
    bool operator == (const linear&) const { return true; }
    bool operator != (const linear&) const { return false; }

public:
    T operator () (T x) const
    {
        return x;
    }
};

/** Derivative of linear function */
template<typename T>
class derivative< linear<T> > :
    public std::unary_function<T, T>
{
public:
    bool operator == (const derivative< linear<T> >&) const { return true; }
    bool operator != (const derivative< linear<T> >&) const { return false; }

public:
    T operator () (T /*x*/) const
    {
        return T(1.0);
    }
};

/** Logistic function: logistic(x) = 1 / ( 1 + exp(-x) ) */
template<typename T>
class logistic :
    public std::unary_function<T, T>
{
public:
    bool operator == (const logistic&) const { return true; }
    bool operator != (const logistic&) const { return false; }

public:
    T operator () (T x) const
    {
        return T(1.0) / ( T(1.0) + exp(-x) );
    }
};

/** Logistic derivative: logistic'(x) = logistic(x) * ( 1 - sigmoid(x) ) */
template<typename T>
class derivative< logistic<T> > :
    public std::unary_function<T, T>
{
public:
    typedef logistic<T> function_type;

    derivative() {}
    derivative(function_type func_) :
        func(func_)
    {}

    T operator () (T x) const
    {
        return func(x) * ( T(1.0) - func(x) );
    }

    bool operator == (const derivative< logistic<T> >& func_) const { return func == func_.func; }
    bool operator != (const derivative< logistic<T> >& func_) const { return func != func_.func; }

public:
    function_type func;
};

/** Logistic function paired with derivative. */
template<typename T>
class function_with_derivative< logistic<T> >
{
public:
    typedef logistic<T>                 function_type;
    typedef derivative< logistic<T> >   derivative_type;

    typedef T                           argument_type;
    typedef std::pair<T, T>             result_type;

    function_with_derivative( function_type func_ = function_type() ) :
        func(func_)
    {
    }

    result_type operator () (argument_type x) const
    {
        typename function_type::result_type value = func(x);
        return result_type( value, value * (T(1.0) - value) );
    }

    bool operator == (const function_with_derivative< logistic<T> >& func_) const { return func == func_.func; }
    bool operator != (const function_with_derivative< logistic<T> >& func_) const { return func != func_.func; }

public:
    function_type   func;
};

/** Hiperbolic tangent function: tanh(x) = sh(x) / ch(x) = (exp(x) - exp(-x)) / (exp(x) + exp(-x)) */
template<typename T>
class hyperbolic_tangent :
    public std::unary_function<T, T>
{
public:
    bool operator == (const hyperbolic_tangent&) const { return true; }
    bool operator != (const hyperbolic_tangent&) const { return false; }

public:
    T operator () (T x) const
    {
        return tanh(x);
    }
};

/** Hiperbolic tangent derivative: tanh'(x) = 1 / ch^2(x) */
template<typename T>
class derivative< hyperbolic_tangent<T> > :
    public std::unary_function<T, T>
{
public:
    typedef hyperbolic_tangent<T>   function_type;

    derivative() {}
    derivative(function_type htangent_) :
        htangent(htangent_) {}

    T operator () (T x) const
    {
        return T(1.0) - htangent(x) * htangent(x);
    }

    bool operator == (const derivative< hyperbolic_tangent<T> >& func) const { return htangent == func.htangent; }
    bool operator != (const derivative< hyperbolic_tangent<T> >& func) const { return htangent != func.htangent; }

public:
    function_type htangent;
};

/** Hiperbolic tangent function paired with derivative. */
template<typename T>
class function_with_derivative< hyperbolic_tangent<T> >
{
public:
    typedef hyperbolic_tangent<T>               function_type;
    typedef derivative< hyperbolic_tangent<T> > derivative_type;

    typedef T                                   argument_type;
    typedef std::pair<T, T>                     result_type;
    
    function_with_derivative( function_type func_ = function_type() ) :
        func(func_)
    {
    }

    result_type operator () (argument_type x) const
    {
        typename function_type::result_type value = func(x);
        return result_type(value, argument_type(1.0) - value * value);
    }

    bool operator == (const function_with_derivative< hyperbolic_tangent<T> >& func_) const { return func == func_.func; }
    bool operator != (const function_with_derivative< hyperbolic_tangent<T> >& func_) const { return func != func_.func; }

public:
    function_type   func;
};

/** Base class for virtual functions */
template<typename Argument, typename Result>
class abstract_function :
    public std::unary_function<Argument, Result>
{
public:
    virtual Result operator () (Argument x) = 0;

    virtual ~abstract_function() {}
};

/** Function wrapper to make functor abstract. */
template<typename Functor>
class abstract_functor :
    public abstract_function<typename Functor::argument_type, typename Functor::result_type>
{
public:
    typedef Functor function_type;

    abstract_functor( function_type func_ = function_type() ) :
        func(func_)
    {}

    typename Functor::result_type operator () (typename Functor::argument_type x)
    {
        return func(x);
    }

    bool operator == (const abstract_functor& func_) const { return func == func_.func; }
    bool operator != (const abstract_functor& func_) const { return func != func_.func; }

    function_type func;
};

/** Make functor abstract */
template<typename Functor>
abstract_functor<Functor> make_abstract(Functor func)
{
    return abstract_functor<Functor>(func);
}

/** Copyable wrapper around virtual fuction */
template<typename T>
class generic_function :
    public std::unary_function<T, T>
{
public:
    typedef abstract_function<T, T>             function_type;
    typedef boost::shared_ptr<function_type>    function_ptr;

    generic_function() {}
    generic_function(function_ptr func_) :
        func(func_)
    {}

    T operator () (T x)
    {
        assert(func);
        (*func)(x);
    }

    function_ptr func;
};

/** Make functor generic */
template<typename Functor>
generic_function<typename Functor::result_type> make_generic(Functor func)
{
    typedef generic_function<typename Functor::result_type> generic_function;
    typedef typename generic_function::function_ptr         abstract_function_ptr;

    abstract_function_ptr function( new abstract_functor<Functor>(func) );
    return generic_function(function);
}

/** Copyable wrapper around virtual function derivative */
template<typename T>
class derivative< generic_function<T> > :
    public std::unary_function<T, T>
{
public:
    typedef abstract_function<T, T>             function_type;
    typedef boost::shared_ptr<function_type>    function_ptr;

    derivative() {}
    derivative(function_ptr func_) :
        func(func_)
    {}

    T operator () (T x)
    {
        assert(func);
        (*func)(x);
    }

    function_ptr func;
};

/** Make functor derivative generic */
template<typename Functor>
derivative< generic_function<typename Functor::result_type> > make_generic_derivative(Functor func)
{
    typedef generic_function<typename Functor::result_type>     generic_function;
    typedef derivative<generic_function>                        generic_function_derivative;
    typedef typename generic_function_derivative::function_ptr  abstract_function_ptr;

    abstract_function_ptr function( new abstract_functor< derivative<Functor> >( derivative<Functor>(func) ) );
    return generic_function_derivative(function);
}

/** Copyable wrapper around virtual function with derivative */
template<typename T>
class function_with_derivative< generic_function<T> > :
    public std::unary_function< T, std::pair<T, T> >
{
public:
    typedef std::unary_function< T, std::pair<T, T> >   base_type;
    typedef abstract_function< T, std::pair<T, T> >     function_type;
    typedef boost::shared_ptr<function_type>            function_ptr;

public:
    function_with_derivative() {}
    function_with_derivative(function_ptr func_) :
        func(func_)
    {}

    typename base_type::result_type operator () (typename base_type::argument_type x)
    {
        assert(func);
        return (*func)(x);
    }

    function_ptr func;
};

/** Make functor derivative generic */
template<typename Functor>
function_with_derivative< generic_function<typename Functor::result_type> > make_generic_function_with_derivative(Functor func)
{
    typedef generic_function<typename Functor::result_type>         generic_function;
    typedef function_with_derivative<generic_function>              generic_function_with_derivative;
    typedef typename generic_function_with_derivative::function_ptr abstract_function_ptr;

    abstract_function_ptr function( new abstract_functor< function_with_derivative<Functor> >( function_with_derivative<Functor>(func) ) );
    return generic_function_with_derivative(function);
}

template<typename T>
class multi_function
{
public:
    typedef T   result_type;
    typedef T   value_type;
};

template<typename T>
class abstract_multi_function :
    public multi_function<T>
{
public:
    virtual T operator () (const T* begin, const T* end) = 0;

    virtual ~abstract_multi_function() {}
};

/** Function wrapper to make multi functor abstract. */
template<typename Functor>
class abstract_multi_functor :
    public abstract_multi_function<typename Functor::value_type>
{
public:
    typedef Functor function_type;

    abstract_multi_functor( function_type func_ = function_type() ) :
        func(func_)
    {}

    typename Functor::result_type operator () (const typename Functor::value_type*  begin,
                                               const typename Functor::value_type*  end)
    {
        return func(begin, end);
    }

    bool operator == (const abstract_multi_functor& func_) const { return func == func_.func; }
    bool operator != (const abstract_multi_functor& func_) const { return func != func_.func; }

    function_type func;
};

/** Make functor abstract */
template<typename Functor>
abstract_multi_functor<Functor> make_multi_abstract(Functor func)
{
    return abstract_multi_functor<Functor>(func);
}

template<typename T>
class select :
    public multi_function<T>
{
public:
    select(int index_ = 0) : 
        index(index_)
    {}

    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   end)
    {
        T ret = *(begin + index);
        return ret;
    }

    bool operator == (const select& func_) const { return index == func_.index; }
    bool operator != (const select& func_) const { return index != func_.index; }

    int index;
};

template<typename T>
class select_anti_quad :
    public multi_function<T>
{
public:
    select_anti_quad(int index_ = 0) : 
        index(index_)
    {}

    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   end)
    {
        T ret = *(begin + index)* abs(*(begin + index));
        return ret;
    }

    bool operator == (const select_anti_quad& func_) const { return index == func_.index; }
    bool operator != (const select_anti_quad& func_) const { return index != func_.index; }

    int index;
};

template<typename T>
class constant :
    public multi_function<T>
{
public:
    constant( T val_ = T() ) :
        val(val_)
    {}

    template<typename Iterator>
    T operator () (Iterator   /*begin*/,
                   Iterator   /*end*/)
    {
        return val;
    }

    bool operator == (const constant& func_) const { return val == func_.val; }
    bool operator != (const constant& func_) const { return val != func_.val; }

    T val;
};

template<typename T>
class sum :
    public multi_function<T>
{
public:
    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   end)
    {
        T ret = T();
        for (Iterator i = begin; i != end; ++i)
        {
            ret += *i;
        }
        return ret;
    }

    bool operator == (const sum&) const { return true; }
    bool operator != (const sum&) const { return false; }
};

template<typename T>
class select_product :
    public multi_function<T>
{
public:
    select_product(int _first = 0, int _second = 0) : 
        first(_first), 
        second(_second)
    {}

    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   /*end*/)
    {
        T ret = (*(begin + first)) * (*(begin + second));
        return ret;
    }

    bool operator == (const select_product& func_) const { return first == func_.first && second == func_.second; }
    bool operator != (const select_product& func_) const { return first != func_.first || second != func_.second; }

public:
    int first;
    int second;
};

template<typename T>
class select_sum :
    public multi_function<T>
{
public:
    select_sum(int _first = 0, int _second = 0) : 
        first(_first), 
        second(_second)
    {}

    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   /*end*/)
    {
        T ret = (*(begin + first)) + (*(begin + second));
        return ret;
    }

    bool operator == (const select_sum& func_) const { return first == func_.first && second == func_.second; }
    bool operator != (const select_sum& func_) const { return first != func_.first || second != func_.second; }

public:
    int first;
    int second;
};

template<typename T>
class select_sum3 :
    public multi_function<T>
{
public:
    select_sum3(int _first = 0, int _second = 0, int _third = 0) : 
        first(_first), 
        second(_second),
        third(_third)
    {}

    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   /*end*/)
    {
        T ret = (*(begin + first)) + (*(begin + second)) + (*(begin + third));
        return ret;
    }

    bool operator == (const select_sum3& func_) const { return first == func_.first && second == func_.second && third == func_.third; }
    bool operator != (const select_sum3& func_) const { return first != func_.first || second != func_.second || third != func_.third; }

public:
    int first;
    int second;
    int third;
};


template<typename T>
class select_subtract :
    public multi_function<T>
{
public:
    select_subtract(int _first = 0, int _second = 0) : 
        first(_first), 
        second(_second)
    {}

    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   /*end*/)
    {
        T ret = (*(begin + first)) - (*(begin + second));
        return ret;
    }

    bool operator == (const select_subtract& func_) const { return first == func_.first && second == func_.second; }
    bool operator != (const select_subtract& func_) const { return first != func_.first || second != func_.second; }

public:
    int first;
    int second;
};

template<typename T>
class gaussian :
    public multi_function<T>
{
public:
    gaussian() : 
        sigma( T() ) 
    {}

    template<typename Iterator>
    gaussian(Iterator meanBegin, 
             Iterator meanEnd, 
             T        sigma_) : 
        mean(meanBegin, meanEnd), 
        sigma(sigma_) 
    {}

    template<typename Iterator>
    T operator () (Iterator   begin,
                   Iterator   end)
    {
        T distance = T();
        typename std::vector<T>::iterator j = mean.begin();
        for (Iterator i = begin; i != end; ++i, ++j)
        {
            distance += (*i - *j) * (*i - *j);
        }
        return std::exp(-distance/(2*sigma*sigma));
    }

    bool operator == (const gaussian& func_) const 
    { 
        if ( mean.size() != func_.mean.size() ) {
            return false;
        }

        for (size_t i = 0; i<mean.size(); ++i) 
        {
            if (mean[i] != func_.mean[i]) {
                return false;
            }
        }

        return sigma == func_.sigma;
    }

    bool operator != (const gaussian& func_) const 
    {
        return !(*this == func_);
    }

public:
    std::vector<T>  mean;
    T               sigma;
};

/** Copyable wrapper around virtual fuction */
template<typename T>
class generic_multi_function :
    public multi_function<T>
{
public:
    typedef abstract_multi_function<T>          function_type;
    typedef boost::shared_ptr<function_type>    function_ptr;

    generic_multi_function() {}
    generic_multi_function(function_ptr func_) :
        func(func_)
    {}

    template<typename Iterator>
    T operator () (Iterator begin, Iterator end) const
    {
        std::vector<T> tmp(begin, end);
        return (*func)( &tmp[0], &tmp[0] + tmp.size() );
    }

    T operator () (const T* begin, const T* end) const
    {
        return (*func)(begin, end);
    }

    T operator () (const typename std::vector<T>::const_iterator& begin,
                   const typename std::vector<T>::const_iterator& end) const
    {
        return (*func)( &(*begin), &(*end) );
    }

    function_ptr func;
};

/** Make functor generic */
template<typename Functor>
generic_multi_function<typename Functor::result_type> make_multi_generic(Functor func)
{
    typedef generic_multi_function<typename Functor::result_type>   generic_function;
    typedef typename generic_function::function_ptr                 abstract_function_ptr;

    abstract_function_ptr function( new abstract_multi_functor<Functor>(func) );
    return generic_function(function);
}

} // namespace learn

#endif // __WALKER_YARD_LEARNING_FUNCTION_HPP__
