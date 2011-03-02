#ifndef __WALKER_YARD_LEARNING_NEURAL_NETWORK_HPP__
#define __WALKER_YARD_LEARNING_NEURAL_NETWORK_HPP__

#include "function.hpp"
#include <boost/bind.hpp>
#include <boost/random.hpp>
#include <boost/shared_array.hpp>

namespace learn {
    
// forward
namespace io {
    class access;
}

/** Feed forward neural network. Learns using back propagation algorithm. */
template< typename ValueType		= float,
          typename ActivateFunction	= logistic<ValueType> >
class neural_network
{
friend class io::access;
public:
    struct link;
    struct neuron;
	struct layer;

	typedef ValueType			value_type;
    typedef ActivateFunction	activate_function;

    typedef boost::shared_array<link>		link_array;
    typedef boost::shared_array<neuron>		neuron_array;
	typedef boost::shared_array<layer>		layer_array;

	typedef link*		    link_iterator;
	typedef neuron*		    neuron_iterator;
	typedef layer*		    layer_iterator;

	typedef const link*     link_const_iterator;
	typedef const neuron*   neuron_const_iterator;
	typedef const layer*    layer_const_iterator;

    struct link
    {
    friend class io::access;
    friend class neural_network;
	public:
        link( neuron_iterator	source_ = neuron_iterator(),
			  neuron_iterator	target_ = neuron_iterator(),
              value_type		weight_ = value_type() ) :
            source(source_),
            target(target_),
            weight(weight_)
        {}

        void set_source(neuron_iterator source_) { source = source_; }
        void set_target(neuron_iterator target_) { target = target_; }

        neuron_iterator source_neuron() { return source; }
        neuron_iterator target_neuron() { return target; }

        neuron_const_iterator source_neuron() const { return source; }
        neuron_const_iterator target_neuron() const { return target; }

    public:
        value_type			weight;

    private:
        neuron_iterator		source;
		neuron_iterator		target;
    };

	class neuron
    {
    friend class io::access;
    friend class neural_network;
	public:
        typedef ActivateFunction				            activate_function_t;
        typedef function_with_derivative<activate_function> function_with_derivative_t;

    public:
        neuron( link_iterator               firstLink_ = link_iterator(),
			    link_iterator               endLink_   = link_iterator(),
                function_with_derivative_t  func       = function_with_derivative_t() ) :
			input( value_type() ),
            output( value_type() ),
            errorGradient( value_type() ),
            activateFunc(func),
			firstLink(firstLink_),
			endLink(endLink_)
        {
		}

		/** Setup links range */
		void set_range(link_iterator firstLink_, 
                       link_iterator endLink_)
		{
			firstLink = firstLink_;
			endLink   = endLink_;
		}		
        
        size_t num_links() const    { return std::distance(firstLink, endLink); }
        link_iterator first_link()  { return firstLink; }
        link_iterator end_link()    { return endLink; }

        link_const_iterator first_link() const { return firstLink; }
        link_const_iterator end_link() const   { return endLink; }

    public:
        mutable value_type          input;
        mutable value_type          output;
        mutable value_type          gradient;
        mutable value_type          errorGradient;
        function_with_derivative_t	activateFunc;

    private:
		// topology
        link_iterator	firstLink;
		link_iterator	endLink;
    };

    class layer
    {
    friend class io::access;
    friend class neural_network;
    public:
        layer( neuron_iterator firstNeuron_ = neuron_iterator() ,
		       neuron_iterator endNeuron_   = neuron_iterator() ) :
            firstNeuron(firstNeuron_),
			endNeuron(endNeuron_)
        {}

		size_t num_neurons() const		{ return std::distance(firstNeuron, endNeuron); }
        neuron_iterator first_neuron()	{ return firstNeuron; }
        neuron_iterator end_neuron()	{ return endNeuron; }

        neuron_const_iterator first_neuron() const  { return firstNeuron; }
        neuron_const_iterator end_neuron() const    { return endNeuron; }

        neuron& get_neuron(unsigned i)  
        { 
            assert(firstNeuron + i < endNeuron);
            return *(firstNeuron + i); 
        }

        const neuron& get_neuron(unsigned i) const 
        { 
            assert(firstNeuron + i < endNeuron);
            return *(firstNeuron + i); 
        }

		/** Setup neurons range */
		void set_range(neuron_iterator firstNeuron_, 
                       neuron_iterator endNeuron_)
		{
			firstNeuron = firstNeuron_;
			endNeuron   = endNeuron_;
		}

    private:
		// topology
        neuron_iterator	firstNeuron;
		neuron_iterator	endNeuron;
    };

public:
    neural_network(bool useIndent_ = false) :
        useIndent(useIndent_)
    {
    }

    neural_network(const neural_network& nn) :
		numLayers(nn.numLayers),
		numNeurons(nn.numNeurons),
		numLinks(nn.numLinks),
        useIndent(nn.useIndent),
		layers(new layer[numLayers]),
		neurons(new neuron[numNeurons]),
		links(new link[numLinks])
    {
		for (size_t i = 0; i<numLayers; ++i) 
		{
			layers[i]			  = nn.layers[i];
			layers[i].firstNeuron = neurons.get() + std::distance( nn.neurons.get(), nn.layers[i].firstNeuron );
			layers[i].endNeuron   = neurons.get() + std::distance( nn.neurons.get(), nn.layers[i].endNeuron );
		}

		for (size_t i = 0; i<numNeurons; ++i) 
		{
			neurons[i]			 = nn.neurons[i];
			neurons[i].firstLink = links.get() + std::distance( nn.links.get(), nn.neurons[i].firstLink );
			neurons[i].endLink   = links.get() + std::distance( nn.links.get(), nn.neurons[i].endLink );
		}

		for (size_t i = 0; i<numLinks; ++i) 
		{
			links[i]		= nn.links[i];
			links[i].source = neurons.get() + std::distance( nn.neurons.get(), nn.links[i].source );
			links[i].target = neurons.get() + std::distance( nn.neurons.get(), nn.links[i].target );
		}
    }

	neural_network& operator = (const neural_network& nn)
	{
		neural_network tmp(nn);
		this->swap(tmp);
		return *this;
	}

	void swap(neural_network& nn) throw()
	{
		std::swap(numLayers,    nn.numLayers);
		std::swap(numNeurons,   nn.numNeurons);
		std::swap(numLinks,     nn.numLinks);
		std::swap(useIndent,    nn.useIndent);

		layers.swap(nn.layers);
		neurons.swap(nn.neurons);
		links.swap(nn.links);
	}

    size_t num_inputs() const       { return layers[0].num_neurons(); }
    size_t num_outputs() const      { return layers[numLayers - 1].num_neurons(); }
    size_t num_params() const       { return numLinks; }
    size_t num_components() const   { return layers[numLayers - 1].num_neurons(); }

    template<typename InIterator, typename OutIterator>
    void compute(InIterator     beginInput,
                 InIterator     endInput,
                 OutIterator    out) const
    {
        feed(beginInput, endInput);
		for (neuron_iterator neuronIter  = layers[numLayers - 1].first_neuron(); 
							 neuronIter != layers[numLayers - 1].end_neuron(); 
							 ++neuronIter) 
		{
            *out++ = neuronIter->output;
        }
    }

    template<typename InIterator1, typename InIterator2>
    void update( InIterator1    beginInput,
                 InIterator1    endInput,
                 InIterator2    beginNewOut,
                 InIterator2    endNewOut,
                 value_type     stepSize )
    {
        feed(beginInput, endInput);
        backpropagate(beginNewOut, endNewOut, stepSize);
    }

    template<typename InIterator>
    void set_params(InIterator beginParam,
                    InIterator endParam)
    {
        for (link_iterator linkIter  = first_link(); 
					       linkIter != end_link(); 
					       ++linkIter, ++beginParam) 
	    {
            assert(beginParam != endParam);
            linkIter->weight = *beginParam;
        }
    }

    template<typename OutIterator>
    void get_params(OutIterator params)
    {
        for (link_iterator linkIter  = first_link(); 
					       linkIter != end_link(); 
					       ++linkIter) 
	    {
            *params++ = linkIter->weight;
        }
    }

    template<typename InIterator, typename OutIterator>
    void get_gradient(InIterator beginInput, InIterator endInput, OutIterator outGrad)
    {
        std::vector<value_type> out(num_outputs());
        compute(beginInput, endInput, out.begin());

        // compute last layer gradients
        neuron_iterator neuronIter;
        for (neuronIter  = layers[numLayers-1].first_neuron(); 
             neuronIter != layers[numLayers-1].end_neuron();
             ++neuronIter)
        {
            assert( neuronIter != layers[numLayers-1].end_neuron() );
            neuronIter->errorGradient = neuronIter->gradient;
        }

        // compute neuron gradients
        for (int i = numLayers - 2; i >= 0; --i)
        {
            for (neuronIter  = layers[i].first_neuron();
                 neuronIter != layers[i].end_neuron();
                 ++neuronIter)
            {
                neuronIter->errorGradient = value_type();
                for (link_iterator linkIter  = neuronIter->first_link(); 
							       linkIter != neuronIter->end_link(); 
							       ++linkIter) 
			    {
                    neuronIter->errorGradient += linkIter->target->errorGradient * linkIter->weight;
                }
                neuronIter->errorGradient *= neuronIter->gradient;
            }
        }
        for (link_iterator linkIter  = first_link(); 
					       linkIter != end_link(); 
					       ++linkIter) 
	    {
            *outGrad++ = linkIter->target->errorGradient * linkIter->source->output;
        }
    }

    /** Remove all layers from neural network */
    void clear() 
    {
        layers.clear();
        numLinks = 0;
    }

	/** Allocate neural network */
	void allocate( size_t numLayers_,
			       size_t numNeurons_,
				   size_t numLinks_ )
	{
		numLayers  = numLayers_;
		numNeurons = numNeurons_;
		numLinks   = numLinks_;

		layers.reset(new layer[numLayers]);
		neurons.reset(new neuron[numNeurons]);
		links.reset(new link[numLinks]);
	}

    size_t num_links() const    { return numLinks; }
    size_t num_neurons() const  { return numNeurons; }
    size_t num_layers() const   { return numLayers; }

    layer_iterator  first_layer()    { return layers.get(); }
    layer_iterator  end_layer()      { return layers.get() + numLayers; }

    neuron_iterator first_neuron()   { return neurons.get(); }
    neuron_iterator end_neuron()     { return neurons.get() + numNeurons; }

    link_iterator   first_link()     { return links.get(); }
    link_iterator   end_link()       { return links.get() + numLinks; }

    layer_const_iterator  first_layer() const    { return layers.get(); }
    layer_const_iterator  end_layer() const      { return layers.get() + numLayers; }

    neuron_const_iterator first_neuron() const   { return neurons.get(); }
    neuron_const_iterator end_neuron() const     { return neurons.get() + numNeurons; }

    link_const_iterator   first_link() const     { return links.get(); }
    link_const_iterator   end_link() const       { return links.get() + numLinks; }

private:
    template<typename InIterator>
    void feed(InIterator beginInput,
              InIterator endInput) const
    {
        assert(numLayers > 0);

        // feed input
		neuron_iterator neuronIter = layers[0].first_neuron();
        for (InIterator iter  = beginInput;
                        iter != endInput;
                        ++iter, ++neuronIter)
        {
            assert( neuronIter != layers[0].end_neuron() );
            neuronIter->input  = *iter;
            neuronIter->output = *iter;
        }

        // feed 1.0 to the threshold neuron
        if (useIndent) 
        {
            assert( neuronIter != layers[0].end_neuron() );
            neuronIter->input  = value_type(1);
            neuronIter->output = value_type(1);
        }

        // through the net
        for (neuron_iterator neuronIter  = layers[0].first_neuron();
                             neuronIter != layers[0].end_neuron();
                             ++neuronIter)
        {
		    for (link_iterator linkIter  = neuronIter->firstLink; 
						       linkIter != neuronIter->endLink; 
						       ++linkIter)
		    {
			    linkIter->target->input += neuronIter->output * linkIter->weight;
		    }
        }

        for (size_t i = 1; i<numLayers; ++i) 
		{
            for (neuron_iterator neuronIter  = layers[i].first_neuron();
                                 neuronIter != layers[i].end_neuron();
                                 ++neuronIter)
            {
                typename neuron::function_with_derivative_t::result_type res = neuronIter->activateFunc(neuronIter->input);

			    neuronIter->output   = res.first;
                neuronIter->gradient = res.second;
                neuronIter->input    = value_type();

                for (link_iterator linkIter  = neuronIter->firstLink; 
							       linkIter != neuronIter->endLink; 
							       ++linkIter)
			    {
				    linkIter->target->input += neuronIter->output * linkIter->weight;
			    }
            }
		}
    }

    template<typename InIterator>
    void backpropagate(InIterator beginOut,
                       InIterator endOut,
                       value_type stepSize)
    {
        assert(numLayers > 0);

        // compute last layer gradients
        neuron_iterator neuronIter = layers[numLayers-1].first_neuron();
        for (InIterator out  = beginOut;
                        out != endOut;
                        ++out, ++neuronIter)
        {
            assert( neuronIter != layers[numLayers-1].end_neuron() );
            neuronIter->errorGradient = neuronIter->gradient * ( (*out)  - neuronIter->output );
        }

        // compute neuron gradients
        for (int i = numLayers - 2; i >= 0; --i)
        {
            for (neuronIter  = layers[i].first_neuron();
                 neuronIter != layers[i].end_neuron();
                 ++neuronIter)
            {
                neuronIter->errorGradient = value_type();
                for (link_iterator linkIter  = neuronIter->first_link(); 
							       linkIter != neuronIter->end_link(); 
							       ++linkIter) 
			    {
                    neuronIter->errorGradient += linkIter->target->errorGradient * linkIter->weight;
                }
                neuronIter->errorGradient *= neuronIter->gradient;
            }
        }

        // correct weights        
        for (link_iterator linkIter  = first_link(); 
					       linkIter != end_link(); 
					       ++linkIter) 
	    {
            linkIter->weight += stepSize * linkIter->target->errorGradient * linkIter->source->output;
        }
    }

private:
	// topology
	size_t			numLayers;
	size_t			numNeurons;
	size_t			numLinks;
    bool            useIndent;

	layer_array		layers;
	neuron_array	neurons;
	link_array		links;
};

/** Set all weights to constant. */
template<typename T>
struct const_weight_distributor :
    public std::unary_function<T, void>
{
    const_weight_distributor( T val_ = T(0.5) ) :
        val(val_)
    {}

    T operator () (void) const { return val; }

    T val;
};

/** Setup weights using uniform distribution. Avoids zero weight values. */
template<typename T>
struct uniform_weight_distributor :
    public std::unary_function<T, void>
{
    typedef boost::uniform_real<T>                                  distribution;
    typedef boost::variate_generator<boost::mt19937&, distribution> variate_generator;

    static boost::mt19937 rng;

    uniform_weight_distributor( T minVal = T(-0.5),
                                T maxVal = T(0.5),
                                T guard_ = T(0.1) ) :
        generator( rng, distribution(minVal, maxVal) ),
        guard(guard_)
    {
    }

    T operator () (void) 
    { 
        T weight = generator();
        if (weight < T(0.0) && weight > -guard) {
            return -guard;
        }
        else if (weight > T(0.0) && weight < guard) {
            return guard;
        }

        return weight;
    }

    variate_generator   generator;
    T                   guard;
};

template<typename T>
boost::mt19937 uniform_weight_distributor<T>::rng;

template< typename ValueType,
          typename ActivateFunction >
struct neural_network_composer
{
public:
    typedef ValueType                                       value_type;
    typedef ActivateFunction                                activate_function;
    typedef neural_network<value_type, activate_function>   neural_network_t;

private:
    struct link
    {
        size_t      tl, tn;
        value_type  weight;

        link( size_t        tl      = 0, 
              size_t        tn      = 0,
              value_type    weight  = value_type() ) :
            tl(tl),
            tn(tn),
            weight(weight)
        {}
    };

    struct neuron
    {
        std::vector<link> links;

        neuron(size_t numLinks = 0) :
            links(numLinks)
        {}
    };

    struct layer
    {
        std::vector<neuron> neurons;

        layer(size_t numNeurons = 0) :
            neurons(numNeurons)
        {}
    };

public:
    neural_network_composer() :
        numNeurons(0),
        numLinks(0)
    {}

    void add_layer(size_t numLayerNeurons = 0)
    {
        layers.push_back( layer(numLayerNeurons) );
        numNeurons += numLayerNeurons;
    }
    
    void add_neuron(size_t layer, size_t numNeuronLinks = 0)
    {
        layers[layer].neurons.push_back( neuron(numNeuronLinks) );
        numLinks += numNeuronLinks;
        ++numNeurons;
    }

    void add_link(size_t sl, size_t sn, size_t tl, size_t tn, value_type weight)
    {
        layers[sl].neurons[sn].links.push_back( link(tl, tn, weight) );
        ++numLinks;
    }

    size_t get_neuron_id(size_t l, size_t n) const
    {
        size_t id = n;
        for (size_t i = 0; i<l; ++i) {
            id += layers[i].neurons.size();
        }

        return id;
    }

    neural_network_t make_neural_network(bool useThreshold) const
    {
        neural_network_t nn(useThreshold);
        nn.allocate( layers.size(), numNeurons, numLinks );

        // distribute neurons
        {
            typename neural_network_t::layer_iterator  currentLayer    = nn.first_layer();
            typename neural_network_t::neuron_iterator currentNeuron   = nn.first_neuron();
            typename neural_network_t::link_iterator   currentLink     = nn.first_link();
            for (size_t i = 0; i<layers.size(); ++i, ++currentLayer)
            {
                const layer& l = layers[i];
                currentLayer->set_range( currentNeuron, currentNeuron + layers[i].neurons.size() );
                for (size_t j = 0; j < l.neurons.size(); ++j, ++currentNeuron)
                {
                    const neuron& n = l.neurons[j];
                    currentNeuron->set_range( currentLink, currentLink + layers[i].neurons[j].links.size() );
                    for (size_t k = 0; k < n.links.size(); ++k, ++currentLink)
                    {
                        const link& l = n.links[k];
                        currentLink->set_source(currentNeuron);
                        currentLink->set_target(nn.first_neuron() + get_neuron_id(l.tl, l.tn));
                        currentLink->weight = l.weight;
                    }
                }
            }
        }

        return nn;
    }

private:
    size_t              numNeurons;
    size_t              numLinks;
    std::vector<layer>  layers;
};

template<typename ValueType,
         typename ActivateFunction>
void make_cascade_neural_network( neural_network<ValueType, ActivateFunction>&  neuralNetwork,
                                  unsigned                                      numInputs,
                                  unsigned                                      numOutputs,
                                  unsigned                                      numLayers,
                                  bool                                          useIndent)
{
    assert(numLayers > 0);

    // typedefs
    typedef neural_network_composer<ValueType, ActivateFunction>    nn_composer;
    typedef typename nn_composer::neural_network_t                  neural_network;

    // add layers 
    nn_composer nnComposer;

    if (useIndent) {
        nnComposer.add_layer(numInputs + 1);
    }
    else {
        nnComposer.add_layer(numInputs);
    }

    for (unsigned i = 1; i<numLayers; ++i) {
        nnComposer.add_layer(numInputs);
    }
    nnComposer.add_layer(numOutputs);

    // make cascade
    for (unsigned i = 0; i<numLayers-1; ++i)
    {
        for (unsigned j = 0; j<numInputs; ++j)
        {
            // indent
            if (useIndent) {
                nnComposer.add_link( 0, numInputs, i+1, j, 0.5f );
            }

            // cascade
            for (unsigned k = 0; k<numInputs; ++k) {
                nnComposer.add_link( i, j, i+1, k, 0.5f );
            }
        }
    }

    // make output links
    for (unsigned j = 0; j<numInputs; ++j)
    {
        // cascade
        for (unsigned k = 0; k<numOutputs; ++k) {
            nnComposer.add_link( numLayers - 1, j, numLayers, k, 0.5f );
        }
    }

    // make output indent links
    for (unsigned j = 0; j<numOutputs; ++j)
    {        
        if (useIndent) {
            nnComposer.add_link( 0, numInputs, numLayers, j, 0.5f );
        }
    }

    // compose neural network
    neural_network nn = nnComposer.make_neural_network(useIndent);
    neuralNetwork.swap(nn);
}

template<typename ValueType,
         typename ActivateFunction,
         typename WeightDistributor>
void distribute_weights(neural_network<ValueType, ActivateFunction>&  neuralNetwork,
                        WeightDistributor                             weightDistributor)
{
    typedef neural_network<ValueType, ActivateFunction> neural_network;
    for (typename neural_network::link_iterator linkIter  = neuralNetwork.first_link();
                                                linkIter != neuralNetwork.end_link();
                                                ++linkIter)
    {
        linkIter->weight = weightDistributor();
    }
}

template<typename ValueType,
         typename ActivateFunction,
         typename Function>
void setup_function(neural_network<ValueType, ActivateFunction>&  neuralNetwork,
                    unsigned                                      layer,
                    Function                                      activateFunc)
{
    typedef neural_network<ValueType, ActivateFunction> neural_network;
    for (typename neural_network::neuron_iterator neuronIter  = (neuralNetwork.first_layer() + layer)->first_neuron();
                                                  neuronIter != (neuralNetwork.first_layer() + layer)->end_neuron();
                                                  ++neuronIter)
    {
        neuronIter->activateFunc = activateFunc;
    }
}

} // namespace learn

#endif // __WALKER_YARD_LEARNING_NEURAL_NETWORK_HPP__
