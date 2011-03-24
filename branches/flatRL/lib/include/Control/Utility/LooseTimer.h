#ifndef __WALKER_YARD_CONTROL_LOOSE_TIMER_H__
#define __WALKER_YARD_CONTROL_LOOSE_TIMER_H__

#include <boost/noncopyable.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <slon/Thread/StartStopTimer.h>
#include <vector>

namespace ctrl {

class LooseTimer :
    public slon::Timer
{
private:
	typedef std::vector<double> double_vector;

public:
    LooseTimer(double tolerance = 0.000001);

    bool	isPaused() const;
    void    togglePause(bool toggle);

	bool	isRealtime() const;
	void	toggleRealtime(bool toggle);

    // Override Timer
    double	getTime() const;
    void	setTime(double time_);
    void	setTimeScale(double timeScale_);

	bool	isBounded() const;

    double  getMaxTime() const;
	double  getTimeScale() const;
	double  getTolerance() const;

	void    addBarrier(double barrier);
	void    removeBarrier(double barrier);

private:
	bool	myIsBounded() const;
    double  myGetMaxTime() const;
    void	mySetTime(double time);

private:
	double_vector	barriers;
    bool            pause;
	bool			realtime;
    mutable double  time;
    double          timeScale;
	double			tolerance;

    mutable boost::shared_mutex mutex;
    mutable slon::delta_timer   deltaTimer;
};

typedef boost::intrusive_ptr<LooseTimer>		loose_timer_ptr;
typedef boost::intrusive_ptr<const LooseTimer>	const_loose_timer_ptr;

class TimeBarrier :
	public boost::noncopyable
{
public:
	TimeBarrier()
	{
	}

	TimeBarrier(const loose_timer_ptr& timer_, double val_)
	:	timer(timer_)
	,	val(val_)
	{
		assert(timer);
		timer->addBarrier(val);
	}

	~TimeBarrier()
	{
		if (timer) {
			timer->removeBarrier(val);
		}
	}

	void swap(TimeBarrier& other) throw()
	{
		timer.swap(other.timer);
		std::swap(val, other.val);
	}

	double value() const { return val; }

	void reset() 
	{ 
		if (timer) 
		{
			timer->removeBarrier(val);
			timer.reset(); 
		}
	}

	operator bool () { return (timer.get() != 0); }

private:
	loose_timer_ptr	timer;
	double			val;
};

} // namespace ctrl

#endif // __WALKER_YARD_CONTROL_LOOSE_TIMER_H__
