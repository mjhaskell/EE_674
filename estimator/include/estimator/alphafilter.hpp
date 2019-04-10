#ifndef ALPHAFILTER_HPP
#define ALPHAFILTER_HPP

class AlphaFilter
{
public:
    AlphaFilter(double alpha=0.5, double y0=0.0) :
        m_alpha{alpha},
        m_y{y0}
    {
    }

    double update(double u)
    {
        m_y = m_alpha*m_y + (1.0 - m_alpha)*u;
        return m_y;
    }

private:
    double m_alpha;
    double m_y;
};

#endif /* ALPHAFILTER_HPP */
